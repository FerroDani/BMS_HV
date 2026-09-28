#!/usr/bin/env python3
"""
Hardware-in-the-loop style regression of the BMS HV master firmware.

The real ARM firmware (ELF built from the repository) runs in Renode on an
STM32F446 model; the two L9963T transceivers and the L9963E daisy chain are
emulated by L9963Sim.cs (see that file for the behaviour that is modelled).

Usage:  run_tests.py [scenario-name-substring ...]   (default: all)
Output: <out>/<scenario>/{uart.txt,can.csv,trace.txt,renode.txt} + results.json/.md
"""
import json
import os
import re
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
FW = os.environ.get("BMS_FW", os.path.abspath(os.path.join(HERE, "..", "..")))
OUT = os.environ.get("BMS_TEST_OUT", os.path.join(HERE, "out"))
RENODE = os.environ.get("RENODE", "renode")   # Renode 1.15.x (portable dotnet build tested)
BLD = os.path.join(OUT, "build")

# ----------------------------------------------------------------------------- build
VARIANTS = {
    "n1": "-DBMS_N_SLAVES=1",
    "n11": "-DBMS_N_SLAVES=11",
    "n12": "-DBMS_N_SLAVES=12",
    "n12dual": "-DBMS_N_SLAVES=12 -DBMS_DUAL_RING=1",
    "n1dual": "-DBMS_N_SLAVES=1 -DBMS_DUAL_RING=1",
    "n11fsm": "-DBMS_N_SLAVES=11 -DBMS_ENABLE_FSM=1",
}


def build(variant):
    bdir = os.path.join(BLD, variant)
    os.makedirs(bdir, exist_ok=True)
    defs = VARIANTS[variant]
    stamp = os.path.join(bdir, "defs.txt")
    if os.path.exists(stamp) and open(stamp).read() != defs:
        subprocess.run(["rm", "-rf", bdir])
        os.makedirs(bdir)
    open(stamp, "w").write(defs)
    r = subprocess.run(["make", "-j8", f"BUILD_DIR={bdir}", f"BMS_DEFS={defs}"], cwd=FW,
                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    warnings = [l for l in r.stdout.splitlines() if "warning:" in l]
    if r.returncode != 0:
        print(r.stdout[-3000:])
        raise SystemExit(f"build {variant} failed")
    return os.path.join(bdir, "bms_hv_fsm.elf"), warnings


# ----------------------------------------------------------------------------- run
def run_renode(name, elf, slaves, steps, trace=False):
    d = os.path.join(OUT, name)
    os.makedirs(d, exist_ok=True)
    for f in ("uart.txt", "can.csv", "trace.txt"):
        p = os.path.join(d, f)
        if os.path.exists(p):
            os.remove(p)
    repl = open(os.path.join(HERE, "renode", "bms_hv.repl.in")).read().replace("@SLAVES@", str(slaves))
    open(os.path.join(d, "bms_hv.repl"), "w").write(repl)
    lines = [
        f"include @{HERE}/renode/L9963Sim.cs",
        f"include @{HERE}/renode/BxCanStub.cs",
        'mach create "bms"',
        f"machine LoadPlatformDescription @{d}/bms_hv.repl",
        "sysbus.cpu PerformanceInMips 180",
        'emulation SetGlobalQuantum "0.0001"',
        f"$elf=@{elf}",
        'macro reset',
        '"""',
        '    sysbus LoadELF $elf',
        '"""',
        "runMacro $reset",
        f"sysbus.usart3 CreateFileBackend @{d}/uart.txt true",
        f"sysbus.can1 OpenLog @{d}/can.csv",
        "logLevel 3",
    ]
    if trace:
        lines.append(f"sysbus.spi3.l9963th OpenTrace @{d}/trace.txt")
    t_virtual = 0.0
    for st in steps:
        kind, arg = st
        if kind == "run":
            lines.append(f'emulation RunFor "{arg}"')
            t_virtual += arg
        elif kind == "cmd":
            lines.append(arg)
        elif kind == "snap":
            lines.append(f'echo "SNAP {arg} {t_virtual:.3f}"')
            lines.append("sysbus.spi3.l9963th Summary")
            lines.append("sysbus.gpioPortC ReadDoubleWord 0x14")
            lines.append("sysbus.gpioPortE ReadDoubleWord 0x14")
            lines.append("sysbus.can1 TxCount")
            lines.append(f'echo "ENDSNAP {arg}"')
    lines.append("quit")
    open(os.path.join(d, "test.resc"), "w").write("\n".join(lines) + "\n")
    t0 = time.time()
    r = subprocess.run([RENODE, "--disable-xwt", "--console", "--plain", os.path.join(d, "test.resc")],
                       stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                       timeout=1800)
    open(os.path.join(d, "renode.txt"), "w").write(r.stdout)
    wall = time.time() - t0
    return d, r.stdout, wall


# ----------------------------------------------------------------------------- parse
LOG_RE = re.compile(r"^\[\s*(\d+)\.(\d{3})\] (.*)$")


def parse_uart(d):
    out = []
    p = os.path.join(d, "uart.txt")
    if not os.path.exists(p):
        return out
    for l in open(p, errors="replace"):
        m = LOG_RE.match(l.rstrip("\r\n"))
        if m:
            out.append((int(m.group(1)) + int(m.group(2)) / 1000.0, m.group(3)))
    return out


def parse_snaps(renode_out):
    snaps = {}
    cur = None
    for l in renode_out.splitlines():
        l = l.strip()
        m = re.match(r"^SNAP (\S+) ([\d.]+)$", l)
        if m:
            cur = {"t": float(m.group(2)), "model": [], "hex": []}
            snaps[m.group(1)] = cur
            continue
        if l.startswith("ENDSNAP"):
            cur = None
            continue
        if cur is None:
            continue
        if l.startswith("MODEL"):
            cur["model"].append(l)
        elif re.match(r"^0x[0-9A-Fa-f]+$", l):
            cur["hex"].append(int(l, 16))
    for s in snaps.values():
        h = s["hex"] + [0, 0, 0]
        s["odr_c"], s["odr_e"], s["can_tx"] = h[0], h[1], h[2]
        s["ams_pin"] = (s["odr_c"] >> 6) & 1
        s["err_led"] = (s["odr_e"] >> 5) & 1
        s["warn_led"] = (s["odr_e"] >> 3) & 1
        s["stat2_led"] = (s["odr_e"] >> 4) & 1
        s["dev"] = {}
        s["trx"] = {}
        for m in s["model"]:
            mm = re.match(r"MODEL S(\d+): (\w+)\s+id=\s*(\d+) portH=(\d) (\w+) lock=(\d) ct=(\d) rx=(\d+) ans=(\d+) badcrc=(\d+) spdmis=(\d+) wake=(\d+) sleep=(\d+)\(to (\d+)\) swrst=(\d+) conv=(\d+) override=(\d+) maxgap=(\d+) gpiomsk=0x([0-9A-F]+)", m)
            if mm:
                g = mm.groups()
                s["dev"][int(g[0])] = dict(state=g[1], id=int(g[2]), portH=int(g[3]), fast=g[4] == "FAST", lock=int(g[5]),
                                           ct=int(g[6]), rx=int(g[7]), ans=int(g[8]), badcrc=int(g[9]), spdmis=int(g[10]),
                                           wake=int(g[11]), sleep=int(g[12]), tosleep=int(g[13]), swrst=int(g[14]), conv=int(g[15]),
                                           override=int(g[16]), maxgap=int(g[17]), gpiomsk=int(g[18], 16))
            mm = re.match(r"MODEL (TH|TL): (.*)$", m)
            if mm:
                s["trx"][mm.group(1)] = {k: int(v) for k, v in re.findall(r"(\w+)=(\d+)", mm.group(2))}
    return snaps


def parse_can(d):
    rows = []
    p = os.path.join(d, "can.csv")
    if not os.path.exists(p):
        return rows
    for l in open(p).read().splitlines()[1:]:
        t, i, dlc, data = l.split(",")
        rows.append((float(t), int(i, 16), int(dlc), bytes.fromhex(data)))
    return rows


def status_lines(uart):
    res = []
    for t, s in uart:
        if s.startswith("STATUS "):
            kv = dict(re.findall(r"(\w+)=(\S+)", s))
            kv["t"] = t
            res.append(kv)
    return res


def slave_lines(uart):
    """last S<n> line of every slave: list of cell mV and temperatures"""
    res = {}
    for t, s in uart:
        m = re.match(r"S(\d+) (\w) fail=(\d+) V\[mV\]:([\d ]+) sum=(-?\d+) vb=(-?\d+) T\[0\.1C\]:([-\d ]*)$", s)
        if m:
            res[int(m.group(1))] = dict(t=t, path=m.group(2), fail=int(m.group(3)),
                                        cells=[int(x) for x in m.group(4).split()],
                                        vsum=int(m.group(5)), vbatt=int(m.group(6)),
                                        temps=[int(x) for x in m.group(7).split()])
    return res


# ----------------------------------------------------------------------------- checks
class Check:
    def __init__(self):
        self.items = []

    def __call__(self, cond, text):
        self.items.append((bool(cond), text))
        return cond

    @property
    def ok(self):
        return all(c for c, _ in self.items)


def model_cell_mv(slave, cell_idx, override=None):
    """cell voltages set by the model by default: 3700 + 3*c + (slave-1); c = 1..8,12..14"""
    cells = [1, 2, 3, 4, 5, 6, 7, 8, 12, 13, 14]
    c = cells[cell_idx]
    if override is not None:
        return override
    return 3700 + 3 * c + (slave - 1)


def raw89_mv(mv):
    raw = round(mv * 1000 / 89)
    return (raw * 89 + 500) // 1000


def ntc_poly(v):
    a, b, c, d, e = 121.77018530814999, -92.86284471272114, 39.33876760742037, -8.798397291741638, 0.7020187110716422
    return a + b * v + c * v * v + d * v ** 3 + e * v ** 4


def common_checks(chk, snap, n, fast=True, lock=True, ct=1, allow_rx_overflow=False, check_gap=True):
    th = snap["trx"].get("TH", {})
    chk(th.get("tx_overflow", 1) == 0, f"TH tx queue never overflowed ({th.get('tx_overflow')})")
    if not allow_rx_overflow:
        chk(th.get("rx_overflow", 1) == 0, f"TH rx queue never overflowed ({th.get('rx_overflow')})")
    chk(th.get("tx_txen_low_valid", 1) == 0, "no command ever sent with TXEN low")
    chk(th.get("spi_ncs_high", 1) == 0, "no SPI clocking with NCS high")
    chk(th.get("tx_before_ready", 1) == 0, "no frame before T_WAKEUP of the transceiver")
    for i in range(1, n + 1):
        dv = snap["dev"].get(i)
        if not chk(dv is not None, f"slave {i} present in the model"):
            continue
        chk(dv["state"] == "Normal" and dv["id"] == i, f"S{i}: Normal, chip_ID={i} (got {dv['state']} id={dv['id']})")
        chk(dv["fast"] == fast, f"S{i}: isoSPI {'fast' if fast else 'slow'}")
        chk(dv["lock"] == (1 if lock else 0), f"S{i}: Lock_isoh_isofreq={1 if lock else 0}")
        chk(dv["ct"] == ct, f"S{i}: CommTimeout code {ct} (got {dv['ct']})")
        chk(dv["gpiomsk"] == 0x7F, f"S{i}: GPIO OT/UT diagnostics masked (0x{dv['gpiomsk']:02X})")
        if check_gap:
            chk(dv["override"] == 0, f"S{i}: no Configuration Override ({dv['override']})")
            chk(dv["maxgap"] < 200, f"S{i}: longest gap between accesses {dv['maxgap']} ms < 200 ms (CommTimeout 256 ms)")


def check_nominal(chk, uart, snaps, can, n, dual=False, t_run=5.0):
    st = status_lines(uart)
    ready = [t for t, s in uart if s.startswith("AFE: chain ready")]
    chk(len(ready) == 1, f"chain initialised exactly once ({len(ready)})")
    if ready:
        chk(ready[0] < 0.5, f"chain ready at {ready[0]:.3f} s (< 0.5 s)")
    last = st[-1] if st else {}
    chk(last.get("ams") == "0" and last.get("faults") == "0x00", f"no AMS error / fault (ams={last.get('ams')} faults={last.get('faults')})")
    cycles = int(last.get("cycles", 0))
    t_st = last.get("t", 0) - (ready[0] if ready else 0)
    chk(cycles >= int(t_st * 10) - 1, f"measurement cycles {cycles} in {t_st:.2f} s after ready (period 100 ms)")
    cyc_ms = int(last.get("cycle", "999ms").rstrip("ms"))
    chk(cyc_ms < 60, f"cycle duration {cyc_ms} ms")
    sl = slave_lines(uart)
    for i in range(1, n + 1):
        s = sl.get(i)
        if not chk(s is not None, f"slave {i} reported"):
            continue
        exp = [raw89_mv(model_cell_mv(i, k)) for k in range(11)]
        chk(s["cells"] == exp, f"S{i} cells {s['cells'][:3]}.. == model {exp[:3]}..")
        chk(abs(s["vsum"] - sum(exp)) <= 2, f"S{i} VSUM {s['vsum']} ~ {sum(exp)}")
        vb = round(round(sum(model_cell_mv(i, k) for k in range(11)) / 1.33) * 1.33)
        chk(abs(s["vbatt"] - vb) <= 2, f"S{i} VBATT_DIV {s['vbatt']} ~ {vb}")
        chk(all(abs(t - round(ntc_poly(raw89_mv(2500) / 1000) * 10)) <= 1 for t in s["temps"]), f"S{i} temperatures {s['temps'][:2]}.. ~ 25.4 C")
        chk(s["fail"] == 0, f"S{i} no failed cycles ({s['fail']})")
    # CAN periods
    ids = {}
    for t, i, dlc, data in can:
        ids.setdefault(i, []).append(t)
    exp_periods = {0x200: 10, 0x203: 20, 0x204: 20, 0x206: 100, 0x208: 100, 0x20F: 1000}
    for i, per in exp_periods.items():
        ts = ids.get(i, [])
        if not chk(len(ts) >= 2, f"CAN 0x{i:03X} transmitted ({len(ts)})"):
            continue
        dts = [b - a for a, b in zip(ts, ts[1:])]
        mean = sum(dts) / len(dts)
        chk(abs(mean - per) <= per * 0.15, f"CAN 0x{i:03X} period {mean:.1f} ms (nominal {per})")
    stsys = [data[0] & 0x03 for t, i, dlc, data in can if i == 0x203]
    chk(stsys and stsys[-1] == 1, f"HVB_RX_Status stSys = 1 running (last {stsys[-1] if stsys else None})")
    check_can_payload(chk, uart, can, n, dual)
    snap = snaps.get("end")
    if chk(snap is not None, "final snapshot"):
        chk(snap["ams_pin"] == 0, "AMS_ERROR pin (PC6) low")
        chk(snap["err_led"] == 0, "ERR LED off")
        chk(snap["stat2_led"] == 1, "STAT2 LED on (chain ready)")
        common_checks(chk, snap, n)
        for i in range(1, n + 1):
            dv = snap["dev"].get(i, {})
            chk(dv.get("tosleep", 1) == 0, f"S{i}: never fell asleep by CommTimeout while running")
            want_h = 1 if (dual or i < n) else 0
            chk(dv.get("portH") == want_h, f"S{i}: isotx_en_h={want_h}")


_DB = None


def dbc():
    global _DB
    if _DB is None:
        import cantools
        _DB = cantools.database.load_file(os.path.join(FW, "Lib/SCan_HVCB/HVCB.dbc"), strict=False)
    return _DB


def last_decoded(can, fid):
    for t, i, dlc, data in reversed(can):
        if i == fid:
            return dbc().decode_message(fid, data, decode_choices=False)
    return None


def check_can_payload(chk, uart, can, n, dual):
    """decodes the telemetry with the HVCB.dbc of the SCan project and compares it with the firmware log"""
    st = status_lines(uart)[-1]
    sl = slave_lines(uart)
    cells = [(i - 1) * 11 + k for i in range(1, n + 1) for k in range(11)]
    vals = {(i - 1) * 11 + k: sl[i]["cells"][k] for i in range(1, n + 1) for k in range(11)}
    m = last_decoded(can, 0x204)
    pack = int(st["pack"].rstrip("mV"))
    chk(m and abs(m["HVB_uHvb"] - pack / 1000) <= 0.03, f"CAN HVB_uHvb {m['HVB_uHvb'] if m else None} V == pack {pack / 1000} V")
    m = last_decoded(can, 0x206)
    if chk(m is not None, "CAN HVB_RX_VCell decoded"):
        chk(abs(m["HVB_uCellMax"] - max(vals.values()) / 1000) < 0.00015, f"uCellMax {m['HVB_uCellMax']}")
        chk(abs(m["HVB_uCellMin"] - min(vals.values()) / 1000) < 0.00015, f"uCellMin {m['HVB_uCellMin']}")
        mean = sum(vals.values()) / len(vals) / 1000
        chk(abs(m["HVB_uCellMean"] - mean) < 0.0011, f"uCellMean {m['HVB_uCellMean']} ~ {mean:.4f}")
        chk(vals.get(int(m["HVB_idxCell_uMax"])) == max(vals.values()), f"idxCell_uMax {m['HVB_idxCell_uMax']} points to the max cell")
        chk(vals.get(int(m["HVB_idxCell_uMin"])) == min(vals.values()), f"idxCell_uMin {m['HVB_idxCell_uMin']} points to the min cell")
    m = last_decoded(can, 0x208)
    if chk(m is not None, "CAN HVB_RX_TCell decoded"):
        chk(abs(m["HVB_tCellMax"] - 25.4) < 0.06 and abs(m["HVB_tCellMin"] - 25.4) < 0.06, f"tCell {m['HVB_tCellMin']}..{m['HVB_tCellMax']} C")
    m = last_decoded(can, 0x200)
    chk(m is not None and all(v == 0 for v in m.values()), f"HVB_RX_Diagnosis all clear ({m})")
    m = last_decoded(can, 0x20F)
    chk(m is not None and int(m["HVB_noSwVers0"]) == 0x010000 and int(m["HVB_noSwVers1"]) == (n | (256 if dual else 0)),
        f"SW version frame ({m})")


# ----------------------------------------------------------------------------- scenarios
def sc_nominal(variant, n, dual=False, t_run=5.0):
    def run():
        elf, _ = build(variant)
        name = f"nominal_{variant}"
        d, out, wall = run_renode(name, elf, n, [("run", t_run), ("snap", "end")], trace=(n == 1))
        uart, snaps, can = parse_uart(d), parse_snaps(out), parse_can(d)
        chk = Check()
        check_nominal(chk, uart, snaps, can, n, dual=dual, t_run=t_run)
        if dual:
            chk(any("dual ring closed" in s for _, s in uart), "dual ring reported closed")
            chk(snaps["end"]["trx"].get("TL", {}).get("enabled") == 1, "L9963TL enabled")
        return name, chk, wall, d
    return run


def first(uart, pred):
    for t, s in uart:
        if pred(s):
            return t
    return None


def sc_fault(kind):
    """cell fault injection on a 1-slave chain; the reaction time is checked against the FS limits"""
    def run():
        elf, _ = build("n1")
        t_inj = 2.0
        if kind == "ov":
            cmd, limit, bit = "sysbus.spi3.l9963th SetCellMv 1 3 4250", 0.5, "0x01"
        elif kind == "uv":
            cmd, limit, bit = "sysbus.spi3.l9963th SetCellMv 1 12 2700", 0.5, "0x02"
        elif kind == "ot":
            cmd, limit, bit = "sysbus.spi3.l9963th SetGpioMv 1 5 900", 1.0, "0x04"    # ~ 65 C
        elif kind == "ntc_short":
            cmd, limit, bit = "sysbus.spi3.l9963th SetGpioMv 1 4 50", 1.0, "0x10"
        elif kind == "ntc_open":
            cmd, limit, bit = "sysbus.spi3.l9963th SetGpioMv 1 9 4990", 1.0, "0x10"
        name = f"fault_{kind}"
        d, out, wall = run_renode(name, elf, 1, [("run", t_inj), ("snap", "before"), ("cmd", cmd), ("run", 1.5), ("snap", "end")])
        uart, snaps, can = parse_uart(d), parse_snaps(out), parse_can(d)
        chk = Check()
        chk(snaps["before"]["ams_pin"] == 0, "AMS pin low before the fault")
        t_ams = first(uart, lambda s: s.startswith("AMS ERROR asserted"))
        if chk(t_ams is not None, "AMS ERROR asserted"):
            chk(t_ams - t_inj <= limit, f"reaction time {1000 * (t_ams - t_inj):.0f} ms <= {1000 * limit:.0f} ms")
            chk(t_ams - t_inj >= 0.25 if kind in ("ov", "uv") else t_ams - t_inj >= 0.55,
                f"persistence filter applied ({1000 * (t_ams - t_inj):.0f} ms)")
        st = status_lines(uart)
        chk(st and st[-1].get("faults") == bit, f"latched fault {bit} (got {st[-1].get('faults') if st else None})")
        chk(snaps["end"]["ams_pin"] == 1, "AMS_ERROR pin (PC6) high")
        chk(snaps["end"]["err_led"] == 1, "ERR LED on")
        stsys = [data[0] & 3 for t, i, dlc, data in can if i == 0x203 and t / 1000 > t_ams + 0.05] if t_ams else []
        chk(stsys and all(x == 2 for x in stsys), "HVB_RX_Status stSys = 2 (AMS error) after the fault")
        diag = [data for t, i, dlc, data in can if i == 0x200 and t_ams and t / 1000 > t_ams + 0.05]
        chk(diag and any(b != 0 for b in diag[-1]), f"HVB_RX_Diagnosis carries the fault ({diag[-1].hex() if diag else None})")
        return name, chk, wall, d
    return run


def sc_spike():
    def run():
        elf, _ = build("n1")
        name = "fault_ov_spike_150ms"
        steps = [("run", 2.0), ("cmd", "sysbus.spi3.l9963th SetCellMv 1 3 4300"), ("run", 0.15),
                 ("cmd", "sysbus.spi3.l9963th SetCellMv 1 3 3709"), ("run", 1.5), ("snap", "end")]
        d, out, wall = run_renode(name, elf, 1, steps)
        uart, snaps = parse_uart(d), parse_snaps(out)
        chk = Check()
        chk(first(uart, lambda s: s.startswith("AMS ERROR")) is None, "a 150 ms spike does not trip the AMS")
        chk(snaps["end"]["ams_pin"] == 0, "AMS pin low")
        return name, chk, wall, d
    return run


def sc_break_single():
    def run():
        elf, _ = build("n11")
        name = "break_single_ring_n11"
        steps = [("run", 2.0), ("snap", "before"), ("cmd", "sysbus.spi3.l9963th BreakLink 5"), ("run", 3.0), ("snap", "broken"),
                 ("cmd", "sysbus.spi3.l9963th RestoreLink 5"), ("run", 4.0), ("snap", "end")]
        d, out, wall = run_renode(name, elf, 11, steps)
        uart, snaps = parse_uart(d), parse_snaps(out)
        chk = Check()
        t_ams = first(uart, lambda s: s.startswith("AMS ERROR"))
        if chk(t_ams is not None, "communication loss trips the AMS"):
            chk(t_ams - 2.0 <= 0.5, f"reaction {1000 * (t_ams - 2.0):.0f} ms <= 500 ms")
        chk(any("FAULT SLAVE COMMUNICATION: index 6" in s for _, s in uart), "first slave beyond the break (6) reported")
        chk(any("addressing failed, 5/11 slaves answered (no answer from slave 6)" in s for _, s in uart),
            "re-initialisation pinpoints the break (5/11, no answer from slave 6)")
        readies = [t for t, s in uart if s.startswith("AFE: chain ready")]
        chk(len(readies) >= 2 and readies[-1] > 5.0, f"chain re-initialised after the link is restored ({readies})")
        sl = slave_lines(uart)
        chk(all(sl.get(i, {}).get("t", 0) > 8.0 for i in range(1, 12)), "all 11 slaves reported again at the end")
        st = status_lines(uart)
        chk(st and st[-1].get("ready") == "1", "AFE ready at the end")
        chk(snaps["end"]["ams_pin"] == 1, "AMS stays latched after recovery (manual reset required)")
        common_checks(chk, snaps["end"], 11)
        return name, chk, wall, d
    return run


def sc_break_dual():
    def run():
        elf, _ = build("n12dual")
        name = "break_dual_ring_n12"
        steps = [("run", 2.0), ("snap", "before"), ("cmd", "sysbus.spi3.l9963th BreakLink 6"), ("run", 2.5), ("snap", "broken"),
                 ("cmd", "sysbus.spi3.l9963th RestoreLink 6"), ("run", 1.5), ("snap", "end")]
        d, out, wall = run_renode(name, elf, 12, steps)
        uart, snaps = parse_uart(d), parse_snaps(out)
        chk = Check()
        chk(first(uart, lambda s: s.startswith("AMS ERROR")) is None, "a single break of the dual ring does not trip the AMS")
        chk(any("dual ring OPEN" in s for _, s in uart), "ring opening detected")
        chk(any("dual ring closed again" in s for _, s in uart), "ring closing detected")
        lines = [s for t, s in uart if 2.0 < t < 4.5 and "OK via L9963TL" in s]
        moved = sorted(int(re.search(r"slave (\d+)", s).group(1)) for s in lines)
        chk(moved == list(range(7, 13)), f"slaves 7..12 moved to the L9963TL ({moved})")
        st = [x for x in status_lines(uart) if 2.5 < x["t"] < 4.6]
        chk(st and all(x["faults"] == "0x00" for x in st), "no fault while the ring is open")
        slv = [x for x in status_lines(uart) if x["t"] > 3.0]
        cyc = [int(x["cycle"].rstrip("ms")) for x in slv]
        chk(cyc and max(cyc) < 60, f"cycle time with the ring open {max(cyc) if cyc else None} ms")
        # in the cycle in which the ring closes again, the commands of the L9963TL reach the
        # (unread) RX queue of the L9963TH: harmless overflow, flushed before the next command
        common_checks(chk, snaps["end"], 12, allow_rx_overflow=True)
        warn_open = snaps["broken"]["warn_led"]
        chk(warn_open == 1, "WARN LED on while the ring is open")
        chk(snaps["end"]["warn_led"] == 0, "WARN LED off when the ring is closed again")
        sl = slave_lines(uart)
        chk(all(sl.get(i, {}).get("path") == "H" for i in range(1, 13)), "all slaves back on the L9963TH after the ring closed")
        return name, chk, wall, d
    return run


def sc_power_cycle_slave():
    def run():
        elf, _ = build("n11")
        name = "slave4_power_cycle_n11"
        steps = [("run", 2.0), ("cmd", "sysbus.spi3.l9963th PowerOffSlave 4"), ("run", 1.0),
                 ("cmd", "sysbus.spi3.l9963th PowerOnSlave 4"), ("run", 4.0), ("snap", "end")]
        d, out, wall = run_renode(name, elf, 11, steps)
        uart, snaps = parse_uart(d), parse_snaps(out)
        chk = Check()
        chk(any("FAULT SLAVE COMMUNICATION: index 4" in s for _, s in uart), "slave 4 loss detected")
        readies = [t for t, s in uart if s.startswith("AFE: chain ready")]
        chk(len(readies) >= 2, f"chain re-addressed with the power-cycled slave ({readies})")
        sl = slave_lines(uart)
        chk(all(sl.get(i, {}).get("t", 0) > 6.0 for i in range(1, 12)), "all 11 slaves reported at the end")
        common_checks(chk, snaps["end"], 11)
        return name, chk, wall, d
    return run


def sc_warm_reset():
    """MCU reset (watchdog, debugger) while the chain is awake in fast mode"""
    def run():
        elf, _ = build("n11")
        name = "mcu_warm_reset_n11"
        steps = [("run", 2.0), ("snap", "before"), ("cmd", "machine Reset"), ("run", 3.0), ("snap", "end")]
        d, out, wall = run_renode(name, elf, 11, steps)
        uart, snaps = parse_uart(d), parse_snaps(out)
        chk = Check()
        boots = [t for t, s in uart if s.startswith("BMS HV master fw")]
        chk(len(boots) == 2, f"two boots ({len(boots)})")
        chk(any("chain found awake in fast mode" in s for _, s in uart), "chain awake in fast mode detected and reset")
        readies = [t for t, s in uart if s.startswith("AFE: chain ready")]
        chk(len(readies) == 2, f"chain ready after the reset ({readies})")
        if len(readies) == 2 and len(boots) == 2:
            chk(readies[1] - boots[1] < 0.5, f"re-initialisation after reset in {1000 * (readies[1] - boots[1]):.0f} ms")
        chk(first(uart, lambda s: s.startswith("AMS ERROR")) is None, "no AMS error")
        common_checks(chk, snaps["end"], 11)
        return name, chk, wall, d
    return run


def sc_noise():
    def run():
        elf, _ = build("n11")
        name = "isoline_noise_n11"
        steps = [("cmd", "sysbus.spi3.l9963th CorruptEvery 40"), ("run", 6.0), ("snap", "end")]
        d, out, wall = run_renode(name, elf, 11, steps)
        uart, snaps = parse_uart(d), parse_snaps(out)
        chk = Check()
        chk(first(uart, lambda s: s.startswith("AMS ERROR")) is None, "2.5% corrupted frames: no AMS error")
        readies = [t for t, s in uart if s.startswith("AFE: chain ready")]
        chk(len(readies) >= 1, f"chain initialised with noise ({readies})")
        st = status_lines(uart)
        chk(st and int(st[-1]["cycles"]) >= int((st[-1]["t"] - 0.2) * 10) - 1, f"measurement cycles {st[-1]['cycles'] if st else None} at {st[-1]['t'] if st else None} s")
        chk(len(readies) == 1, "no re-initialisation needed: lost broadcasts repaired by unicast writes")
        m = snaps["end"]["model"][0] if snaps["end"]["model"] else ""
        corrupted = int(re.search(r"corrupted=(\d+)", m).group(1)) if m else 0
        chk(corrupted > 100, f"{corrupted} frames corrupted by the model")
        common_checks(chk, snaps["end"], 11)
        return name, chk, wall, d
    return run


def sc_missing_slave():
    def run():
        elf, _ = build("n11")
        name = "fw11_chain10_missing_slave"
        d, out, wall = run_renode(name, elf, 10, [("run", 4.5), ("snap", "end")])
        uart, snaps = parse_uart(d), parse_snaps(out)
        chk = Check()
        chk(any("addressing failed, 10/11 slaves answered (no answer from slave 11)" in s for _, s in uart),
            "diagnostic: 10/11 slaves answered, no answer from slave 11")
        t_ams = first(uart, lambda s: s.startswith("AMS ERROR"))
        chk(t_ams is not None and 2.9 <= t_ams <= 3.2, f"AMS error at the end of the start-up grace time ({t_ams})")
        chk(snaps["end"]["ams_pin"] == 1, "AMS pin high")
        return name, chk, wall, d
    return run


def sc_fw1_on_car():
    """firmware for 1 slave flashed on the complete chain: slave 1 works, the others stay asleep"""
    def run():
        elf, _ = build("n1")
        name = "fw1_on_chain11"
        d, out, wall = run_renode(name, elf, 11, [("run", 3.0), ("snap", "end")])
        uart, snaps = parse_uart(d), parse_snaps(out)
        chk = Check()
        chk(first(uart, lambda s: s.startswith("AMS ERROR")) is None, "no AMS error")
        dv = snaps["end"]["dev"]
        chk(dv[1]["state"] == "Normal" and dv[1]["portH"] == 0, "slave 1 Normal, port H closed (Farthest_Unit)")
        chk(dv[2]["state"] in ("Init", "Sleep") and dv[2]["id"] == 0, "slave 2 at most woken without address (DS fig. 7 opens port H of slave 1)")
        chk(all(dv[i]["state"] == "Sleep" and dv[i]["wake"] == 0 for i in range(3, 12)), "slaves 3..11 never woken")
        return name, chk, wall, d
    return run


def sc_can_noack():
    def run():
        elf, _ = build("n1")
        name = "can_no_ack"
        steps = [("cmd", "sysbus.can1 NoAck true"), ("run", 3.0), ("snap", "end"), ("cmd", "sysbus.can1 Aborted")]
        d, out, wall = run_renode(name, elf, 1, steps)
        uart, snaps = parse_uart(d), parse_snaps(out)
        chk = Check()
        st = status_lines(uart)
        chk(st and int(st[-1]["cycles"]) >= int((st[-1]["t"] - 0.1) * 10) - 1, "measurements keep running with CAN mailboxes stuck")
        chk(first(uart, lambda s: s.startswith("AMS ERROR")) is None, "no AMS error")
        tail = out.split("ENDSNAP end")[-1]
        hx = re.findall(r"0x[0-9A-Fa-f]+", tail)
        chk(hx and int(hx[0], 16) > 50, f"stale frames aborted by the firmware ({int(hx[0], 16) if hx else None})")
        return name, chk, wall, d
    return run


def sc_long_run():
    def run():
        elf, _ = build("n12")
        name = "long_run_60s_n12"
        d, out, wall = run_renode(name, elf, 12, [("run", 60.0), ("snap", "end")])
        uart, snaps = parse_uart(d), parse_snaps(out)
        chk = Check()
        boots = [t for t, s in uart if s.startswith("BMS HV master fw")]
        chk(len(boots) == 1, "no reset (watchdog) in 60 s")
        st = status_lines(uart)
        chk(st and int(st[-1]["cycles"]) >= 590, f"cycles {st[-1]['cycles'] if st else None} in 60 s")
        chk(st and st[-1]["faults"] == "0x00", "no fault")
        sl = slave_lines(uart)
        chk(all(sl.get(i, {}).get("fail", 1) == 0 for i in range(1, 13)), "zero failed reads on 12 slaves")
        common_checks(chk, snaps["end"], 12)
        return name, chk, wall, d
    return run


def sc_simple_fault(name, variant, n, steps_mid, expect_bits, limit, t_inj=2.0, extra=None):
    """generic: inject at t_inj, expect a latched fault set within `limit` seconds"""
    def run():
        elf, _ = build(variant)
        steps = [("run", t_inj), ("snap", "before")] + steps_mid + [("snap", "end")]
        d, out, wall = run_renode(name, elf, n, steps)
        uart, snaps, can = parse_uart(d), parse_snaps(out), parse_can(d)
        chk = Check()
        chk(snaps["before"]["ams_pin"] == 0, "AMS pin low before the fault")
        t_ams = first(uart, lambda s: s.startswith("AMS ERROR asserted"))
        if chk(t_ams is not None, "AMS ERROR asserted"):
            chk(t_ams - t_inj <= limit, f"reaction time {1000 * (t_ams - t_inj):.0f} ms <= {1000 * limit:.0f} ms")
        st = status_lines(uart)
        got = int(st[-1]["faults"], 16) if st else 0
        chk(got & expect_bits, f"latched faults 0x{got:02X} include 0x{expect_bits:02X}")
        chk(snaps["end"]["ams_pin"] == 1, "AMS_ERROR pin (PC6) high")
        boots = [t for t, s2 in uart if s2.startswith("BMS HV master fw")]
        chk(len(boots) == 1, f"no MCU reset ({len(boots)} boots)")
        if extra:
            extra(chk, uart, snaps, can)
        return name, chk, wall, d
    return run


def extra_temp_stale(chk, uart, snaps, can):
    sl = slave_lines(uart)
    chk(sl.get(1, {}).get("t", 0) >= 3.0 and sl[1]["fail"] > 0, "cell voltages still reported while the temperatures are missing")


def sc_break_dual_link0():
    def run():
        elf, _ = build("n12dual")
        name = "break_dual_ring_link0_n12"
        steps = [("run", 2.0), ("cmd", "sysbus.spi3.l9963th BreakLink 0"), ("run", 2.0), ("snap", "broken"),
                 ("cmd", "sysbus.spi3.l9963th RestoreLink 0"), ("run", 1.5), ("snap", "end")]
        d, out, wall = run_renode(name, elf, 12, steps)
        uart, snaps = parse_uart(d), parse_snaps(out)
        chk = Check()
        chk(first(uart, lambda s: s.startswith("AMS ERROR")) is None, "break of the link L9963TH-slave 1: no AMS error")
        chk(any("dual ring OPEN" in s for _, s in uart), "ring opening detected (two-way ring check)")
        chk(snaps["broken"]["warn_led"] == 1, "WARN LED on while the ring is open")
        moved = sorted(int(re.search(r"slave (\d+)", s).group(1)) for t, s in uart if 2.0 < t < 4.0 and "OK via L9963TL" in s)
        chk(moved == list(range(1, 13)), f"all slaves moved to the L9963TL ({moved})")
        chk(any("dual ring closed again" in s for _, s in uart), "ring closing detected")
        common_checks(chk, snaps["end"], 12, allow_rx_overflow=True)
        return name, chk, wall, d
    return run


def sc_bne_stuck():
    def run():
        elf, _ = build("n11")
        name = "bne_stuck_high_n11"
        steps = [("run", 2.0), ("cmd", "sysbus.spi3.l9963th BneStuck 1"), ("run", 2.0), ("snap", "mid"),
                 ("cmd", "sysbus.spi3.l9963th BneStuck 2"), ("run", 2.0), ("snap", "end")]
        d, out, wall = run_renode(name, elf, 11, steps)
        uart, snaps = parse_uart(d), parse_snaps(out)
        chk = Check()
        boots = [t for t, s in uart if s.startswith("BMS HV master fw")]
        chk(len(boots) == 1, f"BNE stuck high / dead transceiver never hang the MCU (no watchdog reset, {len(boots)} boots)")
        t_ams = first(uart, lambda s: s.startswith("AMS ERROR"))
        chk(t_ams is None or t_ams > 4.0, f"BNE stuck high with a working transceiver: data still read, no fault before 4 s ({t_ams})")
        st = [x for x in status_lines(uart) if 2.5 < x["t"] <= 4.0]
        chk(st and all(x["faults"] == "0x00" for x in st), "no fault while only BNE is stuck")
        chk(t_ams is not None and t_ams - 4.0 <= 0.5, f"dead transceiver: communication fault within 500 ms ({t_ams})")
        st = status_lines(uart)
        chk(st and st[-1]["t"] >= 5.0, "main loop still running (status printed)")
        return name, chk, wall, d
    return run


def symbol(elf, name):
    out = subprocess.run(["arm-none-eabi-nm", elf], stdout=subprocess.PIPE, text=True).stdout
    for l in out.splitlines():
        p = l.split()
        if len(p) == 3 and p[2] == name:
            return int(p[0], 16) & ~1
    raise KeyError(name)


def sc_watchdog_flag():
    """a boot after a watchdog reset must latch the AMS error at once. Renode does not emulate the
    reset flags of RCC_CSR, so IWDGRSTF is injected into the variable read by bms_app_init()."""
    def run():
        elf, _ = build("n1")
        name = "boot_after_watchdog_reset"
        hook = symbol(elf, "bms_monitor_init")
        var = symbol(elf, "reset_flags")
        steps = [("cmd", f'sysbus.cpu AddHook 0x{hook:08X} "machine.SystemBus.WriteDoubleWord(0x{var:08X}, 0x20000000)"'),
                 ("run", 1.0), ("snap", "end")]
        d, out, wall = run_renode(name, elf, 1, steps)
        uart, snaps = parse_uart(d), parse_snaps(out)
        chk = Check()
        t_ams = first(uart, lambda s: s.startswith("AMS ERROR"))
        chk(t_ams is not None and t_ams < 0.05, f"AMS error latched at boot ({t_ams})")
        chk(any("FAULT WATCHDOG RESET" in s for _, s in uart), "fault reported as WATCHDOG RESET")
        chk(snaps["end"]["ams_pin"] == 1, "AMS pin high")
        chk(any(s.startswith("AFE: chain ready") for _, s in uart), "the chain is still initialised and measured (diagnostics)")
        return name, chk, wall, d
    return run


def sc_dual_restart(kind):
    """dual ring with a break present when the chain is (re)initialised"""
    def run():
        elf, _ = build("n12dual")
        name = f"dual_break_{kind}_n12"
        if kind == "warm_reset":        # MCU reset while the slaves are awake in fast mode
            steps = [("run", 2.0), ("cmd", "sysbus.spi3.l9963th BreakLink 6"), ("run", 1.0), ("cmd", "machine Reset"),
                     ("run", 3.0), ("snap", "end")]
        elif kind == "master_off":      # master switched off long enough for the slaves to fall asleep
            steps = [("run", 2.0), ("cmd", "sysbus.spi3.l9963th BreakLink 6"), ("run", 1.0),
                     ("cmd", "sysbus.cpu IsHalted true"), ("run", 1.5), ("cmd", "sysbus.cpu IsHalted false"),
                     ("cmd", "machine Reset"), ("run", 3.0), ("snap", "end")]
        else:                           # cold start: slaves never addressed, port H disabled
            steps = [("cmd", "sysbus.spi3.l9963th BreakLink 6"), ("run", 4.0), ("snap", "end")]
        d, out, wall = run_renode(name, elf, 12, steps)
        uart, snaps = parse_uart(d), parse_snaps(out)
        chk = Check()
        boots = [t for t, s in uart if s.startswith("BMS HV master fw")]
        last_boot = max(i for i, (t, s) in enumerate(uart) if s.startswith("BMS HV master fw"))
        after = uart[last_boot:]
        if kind == "cold":
            chk(any("completing from the L9963TL" in s for _, s in after), "bottom addressing stops at slave 7, top completion tried")
            chk(any("not reachable from the L9963TL either" in s for _, s in after), "slaves never addressed cannot be reached from the top (DS: port H disabled after POR)")
            t_ams = first(after, lambda s: s.startswith("AMS ERROR"))
            chk(t_ams is not None and 2.9 <= t_ams <= 3.3, f"AMS error at the end of the start-up grace ({t_ams})")
        else:
            chk(len(boots) == 2, f"two boots ({len(boots)})")
            chk(any("completing from the L9963TL" in s for _, s in after), "addressing completed from the L9963TL")
            ready = [t for t, s in after if s.startswith("AFE: chain ready")]
            chk(len(ready) == 1 and ready[0] < 0.6, f"chain ready with the ring open ({ready})")
            chk(any("dual ring OPEN" in s for _, s in after), "ring reported open")
            chk(first(after, lambda s: s.startswith("AMS ERROR")) is None, "no AMS error")
            sl = slave_lines(after)
            chk(all(sl.get(i, {}).get("path") == ("H" if i <= 6 else "L") for i in range(1, 13)),
                "slaves 1..6 on the L9963TH, 7..12 on the L9963TL")
            chk(all(sl.get(i, {}).get("fail", 1) == 0 for i in range(1, 13)), "no failed read after the start")
            common_checks(chk, snaps["end"], 12, allow_rx_overflow=True)
        return name, chk, wall, d
    return run


def sc_fsm_smoke():
    """relay state machine enabled (not validated: only checks that it coexists with the AFE)"""
    def run():
        elf, _ = build("n11fsm")
        name = "fsm_enabled_smoke_n11"
        d, out, wall = run_renode(name, elf, 11, [("run", 3.0), ("snap", "end")])
        uart, snaps = parse_uart(d), parse_snaps(out)
        chk = Check()
        boots = [t for t, s in uart if s.startswith("BMS HV master fw")]
        chk(len(boots) == 1, "no reset with the FSM running")
        st = status_lines(uart)
        chk(st and st[-1]["ready"] == "1" and int(st[-1]["cycles"]) >= 15, "AFE measuring with the FSM enabled")
        chk(st and st[-1]["faults"] == "0x00", "no fault")
        return name, chk, wall, d
    return run


SCENARIOS = [
    ("nominal_n1", sc_nominal("n1", 1)),
    ("nominal_n11", sc_nominal("n11", 11)),
    ("nominal_n12", sc_nominal("n12", 12)),
    ("nominal_n12dual", sc_nominal("n12dual", 12, dual=True)),
    ("nominal_n1dual", sc_nominal("n1dual", 1, dual=True)),
    ("fault_ov", sc_fault("ov")),
    ("fault_uv", sc_fault("uv")),
    ("fault_ot", sc_fault("ot")),
    ("fault_ntc_short", sc_fault("ntc_short")),
    ("fault_ntc_open", sc_fault("ntc_open")),
    ("fault_spike", sc_spike()),
    ("fault_temp_stale", sc_simple_fault("fault_temp_stale", "n1", 1, [("cmd", "sysbus.spi3.l9963th MuteGpio 1 true"), ("run", 1.5)],
                                         0x40, 1.0, extra=extra_temp_stale)),
    ("fault_ntc_open_low_vtref", sc_simple_fault("fault_ntc_open_low_vtref", "n1", 1,
                                                 [("cmd", "sysbus.spi3.l9963th SetGpioMv 1 6 4800"), ("run", 1.5)], 0x10, 1.0)),
    ("fault_hw_ov_override", sc_simple_fault("fault_hw_ov_override", "n1", 1,
                                             [("cmd", "sysbus.spi3.l9963th SetCellMv 1 2 4350"), ("run", 1.5)], 0x21, 0.5)),
    ("fault_ov_n12", sc_simple_fault("fault_ov_n12_slave12", "n12", 12,
                                     [("cmd", "sysbus.spi3.l9963th SetCellMv 12 14 4230"), ("run", 1.5)], 0x01, 0.5)),
    ("bne_stuck", sc_bne_stuck()),
    ("watchdog_flag", sc_watchdog_flag()),
    ("break_dual_link0", sc_break_dual_link0()),
    ("dual_break_warm_reset", sc_dual_restart("warm_reset")),
    ("dual_break_master_off", sc_dual_restart("master_off")),
    ("dual_break_cold", sc_dual_restart("cold")),
    ("break_single", sc_break_single()),
    ("break_dual", sc_break_dual()),
    ("slave_power_cycle", sc_power_cycle_slave()),
    ("warm_reset", sc_warm_reset()),
    ("noise", sc_noise()),
    ("missing_slave", sc_missing_slave()),
    ("fw1_on_car", sc_fw1_on_car()),
    ("can_noack", sc_can_noack()),
    ("fsm_smoke", sc_fsm_smoke()),
    ("long_run", sc_long_run()),
]


def main():
    sel = sys.argv[1:]
    os.makedirs(OUT, exist_ok=True)
    results = []
    for key, fn in SCENARIOS:
        if sel and not any(s in key for s in sel):
            continue
        try:
            name, chk, wall, d = fn()
            items = chk.items
            ok = chk.ok
        except Exception as e:  # noqa
            import traceback
            name, items, ok, wall, d = key, [(False, "exception: " + traceback.format_exc().splitlines()[-1])], False, 0, ""
        results.append(dict(scenario=key, ok=ok, wall_s=round(wall, 1), checks=[dict(ok=c, text=t) for c, t in items], dir=d))
        print(f"[{'PASS' if ok else 'FAIL'}] {key} ({wall:.0f} s, {sum(1 for c, _ in items if c)}/{len(items)} checks)")
        for c, t in items:
            if not c:
                print(f"      FAILED: {t}")
        sys.stdout.flush()
    tag = os.environ.get("BMS_TEST_TAG", "latest")
    json.dump(results, open(os.path.join(OUT, f"results_{tag}.json"), "w"), indent=1)
    npass = sum(1 for r in results if r["ok"])
    print(f"\n{npass}/{len(results)} scenarios passed")
    return 0 if npass == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())
