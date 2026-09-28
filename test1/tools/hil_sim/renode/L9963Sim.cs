//
// Behavioural model of the L9963T isoSPI transceiver + a daisy chain of L9963E
// for Renode (loaded at runtime with `include @L9963Sim.cs`).
//
// References: L9963E DS13636 rev 11, L9963T DS13590 rev 4.
//
// What is modelled (as strictly as the datasheets allow):
//  L9963T (slave configuration, NSLAVE = 0)
//   - DIS active high disable, T_WAKEUP 1.06 ms, ISOFREQ latched at DIS falling edge
//   - TXEN / ISOFREQ latched at NCS falling edge
//   - RX speed switched immediately, TX speed switched after the latched frame is sent
//   - TX queue 3 frames (FIFO, sent when the line is idle, inter-frame 4/8 bit)
//   - RX queue 20 frames, BNE = queue not empty, NCS assertion pops the head (MISO)
//  L9963E
//   - Sleep / waking (TWAKEUP 2 ms, only low speed frames wake) / Init / Normal
//   - chip_ID written once (Init -> Normal), isotx_en_h (port H), iso_freq_sel, Lock_isoh_isofreq
//   - bit level repeater: frames forwarded up if port H enabled, always forwarded down
//   - a frame at the wrong speed is not decoded (and not forwarded)
//   - CRC-6 (poly 0x59, seed 0x38) checked on every frame, answers carry a CRC
//   - answer to unicast read/write after TANSWER_DELAY, towards the requesting side
//   - broadcast (devid 0): executed, no answer (no "master unit" in this topology)
//   - CommTimeout (32/256/1024/2048 ms) and t_SHUT (60 s) -> Sleep; reset source B lost in Sleep
//   - SW_RST (+GO2SLP): all registers to default except CommTimeout, chip_ID cleared
//   - on-demand conversion (SOC): cells, VBATT_DIV, VSUM, GPIO3..9, d_rdy cleared on read
//   - GPIO measure requires VTREF_EN = 1 and GPIOx_CONFIG = 00 (analog)
//  Fault injection: link breaks, slave power loss, CRC corruption on the isoline.
//
using System;
using System.Collections.Generic;
using System.IO;
using System.Text;
using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.GPIOPort;
using Antmicro.Renode.Time;

namespace Antmicro.Renode.Peripherals.SPI
{
    public static class L9963Frame
    {
        static readonly byte[] lut = {
            0x0, 0x19, 0x32, 0x2b, 0x3d, 0x24, 0xf, 0x16, 0x23, 0x3a, 0x11, 0x8, 0x1e, 0x7, 0x2c, 0x35,
            0x1f, 0x6, 0x2d, 0x34, 0x22, 0x3b, 0x10, 0x9, 0x3c, 0x25, 0xe, 0x17, 0x1, 0x18, 0x33, 0x2a,
            0x3e, 0x27, 0xc, 0x15, 0x3, 0x1a, 0x31, 0x28, 0x1d, 0x4, 0x2f, 0x36, 0x20, 0x39, 0x12, 0xb,
            0x21, 0x38, 0x13, 0xa, 0x1c, 0x5, 0x2e, 0x37, 0x2, 0x1b, 0x30, 0x29, 0x3f, 0x26, 0xd, 0x14 };

        public static byte Crc(ulong w)
        {
            w = (w & 0xFFFFFFFFC0UL) ^ (0x38UL << 34);
            ulong test = 1UL << 39;
            ulong mask = 0x59UL << 33;
            for(int k = 0; k < 4; k++)
            {
                if((w & test) != 0) { w ^= mask; }
                mask >>= 1; test >>= 1;
            }
            byte crc = 0;
            for(int i = 30; i > 0; i -= 6)
            {
                crc = lut[((w >> i) & 0x3F) ^ crc];
            }
            return crc;
        }

        public static ulong Build(int pa, int rw, int devid, int addr, int gsw, uint data)
        {
            ulong f = ((ulong)(pa & 1) << 39) | ((ulong)(rw & 1) << 38) | ((ulong)(devid & 0x1F) << 33)
                    | ((ulong)(addr & 0x7F) << 26) | ((ulong)(gsw & 3) << 24) | ((ulong)(data & 0x3FFFF) << 6);
            return f | Crc(f);
        }

        public static bool CrcOk(ulong f) { return (f & 0x3F) == Crc(f); }
        public static int Pa(ulong f) { return (int)((f >> 39) & 1); }
        public static int Rw(ulong f) { return (int)((f >> 38) & 1); }
        public static int DevId(ulong f) { return (int)((f >> 33) & 0x1F); }
        public static int Addr(ulong f) { return (int)((f >> 26) & 0x7F); }
        public static uint Data(ulong f) { return (uint)((f >> 6) & 0x3FFFF); }

        public static string Describe(ulong f)
        {
            return string.Format("{0}{1} dev={2,2} addr=0x{3:X2} data=0x{4:X5}{5}", Pa(f) == 1 ? "CMD " : "ANS ",
                Rw(f) == 1 ? "W" : "R", DevId(f), Addr(f), Data(f), CrcOk(f) ? "" : " BADCRC");
        }

        public static double Tbit(bool fast) { return fast ? 0.375 : 3.0; }        // us (2.66 / 0.333 Mbit/s)
        public static double Insertion(bool fast) { return 0.5 * Tbit(fast); }    // < 1 bit per device (DS 6.11.5)
        public static double AnswerDelay(bool fast) { return fast ? 4.5 : 9.0; }  // TANSWER_DELAY
    }

    public class L9963EDevice
    {
        public enum St { Off, Sleep, Waking, Init, Normal }

        public L9963EDevice(int position) { Position = position; PowerOnReset(0); }

        public readonly int Position;           // 1 = closest to the L9963TH
        public St State;
        public uint[] Regs = new uint[128];
        public int ChipId;
        public bool PortH, Fast, Lock;
        public double LastComm, WakeAt, ReadyAt, ConvDoneAt = -1;
        public bool ConvGpio;
        public int[] CellMv = new int[15];       // index 1..14
        public int[] GpioMv = new int[10];       // index 3..9
        // statistics
        public long RxCmd, RxBadCrc, RxSpeedMismatch, Answers, Wakeups, Sleeps, TimeoutSleeps, SwResets, Conversions, DroppedWaking, Overrides, SocIgnored;
        public double MaxCommGap;   // longest interval between two commands addressed to this device while Normal
        public bool MuteGpioRegs;   // fault injection: no answer to GPIOx_MEAS reads

        const int REG_DEV_GEN_CFG = 1, REG_FASTCH = 2, REG_BAL3 = 5, REG_ADCV = 13, REG_NCYCLE2 = 15,
                  REG_FSM = 18, REG_GPIO_CONF = 20, REG_VCELLS_EN = 28, REG_VCELL1 = 33, REG_GPIO3 = 52,
                  REG_VSUMBATT = 64, REG_VBATTDIV = 65, REG_VCELL_TH = 11, REG_GPIO3_THR = 21, REG_CSA_GPIO_MSK = 32;
        static readonly double[] CommTimeoutMs = { 32, 256, 1024, 2048 };
        const double TShutUs = 60e6, TWakeUs = 2000;

        void DefaultRegs()
        {
            Array.Clear(Regs, 0, Regs.Length);
            Regs[REG_DEV_GEN_CFG] = 0x40;          // HeartBeatCycle = 4
            Regs[REG_GPIO_CONF] = 0x0A000;         // GPIO7/8 digital by default
            ChipId = 0; PortH = false; Fast = false; Lock = false; ConvDoneAt = -1;
        }

        public void PowerOnReset(double now)
        {
            DefaultRegs();
            State = St.Sleep;
            LastComm = now;
        }

        public void PowerOff() { State = St.Off; }

        void GoSleep(double t, bool timeout)
        {
            State = St.Sleep;
            Sleeps++;
            if(timeout) { TimeoutSleeps++; }
            // reset source B
            Fast = false; Lock = false;
            Regs[REG_DEV_GEN_CFG] &= ~0x300u;
            Regs[REG_BAL3] &= ~0x08000u;
            Regs[REG_ADCV] = 0;
            ConvDoneAt = -1;
            for(int a = REG_VCELL1; a < REG_VCELL1 + 14; a++) { Regs[a] = 0; }
            for(int a = REG_GPIO3; a < REG_GPIO3 + 7; a++) { Regs[a] = 0; }
            Regs[REG_VSUMBATT] = 0; Regs[REG_VBATTDIV] = 0;
        }

        /** lazy evolution of the internal timers up to time t */
        public void Advance(double t)
        {
            if(State == St.Off) { return; }
            if(State == St.Waking && t >= ReadyAt)
            {
                State = ChipId == 0 ? St.Init : St.Normal;
                LastComm = ReadyAt;
            }
            if(State == St.Init && t - WakeAt > TShutUs)
            {
                GoSleep(WakeAt + TShutUs, true);
            }
            if(State == St.Normal)
            {
                double to = CommTimeoutMs[(Regs[REG_FASTCH] >> 16) & 3] * 1000.0;
                if(t - LastComm > to) { GoSleep(LastComm + to, true); }
            }
            if(ConvDoneAt >= 0 && t >= ConvDoneAt && State == St.Normal) { LatchConversion(); }
        }

        static uint Raw89(int mv) { long r = (long)Math.Round(mv * 1000.0 / 89.0); if(r < 0) r = 0; if(r > 0xFFFF) r = 0xFFFF; return (uint)r; }

        void LatchConversion()
        {
            uint en = Regs[REG_VCELLS_EN] & 0x3FFF;
            long sum = 0; long vbat = 0;
            for(int c = 1; c <= 14; c++)
            {
                vbat += CellMv[c];
                uint raw = ((en >> (c - 1)) & 1) != 0 ? Raw89(CellMv[c]) : 0;
                if(raw != 0) { sum += raw; }
                Regs[REG_VCELL1 + c - 1] = raw | 0x10000;
            }
            Regs[REG_VSUMBATT] = (uint)((sum >> 2) & 0x3FFFF);
            long vdiv = (long)Math.Round(vbat / 1.33); if(vdiv > 0xFFFF) vdiv = 0xFFFF;
            Regs[REG_VBATTDIV] = (uint)(vdiv & 0xFFFF) | (uint)((sum & 3) << 16);
            if(ConvGpio)
            {
                bool vtref = (Regs[REG_NCYCLE2] & 0x20000) != 0;
                for(int g = 3; g <= 9; g++)
                {
                    uint cfg = (Regs[REG_GPIO_CONF] >> (4 + 2 * (g - 3))) & 3;
                    uint raw = (vtref && cfg == 0) ? Raw89(GpioMv[g]) : 0;
                    Regs[REG_GPIO3 + g - 3] = raw | 0x10000;
                }
            }
            Regs[REG_ADCV] &= ~0x8000u;  // SOC self clears
            ConvDoneAt = -1;
            Conversions++;
        }

        /** A fault detected by the conversion (cell OV/UV, unmasked GPIO OT/UT) makes the
         *  voltage conversion routine execute a Configuration Override (~30 ms of diagnostics). */
        bool WouldOverride()
        {
            uint en = Regs[REG_VCELLS_EN] & 0x3FFF;
            uint th = Regs[REG_VCELL_TH];
            double ov = ((th >> 8) & 0xFF) * 22.784, uv = (th & 0xFF) * 22.784;
            for(int c = 1; c <= 14; c++)
            {
                if(((en >> (c - 1)) & 1) == 0) { continue; }
                if(CellMv[c] > ov || CellMv[c] < uv) { return true; }
            }
            if(ConvGpio)
            {
                uint msk = Regs[REG_CSA_GPIO_MSK];
                for(int g = 3; g <= 9; g++)
                {
                    uint cfg = (Regs[REG_GPIO_CONF] >> (4 + 2 * (g - 3))) & 3;
                    if(cfg != 0 || ((msk >> (g - 3)) & 1) != 0) { continue; }
                    uint thr = Regs[REG_GPIO3_THR + g - 3];
                    double ut = (thr & 0x1FF) * 11.392, ot = ((thr >> 9) & 0x1FF) * 11.392;
                    double v = (Regs[REG_NCYCLE2] & 0x20000) != 0 ? GpioMv[g] : 0;
                    if(v > ut || v < ot) { return true; }
                }
            }
            return false;
        }

        uint ReadReg(int addr)
        {
            if(addr == REG_DEV_GEN_CFG)
            {
                uint v = Regs[addr] & ~(0x3E000u | 0x1000u | 0x300u);
                v |= (uint)(ChipId & 0x1F) << 13;
                v |= PortH ? 0x1000u : 0;
                v |= (uint)((Regs[addr] >> 8) & 3) << 8;
                return v;
            }
            return Regs[addr] & 0x3FFFF;
        }

        void WriteReg(int addr, uint data, double t, bool broadcast)
        {
            if(State == St.Init)
            {
                // DS 4.1.2: in Init only chip_ID, isotx_en_h and iso_freq_sel are writable
                if(addr != REG_DEV_GEN_CFG) { return; }
                int id = (int)((data >> 13) & 0x1F);
                PortH = (data & 0x1000) != 0;
                Regs[addr] = (Regs[addr] & ~0x300u) | (data & 0x300);
                Fast = ((data >> 8) & 3) == 3;
                if(id != 0) { ChipId = id; State = St.Normal; LastComm = t; }
                return;
            }
            switch(addr)
            {
                case REG_DEV_GEN_CFG:
                {
                    uint keep = Lock ? 0x1300u : 0u;  // lock protects isotx_en_h and iso_freq_sel
                    uint cur = ReadReg(addr);
                    uint nv = (data & ~keep) | (cur & keep);
                    nv = (nv & ~0x3E000u) | ((uint)ChipId << 13);   // chip_ID locked
                    PortH = (nv & 0x1000) != 0;
                    Fast = ((nv >> 8) & 3) == 3;
                    Regs[addr] = nv & 0x3FFFF;
                    break;
                }
                case REG_BAL3:
                    Regs[addr] = data & 0x3FFFF & ~0x10000u;
                    if((data & 0x8000) != 0) { Lock = true; }
                    break;
                case REG_FSM:
                {
                    bool swrst = ((data >> 14) & 3) == 2;
                    bool go2slp = ((data >> 12) & 3) == 2;
                    if(swrst)
                    {
                        uint ct = Regs[REG_FASTCH] & 0x30000;
                        DefaultRegs();
                        Regs[REG_FASTCH] = ct;
                        SwResets++;
                        State = St.Init; WakeAt = t;
                    }
                    if(go2slp) { GoSleep(t, false); }
                    break;
                }
                case REG_ADCV:
                    Regs[addr] = data & 0x3FFFF;
                    if((data & 0x8000) != 0)
                    {
                        if(ConvDoneAt >= 0) { SocIgnored++; break; }  // on-demand cannot interrupt itself
                        uint filt = (data >> 9) & 7;
                        double conv = 700 + 640 * filt;               // ~1.34 ms with ADC_FILTER_SOC = 1
                        ConvGpio = (data & 0x100) != 0;
                        if(ConvGpio) { conv += 1200; }
                        if(WouldOverride()) { conv += 30000; Overrides++; }   // Configuration Override (DS 4.12)
                        ConvDoneAt = t + conv;
                    }
                    break;
                default:
                    if((addr >= REG_VCELL1 && addr < REG_VCELL1 + 14) || (addr >= REG_GPIO3 && addr <= REG_VBATTDIV)) { break; } // read only
                    Regs[addr] = data & 0x3FFFF;
                    break;
            }
        }

        public struct Result { public bool Forward; public ulong Answer; public bool HasAnswer; public double AnswerEnd; public bool AnswerFast; }

        /** A complete frame reached this device at time t on port L (viaH = false) or H. */
        public Result Receive(ulong f, bool fast, double t, bool viaH)
        {
            var r = new Result();
            if(State == St.Off) { return r; }
            Advance(t);
            if(viaH && !PortH) { return r; }                       // ISOH port disabled
            if(State == St.Sleep)
            {
                if(!fast) { State = St.Waking; WakeAt = t; ReadyAt = t + TWakeUs; Wakeups++; }
                return r;                                            // wake-up frame is not interpreted
            }
            if(State == St.Waking) { DroppedWaking++; return r; }
            if(fast != Fast) { RxSpeedMismatch++; return r; }
            r.Forward = viaH ? true : PortH;                         // repeater decision taken as bits arrive
            if(!L9963Frame.CrcOk(f)) { RxBadCrc++; return r; }
            if(L9963Frame.Pa(f) == 0) { return r; }                  // answers just travel through
            int devid = L9963Frame.DevId(f);
            bool bcast = devid == 0;
            if(!bcast && (ChipId == 0 || devid != ChipId)) { return r; }
            RxCmd++;
            if(State == St.Normal && t - LastComm > MaxCommGap) { MaxCommGap = t - LastComm; }
            LastComm = t;
            int addr = L9963Frame.Addr(f);
            bool wr = L9963Frame.Rw(f) == 1;
            bool wasFast = Fast;
            if(wr) { WriteReg(addr, L9963Frame.Data(f), t, bcast); }
            if(!bcast && State == St.Normal && !(MuteGpioRegs && addr >= REG_GPIO3 && addr < REG_GPIO3 + 7))
            {
                Advance(t);
                uint d = ReadReg(addr);
                if(addr >= REG_VCELL1 && addr < REG_VCELL1 + 14 || addr >= REG_GPIO3 && addr < REG_GPIO3 + 7)
                {
                    if(!wr) { Regs[addr] &= ~0x10000u; }             // d_rdy cleared on read
                }
                r.HasAnswer = true;
                r.AnswerFast = wasFast;
                r.Answer = L9963Frame.Build(0, wr ? 1 : 0, ChipId, addr, 0, d);
                r.AnswerEnd = t + L9963Frame.AnswerDelay(wasFast) + 40 * L9963Frame.Tbit(wasFast);
                Answers++;
            }
            return r;
        }

        public string Describe(double now)
        {
            Advance(now);
            return string.Format("S{0}: {1,-7} id={2,2} portH={3} {4} lock={5} ct={6} rx={7} ans={8} badcrc={9} spdmis={10} wake={11} sleep={12}(to {13}) swrst={14} conv={15} override={16} maxgap={17:F0} gpiomsk=0x{18:X2}",
                Position, State, ChipId, PortH ? 1 : 0, Fast ? "FAST" : "slow", Lock ? 1 : 0, (Regs[REG_FASTCH] >> 16) & 3,
                RxCmd, Answers, RxBadCrc, RxSpeedMismatch, Wakeups, Sleeps, TimeoutSleeps, SwResets, Conversions, Overrides, MaxCommGap / 1000, Regs[REG_CSA_GPIO_MSK] & 0x7F);
        }
    }

    public class L9963EChain
    {
        public L9963EChain(IMachine machine, int n)
        {
            this.machine = machine;
            Devs = new L9963EDevice[n];
            for(int i = 0; i < n; i++)
            {
                Devs[i] = new L9963EDevice(i + 1);
                for(int c = 1; c <= 14; c++) { Devs[i].CellMv[c] = (c >= 9 && c <= 11) ? 0 : 3700 + 3 * c + i; }
                for(int g = 3; g <= 9; g++) { Devs[i].GpioMv[g] = 2500; }
            }
            LinkOk = new bool[n + 1];
            for(int i = 0; i <= n; i++) { LinkOk[i] = true; }
        }

        public double Now
        {
            get
            {
                if(machine.SystemBus.TryGetCurrentCPU(out var cpu)) { cpu.SyncTime(); }
                return machine.ElapsedVirtualTime.TimeElapsed.TotalMicroseconds;
            }
        }
        public int N { get { return Devs.Length; } }
        public L9963T Bottom, Top;
        public readonly L9963EDevice[] Devs;
        public readonly bool[] LinkOk;       // link k joins node k and node k+1 (node 0 = TH, node N+1 = TL)
        public int CorruptEvery;             // corrupt one frame every N frames on the isoline (0 = off)
        public long LineFrames, Corrupted, LostOnBreak;
        public StreamWriter Trace;

        readonly IMachine machine;

        ulong MaybeCorrupt(ulong f)
        {
            LineFrames++;
            if(CorruptEvery > 0 && LineFrames % CorruptEvery == 0)
            {
                Corrupted++;
                return f ^ (1UL << 20);
            }
            return f;
        }

        /** A frame leaves node `from` (0 = TH, N+1 = TL, 1..N = device) in direction dir, fully
         *  emitted at tEnd (end of frame at the output of the node). */
        public void Travel(ulong f, bool fast, double tEnd, int from, int dir)
        {
            f = MaybeCorrupt(f);
            int node = from;
            double t = tEnd;
            while(true)
            {
                int next = node + dir;
                int link = dir > 0 ? node : next;
                if(link < 0 || link > N) { return; }
                if(!LinkOk[link]) { LostOnBreak++; return; }
                t += L9963Frame.Insertion(fast);
                if(next == 0) { Bottom?.ScheduleRx(f, fast, t); return; }
                if(next == N + 1) { Top?.ScheduleRx(f, fast, t); return; }
                var d = Devs[next - 1];
                var r = d.Receive(f, fast, t, dir < 0);
                if(Trace != null)
                {
                    Trace.WriteLine(string.Format("{0,12:F1} S{1} {2} {3} {4} fwd={5}{6}", t, next, dir > 0 ? "rxL" : "rxH",
                        fast ? "F" : "s", L9963Frame.Describe(f), r.Forward ? 1 : 0, r.HasAnswer ? " -> answer" : ""));
                }
                if(r.HasAnswer) { Travel(r.Answer, r.AnswerFast, r.AnswerEnd, next, -dir); }
                if(!r.Forward) { return; }
                node = next;
            }
        }
    }

    public class L9963T : ISPIPeripheral, IGPIOReceiver
    {
        public L9963T(IMachine machine, STM32_GPIOPort ncsPort, int ncsPin, STM32_GPIOPort txenPort, int txenPin,
                      STM32_GPIOPort isofreqPort, int isofreqPin, STM32_GPIOPort disPort, int disPin,
                      STM32_GPIOPort bnePort, int bnePin, int slaves = 1, bool isTop = false, L9963T bottom = null)
        {
            this.machine = machine;
            this.bnePort = bnePort;
            this.bnePin = bnePin;
            this.isTop = isTop;
            if(isTop)
            {
                if(bottom == null) { throw new ArgumentException("the top transceiver needs the bottom one"); }
                chain = bottom.chain;
                chain.Top = this;
            }
            else
            {
                chain = new L9963EChain(machine, slaves);
                chain.Bottom = this;
            }
            ncsPort.Connections[ncsPin].Connect(this, PinNcs);
            txenPort.Connections[txenPin].Connect(this, PinTxen);
            isofreqPort.Connections[isofreqPin].Connect(this, PinIsofreq);
            disPort.Connections[disPin].Connect(this, PinDis);
            // Renode artefact: a write to GPIOx_BSRR rewrites all the 16 pins of the port from a
            // snapshot taken before the write, so a BNE change made while another pin of the same
            // port is being written (NCS of the L9963TL is PA8, BNE PA9; TXEN/ISOFREQ/DIS of the
            // L9963TH share port D with BNE) would be lost. The model watches its own BNE pin and
            // restores the right level whenever it is overwritten.
            bnePort.Connections[bnePin].Connect(this, PinBneGuard);
            UpdateBne();
        }

        const int PinNcs = 0, PinTxen = 1, PinIsofreq = 2, PinDis = 3, PinBneGuard = 4;
        const double TWakeupUs = 1060, TxQueueSize = 3, RxQueueSize = 20;

        readonly IMachine machine;
        readonly STM32_GPIOPort bnePort;
        readonly int bnePin;
        readonly bool isTop;
        readonly L9963EChain chain;

        bool ncs = true, txen, isofreq, dis = true, enabled;
        bool txenLatched, isofreqLatched, rxFast, txFastAtEnd, frameValidWindow;
        double readyAt, txBusyUntil;
        readonly List<byte> spiIn = new List<byte>();
        byte[] spiOut;
        int spiOutIdx;
        readonly Queue<ulong> rxQueue = new Queue<ulong>();
        readonly List<double> txStarts = new List<double>();

        // statistics
        public long GuardRestores;
        public long TxFrames, TxOverflow, TxBeforeReady, TxDiscardedValid, RxFrames, RxOverflow, RxSpeedMismatch, SpiWhileNcsHigh, Popped, BadLength;

        double Now { get { return chain.Now; } }
        string Name { get { return isTop ? "TL" : "TH"; } }

        public void Reset()
        {
            // MCU reset: DIS released by the pull-up -> transceiver in stand-by. The chain keeps its state.
            lock(chain) { dis = true; ncs = true; GoStandby(); }
        }

        void GoStandby()
        {
            enabled = false;
            rxQueue.Clear();
            txStarts.Clear();
            spiOut = null;
            UpdateBne();
        }

        int bneStuck;
        bool BneLevel { get { return bneStuck != 0 || (enabled && rxQueue.Count > 0); } }
        void UpdateBne()
        {
            bnePort.OnGPIO(bnePin, BneLevel);
        }

        public void OnGPIO(int number, bool value)
        {
            lock(chain) { OnGPIOLocked(number, value); }
        }

        void OnGPIOLocked(int number, bool value)
        {
            switch(number)
            {
                case PinBneGuard:
                    if(value != BneLevel) { GuardRestores++; UpdateBne(); }
                    return;
                case PinDis:
                    if(value == dis) { return; }
                    dis = value;
                    if(dis) { GoStandby(); }
                    else
                    {
                        enabled = true;
                        readyAt = Now + TWakeupUs;
                        rxFast = isofreq; txFastAtEnd = isofreq;   // latched during the wake-up
                        txBusyUntil = 0;
                    }
                    break;
                case PinTxen: txen = value; break;
                case PinIsofreq: isofreq = value; break;
                case PinNcs:
                    if(value == ncs) { return; }
                    ncs = value;
                    if(!ncs) { NcsFalling(); } else { NcsRising(); }
                    break;
            }
        }

        void NcsFalling()
        {
            spiIn.Clear();
            spiOut = null; spiOutIdx = 0;
            frameValidWindow = enabled && Now >= readyAt;
            if(!enabled) { return; }
            txenLatched = txen;
            isofreqLatched = isofreq;
            rxFast = isofreq;                    // applied immediately to the RX side
            if(rxQueue.Count > 0 && bneStuck != 2)
            {
                ulong f = rxQueue.Dequeue();
                Popped++;
                spiOut = new byte[5];
                for(int i = 0; i < 5; i++) { spiOut[i] = (byte)(f >> (32 - 8 * i)); }
                UpdateBne();
            }
        }

        void NcsRising()
        {
            if(!enabled) { return; }
            double now = Now;
            ulong f = 0;
            for(int i = 0; i < spiIn.Count && i < 5; i++) { f = (f << 8) | spiIn[i]; }
            if(bneStuck == 2) { return; }        // dead transceiver
            if(!txenLatched)
            {
                if(spiIn.Count == 5 && L9963Frame.CrcOk(f)) { TxDiscardedValid++; }   // a real command sent with TXEN low
                if(txStarts.TrueForAll(s => s <= now)) { txFastAtEnd = isofreqLatched; }
                return;
            }
            if(!frameValidWindow) { TxBeforeReady++; return; }
            if(spiIn.Count < 1 || spiIn.Count > 8) { BadLength++; return; }
            txStarts.RemoveAll(s => s <= now);
            if(txStarts.Count >= TxQueueSize) { TxOverflow++; return; }
            bool sp = txFastAtEnd;
            double start = Math.Max(now, txBusyUntil);
            double tb = L9963Frame.Tbit(sp);
            int bits = spiIn.Count * 8;
            txBusyUntil = start + bits * tb + (sp ? 8 : 4) * tb;
            txStarts.Add(start);
            txFastAtEnd = isofreqLatched;        // applied after this frame has been sent
            TxFrames++;
            if(chain.Trace != null) { chain.Trace.WriteLine(string.Format("{0,12:F1} {1} TX {2} {3}", start, Name, sp ? "F" : "s", spiIn.Count == 5 ? L9963Frame.Describe(f) : "len " + spiIn.Count)); }
            if(spiIn.Count != 5) { return; }     // wake-up dummies of other lengths still move the line
            chain.Travel(f, sp, start + bits * tb, isTop ? chain.N + 1 : 0, isTop ? -1 : +1);
        }

        public void ScheduleRx(ulong f, bool fast, double t)
        {
            double delay = Math.Max(0, t - Now);
            machine.ScheduleAction(TimeInterval.FromMicroseconds((ulong)Math.Ceiling(delay)), _ => DeliverRx(f, fast), "L9963T rx");
        }

        void DeliverRx(ulong f, bool fast)
        {
            lock(chain) { DeliverRxLocked(f, fast); }
        }

        void DeliverRxLocked(ulong f, bool fast)
        {
            if(!enabled || bneStuck == 2) { return; }
            if(fast != rxFast) { RxSpeedMismatch++; return; }
            if(rxQueue.Count >= RxQueueSize) { RxOverflow++; return; }
            rxQueue.Enqueue(f);
            RxFrames++;
            if(chain.Trace != null) { chain.Trace.WriteLine(string.Format("{0,12:F1} {1} RX {2}", Now, Name, L9963Frame.Describe(f))); }
            UpdateBne();
        }

        public byte Transmit(byte data)
        {
            lock(chain) { return TransmitLocked(data); }
        }

        byte TransmitLocked(byte data)
        {
            if(ncs) { SpiWhileNcsHigh++; return 0; }
            spiIn.Add(data);
            if(spiOut != null && spiOutIdx < spiOut.Length) { return spiOut[spiOutIdx++]; }
            return 0x00;   // SDO in HiZ, pulled down
        }

        public void FinishTransmission() { }

        // ------------------------ scenario control (monitor) ------------------------
        L9963EDevice Dev(int n)
        {
            if(n < 1 || n > chain.N) { throw new ArgumentException("slave index out of range"); }
            return chain.Devs[n - 1];
        }
        public void SetCellMv(int slave, int cell, int mv) { Dev(slave).CellMv[cell] = mv; }
        public void SetAllCellsMv(int mv)
        {
            foreach(var d in chain.Devs) { for(int c = 1; c <= 14; c++) { if(c < 9 || c > 11) d.CellMv[c] = mv; } }
        }
        public void SetGpioMv(int slave, int gpio, int mv) { Dev(slave).GpioMv[gpio] = mv; }
        public void SetAllGpioMv(int mv) { foreach(var d in chain.Devs) { for(int g = 3; g <= 9; g++) d.GpioMv[g] = mv; } }
        public void BreakLink(int link) { chain.LinkOk[link] = false; }
        public void RestoreLink(int link) { chain.LinkOk[link] = true; }
        public void PowerOffSlave(int slave) { Dev(slave).PowerOff(); }
        public void PowerOnSlave(int slave) { Dev(slave).PowerOnReset(Now); }
        public void CorruptEvery(int n) { chain.CorruptEvery = n; }
        public void MuteGpio(int slave, bool mute) { Dev(slave).MuteGpioRegs = mute; }
        /** 0 = normal, 1 = BNE stuck high while the transceiver keeps working,
         *  2 = transceiver dead (no supply): nothing sent or received, BNE floating high */
        public void BneStuck(int mode) { lock(chain) { bneStuck = mode; UpdateBne(); } }
        public void OpenTrace(string path) { chain.Trace = new StreamWriter(path) { AutoFlush = true }; }
        public void CloseTrace() { if(chain.Trace != null) { chain.Trace.Flush(); chain.Trace.Dispose(); chain.Trace = null; } }
        public int SlaveCount() { return chain.N; }
        public string SlaveState(int slave) { return Dev(slave).State.ToString(); }

        public string Summary()
        {
            var sb = new StringBuilder();
            double now = Now;
            sb.AppendFormat("MODEL t={0:F1}ms line_frames={1} corrupted={2} lost_on_break={3}\n", now / 1000, chain.LineFrames, chain.Corrupted, chain.LostOnBreak);
            foreach(var t in new[] { chain.Bottom, chain.Top })
            {
                if(t == null) { continue; }
                sb.AppendFormat("MODEL {0}: enabled={1} tx={2} tx_overflow={3} tx_before_ready={4} tx_txen_low_valid={5} rx={6} rx_overflow={7} rx_speed_mismatch={8} popped={9} spi_ncs_high={10} bad_len={11} bne_guard={12}\n",
                    t.Name, t.enabled ? 1 : 0, t.TxFrames, t.TxOverflow, t.TxBeforeReady, t.TxDiscardedValid, t.RxFrames, t.RxOverflow, t.RxSpeedMismatch, t.Popped, t.SpiWhileNcsHigh, t.BadLength, t.GuardRestores);
            }
            foreach(var d in chain.Devs) { sb.Append("MODEL ").Append(d.Describe(now)).Append('\n'); }
            return sb.ToString();
        }
    }
}
