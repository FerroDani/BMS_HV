# Firmware BMS HV master – guida rapida

Branch `fw/bms-hv-l9963e` (base: `dev` @ 4ead262). Target STM32F446VE, catena di L9963E tramite due L9963T
(L9963TH su SPI3 = inizio dell'anello, L9963TL su SPI2 = fine dell'anello, usato solo con il doppio anello).

## Compilare

```sh
make -j8                                                   # 1 slave (banco)
make -j8 BUILD_DIR=build_n11 BMS_DEFS="-DBMS_N_SLAVES=11"  # macchina, anello singolo
make -j8 BUILD_DIR=build_n12d BMS_DEFS="-DBMS_N_SLAVES=12 -DBMS_DUAL_RING=1"
```

Output: `build/bms_hv_fsm.{elf,hex,bin}`. Flash con ST-Link / OpenOCD (`openocd.cfg` nel repo) o STM32CubeProgrammer
(`.hex` o `.bin` a 0x08000000). Con un `BUILD_DIR` nuovo per ogni configurazione si evita di mescolare oggetti
compilati con macro diverse.

## Configurazione (`Core/Inc/bms_config.h`, tutte sovrascrivibili con `BMS_DEFS`)

| Macro | Default | Significato |
|---|---|---|
| `BMS_N_SLAVES` | 1 | slave nella catena (1…31) |
| `BMS_DUAL_RING` | 0 | 1 = anello chiuso anche sull'L9963TL |
| `BMS_NTC_GPIO_MASK` | 0x7F | GPIO3..9 con NTC (bit0 = GPIO3). **Al banco senza NTC: 0** |
| `BMS_MEAS_PERIOD_MS` / `BMS_GPIO_EVERY_N_CYCLES` | 100 / 2 | periodo misura celle / temperature ogni N cicli |
| `BMS_SLAVE_COMM_TIMEOUT` | 1 (256 ms) | CommTimeout degli L9963E (3 = 2 s per debug con breakpoint) |
| `BMS_CELL_OV_MV` / `BMS_CELL_UV_MV` | 4200 / 2800 | soglie firmware celle |
| `BMS_CELL_OT_DC` / `BMS_CELL_UT_DC` | 600 / −200 | soglie temperatura [0,1 °C] |
| `BMS_FAULT_TIME_VOLTAGE_MS` / `_TEMP_MS` | 400 / 800 | tempo dall'ultimo campione buono → guasto |
| `BMS_FAULT_TIME_COMM_MS` / `_TEMP_STALE_MS` | 400 / 800 | dati non aggiornati → guasto |
| `BMS_STARTUP_GRACE_MS` | 3000 | tempo concesso alla catena per partire all'accensione |
| `BMS_ENABLE_FSM` | 0 | macchina a stati relè (AIR/precarica) — **non validata, lasciare 0** |
| `BMS_ENABLE_CAN` / `BMS_CAN_BITRATE_KBPS` | 1 / 500 | telemetria HVCB su CAN1 |
| `BMS_ENABLE_LOG` | 1 | log su USART3 (PB10, 115200 8N1) |
| `BMS_ENABLE_IWDG` | 1 | watchdog indipendente 2 s (fermo quando il core è in halt dal debugger) |

## Cosa fa all'accensione

1. Transceiver in stand-by (DIS rilasciato 5 ms), poi DIS basso, ISOFREQ basso, attesa 2 ms.
2. Probe veloce: se trova la catena ancora sveglia in fast (reset dell'MCU a catena alimentata) la resetta
   (SW_RST+GO2SLP; dal lato L9963TL solo GO2SLP, per non perdere l'indirizzo degli slave oltre un'eventuale rottura).
3. Indirizzamento DS 4.1.2.2 (libreria corretta), passaggio a isoSPI fast, lock.
   Doppio anello con rottura: gli slave oltre la rottura vengono completati dall'L9963TL.
4. Configurazione in broadcast + rilettura di ogni slave (un broadcast perso viene riparato con una scrittura unicast):
   celle C1..C8, C12..C14; GPIO3..9 analogici; VTREF on; soglie HW OV 4,3 V / UV 2,5 V; diagnostica GPIO OT/UT
   mascherata; CommTimeout.
5. Ciclo di misura ogni 100 ms, non bloccante (uno slave per giro del main loop).

## LED e uscite

| | |
|---|---|
| STAT1 | lampeggio 1 Hz = firmware vivo |
| STAT2 | acceso = catena pronta; lampeggio veloce = inizializzazione in corso |
| WARN | condizione fuori soglia non ancora latchata, dati non validi, anello doppio aperto |
| ERR + AMS_ERROR (PC6, attivo alto) | guasto AMS latchato fino al reset dell'MCU |

Guasti (`faults`, anche su CAN): 0x01 OV, 0x02 UV, 0x04 OT, 0x08 UT, 0x10 NTC aperto/corto,
0x20 comunicazione celle, 0x40 temperature non aggiornate, 0x80 reset da watchdog.

## Log seriale (esempio)

```
[     0.000] BMS HV master fw 1.0.0 (local) - slaves 11, dual ring 0, meas 100 ms, gpio every 2
[     0.145] AFE: chain ready, 11 slave(s), init 138 ms
[     1.000] STATUS afe=READY ready=1 init=1/1 cycles=9 cycle=13ms ams=0 faults=0x00 active=0x00 pack=450780mV ...
[     1.000] S01 H fail=0 V[mV]: 3703 3706 ... sum=40925 vb=40925 T[0.1C]: 254 254 ...
```
Messaggi diagnostici utili: `addressing failed, k/N slaves answered (no answer from slave k+1)` = rottura del
collegamento isoSPI dopo lo slave k (o slave k+1 non alimentato); `CSA_GPIO_MSK = ..., expected ...: rewriting it`
= broadcast perso e riparato.

## CAN (HVCB, SC26)

0x200 HVB_RX_Diagnosis 10 ms · 0x203 HVB_RX_Status 20 ms (stSys 0 init, 1 running, 2 AMS error) ·
0x204 HVB_RX_Measure 20 ms (uHvb) · 0x206 HVB_RX_VCell 100 ms · 0x208 HVB_RX_TCell 100 ms · 0x20F SW version 1 s.

## Checklist prima prova su hardware

1. Senza HV: alimentare il master, verificare boot e log su USART3.
2. Con 1 slave: firmware default. Senza NTC montate compilare con `BMS_NTC_GPIO_MASK=0`, altrimenti AMS per NTC aperta.
3. Controllare `chain ready`, le 11 tensioni, `sum` ≈ somma, `vb` ≈ tensione del modulo misurata col multimetro.
4. Verificare la curva NTC dell'HV (quella usata è quella del BMS LV): 25 °C ambiente → ~250 nel log.
5. Staccare il cavo isoSPI: entro 0,5 s AMS_ERROR alto e log `FAULT SLAVE COMMUNICATION`.
6. Solo dopo, passare a 11/12 slave e, se cablato, al doppio anello.

## Banco di prova (senza hardware)

`tools/hil_sim/`: il firmware ARM gira in Renode 1.15 con un modello di L9963T + catena di L9963E
(`renode/L9963Sim.cs`) e uno stub bxCAN. `RENODE=/percorso/renode python3 tools/hil_sim/run_tests.py [filtro]`
esegue 31 scenari (nominali 1/11/12 slave e doppio anello, guasti OV/UV/OT/NTC, rotture dell'anello, reset,
rumore, watchdog, CAN) e scrive i log in `tools/hil_sim/out/`. `make -C tools/hil_sim/unit` esegue i test
unitari del monitor di sicurezza sull'host. Serve `pip install cantools` per la verifica dei payload CAN.
