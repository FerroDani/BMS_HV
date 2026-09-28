//
// Minimal bxCAN (STM32F4 CAN1/CAN2) model for Renode.
// The stock STMCAN model never sets INAK while SLEEP is set, so HAL_CAN_Init()
// of STM32Cube fails (Error_Handler). This stub implements the init/sleep
// handshake, the three TX mailboxes and logs every transmitted frame to a CSV.
//   NoAck = true  -> frames stay pending (bench without other nodes), ABRQ frees them
//
using System;
using System.IO;
using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;

namespace Antmicro.Renode.Peripherals.CAN
{
    public class BxCanStub : IDoubleWordPeripheral, IKnownSize
    {
        public BxCanStub(IMachine machine)
        {
            this.machine = machine;
            Reset();
        }

        public void OpenLog(string path)
        {
            log = new StreamWriter(path) { AutoFlush = true };
            log.WriteLine("t_ms,id,dlc,data");
        }

        public long Size { get { return 0x400; } }
        public bool NoAck { get; set; }
        public long TxCount { get; private set; }
        public long Aborted { get; private set; }

        public void Reset()
        {
            Array.Clear(regs, 0, regs.Length);
            regs[MCR] = 0x00010002;
            pending = new bool[3];
        }

        public uint ReadDoubleWord(long offset)
        {
            int i = (int)(offset / 4);
            switch(i)
            {
                case MSR:
                {
                    uint mcr = regs[MCR];
                    uint v = 0;
                    if((mcr & 1) != 0) { v |= 1; }                 // INAK follows INRQ
                    else if((mcr & 2) != 0) { v |= 2; }            // SLAK follows SLEEP
                    return v | (1u << 11);                          // RX pin recessive
                }
                case TSR:
                {
                    uint v = regs[TSR] & 0x00FFFFFF;
                    int code = -1;
                    for(int m = 0; m < 3; m++) { if(!pending[m]) { v |= 1u << (26 + m); if(code < 0) { code = m; } } }
                    return v | ((uint)(code < 0 ? 0 : code) << 24);   // CODE: next free mailbox
                }
                default:
                    return i < regs.Length ? regs[i] : 0;
            }
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            int i = (int)(offset / 4);
            switch(i)
            {
                case MCR:
                    if((value & (1u << 15)) != 0) { Reset(); return; }  // RESET
                    regs[MCR] = value;
                    return;
                case TSR:
                    // RQCPx are rc_w1 (clearing RQCP also clears TXOK/ALST/TERR), ABRQx aborts
                    for(int m = 0; m < 3; m++)
                    {
                        if((value & (1u << (8 * m))) != 0) { regs[TSR] &= ~(0xFu << (8 * m)); }
                        if((value & (1u << (7 + 8 * m))) != 0 && pending[m])
                        {
                            pending[m] = false;
                            Aborted++;
                            regs[TSR] |= 1u << (8 * m);          // RQCP, TXOK = 0
                        }
                    }
                    return;
                case MSR:
                    return;
                default:
                    if(i >= regs.Length) { return; }
                    regs[i] = value;
                    for(int m = 0; m < 3; m++)
                    {
                        if(i == TI0R + 4 * m && (value & 1) != 0) { Transmit(m); }
                    }
                    return;
            }
        }

        void Transmit(int m)
        {
            uint tir = regs[TI0R + 4 * m];
            uint tdtr = regs[TI0R + 4 * m + 1];
            uint tdlr = regs[TI0R + 4 * m + 2];
            uint tdhr = regs[TI0R + 4 * m + 3];
            regs[TI0R + 4 * m] = tir & ~1u;
            uint id = (tir & 4) != 0 ? (tir >> 3) : (tir >> 21);
            int dlc = (int)(tdtr & 0xF);
            if(dlc > 8) { dlc = 8; }
            if(NoAck)
            {
                pending[m] = true;
                return;
            }
            TxCount++;
            regs[TSR] |= 0x3u << (8 * m);                            // RQCP + TXOK
            if(log != null)
            {
                var bytes = new byte[8];
                for(int b = 0; b < 4; b++) { bytes[b] = (byte)(tdlr >> (8 * b)); bytes[4 + b] = (byte)(tdhr >> (8 * b)); }
                log.WriteLine(string.Format("{0:F3},0x{1:X3},{2},{3}", machine.ElapsedVirtualTime.TimeElapsed.TotalMilliseconds, id, dlc,
                    BitConverter.ToString(bytes, 0, dlc).Replace("-", "")));
            }
        }

        const int MCR = 0, MSR = 1, TSR = 2, TI0R = 0x180 / 4;
        readonly uint[] regs = new uint[0x400 / 4];
        bool[] pending = new bool[3];
        StreamWriter log;
        readonly IMachine machine;
    }
}
