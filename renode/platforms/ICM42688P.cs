//
// ICM42688P.cs — fixed-value TDK ICM-42688-P model for Renode (SPI, bank 0).
//
// Loaded at runtime with `include @renode/platforms/ICM42688P.cs`; attached in
// renode/platforms/local-icm42688.repl. Models what src/drivers/icm42688.rs uses:
//
//   0x11 DEVICE_CONFIG   bit 0 SOFT_RESET -> registers to reset values
//   0x1D..0x2A           TEMP, ACCEL X/Y/Z, GYRO X/Y/Z, big-endian i16
//   0x2D INT_STATUS      DATA_RDY (bit 3) always set
//   0x4E PWR_MGMT0       [1:0] accel mode, [3:2] gyro mode; 0 = off
//   0x4F GYRO_CONFIG0    [7:5] FS_SEL: 0 = +-2000 dps (16.384 LSB/dps) .. 7
//   0x50 ACCEL_CONFIG0   [7:5] FS_SEL: 0 = +-16 g (2048 LSB/g) .. 3
//   0x75 WHO_AM_I        0x47
//   0x76 REG_BANK_SEL    other banks are accepted and read as 0
//
// Other bank-0 registers are plain storage. A sensor whose mode is off
// reads -32768, as the datasheet specifies.
//
// Values are set in the CHIP frame, in g, dps and degC, from the Monitor:
//   sysbus.spi1.icm1 AccelerationZ 1.0
// The firmware maps chip -> body (NED) with the mounting rotation in
// src/drivers/orientation.rs. At level rest body specific force is
// (0, 0, -1) g (src/conventions.rs), which is chip (0, 0, +1) g for both
// Roll180 (IMU1) and Pitch180 (IMU2): that is the default.
//
// Transaction framing is on chip-select, NOT FinishTransmission():
// STM32H7_SPI only calls FinishTransmission() when TSIZE > 0, and embassy
// leaves TSIZE = 0. Wire the CS pin to GPIO input 0 (active low).
//
using System;

using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.SPI;

namespace Antmicro.Renode.Peripherals.Sensors
{
    public class ICM42688P : ISPIPeripheral, IGPIOReceiver
    {
        public ICM42688P()
        {
            Temperature = 25.0;
            AccelerationZ = 1.0;
            Reset();
        }

        public void Reset()
        {
            Array.Clear(registers, 0, registers.Length);
            bank = 0;
            BeginTransaction();
        }

        // Chip-frame inputs. Persist across Reset(): they describe the world,
        // not the chip's register state.
        public double AccelerationX { get; set; }
        public double AccelerationY { get; set; }
        public double AccelerationZ { get; set; }
        public double AngularRateX { get; set; }
        public double AngularRateY { get; set; }
        public double AngularRateZ { get; set; }
        public double Temperature { get; set; }

        public byte Transmit(byte data)
        {
            if(expectingAddress)
            {
                expectingAddress = false;
                isRead = (data & ReadFlag) != 0;
                address = (byte)(data & ~ReadFlag);
                // Snapshot the data block once per transaction so a burst
                // read is coherent, as the chip's output latch is.
                LatchSample();
                return 0;
            }

            byte result = 0;
            if(isRead)
            {
                result = ReadRegister(address);
            }
            else
            {
                WriteRegister(address, data);
            }
            address = (byte)((address + 1) & 0x7F);
            return result;
        }

        public void FinishTransmission()
        {
            BeginTransaction();
        }

        // GPIO 0 = CS, active low. Either edge starts a fresh transaction.
        public void OnGPIO(int number, bool value)
        {
            if(number != 0)
            {
                this.Log(LogLevel.Warning, "Unexpected GPIO {0}", number);
                return;
            }
            BeginTransaction();
        }

        private void BeginTransaction()
        {
            expectingAddress = true;
            isRead = false;
        }

        private byte ReadRegister(byte reg)
        {
            if(bank != 0)
            {
                return reg == RegBankSel ? bank : (byte)0;
            }
            if(reg >= RegTempData1 && reg < RegTempData1 + DataBlockLength)
            {
                return sample[reg - RegTempData1];
            }
            switch(reg)
            {
            case RegWhoAmI:
                return WhoAmIValue;
            case RegIntStatus:
                return IntStatusDataReady;
            case RegBankSel:
                return bank;
            default:
                return registers[reg];
            }
        }

        private void WriteRegister(byte reg, byte value)
        {
            if(reg == RegBankSel)
            {
                bank = (byte)(value & 0x07);
                return;
            }
            if(bank != 0)
            {
                return;
            }
            if(reg == RegDeviceConfig && (value & SoftReset) != 0)
            {
                this.Log(LogLevel.Debug, "Soft reset");
                Array.Clear(registers, 0, registers.Length);
                return;
            }
            registers[reg] = value;
        }

        private void LatchSample()
        {
            var pwr = registers[RegPwrMgmt0];
            var accelOn = (pwr & 0x03) != 0;
            var gyroOn = (pwr & 0x0C) != 0;

            var accelFs = (registers[RegAccelConfig0] >> 5) & 0x07;
            var gyroFs = (registers[RegGyroConfig0] >> 5) & 0x07;
            var accelLsbPerG = 2048.0 * (1 << Math.Min(accelFs, 3));
            var gyroLsbPerDps = 16.384 * (1 << gyroFs);

            var temp = ToRaw((Temperature - 25.0) * 132.48);
            Put(0, temp);
            Put(2, accelOn ? ToRaw(AccelerationX * accelLsbPerG) : SensorOff);
            Put(4, accelOn ? ToRaw(AccelerationY * accelLsbPerG) : SensorOff);
            Put(6, accelOn ? ToRaw(AccelerationZ * accelLsbPerG) : SensorOff);
            Put(8, gyroOn ? ToRaw(AngularRateX * gyroLsbPerDps) : SensorOff);
            Put(10, gyroOn ? ToRaw(AngularRateY * gyroLsbPerDps) : SensorOff);
            Put(12, gyroOn ? ToRaw(AngularRateZ * gyroLsbPerDps) : SensorOff);
        }

        private void Put(int offset, short value)
        {
            sample[offset] = (byte)(value >> 8);
            sample[offset + 1] = (byte)value;
        }

        private static short ToRaw(double value)
        {
            var r = Math.Round(value);
            if(r > short.MaxValue)
            {
                return short.MaxValue;
            }
            if(r < short.MinValue + 1)
            {
                return short.MinValue + 1;
            }
            return (short)r;
        }

        private readonly byte[] registers = new byte[128];
        private readonly byte[] sample = new byte[DataBlockLength];
        private byte bank;
        private byte address;
        private bool isRead;
        private bool expectingAddress;

        private const byte ReadFlag = 0x80;
        private const byte RegDeviceConfig = 0x11;
        private const byte RegTempData1 = 0x1D;
        private const int DataBlockLength = 14;
        private const byte RegIntStatus = 0x2D;
        private const byte RegPwrMgmt0 = 0x4E;
        private const byte RegGyroConfig0 = 0x4F;
        private const byte RegAccelConfig0 = 0x50;
        private const byte RegWhoAmI = 0x75;
        private const byte RegBankSel = 0x76;

        private const byte WhoAmIValue = 0x47;
        private const byte IntStatusDataReady = 0x08;
        private const byte SoftReset = 0x01;
        private const short SensorOff = short.MinValue;
    }
}
