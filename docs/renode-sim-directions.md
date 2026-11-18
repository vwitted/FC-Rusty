# Renode: peripheral models and future directions

Status as of 2026-11-18. The harness lives in the Renode workspace that
contains this checkout (not version-controlled); file paths below are
relative to that workspace root unless prefixed `FC-Rusty-Code/`.

## Current state

Loaded on top of the stock platform (`@platforms/cpus/stm32h743.repl`):

| File | Effect |
|---|---|
| `platforms/local-overlay.resc` | Applies the two entries below. |
| `platforms/STM32DMA_ClearsEnable.cs`, `local-dma.repl` | DMA1/DMA2 clear `SxCR.EN` when a non-circular transfer completes. |
| `platforms/local-spi1.repl` | SPI1 is a real `STM32H7_SPI` with RX DMA request 37. |
| `platforms/local-imu.resc` | Fixed-value ICM-42688-P on SPI1 (IMU1) and SPI4 (IMU2). |
| `platforms/ICM42688P.cs`, `local-icm42688.repl` | The IMU model and its bus/CS wiring. |

The DMA change is the load-bearing one. Upstream `STM32DMA` sets TCIF but
never clears `EN`; embassy-stm32 0.4.0 `Transfer::is_running()` polls `EN`
(`dma/dma_bdma.rs`), so every DMA-backed await stayed Pending after its data
moved. On stock, the flight firmware parks in `Icm42688::new` on its first
SPI1 write. Measured against `c70e286` as well as `e07d39c`: not a
regression from the interrupt-executor or 8 kHz tick work.

With the overlay and IMU models, the flight build initialises both IMUs,
seeds the MEKF, and starts GPS, DShot, the fast inner loop, the control loop
and PosKF. `tests/run-tests.sh flight-bringup` (fast tier) asserts this,
including a level MEKF seed; a flipped chip Z was verified to seed at 180 deg
roll and fail it.

### Conventions the IMU model encodes

- Values are set in the **chip** frame: g, deg/s, degC. The firmware applies
  the mounting rotation (`src/drivers/orientation.rs`): IMU1 Roll180
  `(x, -y, -z)`, IMU2 Pitch180 `(-x, y, -z)`.
- At level rest body specific force is `(0, 0, -1) g` (NED,
  `src/conventions.rs`), which is chip `(0, 0, +1) g` for both mountings.
  That is the model default.
- Verified end to end: chip gyro X +10 dps reads body +10.0098 on IMU1 and
  -10.0098 on IMU2 (164 LSB quantisation); chip accel Y +0.5 g on IMU2 reads
  body +0.5 g and seeds roll -14.04 deg (fused 0.25 g).
- Scaling follows `GYRO_CONFIG0`/`ACCEL_CONFIG0` FS_SEL; a sensor whose
  `PWR_MGMT0` mode is off reads -32768.
- Transactions are framed on chip-select (PA4, PB1), not on SPI
  `FinishTransmission()`: `STM32H7_SPI` calls that only when TSIZE > 0, and
  embassy writes CR2 = 0.

Set values from the Monitor, e.g. `sysbus.spi4.icm2 AngularRateZ 10.0`.

## Limits that bound everything below

- **Timing is not representative.** All of a DMA transfer lands in ~zero
  virtual time; there is no timer-triggered DMA (renode/renode #618); the
  DWT cycle counter (0xE0001000) is unmodelled, so DWT-based figures (MPC
  solve time, loop timing) read 0. Renode tests logic and integration, not
  latency or CPU budget.
- **Cost.** Once the 8 kHz chain runs, one virtual second takes ~2.4 min of
  host time. Long scenarios belong in the slow tier or a separate job.
- **Observed rate figures are sim artefacts.** In a 1.2 s run the MEKF
  reported 7047 samples/s with 953 missed while the read task reported
  8213/s. Do not read these as hardware behaviour.

## IMU: next steps, in rising cost

1. **Scripted profiles.** Step or ramp the properties from the `.resc` at
   chosen virtual times (`emulation RunFor` between Monitor writes) to
   exercise MEKF convergence, gyro-bias estimation, and the arming tilt
   gate. No model change needed.
2. **Recorded or generated streams.** Add a sample queue to `ICM42688P.cs`
   fed from a CSV, or adopt Renode's RESD format as `LSM6DSO_IMU.cs` does,
   and pop one sample per data-register read. Sources: the host 6DOF sim
   (`src/sim/`, open loop), or a bench capture replayed through the inverse
   mounting rotation. This makes MEKF/PosKF behaviour comparable between
   host and target builds of the same code.
3. **Sensor error model.** Noise, bias and scale per axis, matching the
   host sim's sensor model so the two do not drift apart.
4. **Closed loop.** Decode motor commands (the order-based decoder in
   `tests/esc_model.py` already works), step physics, feed the IMU. Needs a
   co-simulation channel (Renode's external-control or socket interfaces)
   or a C# plugin. Useful for mode logic and failsafe sequencing; not for
   inner-loop tuning, given the timing limits above.

IMU2 needed no extra work: the same model attaches to stock `spi4`, whose
RX DMA request (83) is already wired.

## Other peripherals

| Peripheral | Bus (firmware) | Stock model | Direction |
|---|---|---|---|
| CRSF receiver | UART5 RX | `STM32F7_USART` | Inject CRSF frames from a socket or file backend. Unlocks arming, mode switches and RC failsafe tests. Highest value per effort. |
| GPS | USART1 | `STM32F7_USART` | Feed UBX NAV-PVT or NMEA, either canned or generated from the host sim. Exercises PosKF fusion, the home latch and the GPS-staleness fallback (`eeeda92`). |
| Barometer | I2C2, 0x76/0x77 | `STM32F7_I2C`; no DPS310/SPL06 model | Write an `II2CPeripheral` model following `ICM42688P.cs`. `BMP180.cs` upstream is a different register map. |
| Magnetometer | I2C2 | none for IST8310/QMC; `IIS2MDC.cs` exists | IIS2MDC is the industrial LIS2MDL and may be register-compatible with `lis2mdl.rs`. Unverified. |
| DShot / ESC | GPIOA via TIM1-paced DMA | timer DMA requests absent | The DMA fix may let transfers complete and the bidir RX window run. The DShot checks still run on stock and have not been re-measured on the overlay; do that first. |
| DWT | 0xE0001000 | absent | A small model returning instruction-count-derived cycles would make timing prints non-zero, although not meaningful. |

## Upstreaming

The `EN` fix is a one-line change to a generic model and likely affects
every STM32 DMA user of Renode. Offering it upstream (renode-infrastructure)
would let the local copy be deleted.
