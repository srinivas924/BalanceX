# BalanceX — Self-Balancing Robot

A two-wheeled robot that stands upright by itself. **Goal 1:** balance the bare chassis. **Goal 2:** balance with a 2 kg weight on the top tray.

| Part | Used |
|---|---|
| Controller | ESP32-WROOM-32 DevKit |
| Tilt sensor | MPU6050 (accelerometer + gyroscope, I2C) |
| Motor driver | L298N dual H-bridge |
| Motors | 2 × 12 V DC gear motors, 300 rpm, 1.4–1.5 A stall, no encoders (yet) |
| Wheels | 70 mm × 40 mm |
| Power | 3S LiPo (11.1 V) → switch → fuse → L298N + LM2596 (5 V for the ESP32) |

Wiring: [docs/CONNECTIONS.md](docs/CONNECTIONS.md)

## Folder layout

```
firmware/BalanceX/      Arduino sketch (open BalanceX.ino; the .h files appear as tabs)
  BalanceX.ino          setup(), loop(), modes, safety, serial/Bluetooth commands
  config.h              ALL pins and tunable values  <-- the file you edit
  control.h             tilt maths, complementary filter, PID, balance controller
  imu.h                 MPU6050 driver (registers, no library needed)
  motors.h              L298N driver (ESP32 core 2.x and 3.x)
docs/
  TUNING_GUIDE.md       step-by-step bring-up and tuning   <-- start here
  ERROR_LOG.md          every error we hit, cause and fix (+ template for hardware testing)
  CONNECTIONS.md        wiring list
tests/
  host_test.cpp         optional PC test: sensor sign maths + physics simulation
```

## Quick start

1. Arduino IDE 2.x + **esp32 by Espressif** board package (tested on 2.0.17 and 3.3.12). No extra libraries needed.
2. Open `firmware/BalanceX/BalanceX.ino`, select **ESP32 Dev Module**, upload.
3. Serial Monitor at **115200**, line ending **Newline**, type `help`.
4. Follow [docs/TUNING_GUIDE.md](docs/TUNING_GUIDE.md) **in order**. The sign checks (steps 2–5) come before any tuning.

None of the numbers are fixed. Every gain, limit and filter value is in `config.h` with a comment explaining it. The main ones can also be changed live over USB or Bluetooth (`kp 25`, `zero`, …), then saved with `show` + copy/paste.

## How it works

```mermaid
flowchart LR
  A[MPU6050<br/>accel + gyro] -->|200 Hz| B[tilt angle<br/>complementary filter]
  B --> C{"error = angle − target"}
  C --> D[PID<br/>KP·e + KI·∫e + KD·rate]
  D --> E[dead-band<br/>compensation]
  E --> F[L298N → motors]
  D -->|averaged output ≈ wheel speed| G[Stage 2<br/>anti-drift]
  G -->|shifts target| C
```

1. **Angle.** The accelerometer gives the tilt from gravity: never drifts, but noisy and fooled by acceleration. The gyro gives the tilt *rate*: smooth, but its integral drifts. The complementary filter combines them: `angle = α·(angle + rate·dt) + (1−α)·accelAngle`.
2. **PID.** Leaning forward → drive forward, under the robot. The D term uses the gyro rate directly. The I term is essential with PWM-driven DC motors (see [ERROR_LOG DEV-04](docs/ERROR_LOG.md#dev-04--simulation-the-first-gains-kp-15-ki-0-kd-05-made-the-robot-run-away-and-fall)).
3. **Stage 2.** If the robot keeps rolling one way, the target angle is tilted the other way so it slows down ([DEV-05](docs/ERROR_LOG.md#dev-05--simulation-with-ki-added-it-still-ran-away-accelerometer-fooled-by-acceleration)).
4. **Safety.** Motors start only after the robot is held upright for 0.5 s, stop beyond 35°, and stop at once if the sensor stops answering.

## What has been verified (and what has not)

| Check | Result |
|---|---|
| Compiles, ESP32 core **3.3.12**, all warnings on | ✅ 0 errors. 1 warning: `BluetoothSerial` deprecated (harmless, [DEV-02](docs/ERROR_LOG.md#dev-02--bluetooth-library-deprecation-warning-on-core-3x)) |
| Compiles, ESP32 core **2.0.17**, all warnings on | ✅ 0 errors, 0 warnings |
| Compiles with `USE_BLUETOOTH 0` on both cores | ✅ |
| Sensor sign maths, all 24 possible board mountings | ✅ `tests/host_test.cpp` |
| Physics simulation with your motor + wheel specs, default gains | ✅ balances; reversed motors fall, as they should |
| Same, across unknown weight (1–2 kg), centre of mass (6–20 cm), battery charge, gearbox efficiency | ✅ 105 / 108 variants, all 81 with centre of mass ≤ 15 cm ([DEV-11](docs/ERROR_LOG.md#dev-11--re-simulation-with-the-real-motors-the-first-defaults-were-too-weak-for-70-mm-wheels)) |
| **Running on the real robot** | ❌ **Not yet.** This is your step: follow TUNING_GUIDE and log results in ERROR_LOG Part D |

Run the PC test (optional, any computer with g++):

```
g++ -std=c++17 -O2 -o host_test tests/host_test.cpp && ./host_test
```

The simulation uses your motor and wheel specs. Weight, centre-of-mass height and gearbox friction are still guesses, and it has no gearbox backlash. It proves the signs and the controller logic and gives sensible starting gains; the real robot sets the final ones.

## Better ways to do this project

In order of how much they would help, especially for the 2 kg goal:

1. **Wheel encoders (biggest improvement, you plan these anyway).** Without them, the robot can only guess its wheel speed from the PWM it sends. That's why it tends to drift and needs the Stage-2 workaround. With encoders you get a proper speed/position loop around the angle loop: it stands still in one spot, does not run away, and you can later make it drive and turn. Easiest route: the encoder version of the same motor (same size, shaft and rpm; usually a 6-wire connector), so the chassis and wheels stay the same. It is 4 extra wires (to free pins such as GPIO 32, 33, 34, 35) plus code using the ESP32's hardware pulse counter.
2. **Replace the L298N before the 2 kg stage.** Your motors stall at 1.4–1.5 A, which is inside the L298N's 2 A per channel, so it is **fine for balancing the bare chassis**. But it is an old bipolar-transistor driver that loses about 2–4 V of the 11.1 V battery as heat: roughly 20–35 % of the voltage your motors could get, and it hurts most when catching a heavy load. A MOSFET driver loses almost nothing; for 1.5 A-stall motors, e.g. two DRV8871 or DRV8874 boards (one per motor, both comfortably above 1.5 A). Avoid the popular TB6612FNG here: it is rated only 1.2 A continuous. It would use the same ESP32 pins with small code changes.
3. **Battery-voltage compensation.** As the LiPo drains from 12.6 V to about 10.5 V, the same PWM gives about 17 % less voltage, so the robot gets "lazier" during a session. A voltage divider to an ADC pin (GPIO 34/35) lets the code scale the PWM to keep the behaviour constant.
4. **Check the gearbox backlash.** Wiggle each wheel by hand. Play in the gears is the classic cause of small, never-ending wobbles in DC-motor balancing robots. Low-backlash motors make tuning much easier.
5. **Mount the sensor rigidly**, close to the robot's centre, away from the motors (vibration) and motor cables (I2C noise).
6. **Later: LQR instead of PID.** Once encoders give you wheel speed, a state-space controller (LQR) designed from a model of the robot handles angle and speed together. It is more systematic than hand-tuning two PID loops, and a strong topic for a project report.
7. **Alternative design (for a future version, not this one):** stepper motors (NEMA 17 + TMC2209 drivers) are popular for DIY balancing robots because they have no backlash and their speed is exact without encoders. The cost is weight, current draw and less torque at speed, so with a 2 kg payload a good DC motor + encoder + MOSFET driver is usually the better choice.

**Recommended path:** get the current hardware balancing with this code first. It will teach you the tuning and sign checks. Then add encoders, and replace the L298N before attempting the 2 kg payload.
