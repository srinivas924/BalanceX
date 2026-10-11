# BalanceX — Error Log and Fixes

This file records every problem we ran into while building BalanceX: what we saw, why it happened, how we found the cause, and what we changed. The aim is that anyone on the team can answer *"What errors did you face and how did you fix them?"* with specific examples and evidence.

**How this file is organised**

| Part | What it contains | Status |
|---|---|---|
| [A. Errors found while writing the code](#a-errors-found-while-writing-the-code) | Real compiler messages and simulation failures from development, with evidence | Done (2026-10-11) |
| [B. Problems designed out in advance](#b-problems-designed-out-in-advance) | Well-known balancing-robot / ESP32 pitfalls the code guards against, and where | Done |
| [C. Troubleshooting table](#c-troubleshooting-table-symptom--cause--fix) | Symptom → likely cause → fix, for problems you may still meet | Reference |
| [D. Hardware testing log](#d-hardware-testing-log--fill-this-in) | **Your** entries from testing the real robot, using the template | To fill in |

**Be honest in a viva.** Part A happened. Part B lists problems we *prevented* by design; we never saw them on the robot, so describe them that way ("we knew X is a common problem, so we did Y"). Part D is where real hardware stories go. Write those down **the same day** they happen, while the details are fresh.

---

## A. Errors found while writing the code

The firmware was compiled with **arduino-cli** for the board `ESP32 Dev Module` (`esp32:esp32:esp32`) on **two** versions of the ESP32 Arduino core: **2.0.17** (the last 2.x release) and **3.3.12** (a current release). Warnings were set to "all". The control maths was also tested on a PC with `tests/host_test.cpp`.

### DEV-01 — PWM functions have different names on ESP32 core 2.x and 3.x

| | |
|---|---|
| **Stage** | Writing the motor driver (`motors.h`) |
| **Symptom** | Motor PWM code written the "old" way does not compile on core 3.x, and the "new" way does not compile on core 2.x. Real messages from our test compile: |
| | core 3.3.12: `error: 'ledcSetup' was not declared in this scope` and `error: 'ledcAttachPin' was not declared in this scope; did you mean 'ledcAttach'?` |
| | core 2.0.17: `error: 'ledcAttach' was not declared in this scope` |
| **Root cause** | Espressif rewrote the LEDC (PWM) API in core 3.0. Old: `ledcSetup(channel, freq, bits)` + `ledcAttachPin(pin, channel)` + `ledcWrite(channel, duty)`. New: `ledcAttach(pin, freq, bits)` + `ledcWrite(pin, duty)`. Most online tutorials still use the old one. |
| **How we found it** | Compiled a 2-line test sketch with each API against each core version, and read `esp32-hal-ledc.h` in both cores. |
| **Fix** | `motors.h` checks `ESP_ARDUINO_VERSION_MAJOR` and uses the matching functions (`attachPwm()` / `writePwm()`). The `.ino` stops with a clear `#error` if the core is older than 2.x. |
| **Verified** | Full firmware compiles with 0 errors on 2.0.17 and 3.3.12. |
| **One-line explanation** | "The ESP32 PWM library changed its function names in version 3, so we wrote a small wrapper that picks the right one at compile time." |

### DEV-02 — Bluetooth library deprecation warning on core 3.x

| | |
|---|---|
| **Stage** | First full compile on core 3.3.12 |
| **Symptom** | `warning: 'BluetoothSerial' is deprecated: BluetoothSerial won't be supported in version 4.0.0 by default [-Wdeprecated-declarations]` |
| **Root cause** | Espressif plans to drop Bluetooth Classic serial from the default build in core 4.0. It still works in every 2.x and 3.x release. |
| **Fix** | Kept it, because tuning without a USB cable pulling on the robot is very useful. Bluetooth is behind a switch: `#define USE_BLUETOOTH 1` in `config.h`. Set it to `0` to remove it completely. |
| **Verified** | Compiles with `USE_BLUETOOTH 1` and `0` on both core versions. The warning only appears on 3.x and is harmless. |
| **Future** | If a later core removes it: use BLE or a small Wi-Fi web page instead. |

### DEV-03 — Program size with Bluetooth is 84–85 % of flash

| | |
|---|---|
| **Stage** | First full compile |
| **Observation** | `Sketch uses 1107199 bytes (84%)` (core 3.3.12) / `1120541 bytes (85%)` (core 2.0.17) of the 1,310,720-byte app partition. Without Bluetooth: about 23–24 %. |
| **Root cause** | The Bluetooth Classic stack is about 0.8 MB on its own. |
| **Risk** | Adding more libraries later (e.g. Wi-Fi) can push it past 100 % and give `Sketch too big` / `text section exceeds available space in board`. |
| **Fix if it happens** | Arduino IDE → **Tools → Partition Scheme → "Huge APP (3MB No OTA/1MB SPIFFS)"**, or set `USE_BLUETOOTH 0`. |

### DEV-04 — Simulation: the first gains (KP 15, KI 0, KD 0.5) made the robot run away and fall

| | |
|---|---|
| **Stage** | Checking the control logic with a physics simulation (`tests/host_test.cpp`) before touching hardware |
| **Symptom** | The simulated robot recovered from a 5° lean, then started rolling in one direction, faster and faster, and fell after a few seconds. With KI = 0 (and no anti-drift loop) this happened for **every** KP/KD combination in the test grid. |
| **Root cause** | A DC motor driven by PWM behaves like a **speed** control (more PWM → more speed, not more force). To catch a fall, the wheels must keep **accelerating** while the robot leans. Plain PD only gives "speed proportional to lean", which is not enough. A linear stability analysis of the same model showed one unstable pole for every KD when KI = 0 (+1.7 to +2.1 rad/s for KP 15, i.e. the speed error doubles roughly every 0.35 s). |
| **How we found it** | The test printed FELL. We traced angle, speed, PWM and I term every 0.25 s and saw the speed growing steadily. Then we computed the eigenvalues of the linearised model. |
| **Fix** | Use a non-zero KI. The I term keeps increasing the PWM while the lean lasts, which produces the needed acceleration. Analysis showed the unstable pole disappears for KI ≳ 50 (in our units). |
| **One-line explanation** | "Our motors are speed-controlled by PWM, so a plain PD controller cannot produce the acceleration needed to stay under the robot. The integral term supplies it." |

### DEV-05 — Simulation: with KI added it still ran away (accelerometer fooled by acceleration)

| | |
|---|---|
| **Stage** | Same simulation, after DEV-04 |
| **Symptom** | Better, but many runs still drifted off, the I term hit its limit (±150) and the robot fell after 3–5 s. |
| **Root cause** | While the robot **accelerates**, the accelerometer cannot tell acceleration from tilt. In the trace the true lean was about −1.5° but the filtered angle read about −0.4°: a constant error of about 1° exactly while it was speeding up, so it kept speeding up. |
| **How we found it** | Printed true angle vs filtered angle side by side. The gap grew with the acceleration (≈ atan(a/g)). |
| **Fix** | Turned on the **Stage-2 anti-drift loop** by default (`SPEED_KP = 0.02`). The averaged motor output stands in for wheel speed; if the robot keeps rolling one way, the target angle is tilted the other way. The proper long-term fix is wheel encoders (see the README's "Better ways"). |
| **Verified** | With KI = 100 and SPEED_KP = 0.02, the simulated robot balanced for **every** KP 10–60 × KD 0.25–2.0 tried, with and without a 2 kg payload. With KI = 0 and SPEED_KP = 0, none did. Run `tests/host_test.cpp` to reproduce both maps. |

### DEV-06 — Simulation: without dead-band compensation the robot did not settle

| | |
|---|---|
| **Symptom** | With `MOTOR_MIN_PWM = 0`, the simulated robot (gearbox friction ≈ 6 % of stall torque) fell or kept wobbling. With 40 it balanced. |
| **Root cause** | Small corrections (e.g. PWM 10) do not overcome gearbox friction, so the wheels do nothing until the lean is already large. |
| **Fix** | Kept the dead-band compensation in `shapeCommand()` and made measuring `MOTOR_MIN_PWM` with the `ramp` test a required step in the tuning guide. |
| **Re-checked with the real motor specs (DEV-11)** | The single middle-case robot now balanced even with `minpwm 0`, but across the 81 likely robot variants only 54 did, against 81 with `minpwm 40`. With a stiffer gearbox (friction 2–3× higher) **none** balanced without it, and 77–78 of 81 did with `minpwm 60`. Conclusion unchanged: measure it. |

### DEV-07 — Simulation: good gains depend strongly on the motors

| | |
|---|---|
| **Observation** | We swept the guessed motor specs. Gains that worked for 300 rpm motors failed for 150 rpm motors. 150 rpm motors needed roughly 2× KP, KI and KD without payload, and about 4× with the 2 kg payload. |
| **Consequence** | The defaults in `config.h` are only a starting point. The comment ranges were widened (KP 10–80, KI 50–400, KD 0.25–3) and the tuning guide explains how to scale. **We needed the real motor specs to give better starting values. They arrived later; see DEV-11.** |

### DEV-08 — Firmware and simulation could drift apart

| | |
|---|---|
| **Problem** | The first simulation copied the controller lines from the `.ino`. Any later change to the firmware would silently make the test meaningless. |
| **Fix** | Moved the complete controller (`Tuning`, `BalanceController`, `shapeCommand`, filter, PID) into `control.h`, which has no Arduino code. The firmware **and** the PC test both use that same file. |

### DEV-09 — Safety bug found in code review: motors could start with the robot lying on its side

| | |
|---|---|
| **Stage** | Reading through the finished `BalanceX.ino` before the first upload |
| **Problem** | The tilt angle is `atan2(-front, up)`. Lying on its **side**, gravity points along the wheel axle, so both `front` and `up` are about 0 g and the "angle" is `atan2` of two noise values: it can be anything, including ≈ 0°. If it happened to stay within 3° of the setpoint for 0.5 s, balancing (and the motors) would start with the robot on its side. |
| **Root cause** | The start/fall checks only looked at the tilt *around the axle*, not at whether the robot was on its side. |
| **Fix** | `readSensors()` also tracks gravity along the axle (`sidewaysG`, smoothed over ~0.1 s so a vibration spike cannot trigger it). Balancing only starts if it is below `SIDEWAYS_LIMIT_G` (0.5 g ≈ 30°), and the motors cut out if it goes above. |
| **Verified** | Compiles on both cores. Must still be checked on the robot: lay it on its side in `go` mode; the LED must keep slow-blinking and the wheels must stay still. |
| **One-line explanation** | "A tilt sensor can't measure forward/back lean when the robot lies on its side, so we added a second check that keeps the motors off in that position." |

### DEV-10 — Checked and found to be a false alarm: `Wire.begin()` overloads

| | |
|---|---|
| **Concern** | The ESP32 `Wire` library also has an I2C *slave* `begin(uint8_t addr, int sda, int scl, uint32_t freq)`. We worried that `Wire.begin(sda, scl, freq)` with `uint8_t` pin numbers could pick it by mistake. |
| **Check** | Compiled a test sketch on both cores and read `Wire.h`. The slave version needs **4** arguments, so a 3-argument call can only be the master version. |
| **Result** | Not a problem. Pins are declared `int` anyway, because that is the type `Wire.begin()` expects. **Lesson:** check assumptions against the real library source before "fixing" them. |

### DEV-11 — Re-simulation with the real motors: the first defaults were too weak for 70 mm wheels

| | |
|---|---|
| **Stage** | After the team supplied the specs: 12 V, 300 rpm gear motors, 1.4–1.5 A stall current, 70 mm wheels, MPU6050 flat, no encoders |
| **What we did** | Worked out the motor's torque from the specs: 12 V ÷ 31.4 rad/s = 0.38 N·m per amp, × 1.45 A = 0.55 N·m stall torque per motor with a perfect gearbox, so 0.55–1.1 N·m for both motors depending on gearbox efficiency. We put that and the 35 mm wheel radius into `tests/host_test.cpp`. The robot's weight and centre-of-mass height are still not measured, so we swept them: 1–2 kg, centre of mass 6–20 cm above the axle, battery 10.5 / 11.1 / 12.6 V, and three gearbox efficiencies. |
| **Symptom** | The first defaults (KP 20, KI 100, KD 0.75) failed in 17 of 54 variants in the first sweep, mostly with a nearly empty battery (10.5 V) or a high centre of mass. |
| **Root cause** | Smaller wheels mean less ground speed per PWM step, and a lower battery means less voltage per PWM step. Both reduce the effective loop gain, so the same KP/KI/KD "pushes" less. |
| **Fix** | Compared six candidate gain sets on the same sweep and chose **KP 30, KI 250, KD 1.0**. It balanced 105 of 108 variants, and **all 81** with the centre of mass at 15 cm or lower (likely for this chassis, with the battery on the bottom plate). The 3 failures all have the centre of mass at 20 cm. It was also among the lowest KP/KD of the candidates, which matters because real gearbox backlash and noise (not in the simulation) punish high KP/KD. |
| **Also seen** | Failures cluster at the lowest battery voltage. That supports tuning on a charged battery and, later, battery-voltage compensation (README, "Better ways" 3). |
| **Verified** | `tests/host_test.cpp` Test 3 now checks the config.h gains across this whole sweep. |
| **Caution** | Differences between neighbouring gain sets are within what a simplified model can tell apart. The real robot decides. |

---

## B. Problems designed out in advance

These are classic failures of ESP32 / MPU6050 / balancing-robot projects. We did **not** see them; the code is written so they should not happen, or so they are easy to diagnose if they do.

| ID | Known problem | What the code does | Where |
|---|---|---|---|
| P-01 | `Serial.readStringUntil()` waits up to 1 s for a newline, so the robot falls while you type | Non-blocking `LineReader` collects characters each loop | `BalanceX.ino` |
| P-02 | Printing too much stalls the control loop and makes balancing worse | 4 KB serial TX buffer, telemetry limited to 20 Hz, `show` reports the longest loop time and late loops | `setup()`, `printTelemetry()`, `printSettings()` |
| P-03 | Sensor wire comes loose → code keeps the last motor command → robot races off | Any failed I2C read stops the motors at once; balancing only resumes after the robot is stood up again; the sensor is restarted automatically | `handleImuFailure()` |
| P-04 | A disconnected / unpowered MPU6050 can return all zeros, which looks like "level" | A reading with less than 0.2 g total is treated as a failed read | `imu::readUncalibrated()` |
| P-05 | Integral wind-up: after a fall or while held in the air, the I term grows huge and the robot lurches when restarted | I term clamped (`I_TERM_LIMIT`) and cleared on every fall, stop and mode change | `PidController`, `stopMotors()` |
| P-06 | Differentiating a noisy angle makes the D term jittery, and changing the setpoint causes a "derivative kick" | D term uses the gyro rate directly | `PidController::update()` |
| P-07 | Changing KI live makes the output jump | The I term stores `KI·error·dt`, not `error·dt` | `PidController::update()` |
| P-08 | Gyro offset makes the angle drift | Offset measured at start-up (500 samples); if the readings spread more than 2 °/s the robot moved, so it repeats | `imu::calibrateGyro()` |
| P-09 | Gyro-only angle drifts, accelerometer-only angle is noisy | Complementary filter, `COMP_FILTER_ALPHA` tunable live | `ComplementaryFilter` |
| P-10 | Wrong axis or sign for the sensor mounting → robot drives *away* from the fall | User sets only UP and FRONT axes; the gyro axis and sign are **calculated** (cross product), wrong combinations stop the compile (`static_assert`), and `sensor` mode warns if gyro and accelerometer disagree. Checked for all 24 possible mountings in `tests/host_test.cpp` | `config.h` §5, `control.h` |
| P-11 | Clone MPU6050 chips report WHO_AM_I ≠ 0x68 and some libraries refuse to start | Own register-level driver; prints the ID and carries on | `imu.h` |
| P-12 | ESP32 resets when motors start (battery sag → brownout) and you cannot tell why | Last reset reason printed at boot, with an explicit brownout warning | `setup()` |
| P-13 | Motors twitch during boot | All motor pins forced LOW as the very first line of `setup()` | `motors::forcePinsLow()` |
| P-14 | A long pause (e.g. calibration) makes `dt` huge and the filter jumps | `dt` capped at 50 ms | `loop()` |
| P-15 | Robot jumps to life when switched on lying down | Starts only after being held within 3° of upright for 0.5 s (and not on its side, DEV-09); motors off beyond 35° | `runBalance()` |
| P-16 | Tuned values are lost or "the code change does nothing" | Live changes are never saved to flash, so `config.h` is always the truth. `show` prints them ready to paste | `printSettings()` |

---

## C. Troubleshooting table (symptom → cause → fix)

### Compiling and uploading

| Symptom | Likely cause | Fix |
|---|---|---|
| `'ledcSetup' was not declared in this scope` in *other* example code | Code written for ESP32 core 2.x, you have 3.x | See DEV-01; our code already handles both |
| `Update the ESP32 board package to version 2.x or 3.x` | Very old core (1.0.x) | Boards Manager → esp32 by Espressif → install 3.x |
| Errors about Bluetooth / `BluetoothSerial` not available | Wrong board selected (S2/S3/C3 have no Bluetooth Classic) | Tools → Board → **ESP32 Dev Module**, or `USE_BLUETOOTH 0` |
| `Sketch too big` | Too many features for the default partition | DEV-03 |
| `Failed to connect to ESP32: Timed out waiting for packet header` | Board not in download mode | Hold **BOOT**, click Upload, release when "Connecting…" appears |
| No COM port | Missing USB driver (CP210x or CH340, depending on the board) or a charge-only cable | Install the driver; try another cable |
| `error: static assertion failed: IMU_UP_AXIS and IMU_FWD_AXIS cannot be the same axis` | Mounting settings in `config.h` §5 are wrong | Re-do TUNING_GUIDE step 2 |
| Compiles but other `.h` files "not found" | Folder name ≠ sketch name | The folder must be called `BalanceX` and contain `BalanceX.ino` |

### At start-up / sensor

| Symptom | Likely cause | Fix |
|---|---|---|
| `MPU6050 not found at 0x68` repeating | SDA/SCL swapped, no 3V3/GND, AD0 floating, bad breadboard contact | Check wires 23–28 in CONNECTIONS.md; SDA = 21, SCL = 22 |
| `ERROR: MPU6050 not answering` while running, LED blinking fast | Loose wire or motor noise on the I2C wires | Shorter, twisted I2C wires away from motor cables; `I2C_CLOCK_HZ = 100000` |
| `Gyro calibration ... robot moved` repeating | Robot was moved / vibrating during start-up | Lay it still; retry with `calib` |
| Angle drifts slowly while the robot is still | Calibration done while moving | `calib` while perfectly still |
| Angle jumps when wheels kick | Accelerometer trusted too much | Raise `alpha` (0.995), or `MPU_DLPF_CFG` 4 |
| Garbage characters in Serial Monitor | Wrong baud | 115200 |
| Commands do nothing | Serial Monitor line ending is "No line ending" | Set to "Newline" |

### Motors and power

| Symptom | Likely cause | Fix |
|---|---|---|
| A motor always runs at full speed (no speed control) | ENA/ENB jumper still fitted: the enable pin is tied to 5 V and also fights the ESP32 pin | Power off, remove the ENA/ENB jumpers (CONNECTIONS.md §6.2) |
| A motor never turns, even in `motor` test | ENA/ENB wire not connected, motor wire loose, no battery voltage at the L298N | Check wires 13–22; measure the L298N supply with a multimeter |
| Motor turns only one way | One IN pin not connected | Check IN1–IN4 wiring |
| Wheel turns the wrong way in `motor` test | Motor wired the other way round (normal for mirror-mounted motors) | `LEFT_/RIGHT_MOTOR_INVERT = true` |
| ESP32 restarts when motors start, boot shows `last reset: BROWNOUT` | Supply dip: LM2596 input too low, weak/empty battery, thin ground wire | Charge the battery; check LM2596 = 5.00 V under load; thick common ground |
| L298N very hot | Normal-ish for L298N (it wastes 2–4 V as heat), worse at high PWM frequency | Heatsink; try `PWM_FREQ_HZ` 5000; consider a MOSFET driver (README) |
| Wheels feel weak at low speed | L298N + 20 kHz PWM | Try `PWM_FREQ_HZ` 5000 or 1000, then re-measure `minpwm` |

### Balancing

| Symptom | Likely cause | Fix |
|---|---|---|
| Robot drives **away** from the fall and slams down immediately | Sign error: sensor or motor direction | Stop. Re-do TUNING_GUIDE steps 2, 3 and 5 |
| Falls slowly, feels "lazy" | KP too low, or `minpwm` too low | Raise KP ×1.5; measure `minpwm` |
| Fast shaking / buzzing | KD too high, or KP far too high | Lower KD ×0.7 |
| Rocks back and forth, getting bigger | KP too high or KD too low | Lower KP or raise KD |
| Balances but rolls away in one direction | Setpoint off, KI or SPEED_KP too low | `zero` again; raise KI and/or `skp` |
| Large slow rocking over a long distance | SPEED_KP or KI too high | Lower `skp`, then KI |
| Turns in a circle | Motors not equal | Lower the stronger side's `*_MOTOR_GAIN` |
| Balanced worse after adding the 2 kg weight | Balance point and inertia changed | `zero` again, raise all gains, see TUNING_GUIDE step 8 |
| Behaviour changes as the battery runs down | PWM gives less voltage with a lower battery | Tune on a charged battery; future: battery-voltage compensation |

---

## D. Hardware testing log — fill this in

Copy the template for every problem. Short notes are fine; the **evidence** (numbers, a Serial Plotter screenshot, a phone video) is what makes the explanation convincing later.

```markdown
### HW-01 — <short title>

| | |
|---|---|
| **Date / who** | 2026-__-__ / <names> |
| **Stage** | wiring / upload / sensor test / motor test / ramp / first balance / tuning / payload |
| **Symptom** | What we saw, exactly (paste the Serial Monitor line if there is one) |
| **Root cause** | Why it happened (once we knew) |
| **How we found it** | Steps / measurements that led to the cause (multimeter values, plots, ...) |
| **Fix** | What we changed: wire / config.h value (old → new) / code (file + function) |
| **Verified** | How we know it is fixed |
| **Evidence** | photo / video / screenshot file name |
| **One-line explanation** | How we would explain it in one sentence |
```

### Gain tuning record

Record every gain change during tuning. It shows *how* you reached the final values.

| Date | KP | KI | KD | SPEED_KP | setpoint | minpwm | Payload | What happened |
|---|---|---|---|---|---|---|---|---|
| | 30 | 250 | 1.0 | 0.02 | | | none | (defaults) |
| | | | | | | | | |
