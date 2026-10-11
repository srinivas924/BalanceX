# BalanceX — Bring-up and Tuning Guide

Do the steps **in order**. Steps 2–5 check the signs (which way is "forward" for the sensor and for each motor). If any sign is wrong, the robot drives *away* from a fall instead of under it, and no amount of tuning will fix that.

Write down anything unexpected in [ERROR_LOG.md](ERROR_LOG.md), Part D.

---

## 0. Safety first

- **Wheels off the ground** (robot on a box, axle free) for steps 1–5.
- Keep your hand near the battery switch. `stop` (Serial/Bluetooth) also cuts the motors.
- Battery charged, low-voltage alarm plugged in, fuse fitted. With two motors of 1.4–1.5 A stall current, the worst case is about 3 A for the motors plus a little for the electronics, so a fuse of about 5 A (slow-blow) fits, not 20 A. Your wires and switch must be rated for at least that.
- First balance attempts: on the floor, in an open space, hands around the robot (not touching), and `maxpwm 150` to keep it gentle.
- Mark the **FRONT** of the robot with tape. Every direction in this project is relative to that mark.

## 1. Upload the firmware

1. Install **Arduino IDE 2.x**.
2. **File → Preferences → Additional boards manager URLs**: `https://espressif.github.io/arduino-esp32/package_esp32_index.json`
3. **Tools → Board → Boards Manager** → install **esp32 by Espressif Systems**. Tested with **2.0.17** and **3.3.12**; any 2.x or 3.x should work.
4. Open `firmware/BalanceX/BalanceX.ino`. The other `.h` files open as tabs automatically.
5. **Tools → Board → esp32 → ESP32 Dev Module**, choose the COM port, then **Upload**. If it says `Timed out waiting for packet header`, hold the **BOOT** button while it says "Connecting…".
6. **Serial Monitor**: **115200 baud**, line ending **"Newline"**.

You should see something like:

```
=== BalanceX self-balancing robot ===
ESP32 Arduino core 3.3.12 | last reset: power on
Bluetooth ready: pair your phone with "BalanceX"
IMU mounting: up = +Z, front = +X  ->  tilt rate from gyro +Y
MPU6050 found (WHO_AM_I = 0x68)
Gyro calibration 1/5: keep the robot completely STILL...
  done. offset X+0.85 Y-1.20 Z+0.33 deg/s (noise 0.40)
```

If the MPU6050 is not found, see [ERROR_LOG.md](ERROR_LOG.md) Part C.

> **Bluetooth (optional, Android):** pair with "BalanceX", then use the app *Serial Bluetooth Terminal*. All commands work the same as over USB, and no cable pulls on the robot while it balances.

## 2. Sensor mounting  →  `config.h` section 5

Type `sensor`. Ten lines per second appear:

```
acc g X+0.02 Y-0.01 Z+0.99 | gyro dps X  +0.1 Y  -0.2 Z  +0.0 | gravity: +Z | accAngle   +1.2  rate   -0.2  ANGLE   +1.1
```

1. **Stand the robot upright.** Read `gravity:`, e.g. `+Z` → `IMU_UP_AXIS = 2`, `IMU_UP_SIGN = +1`. (X = 0, Y = 1, Z = 2. A `-` means sign −1.) Your board is mounted flat, so expect `+Z` (chip side up) or `-Z` (board upside down).
2. **Lay the robot on its BACK** (front mark facing the ceiling). Read `gravity:` again, e.g. `+X` → `IMU_FWD_AXIS = 0`, `IMU_FWD_SIGN = +1`. For a flat board this will be X or Y: the printed arrow that points toward the front (`+`) or the back (`-`).
3. Put those four values in `config.h` and upload again.
4. Type `sensor` again and check:
   - Upright: `ANGLE` is near 0 (within a few degrees).
   - Tip toward the FRONT: `ANGLE` goes **positive**. Tip back: **negative**.
   - Tip it about 20° and back a few times. You should see `gyro and accelerometer agree ... OK`. A `!!` line means an axis or sign is wrong: repeat this step.

The gyro axis is calculated from these two automatically, so you never set it by hand.

## 3. Motor directions  →  `LEFT_/RIGHT_MOTOR_INVERT`

**Wheels off the ground.** Type `motor`. The robot announces each step:

```
step 1/9: LEFT wheel  FORWARD  (should roll the robot toward the FRONT mark)
```

For every "FORWARD" step, the wheel must turn so that the robot *would* roll toward the front mark. "LEFT" is the robot's left when you stand behind it, facing the front mark.

- Wrong direction → set that side's `*_MOTOR_INVERT = true` (or, power OFF, swap its two motor wires at the L298N).
- The wrong wheel moves (right instead of left) → the motor cables are swapped between OUT1/2 and OUT3/4.

Upload and repeat until all steps are correct.

## 4. Motor dead-band  →  `MOTOR_MIN_PWM`

**Wheels off the ground.** Type `ramp`. The PWM rises by 5 every 0.4 s:

```
  ramp pwm = 35
  ramp pwm = 40     <- left wheel starts turning
  ramp pwm = 45
  ramp pwm = 50     <- both wheels turning steadily
```

Type `stop` once **both** wheels turn steadily, and use that number: `minpwm 50` now, and `MOTOR_MIN_PWM = 50.0f` in `config.h`. The simulation showed this step matters (ERROR_LOG DEV-06). Re-measure if you change `PWM_FREQ_HZ` or the battery voltage changes a lot.

## 5. Final sign check (most important)

**Wheels still off the ground.** Type `go`. Hold the robot upright and still. After 0.5 s the LED turns solid and the monitor says `Upright - balancing! (motors ON)`.

Now **tilt it slightly toward the front**: both wheels must spin **forward** (toward the front mark), as if driving under the fall. Tilt back: both spin backward.

If they spin the wrong way, **do not continue**. Something in step 2 or 3 is wrong.

Two safety checks while you are here:
- Tilt it past 35°: the motors must stop, with the message `Fell over ...`.
- Type `go` again and lay the robot **on its side**: the LED must keep slow-blinking and the wheels must stay still (ERROR_LOG DEV-09).

## 6. Find the balance point  →  `BALANCE_SETPOINT_DEG`

Type `stop`. Hold the robot by the top and find the angle where it feels weightless (does not want to fall either way). Hold it there and type `zero`. It prints e.g. `setpoint = -2.35 deg`.

## 7. First balance and tuning

Start with the defaults: KP 30, KI 250, KD 1.0, SPEED_KP 0.02. They come from simulating your motors (12 V, 300 rpm, 1.45 A stall) and 70 mm wheels. For the first tries also type `maxpwm 150`.

Type `go`, hold it upright on the floor, and let go gently once it starts.

**Watch it live:** open **Tools → Serial Plotter** (it has its own box for typing commands) and type `plot 2`. You get angle, target, output and the separate P, I and D parts.

**Change one value at a time, in steps of about ×1.5 or ÷1.5:**

| What you see | Change |
|---|---|
| Wheels react, but too weakly; falls slowly | `kp` up |
| Falls without the wheels doing much | `minpwm` up (step 4), then `kp` up |
| Fast buzzing / jitter (many times per second) | `kd` down |
| Rocks back and forth faster and faster, then falls | `kp` down **or** `kd` up |
| Oscillates about once per second and slowly gets worse | `kd` up |
| Stands, but always rolls off in the same direction | `zero` again, then `ki` up, then `skp` up |
| Big, slow rocking over half a metre or more | `skp` down, then `ki` down |
| Turns in a circle | lower the stronger side's `*_MOTOR_GAIN` (config.h) |
| Wheels slam to full power on small leans | `kp` or `maxpwm` down |

**Simulation hint (ERROR_LOG DEV-04 to DEV-11):** with KI and SPEED_KP switched on, a wide range of KP (10–60) and KD (0.25–2) worked. With KI = 0 **and** SPEED_KP = 0, nothing worked; the robot balanced briefly and then ran away. So if it "almost works but runs away", the answer is usually KI / `skp` / `zero`, not KP. A nearly empty battery made it fail more often, so tune on a charged battery.

Once it stands: raise `maxpwm` back to 255, and push it gently to check it recovers.

## 8. Save the values

Type `show`. Copy the printed lines into `config.h`, replacing the old values, and upload. Live changes are **never** saved automatically. This is on purpose: `config.h` always shows the values the robot is really using.

## 9. Adding the 2 kg payload

1. Fix the weight firmly, as low and centred as the design allows. A higher weight is harder to balance and needs more motor torque.
2. Redo **step 6** (`zero`): the balance point moves with the weight.
3. Start from your working gains and raise KP, KI and KD together (×1.5, then ×2). A heavier, taller robot falls more slowly but needs much more force to catch.
4. Watch the L298N temperature and the battery. 2 kg on top is near the limit of an L298N with small gear motors (see the README's "Better ways").
5. Record everything in the gain tuning table in ERROR_LOG.md Part D.

## Command reference

| Command | What it does |
|---|---|
| `help` | List commands |
| `show` | Print all current values in `config.h` format + loop timing |
| `kp` / `ki` / `kd` `<value>` | Set PID gains (no value = print current) |
| `sp <deg>` | Set the balance setpoint |
| `zero` | Setpoint = current angle |
| `alpha <value>` | Complementary filter (0.95–0.999) |
| `minpwm` / `maxpwm` `<value>` | Motor dead-band / output limit |
| `skp <value>` | Stage-2 anti-drift gain |
| `plot 0/1/2` | Telemetry off / basic / with P, I, D parts |
| `go` | Balance mode (starts when held upright) |
| `stop` | Motors off |
| `sensor` | Sensor test (motors off) |
| `motor` | Motor direction test (wheels off the ground) |
| `ramp` | Dead-band finder (wheels off the ground) |
| `calib` | Re-measure gyro offset (robot still) |

**Status LED (GPIO 2):** solid = balancing · slow blink = waiting to be stood up · fast blink = sensor error · short flash every 2 s = stopped / test mode.
