# BalanceX on the Arduino Uno — Bring-up and Tuning

Firmware: `firmware/BalanceX_Uno/` · Wiring: [CONNECTIONS_UNO.md](CONNECTIONS_UNO.md) · Why it changed: [ERROR_LOG.md](ERROR_LOG.md) DEV-12 to DEV-17

Do the steps **in order**. Step 3 checks the signs. If one is wrong, the robot drives *away* from a fall, and no tuning can fix that.

## 0. Safety

- **Wheels off the ground** (robot on a box) for steps 1–3.
- Keep a hand near the main switch. `s` in the Serial Monitor stops the motors at once.
- First balance: on the floor, open space, hands around the robot without touching it.

## 1. Upload

1. Arduino IDE → open `firmware/BalanceX_Uno/BalanceX_Uno.ino`. `config.h` and `balance.h` open as tabs. **No libraries to install**: the MPU6050 is read directly, so the Adafruit libraries are no longer needed.
2. **Tools → Board → Arduino Uno**, choose the port, **Upload**.
3. **Serial Monitor at 115200 baud.** Any line ending works.

At start-up the robot calibrates for 3 s. **Hold it upright, at its balance point, and still** (same as your previous sketch):

```
=== BalanceX - Arduino Uno ===
MPU6050 ready.
Calibrating 1/5: hold the robot UPRIGHT at its balance point and STILL...
  done: zero angle -1.84 deg, gyro offset -0.712 deg/s, spread 1.2
```

If it says `the robot moved - again`, it repeats (up to 5 times; the last try is used anyway, with a warning). Small hand tremor is fine: it is averaged out.

## 2. What it does

1. Calibrates (3 s).
2. Starts balancing once it has been held within 5° of upright, still, for 0.8 s. Message `BALANCING (motors on). Holding position.`
3. Holds its position for 3 s.
4. Ramps up to `FORWARD_SPEED_RPM` (10 RPM ≈ 3.7 cm/s with 70 mm wheels) and drives forward in a straight line. Message `Driving forward.`
5. Past 25° it switches the motors off (`FELL`). Stand it up and hold it still to restart.

Commands (send `?` for the list):

| Command | What it does |
|---|---|
| `s` | **Stop** (motors off). `g` starts again |
| `t` | Motor/encoder test (**wheels off the ground**) |
| `i` | Sensor view: angle, rate, encoder counts (motors off). `i` again to leave |
| `m` | Telemetry on/off (for **Tools → Serial Plotter**) |
| `z` | Current angle = balance point (hold it there, still) |
| `r` | Recalibrate gyro offset and zero angle |
| `c` | Show current values in `config.h` format + loop timing |
| `p` `d` `v` `o` `h` `f` `k` `x` `a` + value | Set KP, KD, SPEED_KP, POS_KP, HEADING_KP, forward RPM, friction %, MAX_PWM, filter alpha. Without a value: print it |

Live changes are lost at reset. When the robot behaves well, send `s`, then `c`, and paste the printed lines into `config.h`.

## 3. Sign checks (wheels off the ground)

Your sign values from the previous sketch are already in `config.h`. These three checks confirm them. Each one tests one link in the chain.

**3a. Encoders: `i`**, then turn each wheel by hand in the robot's FORWARD direction. `L` (left) and `R` (right) must **count up**. If one counts down, flip that wheel's `*_ENCODER_SIGN`. This defines "forward" for everything else.

**3b. Motors: `t`.** Each wheel turns briefly on its own. A good result looks like this (your numbers will differ):

```
LEFT : own encoder 1730, other 2, 148 RPM
   OK: positive command -> encoder counts up
RIGHT: own encoder 1702, other -1, 145 RPM
   OK: positive command -> encoder counts up
```

- `WRONG WAY` on one wheel: flip that wheel's `*_MOTOR_SIGN`.
- `WRONG WAY` on **both** wheels: flip **both** `*_MOTOR_SIGN`s **and** `BALANCE_SIGN` (together they keep your old, working balance direction).
- `the OTHER encoder moved`: motor or encoder cables are swapped between left and right.

Note the RPM it reports: that is your motor speed at PWM 140, useful for the ERROR_LOG.

**3c. Angle: `i`**, robot upright, then tip it toward the FRONT (the direction the wheels rolled in 3a). `accel` (accelerometer angle) must go **positive**, and `rate` must be positive **while** it is tipping forward.

| `accel` | `rate` | Fix |
|---|---|---|
| + | + | nothing, correct |
| − | − | flip `BALANCE_SIGN` |
| + | − | flip `GYRO_SIGN` |
| − | + | flip `ANGLE_SIGN` |

With all signs right, `pitch` (the filtered angle the controller uses) follows `accel` smoothly.

**3d. Together:** `f 0`, then `g`, hold the robot upright (wheels still off the ground) until it says `BALANCING`. Tilt it slightly forward: both wheels spin **forward**. Tilt back: backward. Then tilt past 25°: motors stop with `FELL`. (In the air the wheels may also speed up on their own after a while, because the robot thinks it is rolling away. That is normal: send `s`.)

Safety net: if a wheel keeps turning against its own command while balancing, the firmware stops with `DIRECTION FAULT` instead of racing off.

## 4. First balance

1. `f 0` (balance in place, no driving) and `g`. Hold it upright on the floor and let go gently after `BALANCING`.
2. **Tools → Serial Plotter** shows `pitch`, `target`, `rpm`, `want`, `turn` and the PWM of each wheel (divided by 10).
3. When it stands calmly: `f 10` (or press reset and let it run with the default).

## 5. Tuning

Change **one** value at a time, in steps of about ×1.3.

| What you see | Change |
|---|---|
| Fast buzzing / jitter, wheels twitching around upright | `k` down (friction compensation, e.g. `k 50`) |
| Slow rocking, wheels "stick" and then jerk near upright | `k` up (e.g. `k 85`) |
| Wobbles back and forth about twice a second, steadily | `d` up (e.g. `d 2.0`), or `p` down |
| Rocks back and forth faster and faster | `p` down or `d` up |
| Reacts weakly, sags before the wheels catch it | `p` up |
| Large slow surges forward and back (1–3 s) | `v` up a little (e.g. `v 0.1`) |
| Creeps away / does not hold its place | `o` up (e.g. `o 4`) |
| Does not reach the forward speed, or the speed hunts | `v` up |
| Curves away from a straight line | `h` up (straight-line loop) |
| Wiggles left-right while driving | `h` down |
| Falls when pushed | `p` up, and check `x` is 255 |

What the simulation found (tests/uno_sim_test.cpp, 64 robot variants per setting):

- **`v` (SPEED_KP) matters most.** 0.06 failed often; 0.08–0.11 worked. What counts is KP × SPEED_KP, so if you lower `p` a lot, raise `v` a little.
- **`o` (POS_KP) 3–4** worked; 2 let many variants drift.
- KP 40–60 with KD 1.3–1.9 all worked with the default `v` and `o`.
- An MPU6050 mounted high (about 18 cm above the axle) made the robot wobble about twice a second with the default gains; KD 1.9–2.0 fixed it. Mount the sensor low and rigid if you can.

Gearbox play makes a big difference: wiggle each wheel by hand. A lot of play usually means a lower `k` is needed.

## 6. Notes

- **Battery**: tune with a charged battery. As the LiPo drains from 12.6 V towards 11 V the motors get weaker for the same PWM.
- **Zero angle**: if it always leans and creeps one way at the start, recalibrate (`r`) holding it more exactly at the balance point, or send `z` while holding it there. The position loop also compensates for a slightly wrong zero by itself.
- **USB and the buck converter**: see [CONNECTIONS_UNO.md](CONNECTIONS_UNO.md) — never power the Uno from USB and the buck converter's 5 V at the same time.
- **PC simulation** (optional): `g++ -std=c++17 -O2 -o uno_sim_test tests/uno_sim_test.cpp && ./uno_sim_test` runs this firmware's controller against a physics model of the robot.
