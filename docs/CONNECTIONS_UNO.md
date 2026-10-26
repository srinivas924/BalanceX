# Self-Balancing Robot — Connection Guide (Arduino Uno R3)

This guide matches `firmware/BalanceX_Uno`. (The older ESP32 wiring is in [CONNECTIONS.md](CONNECTIONS.md).)

## 1. Arduino Uno R3 pin connections

### MPU6050 — tilt sensor

| MPU6050 pin | Arduino Uno R3 |
|---|---|
| VCC | 3.3V |
| GND | GND |
| SDA | A4 |
| SCL | A5 |

Mount the MPU6050 firmly and keep its orientation consistent with the angle calculation in the code (pitch from accelerometer X and Z, rate from gyro Y).

- Mount it on something stiff, near the middle of the robot, not on a plate that the motors shake. Double-sided foam tape is fine; a loose board makes the angle reading vibrate.
- Keep the four I2C wires short and away from the motor wires. The firmware reads the sensor at 400 kHz; if you see `IMU read failed`, set `I2C_CLOCK_HZ = 100000` in `config.h`.

### L298N — motor driver

| L298N pin | Arduino Uno R3 |
|---|---|
| ENA | D5 (PWM) |
| IN1 | D7 |
| IN2 | D8 |
| ENB | D6 (PWM) |
| IN3 | D9 |
| IN4 | D10 |

- Remove the ENA and ENB jumper caps because D5 and D6 control motor speed.
- L298N OUT1/OUT2 connect to the **left motor**.
- L298N OUT3/OUT4 connect to the **right motor**.

### Wheel encoders

| Encoder signal | Arduino Uno R3 |
|---|---|
| Left encoder A | D2 |
| Left encoder B | D4 |
| Right encoder A | D3 |
| Right encoder B | D12 |
| Encoder VCC | 5V, if the encoder is rated for 5V |
| Encoder GND | GND |

The A signals use the Uno's external-interrupt pins. Keep each encoder attached to the same motor identified above.

The firmware reads these four pins directly from the port registers (D2/D3/D4 = port D, D12 = port B), which is much faster than `digitalRead()`. If you ever move an encoder wire, update the two interrupt functions in `BalanceX_Uno.ino` as well. The sketch refuses to compile if `config.h` and the interrupt code disagree.

## 2. Battery, fuse, main switch and buck converter

The robot uses a 3S LiPo battery (11.1V nominal; 12.6V fully charged), an inline fuse, a main switch, and a buck converter.

### Suggested power distribution

```text
                    3S LiPo Battery
                     +           -
                     |           |
              Inline Fuse        +-------------------- Common Ground
                     |
                 Main Switch
                     |
                 Fused +V
                     |
           +---------+------------------+
           |                            |
      L298N motor supply          Buck converter IN+
                                        Buck IN- ---- Common Ground

      Buck converter OUT+ (regulated 5V) ---> Arduino 5V*
      Buck converter OUT- -------------------> Common Ground

      MPU6050 GND, encoder GND, Arduino GND and L298N GND
      must share a common ground reference.
```

- Put the fuse in series with the battery positive lead, close to the battery.
- Connect the switched battery-positive supply to the L298N motor-supply input and buck-converter input, following the board labels.
- Measure and verify the buck converter's output before connecting electronics.
- The buck converter's regulated 5V may power the Uno's 5V pin only when USB power is disconnected, to avoid competing power sources.
- Never connect the LiPo directly to the Uno's 5V pin.
- Choose a fuse appropriate for the wiring, battery and motor stall current.

**Important:** L298N boards vary. If the board has a `5V-EN` regulator jumper, confirm the board's documentation and jumper arrangement before connecting the buck converter's 5V output to the L298N 5V terminal. Do not connect two 5V supplies together.

**Tuning with the USB cable plugged in:** disconnect the buck converter's 5V wire from the Uno first (the Uno then runs from USB, the motors still from the battery; keep the common ground). Reconnect it, with USB unplugged, for running without the cable.

## 3. Before powering on

1. Keep the wheels raised for initial tests.
2. Verify the buck converter output with a multimeter before connecting the Uno.
3. Check motor direction and encoder signs with the `t` and `i` tests ([UNO_GUIDE.md](UNO_GUIDE.md), step 3).
4. The sketch automatically arms after calibration when the robot is upright and still; keep the main switch accessible. `s` in the Serial Monitor stops the motors.
5. Disconnect motor power immediately if the robot shakes violently or commands full PWM.

## 4. Quick pin summary

| Function | Pin(s) |
|---|---|
| MPU6050 I2C | SDA A4, SCL A5 |
| Left encoder | A D2, B D4 |
| Right encoder | A D3, B D12 |
| L298N left motor control | ENA D5, IN1 D7, IN2 D8 |
| L298N right motor control | ENB D6, IN3 D9, IN4 D10 |
| Battery | 3S LiPo, 11.1V nominal |
| Logic supply | Regulated 5V from buck converter, with safe power-source arrangement |
