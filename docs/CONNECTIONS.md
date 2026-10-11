# Self-Balancing Robot — Complete Connection List

This document lists the wiring connections for **one robot**. Build the second robot independently with its own components and battery.

> **ESP32 assumption:** Classic ESP32-WROOM-32 DevKit. Other ESP32 variants may use a different pinout.

## 1. Battery and Power Connections

| No. | From | To |
|---:|---|---|
| 1 | 3S LiPo battery positive (+) | XT60 positive (+) |
| 2 | XT60 positive wire | ON/OFF switch terminal 1 |
| 3 | Switch terminal 2 | Fuse terminal 1 |
| 4 | Fuse terminal 2 | LM2596 IN+ |
| 5 | Fuse terminal 2 | L298N VIN / motor supply input |
| 6 | 3S LiPo battery negative (-) | XT60 negative (-) |
| 7 | XT60 negative wire | Common GND |
| 8 | Common GND | LM2596 IN- |
| 9 | Common GND | LM2596 OUT- |
| 10 | Common GND | L298N GND |
| 11 | LM2596 OUT+ (adjusted to 5.00 V) | ESP32 VIN / 5V |
| 12 | Common GND | ESP32 GND |

## 2. ESP32 to L298N Connections

| No. | ESP32 pin | L298N pin |
|---:|---|---|
| 13 | GPIO 25 | ENA |
| 14 | GPIO 26 | IN1 |
| 15 | GPIO 27 | IN2 |
| 16 | GPIO 14 | ENB |
| 17 | GPIO 18 | IN3 |
| 18 | GPIO 19 | IN4 |

## 3. L298N to Motors

| No. | L298N output | Connect to |
|---:|---|---|
| 19 | OUT1 | Left motor terminal 1 |
| 20 | OUT2 | Left motor terminal 2 |
| 21 | OUT3 | Right motor terminal 1 |
| 22 | OUT4 | Right motor terminal 2 |

If a motor rotates in the wrong direction during testing, disconnect power before swapping its two output wires or changing the direction logic in code.

## 4. ESP32 to MPU6050 Connections

| No. | MPU6050 pin | ESP32 connection |
|---:|---|---|
| 23 | VCC | 3V3 |
| 24 | GND | Common GND |
| 25 | SDA | GPIO 21 |
| 26 | SCL | GPIO 22 |
| 27 | INT | GPIO 23 (optional) |
| 28 | AD0 | GND (I2C address 0x68) |
| 29 | XDA | Leave unconnected |
| 30 | XCL | Leave unconnected |

Use a breakout compatible with 3.3 V logic. I2C pull-up resistors must pull up to 3.3 V, not 5 V.

## 5. Other Components

| No. | Component | Connection |
|---:|---|---|
| 31 | LiPo low-voltage alarm | Matching battery balance connector |
| 32 | XT60 connectors | Main battery power leads |
| 33 | Breadboard | Low-current signal connections only |
| 34 | 100 nF capacitor (recommended) | Across each motor's terminals, close to the motor |

## 6. Important Setup and Safety Checks

1. **LM2596:** Before connecting the ESP32, adjust the buck converter output to 5.00 V and verify it with a multimeter.
2. **L298N ENA/ENB:** Remove the ENA and ENB jumpers when using ESP32 PWM speed control.
3. **L298N 5 V pin:** Follow the exact module's regulator/jumper arrangement. Do not connect the buck output to this pin blindly.
4. **Common ground:** ESP32, MPU6050, LM2596 and L298N must share a common ground.
5. **Battery voltage:** A 3S LiPo is 11.1 V nominal and 12.6 V when fully charged.
6. **Fuse rating:** Do not assume a 20 A fuse is suitable. Check the battery, any BMS/protection, wiring, switch, driver and motor stall-current ratings. The fuse does not replace a BMS.
7. **Battery protection:** Confirm whether suitable battery protection is present. The listed components do not explicitly include a BMS.
8. **Breadboard:** Do not route battery or motor current through the breadboard.
9. **Initial test:** Keep the wheels lifted during initial motor tests. Verify sensor readings and motor directions before attempting to balance.
10. **Two robots:** Build and power each robot independently. Do not connect the two robots' battery outputs together.

## 7. Power Path Summary

- **Positive:** Battery (+) → XT60 (+) → ON/OFF switch → fuse → branch to LM2596 IN+ and L298N motor supply.
- **Negative:** Battery (-) → XT60 (-) → common GND → LM2596 IN-, LM2596 OUT-, L298N GND and ESP32 GND.
- **Logic supply:** LM2596 OUT+ (verified 5.00 V) → ESP32 VIN/5V.
- **Sensor supply/signals:** ESP32 3V3, GPIO 21 and GPIO 22 → MPU6050 VCC, SDA and SCL.

**Note:** The L298N module's motor supply terminal may be labelled `VS`, `12V`, or `VIN`; verify the label on your exact board. Confirm its 5 V regulator jumper configuration before applying power.
