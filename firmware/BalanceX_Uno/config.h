// =====================================================================================
//  config.h  -  every pin, calibration value and tuning knob for the Arduino Uno robot.
// =====================================================================================
//
//  Labels used below
//    [FIXED]  matches docs/CONNECTIONS_UNO.md. Change only if you re-wire the robot.
//    [YOURS]  your own calibration results, copied unchanged from your previous sketch.
//    [TUNE]   a starting value. Expect to change it while watching the robot.
//
//  "(live)" values can also be changed from the Serial Monitor without re-flashing
//  (type  ?  for the list). Live changes are lost on reset: when the robot behaves well,
//  type  c  and copy the printed lines back into this file.
//
//  Directions used everywhere in this sketch:
//    FORWARD  = the direction in which BOTH normalized encoder counts increase.
//    pitch    = lean angle, POSITIVE = leaning FORWARD.
//    command  = motor command, POSITIVE = drive FORWARD.
//  Your sign settings in section 2 already produce these directions; the  t  (motor/
//  encoder) and  i  (IMU) tests in docs/UNO_GUIDE.md confirm them on the real robot.
// =====================================================================================
#pragma once
#include <stdint.h>

// -------------------------------------------------------------------------------------
// 1. PINS  [FIXED]  - Arduino Uno R3
// -------------------------------------------------------------------------------------
// L298N channel A -> LEFT motor (OUT1/OUT2). Remove the ENA jumper.
constexpr uint8_t PIN_LEFT_EN   = 5;    // ENA, PWM
constexpr uint8_t PIN_LEFT_IN1  = 7;    // IN1
constexpr uint8_t PIN_LEFT_IN2  = 8;    // IN2
// L298N channel B -> RIGHT motor (OUT3/OUT4). Remove the ENB jumper.
constexpr uint8_t PIN_RIGHT_EN  = 6;    // ENB, PWM
constexpr uint8_t PIN_RIGHT_IN1 = 9;    // IN3
constexpr uint8_t PIN_RIGHT_IN2 = 10;   // IN4

// Wheel encoders. The A signals must stay on D2 and D3 (the Uno's two external interrupts).
// The interrupt code reads these pins straight from the port registers for speed:
// D2 = PD2, D3 = PD3, D4 = PD4, D12 = PB4. Re-wiring them means editing the two ISRs too.
constexpr uint8_t PIN_LEFT_ENC_A  = 2;
constexpr uint8_t PIN_LEFT_ENC_B  = 4;
constexpr uint8_t PIN_RIGHT_ENC_A = 3;
constexpr uint8_t PIN_RIGHT_ENC_B = 12;

// MPU6050 on SDA = A4, SCL = A5. AD0 unconnected or to GND -> address 0x68.
constexpr uint8_t MPU_I2C_ADDR = 0x68;

// -------------------------------------------------------------------------------------
// 2. YOUR CALIBRATION  [YOURS]  - unchanged from your previous sketch
// -------------------------------------------------------------------------------------
// Encoder counts per wheel revolution (CHANGE interrupt on the A channel).
constexpr float LEFT_COUNTS_PER_WHEEL_REV  = 1325.1f;
constexpr float RIGHT_COUNTS_PER_WHEEL_REV = 1325.1f;

// Encoder signs: forward wheel movement -> positive count on BOTH wheels.
constexpr int8_t LEFT_ENCODER_SIGN  = +1;
constexpr int8_t RIGHT_ENCODER_SIGN = -1;

// Motor signs: positive command -> wheel turns forward.
constexpr int8_t LEFT_MOTOR_SIGN  = +1;
constexpr int8_t RIGHT_MOTOR_SIGN = -1;

// IMU signs. Angle = ANGLE_SIGN * atan2(accelX, accelZ), rate = GYRO_SIGN * gyroY,
// controller output = BALANCE_SIGN * (KP * angle + KD * rate), exactly as before.
// This sketch folds BALANCE_SIGN into the angle and rate (PITCH_SIGN / RATE_SIGN below),
// so that inside the controller "positive pitch" always means "leaning forward".
constexpr float ANGLE_SIGN   = +1.0f;
constexpr float GYRO_SIGN    = -1.0f;
constexpr float BALANCE_SIGN = -1.0f;
constexpr float PITCH_SIGN   = BALANCE_SIGN * ANGLE_SIGN;   // derived, do not edit
constexpr float RATE_SIGN    = BALANCE_SIGN * GYRO_SIGN;    // derived, do not edit

// Lowest PWM at which each wheel turns (wheels lifted). The right motor needs more.
// These no longer switch the motor straight to 80/100: see FRICTION_COMP_PERCENT.
constexpr float LEFT_MIN_PWM  = 80.0f;
constexpr float RIGHT_MIN_PWM = 100.0f;

// -------------------------------------------------------------------------------------
// 3. TIMING
// -------------------------------------------------------------------------------------
constexpr uint32_t CONTROL_PERIOD_US   = 5000;   // 200 Hz control loop
constexpr uint32_t TELEMETRY_PERIOD_MS = 50;     // one telemetry line every 50 ms (20 Hz)
constexpr bool     TELEMETRY_AT_BOOT   = true;   // (live: m) on/off
constexpr uint32_t SERIAL_BAUD         = 115200;

// -------------------------------------------------------------------------------------
// 4. MPU6050
// -------------------------------------------------------------------------------------
// 400 kHz I2C reads the sensor in about 0.4 ms (100 kHz: about 1.6 ms). If you ever see
// "IMU read failed", use 100000: long or noisy I2C wires do not like 400 kHz.
constexpr uint32_t I2C_CLOCK_HZ     = 400000;
constexpr uint32_t I2C_TIMEOUT_US   = 3000;     // a stuck I2C bus gives up after 3 ms
                                                // (instead of freezing with the motors on)

// Digital low-pass filter inside the MPU6050 (register CONFIG):
//   value :  1      2      3      4      5
//   accel :  184    94     44     21     10   Hz
//   delay :  2.0    3.0    4.9    8.5    13.8 ms
// 3 = 44 Hz, the same as your previous MPU6050_BAND_44_HZ. Higher numbers filter more
// motor vibration but add delay, and delay is what makes a balancing robot sluggish.
constexpr uint8_t MPU_DLPF_CFG = 3;            // [TUNE] 2 .. 4

// Start-up calibration: the robot must be upright, at its balance point, and still.
// It measures the gyro offset and the "zero" angle, like your previous sketch.
constexpr uint32_t CALIBRATION_MS        = 3000;
constexpr float    CALIB_MAX_GYRO_SPREAD = 10.0f;  // deg/s; more than this = robot moved, repeat

// Complementary filter: angle = ALPHA * (angle + rate * dt) + (1 - ALPHA) * accelAngle.
// 0.99 at 200 Hz = 0.5 s time constant. Your previous value was 0.985.
constexpr float COMP_FILTER_ALPHA = 0.99f;     // [TUNE] (live: a) 0.98 .. 0.995

// Extra low-pass filter on the gyro rate used by the KD term only (0 = off).
// The angle itself always integrates the unfiltered rate, so this adds no angle lag.
constexpr float RATE_FILTER_HZ = 40.0f;        // [TUNE] 20 .. 60, or 0

// -------------------------------------------------------------------------------------
// 5. BALANCE (inner loop)  -  command = KP * (pitch - target) + KD * pitchRate
// -------------------------------------------------------------------------------------
// Units are PWM counts directly (no hidden scaling): KP = PWM per degree,
// KD = PWM per deg/s. Your previous KP 8 / KD 0.75 went through a 2.6x mapping and
// equal about KP 21 / KD 2.0 here, plus a jump to PWM 80/100 at 0.25 deg.
// Defaults chosen with tests/uno_sim_test.cpp: they balanced and drove straight in all 64
// simulated variants of this robot (weight, height, motor, gearbox play, battery, zero error).
//   wobbles back and forth about twice a second -> KD up (or KP down)
//   wheels stick / it sags before reacting       -> KP up
constexpr float KP = 50.0f;                    // [TUNE] (live: p) 35 .. 65
constexpr float KD = 1.6f;                     // [TUNE] (live: d) 1.2 .. 2.2

// -------------------------------------------------------------------------------------
// 6. MOTOR OUTPUT
// -------------------------------------------------------------------------------------
// Friction compensation. Your measured LEFT/RIGHT_MIN_PWM is the PWM needed to START a
// wheel turning; a turning wheel needs less. So this share of it is added to each motor:
// in the direction the motor is already turning (from its encoder), or, when it is still,
// in the commanded direction, blended in over the first FRICTION_RAMP_PWM of command.
// Your previous sketch switched from 0 straight to PWM 80/100 whenever the lean passed
// 0.25 deg, back and forth; that on/off jump was a main cause of the fast vibration.
//   wheels still buzz around upright -> lower the percent
//   robot rocks slowly, wheels "stick" near upright -> raise the percent
constexpr float FRICTION_COMP_PERCENT = 70.0f; // [TUNE] (live: k) 40 .. 90
constexpr float FRICTION_RAMP_PWM     = 10.0f; // [TUNE] 5 .. 20
constexpr float FRICTION_SPEED_RPM    = 5.0f;  // above this encoder speed a motor counts as turning
constexpr float MOTOR_ZERO_BAND       = 0.5f;  // smaller commands = motor off

// Largest PWM the controller may use. 255 is full battery voltage; your previous 180
// limited how hard the robot could catch itself. Lower it for gentle first tests.
constexpr float MAX_PWM = 255.0f;              // [TUNE] (live: x)

// -------------------------------------------------------------------------------------
// 7. FORWARD DRIVING (outer loop)  -  uses the encoders
// -------------------------------------------------------------------------------------
// A position reference moves forward at the target speed; the robot follows it.
//   target = POS_KP * (referencePosition - position) + SPEED_KP * (targetRpm - rpm)
// The result is the lean angle (deg) handed to the balance loop: to go faster the robot
// leans forward, to slow down it leans back. The position term also cancels a slightly
// wrong zero angle, so the robot does not roll away even if calibration was not perfect.
// Position and speed are over the ground: the encoders (which turn with the body) plus
// the body's own tilt, see groundRev() in balance.h.
//
// In simulation SPEED_KP mattered most: 0.06 failed often, 0.08 .. 0.11 worked. The
// speed loop's real strength is KP x SPEED_KP (PWM per RPM), so if you lower KP a lot,
// raise SPEED_KP a little.
//
// 1 wheel RPM = 0.37 cm/s with 70 mm wheels (wheel circumference x RPM / 60).
constexpr float FORWARD_SPEED_RPM     = 10.0f; // [TUNE] (live: f) 0 = just balance in place
constexpr float SPEED_RAMP_RPM_PER_S  = 4.0f;  // how quickly that speed is reached
constexpr uint32_t DRIVE_DELAY_MS     = 3000;  // balance in place this long before driving
constexpr float MAX_SPEED_RPM         = 40.0f; // limit for the live  f  command

constexpr float SPEED_KP = 0.09f;              // [TUNE] (live: v) deg per RPM, 0.08 .. 0.11
constexpr float POS_KP   = 3.0f;               // [TUNE] (live: o) deg per wheel revolution, 3 .. 4
constexpr float MAX_TARGET_DEG = 6.0f;         // outer loop may lean the robot at most this much
constexpr float POS_ERR_LIMIT_REV = 1.0f;      // pushed further than this: the reference moves
                                               // along instead of dragging the robot back
constexpr float SPEED_FILTER_TAU_S = 0.04f;    // smoothing of the encoder speed

// -------------------------------------------------------------------------------------
// 8. STRAIGHT LINE (heading hold)  -  uses the encoders
// -------------------------------------------------------------------------------------
// Keeps both wheels at the same travelled distance:
//   turn = HEADING_KP * (leftRev - rightRev) + HEADING_KD * (leftRpm - rightRpm)
// left command - turn, right command + turn. This replaces your RPM sync, which only
// matched the speeds and so never corrected a turn that had already happened.
constexpr float HEADING_KP   = 150.0f;         // [TUNE] (live: h) PWM per revolution of difference
constexpr float HEADING_KD   = 0.6f;           // [TUNE] PWM per RPM of difference
constexpr float MAX_TURN_PWM = 40.0f;

// -------------------------------------------------------------------------------------
// 9. SAFETY  (start / fall values are [YOURS])
// -------------------------------------------------------------------------------------
constexpr float    START_ANGLE_DEG  = 5.0f;    // arm when within 5 deg of the zero angle ...
constexpr float    START_RATE_DPS   = 15.0f;   // ... moving slower than 15 deg/s ...
constexpr uint32_t START_HOLD_MS    = 800;     // ... for 0.8 s
constexpr float    FALL_ANGLE_DEG   = 25.0f;   // motors off beyond this lean

// Direction check while balancing: a wheel turning against its own command for this long
// means a motor or encoder sign is wrong. Motors stop instead of the robot racing off.
constexpr float    DIR_FAULT_MIN_CMD = 80.0f;  // PWM (filtered command)
constexpr float    DIR_FAULT_MIN_RPM = 15.0f;  // RPM in the opposite direction
constexpr uint32_t DIR_FAULT_MS      = 500;

// Motor / encoder test (command  t , WHEELS OFF THE GROUND)
constexpr float    MOTOR_TEST_PWM = 140.0f;
constexpr uint32_t MOTOR_TEST_MS  = 700;
