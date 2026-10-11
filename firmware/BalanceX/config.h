// =====================================================================================
//  config.h  -  EVERY number you are expected to change lives in this one file.
// =====================================================================================
//
//  Labels used below
//    [FIXED]  comes from docs/CONNECTIONS.md. Change only if you re-wire the robot.
//    [CHECK]  depends on how YOUR robot is assembled (sensor mounting, motor wiring).
//             Set these correctly FIRST, using the tests in docs/TUNING_GUIDE.md.
//    [TUNE]   a starting guess. You are expected to change it while watching the robot.
//
//  Live tuning: the [TUNE] values marked "(live)" can also be changed over the Serial
//  Monitor or Bluetooth without re-flashing (type  help ). Live changes are lost on reset,
//  so when the robot behaves well, type  show  and copy the printed numbers back here.
//
//  Directions used everywhere in this project:
//    Put a piece of tape on one side of the robot and call it the FRONT.
//    "Forward"  = rolling toward the FRONT mark.
//    "Left"     = the robot's left when you stand behind it, facing the FRONT mark.
//    "Angle"    = how far the robot leans. POSITIVE = leaning toward the FRONT.
//
//  This robot: 12 V geared DC motors, 300 rpm, 1.4-1.5 A stall, no encoders (yet);
//              70 mm x 40 mm wheels; MPU6050 mounted flat; 3S LiPo; L298N driver.
// =====================================================================================
#pragma once
#include <stdint.h>

// -------------------------------------------------------------------------------------
// 1. PINS  [FIXED]  - classic ESP32-WROOM-32 DevKit, see docs/CONNECTIONS.md
// -------------------------------------------------------------------------------------
// Pin numbers are 'int' because that is the type Wire.begin(int sda, int scl, ...) expects.

// L298N channel A -> LEFT motor (OUT1 / OUT2)
constexpr int PIN_LEFT_PWM  = 25;   // ENA  (the ENA jumper on the L298N must be REMOVED)
constexpr int PIN_LEFT_IN1  = 26;   // IN1
constexpr int PIN_LEFT_IN2  = 27;   // IN2

// L298N channel B -> RIGHT motor (OUT3 / OUT4)
constexpr int PIN_RIGHT_PWM = 14;   // ENB  (the ENB jumper on the L298N must be REMOVED)
constexpr int PIN_RIGHT_IN1 = 18;   // IN3
constexpr int PIN_RIGHT_IN2 = 19;   // IN4

// MPU6050 (I2C). GPIO 23 (INT) is wired but not needed: the code reads the sensor
// on a fixed timer instead of waiting for the interrupt.
constexpr int     PIN_I2C_SDA  = 21;
constexpr int     PIN_I2C_SCL  = 22;
constexpr uint8_t MPU_I2C_ADDR = 0x68;   // AD0 -> GND gives 0x68 (AD0 -> 3V3 would give 0x69)

// On-board LED used as a status light (most ESP32 DevKits have a blue LED on GPIO 2).
//   solid = balancing | slow blink = waiting to be stood up | fast blink = sensor error
//   short flash every 2 s = stopped / test mode.     Set to -1 if your board has no LED.
constexpr int PIN_STATUS_LED = 2;

// -------------------------------------------------------------------------------------
// 2. COMMUNICATION
// -------------------------------------------------------------------------------------
constexpr uint32_t SERIAL_BAUD = 115200;   // Serial Monitor / Plotter must use the same speed.

// 1 = also accept commands + send data over Bluetooth Classic, so you can tune the robot
//     from an Android phone ("Serial Bluetooth Terminal" app) with no USB cable pulling on it.
//     iPhones cannot use Bluetooth Classic serial - set 0 if you do not need it.
// 0 = USB only (smaller program, faster upload).
#define USE_BLUETOOTH 1
#define BT_DEVICE_NAME "BalanceX"   // two robots? give each its own name, e.g. "BalanceX-1"

constexpr uint32_t TELEMETRY_PERIOD_MS = 50;   // [TUNE] print angle/output every 50 ms (20 Hz)
constexpr uint8_t  TELEMETRY_AT_BOOT   = 1;    // 0 = off, 1 = angle/target/output, 2 = + P/I/D terms

// -------------------------------------------------------------------------------------
// 3. CONTROL LOOP TIMING
// -------------------------------------------------------------------------------------
// The whole read-sensor -> compute -> drive-motors cycle runs once every LOOP_PERIOD_US.
// 5000 us = 200 Hz. Faster is not automatically better: the motors/gearbox cannot react
// faster than ~50-100 Hz anyway. If you change this, re-check COMP_FILTER_ALPHA.
constexpr uint32_t LOOP_PERIOD_US = 5000;      // [TUNE]

// -------------------------------------------------------------------------------------
// 4. MPU6050 SETTINGS
// -------------------------------------------------------------------------------------
constexpr uint32_t I2C_CLOCK_HZ   = 400000;    // 400 kHz. Use 100000 if you see I2C errors
                                               // (long wires, wires running next to motor cables).
constexpr uint16_t I2C_TIMEOUT_MS = 10;        // give up on a stuck I2C read after 10 ms

// Digital Low-Pass Filter inside the MPU6050 (register CONFIG, DLPF_CFG):
//   value : 0     1     2     3     4     5     6
//   gyro  : 256   188   98    42    20    10    5    Hz bandwidth
//   delay : ~0    1.9   2.8   4.8   8.3   13.4  18.6 ms
// More filtering = less motor vibration noise, but more delay (delay makes balancing harder).
constexpr uint8_t MPU_DLPF_CFG = 3;            // [TUNE] 3 (42 Hz) is a good start; try 2 or 4

// Gyro calibration at power-up: the robot must be STILL (lying on its side is fine).
constexpr uint16_t GYRO_CALIB_SAMPLES        = 500;   // ~1 second
constexpr float    GYRO_CALIB_MAX_SPREAD_DPS = 2.0f;  // if readings vary more than this, the robot
                                                      // was moving -> calibration is repeated

// -------------------------------------------------------------------------------------
// 5. SENSOR MOUNTING  [CHECK]   (how to find these: docs/TUNING_GUIDE.md, step 2)
// -------------------------------------------------------------------------------------
// Axis numbers: 0 = X, 1 = Y, 2 = Z (X and Y arrows are printed on the board; Z comes out
// of the chip side). Sign: +1 if the arrow points that way, -1 if it points the opposite way.
//
//   IMU_UP_AXIS / IMU_UP_SIGN   : the axis pointing UP while the robot stands upright.
//        In "sensor" mode, stand the robot upright: the line says e.g. "gravity: +Z".
//   IMU_FWD_AXIS / IMU_FWD_SIGN : the axis pointing to the FRONT mark.
//        In "sensor" mode, lay the robot on its BACK (front mark facing the ceiling):
//        the line now says e.g. "gravity: +X".
//
//   Example A - board flat on the middle plate, chip up, X arrow to the front:
//        UP = 2 (Z), +1      FWD = 0 (X), +1
//   Example B - board standing vertical, chip facing the front, Y arrow up:
//        UP = 1 (Y), +1      FWD = 2 (Z), +1
//
// THIS ROBOT: the MPU6050 is mounted FLAT, so UP is the Z axis (+1 if the chip side faces
// up, -1 if the board is upside down). FWD is X or Y, depending on which printed arrow
// points to the front mark - confirm both with "sensor" mode before the first balance.
//
// The gyro axis (the one parallel to the wheel axle) and its sign are worked out
// automatically from these two, so they can never disagree with each other.
constexpr int IMU_UP_AXIS  = 2;   // [CHECK]
constexpr int IMU_UP_SIGN  = +1;  // [CHECK]
constexpr int IMU_FWD_AXIS = 0;   // [CHECK]
constexpr int IMU_FWD_SIGN = +1;  // [CHECK]

// -------------------------------------------------------------------------------------
// 6. ANGLE FILTER  (complementary filter)
// -------------------------------------------------------------------------------------
//   angle = ALPHA * (angle + gyroRate * dt) + (1 - ALPHA) * accelAngle
// ALPHA close to 1 trusts the gyro more (smooth, but slowly drifts);
// the small (1 - ALPHA) part of accelerometer pulls the drift back to the true angle.
// Time constant = ALPHA * dt / (1 - ALPHA):  0.99 at 200 Hz -> ~0.5 s.
//   Angle jumps when the motors kick?  -> raise ALPHA (0.995)
//   Angle slowly wanders / lags?       -> lower ALPHA (0.98)
constexpr float COMP_FILTER_ALPHA = 0.99f;     // [TUNE] (live: alpha) range 0.95 .. 0.999

// -------------------------------------------------------------------------------------
// 7. BALANCE PID  - the main tuning knobs
// -------------------------------------------------------------------------------------
//   error  = angle - target          (degrees; positive = leaning forward)
//   output = KP*error + KI*sum(error*dt) + KD*tiltRate     (PWM counts, -255 .. +255)
//   positive output = both wheels drive FORWARD, i.e. underneath the fall.
//
// Units: KP  = PWM per degree            e.g. 30 -> 2 deg of lean gives 60 PWM
//        KI  = PWM per (degree*second)   e.g. 250 -> 1 deg held for 0.4 s adds 100 PWM
//        KD  = PWM per (degree/second)   (uses the gyro rate directly - no noisy differencing)
//
// Why KI is NOT zero here: a DC motor on PWM behaves like a SPEED control (more PWM ->
// more speed), but catching a fall needs the wheels to keep ACCELERATING. The I term
// keeps raising the PWM while the robot still leans, which gives that acceleration.
// With KI = 0 the robot typically balances for a moment and then runs away and falls.
// (Found with the PC simulation in tests/host_test.cpp - see docs/ERROR_LOG.md.)
//
// Starting values come from simulating THIS robot's motors and wheels (12 V, 300 rpm,
// 1.45 A stall, 70 mm wheels) for robot weights of 1-2 kg and a centre of mass 6-15 cm
// above the axle (ERROR_LOG DEV-11). The real robot will still need tuning: the simulation
// has no gearbox backlash. Symptom -> fix table: docs/TUNING_GUIDE.md, step 7.
constexpr float KP = 30.0f;             // [TUNE] (live: kp) typical range 10 .. 80
constexpr float KI = 250.0f;            // [TUNE] (live: ki) typical range 50 .. 400
constexpr float KD = 1.0f;              // [TUNE] (live: kd) typical range 0.25 .. 3.0
constexpr float I_TERM_LIMIT = 150.0f;  // [TUNE] the I term can never add more than this many PWM
                                        //        (anti wind-up: stops it "storing up" huge values)

// The angle reading at which the robot is actually balanced. It is never exactly 0:
// the sensor is not perfectly level and the weight is not exactly above the axle.
// Easiest way: hold the robot at the point where it feels weightless and type  zero .
// Re-do this whenever you add/move weight (e.g. the 2 kg payload).
constexpr float BALANCE_SETPOINT_DEG = 0.0f;   // [TUNE] (live: sp / zero)

// -------------------------------------------------------------------------------------
// 8. SAFETY
// -------------------------------------------------------------------------------------
constexpr float    FALL_ANGLE_DEG  = 35.0f;  // [TUNE] motors switch OFF beyond this lean
// Lying on its SIDE, gravity points along the wheel axle and the tilt angle is meaningless
// (atan2 of two near-zero numbers). If more than this much gravity (in g) acts along the
// axle, the robot counts as fallen sideways and the motors stay off. 0.5 g = about 30 deg.
constexpr float    SIDEWAYS_LIMIT_G = 0.5f;
constexpr float    START_ANGLE_DEG = 3.0f;   // [TUNE] balancing starts when held within this
constexpr uint32_t START_HOLD_MS   = 500;    //        ...for this long (so it does not jump to life)
constexpr bool     START_BALANCING_ON_BOOT = true;  // false = motors stay off until you type  go

// -------------------------------------------------------------------------------------
// 9. MOTORS / L298N
// -------------------------------------------------------------------------------------
// ESP32 PWM (LEDC). Rule: PWM_FREQ_HZ * 2^PWM_RES_BITS must be <= 80 MHz.
// 20 kHz is above human hearing (no whine). The L298N is a slow, old driver: if the wheels
// feel weak at low speed or the L298N gets very hot, try 5000 or 1000 (a whine is normal).
// Re-measure MOTOR_MIN_PWM (ramp test) after changing the frequency.
constexpr uint32_t PWM_FREQ_HZ  = 20000;     // [TUNE]
constexpr uint8_t  PWM_RES_BITS = 8;         // 8 bits -> duty 0 .. 255
constexpr int      PWM_MAX      = (1 << PWM_RES_BITS) - 1;

// Largest PWM the balance controller may use. Lower it (e.g. 150) for gentle first tests.
constexpr float MOTOR_MAX_PWM = 255.0f;      // [TUNE] (live: maxpwm)

// Dead-band compensation: DC gear motors do not move at all below some PWM (friction).
// Measure it with the  ramp  command (wheels off the ground) and put the value here.
// Any non-zero output is then mapped to  MIN..MAX  so small corrections are not lost.
// This matters: in simulation the same robot balanced with it and fell without it.
constexpr float MOTOR_MIN_PWM = 0.0f;        // [TUNE] (live: minpwm) typical 30 .. 90 for L298N + 12 V
constexpr float MOTOR_ZERO_BAND = 1.0f;      // outputs smaller than this are treated as "stop"

// Motor direction. In "motor" test mode each wheel must turn FORWARD when told to.
// If one turns backward, set its INVERT to true (or swap its two wires at the L298N, power OFF).
constexpr bool LEFT_MOTOR_INVERT  = false;   // [CHECK]
constexpr bool RIGHT_MOTOR_INVERT = false;   // [CHECK]

// If the robot slowly turns in a circle while balancing, one motor is stronger.
// Reduce the stronger side a little, e.g. 0.95.
constexpr float LEFT_MOTOR_GAIN  = 1.0f;     // [TUNE]
constexpr float RIGHT_MOTOR_GAIN = 1.0f;     // [TUNE]

// Test modes (wheels OFF the ground!)
constexpr float    MOTOR_TEST_PWM = 150.0f;  // speed used by the  motor  direction test
constexpr float    RAMP_STEP_PWM  = 5.0f;    // ramp  test: add this much PWM ...
constexpr uint32_t RAMP_STEP_MS   = 400;     //             ... every 400 ms

// -------------------------------------------------------------------------------------
// 10. STAGE 2 - stop the robot from drifting away
// -------------------------------------------------------------------------------------
// Without wheel encoders we estimate wheel speed from the average motor output.
// If the robot keeps rolling one way, this tilts the target angle the other way so it
// leans back and stops:   target = setpoint - SPEED_KP * averageOutput
// Needed because while the robot accelerates, the accelerometer cannot tell
// "accelerating" from "leaning", so the angle estimate is slightly wrong exactly when
// the robot starts to run away. 0 = off (useful to see the inner PID loop on its own).
//   Robot still runs off in one direction -> raise.   Slow, large rocking -> lower.
constexpr float SPEED_KP             = 0.02f; // [TUNE] (live: skp) try 0.005 .. 0.05
constexpr float SPEED_FILTER_TAU_S   = 0.5f;  // [TUNE] averaging time for the speed estimate
constexpr float MAX_TARGET_SHIFT_DEG = 5.0f;  // [TUNE] stage 2 may move the target at most this much
