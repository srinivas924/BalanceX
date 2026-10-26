/*
  =====================================================================================
   BalanceX_Uno - two-wheel self-balancing robot
   Arduino Uno R3 + MPU6050 + L298N + 2 DC gear motors with encoders + 3S LiPo
  =====================================================================================

   Files (keep them together in this folder; the Arduino IDE shows them as tabs):
     BalanceX_Uno.ino  hardware: sensor, encoders, motors, serial commands, telemetry
     config.h          ALL pins, your calibration values and the tuning knobs
     balance.h         the controller (also tested on a PC by tests/uno_sim_test.cpp)

   No extra libraries: the MPU6050 is read directly over I2C (Wire is built in).

   Every 5 ms (200 Hz):
     1. read accelerometer X/Z + gyro Y (one 0.4 ms I2C read) and both encoder counts
     2. tilt angle (complementary filter) and wheel speeds
     3. forward loop: position + speed error -> target lean angle
     4. balance loop: KP * (angle - target) + KD * rate -> PWM
     5. straight-line loop: left/right travel difference -> PWM difference
     6. friction compensation per motor -> L298N

   Start-up: hold the robot UPRIGHT and STILL at its balance point while it calibrates
   (3 s). It starts balancing 0.8 s later, holds its place for 3 s, then drives slowly
   forward in a straight line. Send  s  to stop,  g  to start again,  ?  for all commands.
   Serial Monitor: 115200 baud. Any line ending works.

   SAFETY: run the sign tests in docs/UNO_GUIDE.md with the wheels OFF the ground first.
  =====================================================================================
*/
#include <Arduino.h>
#include <Wire.h>
#include <avr/wdt.h>
#include <stdlib.h>
#include <string.h>

#include "config.h"
#include "balance.h"

static_assert(PIN_LEFT_ENC_A == 2 && PIN_LEFT_ENC_B == 4 && PIN_RIGHT_ENC_A == 3 && PIN_RIGHT_ENC_B == 12,
              "the encoder interrupts read D2/D4/D3/D12 straight from PIND/PINB - update them too");

// =====================================================================================
//  Non-blocking serial output
//  The Uno's serial buffer holds only 64 characters. Printing more than that makes
//  Serial.print() WAIT until the characters are sent (115200 baud = 87 us each). Your
//  previous sketch printed ~150 characters every 100 ms and so froze the control loop
//  for ~9 ms each time: a 10 Hz jolt to the motors. Here everything is queued and handed
//  to Serial only as fast as it can take it, so printing never delays the control loop.
//  If the queue is full, text is dropped (never the control loop).
// =====================================================================================
class TxQueue : public Print {
 public:
  using Print::write;
  size_t write(uint8_t c) override {
    const uint8_t next = (uint8_t)((head + 1) % SIZE);
    if (next == tail) return 0;        // full: drop
    buf[head] = c;
    head = next;
    return 1;
  }
  uint8_t freeSpace() const { return (uint8_t)((tail + SIZE - head - 1) % SIZE); }
  void service() {
    int room = Serial.availableForWrite();
    while (room-- > 0 && tail != head) {
      Serial.write(buf[tail]);
      tail = (uint8_t)((tail + 1) % SIZE);
    }
  }
  void flushBlocking() {               // only used while the motors are off
    while (tail != head) {
      service();
      wdt_reset();
    }
  }

 private:
  static const uint8_t SIZE = 200;
  uint8_t buf[SIZE];
  volatile uint8_t head = 0, tail = 0;
};

TxQueue out;

// =====================================================================================
//  Wheel encoders
//  Same counting rule as your previous sketch (CHANGE on A; A == B -> +1, else -1), so
//  your encoder signs and counts per revolution stay valid. The pins are read straight
//  from the port registers: ~0.2 us instead of ~4 us for each digitalRead().
// =====================================================================================
volatile long leftEncoderRaw = 0;
volatile long rightEncoderRaw = 0;

void leftEncoderISR() {
  const uint8_t pd = PIND;
  const bool a = pd & _BV(PD2);        // D2
  const bool b = pd & _BV(PD4);        // D4
  if (a == b) leftEncoderRaw++;
  else        leftEncoderRaw--;
}

void rightEncoderISR() {
  const bool a = PIND & _BV(PD3);      // D3
  const bool b = PINB & _BV(PB4);      // D12
  if (a == b) rightEncoderRaw++;
  else        rightEncoderRaw--;
}

// A long is 4 bytes and the Uno copies 1 byte at a time: block the interrupts while
// copying, or a count could be read half old, half new.
void readEncoders(long& left, long& right) {
  noInterrupts();
  left = leftEncoderRaw;
  right = rightEncoderRaw;
  interrupts();
}

// Same, with your encoder signs applied (+ = forward on both wheels).
void readEncodersForward(long& left, long& right) {
  readEncoders(left, right);
  left *= LEFT_ENCODER_SIGN;
  right *= RIGHT_ENCODER_SIGN;
}

// =====================================================================================
//  MPU6050 (register access, no library)
//  Register map: "MPU-6000/MPU-6050 Register Map and Descriptions", rev 4.2.
// =====================================================================================
const uint8_t MPU_REG_SMPLRT_DIV   = 0x19;
const uint8_t MPU_REG_CONFIG       = 0x1A;
const uint8_t MPU_REG_GYRO_CONFIG  = 0x1B;
const uint8_t MPU_REG_ACCEL_CONFIG = 0x1C;
const uint8_t MPU_REG_ACCEL_XOUT_H = 0x3B;   // 14 bytes: accel X Y Z, temperature, gyro X Y Z
const uint8_t MPU_REG_PWR_MGMT_1   = 0x6B;

const uint8_t MPU_GYRO_500DPS = 1 << 3;      // +-500 deg/s, as in your previous sketch
const uint8_t MPU_ACCEL_4G    = 1 << 3;      // +-4 g,       as in your previous sketch
const float   GYRO_LSB_PER_DPS = 65.5f;
const float   ACCEL_LSB_PER_G  = 8192.0f;

float gyroBiasDps = 0.0f;                    // measured by calibrate()

bool imuWriteReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_I2C_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool imuReadRegs(uint8_t reg, uint8_t* buf, uint8_t len) {
  Wire.beginTransmission(MPU_I2C_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;   // false = repeated start
  if (Wire.requestFrom((uint8_t)MPU_I2C_ADDR, len) != len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = (uint8_t)Wire.read();
  return true;
}

bool imuBegin() {
  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(I2C_TIMEOUT_US, true);   // never hang on a disturbed I2C bus
#endif
  imuWriteReg(MPU_REG_PWR_MGMT_1, 0x80);       // reset (no ACK check: the chip resets)
  delay(100);
  bool ok = imuWriteReg(MPU_REG_PWR_MGMT_1, 0x01);   // wake up, clock from the X gyro
  delay(10);
  ok = ok && imuWriteReg(MPU_REG_SMPLRT_DIV, 0);     // 1 kHz internal sample rate
  ok = ok && imuWriteReg(MPU_REG_CONFIG, MPU_DLPF_CFG & 0x07);
  ok = ok && imuWriteReg(MPU_REG_GYRO_CONFIG, MPU_GYRO_500DPS);
  ok = ok && imuWriteReg(MPU_REG_ACCEL_CONFIG, MPU_ACCEL_4G);
  uint8_t check = 0;
  return ok && imuReadRegs(MPU_REG_GYRO_CONFIG, &check, 1) && check == MPU_GYRO_500DPS;
}

// Accelerometer X and Z (g) and gyro Y (deg/s, offset NOT removed).
bool imuRead(float& axG, float& azG, float& gyDps) {
  uint8_t b[14];
  if (!imuReadRegs(MPU_REG_ACCEL_XOUT_H, b, sizeof(b))) return false;
  const int16_t ax = (int16_t)((b[0] << 8) | b[1]);
  const int16_t ay = (int16_t)((b[2] << 8) | b[3]);
  const int16_t az = (int16_t)((b[4] << 8) | b[5]);
  const int16_t gy = (int16_t)((b[10] << 8) | b[11]);
  axG = ax / ACCEL_LSB_PER_G;
  azG = az / ACCEL_LSB_PER_G;
  gyDps = gy / GYRO_LSB_PER_DPS;
  // A disconnected sensor often reads all zeros. A real one always feels ~1 g of gravity.
  const float ayG = ay / ACCEL_LSB_PER_G;
  return axG * axG + ayG * ayG + azG * azG > 0.04f;
}

// =====================================================================================
//  Motors (L298N)
// =====================================================================================
void driveMotor(uint8_t enPin, uint8_t in1Pin, uint8_t in2Pin, int electricalPwm) {
  const int duty = electricalPwm >= 0 ? electricalPwm : -electricalPwm;
  if (electricalPwm > 0) {
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
  } else if (electricalPwm < 0) {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
  } else {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
  }
  analogWrite(enPin, duty > 255 ? 255 : duty);
}

int lastLeftPwm = 0, lastRightPwm = 0;

// left / right: PWM, + = forward. Your motor signs turn that into the L298N direction.
void setMotors(float left, float right) {
  lastLeftPwm = (int)lroundf(left);
  lastRightPwm = (int)lroundf(right);
  driveMotor(PIN_LEFT_EN, PIN_LEFT_IN1, PIN_LEFT_IN2, lastLeftPwm * LEFT_MOTOR_SIGN);
  driveMotor(PIN_RIGHT_EN, PIN_RIGHT_IN1, PIN_RIGHT_IN2, lastRightPwm * RIGHT_MOTOR_SIGN);
}

// =====================================================================================
//  State
// =====================================================================================
enum class Ui : uint8_t { Run, SensorView, MotorTest };

bx::Controller ctrl;
Ui ui = Ui::Run;
bool telemetryOn = TELEMETRY_AT_BOOT;

uint32_t lastControlUs = 0;
uint32_t lastPrintMs = 0;
uint32_t maxWorkUs = 0;          // longest control step (should stay far below 5000)
uint32_t lateSteps = 0;          // control steps that started more than 2.5 ms late
uint16_t imuFailures = 0;        // consecutive failed sensor reads

// motor test
uint8_t  testPhase = 0;
uint32_t testPhaseStartMs = 0;
long     testStartLeft = 0, testStartRight = 0;

// serial input
char     lineBuf[24];
uint8_t  lineLen = 0;
uint32_t lastCharMs = 0;

// =====================================================================================
//  Calibration (robot upright, at its balance point, still)
// =====================================================================================
bool calibrate() {
  setMotors(0, 0);
  const uint8_t ATTEMPTS = 5;
  for (uint8_t attempt = 1; attempt <= ATTEMPTS; attempt++) {
    Serial.print(F("Calibrating "));
    Serial.print(attempt);
    Serial.println(F("/5: hold the robot UPRIGHT at its balance point and STILL..."));
    double gyroSum = 0.0, pitchSum = 0.0;
    float gyroMin = 1e9f, gyroMax = -1e9f;
    uint16_t n = 0;
    const uint32_t start = millis();
    while (millis() - start < CALIBRATION_MS) {
      wdt_reset();
      const uint32_t t0 = micros();
      float ax, az, gy;
      if (imuRead(ax, az, gy)) {
        gyroSum += gy;
        pitchSum += bx::accelPitchDeg(ax, az);
        if (gy < gyroMin) gyroMin = gy;
        if (gy > gyroMax) gyroMax = gy;
        n++;
      }
      while (micros() - t0 < CONTROL_PERIOD_US) {}
    }
    if (n < CALIBRATION_MS / 10) {           // fewer than half the readings worked
      Serial.println(F("  sensor reads failing - again"));
      continue;
    }
    const bool still = gyroMax - gyroMin <= CALIB_MAX_GYRO_SPREAD;
    if (!still && attempt < ATTEMPTS) {
      Serial.println(F("  the robot moved - again"));
      continue;
    }
    // Still, or the last attempt: use it (averaging over 3 s cancels small hand tremor).
    gyroBiasDps = (float)(gyroSum / n);
    ctrl.zeroDeg = (float)(pitchSum / n);
    Serial.print(still ? F("  done: zero angle ") : F("  moved, but using it: zero angle "));
    Serial.print(ctrl.zeroDeg, 2);
    Serial.print(F(" deg, gyro offset "));
    Serial.print(gyroBiasDps, 3);
    Serial.print(F(" deg/s, spread "));
    Serial.println(gyroMax - gyroMin, 1);
    if (!still) Serial.println(F("  If it drifts or leans, send s then r to calibrate again."));
    lastControlUs = micros();
    return still;
  }
  Serial.println(F("  CALIBRATION FAILED (sensor). Check the MPU6050 wiring, then send r."));
  lastControlUs = micros();
  return false;
}

// =====================================================================================
//  Event messages
// =====================================================================================
void reportEvent(bx::Event e) {
  switch (e) {
    case bx::EV_ARMED:    out.println(F("BALANCING (motors on). Holding position.")); break;
    case bx::EV_DRIVING:  out.println(F("Driving forward.")); break;
    case bx::EV_FELL:     out.println(F("FELL: motors off. Stand it up and hold still to restart.")); break;
    case bx::EV_TIMING:   out.println(F("TIMING FAULT: control step late, motors off.")); break;
    case bx::EV_DIR_LEFT:
      out.println(F("DIRECTION FAULT: LEFT wheel turns against its command. Motors off."));
      out.println(F("  Run the  t  test (wheels lifted). Send g to retry."));
      break;
    case bx::EV_DIR_RIGHT:
      out.println(F("DIRECTION FAULT: RIGHT wheel turns against its command. Motors off."));
      out.println(F("  Run the  t  test (wheels lifted). Send g to retry."));
      break;
    default: break;
  }
}

// =====================================================================================
//  Motor / encoder test (WHEELS OFF THE GROUND)
//  Drives each wheel FORWARD in turn and checks that its own encoder counts up.
// =====================================================================================
const uint32_t TEST_PAUSE_MS = 600;

// phases: 0 = left forward, 1 = pause, 2 = right forward, 3 = pause
void startMotorTest() {
  ctrl.stop();
  setMotors(0, 0);
  ui = Ui::MotorTest;
  testPhase = 0;
  testPhaseStartMs = millis();
  readEncodersForward(testStartLeft, testStartRight);
  out.println(F("MOTOR TEST - wheels must be OFF the ground. Send s to abort."));
}

void judgeWheel(const __FlashStringHelper* name, long own, long other, float rpm) {
  out.print(name);
  out.print(F(": own encoder "));
  out.print(own);
  out.print(F(", other "));
  out.print(other);
  out.print(F(", "));
  out.print(rpm, 0);
  out.println(F(" RPM"));
  const long minCounts = (long)(LEFT_COUNTS_PER_WHEEL_REV / 20.0f);   // 1/20 of a turn
  if (labs(other) > labs(own)) {
    out.println(F("   the OTHER encoder moved: motor or encoder cables are swapped"));
  } else if (own > minCounts) {
    out.println(F("   OK: positive command -> encoder counts up"));
  } else if (own < -minCounts) {
    out.println(F("   WRONG WAY: flip this wheel's MOTOR_SIGN in config.h"));
  } else {
    out.println(F("   no movement seen: check motor power, ENA/ENB jumpers, encoder wiring"));
  }
}

// Runs every control step while the test is active.
void runMotorTest() {
  long l, r;
  readEncodersForward(l, r);
  const bool drivePhase = (testPhase % 2 == 0);
  if (millis() - testPhaseStartMs < (drivePhase ? MOTOR_TEST_MS : TEST_PAUSE_MS)) {
    if (testPhase == 0) setMotors(MOTOR_TEST_PWM, 0);
    else if (testPhase == 2) setMotors(0, MOTOR_TEST_PWM);
    else setMotors(0, 0);
    return;
  }
  setMotors(0, 0);
  if (testPhase == 0) judgeWheel(F("LEFT "), l - testStartLeft, r - testStartRight, ctrl.leftRpm);
  if (testPhase == 2) judgeWheel(F("RIGHT"), r - testStartRight, l - testStartLeft, ctrl.rightRpm);
  testPhase++;
  testPhaseStartMs = millis();
  if (testPhase == 2) {          // the right wheel is measured from here
    testStartLeft = l;
    testStartRight = r;
  }
  if (testPhase == 4) {
    out.println(F("Both OK? Then also check the pitch sign with  i  (docs/UNO_GUIDE.md)."));
    out.println(F("Motors stay off; send g to balance."));
    ui = Ui::Run;
  }
}

// =====================================================================================
//  Serial commands
// =====================================================================================
void printSettings() {
  const bx::Settings& s = ctrl.set;
  Serial.println(F("\n// ---- current values (paste into config.h) ----"));
  Serial.print(F("constexpr float KP = "));          Serial.print(s.kp, 2);              Serial.println(F("f;"));
  Serial.print(F("constexpr float KD = "));          Serial.print(s.kd, 3);              Serial.println(F("f;"));
  Serial.print(F("constexpr float SPEED_KP = "));    Serial.print(s.speedKp, 3);         Serial.println(F("f;"));
  Serial.print(F("constexpr float POS_KP = "));      Serial.print(s.posKp, 2);           Serial.println(F("f;"));
  Serial.print(F("constexpr float HEADING_KP = "));  Serial.print(s.headingKp, 1);       Serial.println(F("f;"));
  Serial.print(F("constexpr float FORWARD_SPEED_RPM = ")); Serial.print(s.forwardRpm, 1); Serial.println(F("f;"));
  Serial.print(F("constexpr float FRICTION_COMP_PERCENT = ")); Serial.print(s.frictionPercent, 0); Serial.println(F(".0f;"));
  Serial.print(F("constexpr float MAX_PWM = "));     Serial.print(s.maxPwm, 0);          Serial.println(F(".0f;"));
  Serial.print(F("constexpr float COMP_FILTER_ALPHA = ")); Serial.print(s.alpha, 4);     Serial.println(F("f;"));
  Serial.print(F("// zero angle "));                 Serial.print(ctrl.zeroDeg, 2);
  Serial.print(F(" deg, gyro offset "));             Serial.print(gyroBiasDps, 3);
  Serial.println(F(" deg/s (measured at start-up)"));
  Serial.print(F("// longest control step "));       Serial.print(maxWorkUs);
  Serial.print(F(" us of "));                        Serial.print(CONTROL_PERIOD_US);
  Serial.print(F(" us, late steps "));               Serial.println(lateSteps);
  maxWorkUs = 0;
}

void printHelp() {
  Serial.println(F("\n---------------------------- COMMANDS ----------------------------"));
  Serial.println(F(" s        STOP (motors off)          g   go: balance when held upright"));
  Serial.println(F(" t        motor/encoder test (WHEELS OFF THE GROUND)"));
  Serial.println(F(" i        sensor view on/off: angle, rate, encoder counts (motors off)"));
  Serial.println(F(" m        telemetry on/off (for Tools > Serial Plotter)"));
  Serial.println(F(" z        zero: current angle = balance point (hold it there, still)"));
  Serial.println(F(" r        recalibrate gyro offset and zero angle (motors off)"));
  Serial.println(F(" c        show current values (config.h format) and loop timing"));
  Serial.println(F(" p <v>    KP            d <v>  KD          v <v>  SPEED_KP"));
  Serial.println(F(" o <v>    POS_KP        h <v>  HEADING_KP  f <v>  forward speed (RPM)"));
  Serial.println(F(" k <v>    friction compensation %          x <v>  MAX_PWM"));
  Serial.println(F(" a <v>    complementary filter alpha"));
  Serial.println(F(" A letter without a value prints the current value. Example:  p 30"));
  Serial.println(F("------------------------------------------------------------------"));
}

bool safeToBlock() {             // long (blocking) prints only while the motors are off
  return ctrl.mode != bx::Mode::Balancing && ui != Ui::MotorTest;
}

void setValue(const __FlashStringHelper* name, float& target, const char* arg, float lo, float hi) {
  if (*arg != '\0') {
    const float v = (float)atof(arg);
    if (v < lo || v > hi) {
      out.print(name);
      out.print(F(" must be "));
      out.print(lo, 3);
      out.print(F(" .. "));
      out.println(hi, 3);
      return;
    }
    target = v;
  }
  out.print(name);
  out.print(F(" = "));
  out.println(target, 3);
}

void handleCommand(char* line) {
  while (*line == ' ') line++;
  const char cmd = line[0];
  char* arg = line + 1;
  while (*arg == ' ') arg++;
  bx::Settings& s = ctrl.set;

  switch (cmd) {
    case 's': case 'S':
      ctrl.stop();
      setMotors(0, 0);
      ui = Ui::Run;
      out.println(F("STOPPED. Send g to balance again."));
      break;
    case 'g': case 'G':
      ui = Ui::Run;
      ctrl.go();
      out.println(F("GO: hold the robot upright and still to start."));
      break;
    case 't': case 'T': startMotorTest(); break;
    case 'i': case 'I':
      if (ui == Ui::SensorView) {
        ui = Ui::Run;
        out.println(F("sensor view off (motors stay off; send g to balance)"));
      } else {
        ctrl.stop();
        setMotors(0, 0);
        ui = Ui::SensorView;
        out.println(F("SENSOR VIEW (motors off). Tilt forward: accel and rate +. Roll forward: L and R +."));
      }
      break;
    case 'm': case 'M':
      telemetryOn = !telemetryOn;
      out.println(telemetryOn ? F("telemetry on") : F("telemetry off"));
      break;
    case 'z': case 'Z':
      ctrl.zeroHere();
      out.print(F("zero angle = "));
      out.println(ctrl.zeroDeg, 2);
      break;
    case 'r': case 'R':
      if (!safeToBlock()) { out.println(F("send s first")); break; }
      ctrl.stop();
      out.flushBlocking();
      calibrate();
      ctrl.restartEstimates();
      out.println(F("Send g to balance."));
      break;
    case 'c': case 'C':
      if (!safeToBlock()) { out.println(F("send s first")); break; }
      out.flushBlocking();
      printSettings();
      break;
    case '?':
      if (!safeToBlock()) { out.println(F("send s first")); break; }
      out.flushBlocking();
      printHelp();
      break;
    case 'p': setValue(F("KP"), s.kp, arg, 1.0f, 200.0f); break;
    case 'd': setValue(F("KD"), s.kd, arg, 0.0f, 10.0f); break;
    case 'v': setValue(F("SPEED_KP"), s.speedKp, arg, 0.0f, 1.0f); break;
    case 'o': setValue(F("POS_KP"), s.posKp, arg, 0.0f, 50.0f); break;
    case 'h': setValue(F("HEADING_KP"), s.headingKp, arg, 0.0f, 1000.0f); break;
    case 'f': setValue(F("FORWARD_SPEED_RPM"), s.forwardRpm, arg, -MAX_SPEED_RPM, MAX_SPEED_RPM); break;
    case 'k': setValue(F("FRICTION_COMP_PERCENT"), s.frictionPercent, arg, 0.0f, 120.0f); break;
    case 'x': setValue(F("MAX_PWM"), s.maxPwm, arg, 50.0f, 255.0f); break;
    case 'a': setValue(F("COMP_FILTER_ALPHA"), s.alpha, arg, 0.9f, 0.999f); break;
    case '\0': break;
    default:
      out.print(F("unknown command '"));
      out.print(cmd);
      out.println(F("' - send ? for the list"));
  }
}

// Reads characters without ever waiting. A line runs on Enter, or 80 ms after the last
// character (so it also works with "No line ending" in the Serial Monitor).
void pollSerial() {
  while (Serial.available() > 0) {
    const char c = (char)Serial.read();
    lastCharMs = millis();
    if (c == '\n' || c == '\r') {
      if (lineLen > 0) {
        lineBuf[lineLen] = '\0';
        lineLen = 0;
        handleCommand(lineBuf);
      }
    } else if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    }
  }
  if (lineLen > 0 && millis() - lastCharMs > 80) {
    lineBuf[lineLen] = '\0';
    lineLen = 0;
    handleCommand(lineBuf);
  }
}

// =====================================================================================
//  Telemetry (name:value pairs for the Serial Plotter)
// =====================================================================================
void printTelemetry() {
  const uint32_t now = millis();
  if (now - lastPrintMs < (ui == Ui::SensorView ? 100UL : TELEMETRY_PERIOD_MS)) return;
  if (out.freeSpace() < 110) return;     // still sending the previous line: skip this one
  lastPrintMs = now;

  if (ui == Ui::SensorView) {
    long l, r;
    readEncodersForward(l, r);
    out.print(F("pitch:"));   out.print(ctrl.pitch, 2);
    out.print(F(",accel:"));  out.print(ctrl.accPitch, 2);
    out.print(F(",rate:"));   out.print(ctrl.rate, 1);
    out.print(F(",L:"));      out.print(l);
    out.print(F(",R:"));      out.print(r);
    out.print(F(",Lrpm:"));   out.print(ctrl.leftRpm, 1);
    out.print(F(",Rrpm:"));   out.println(ctrl.rightRpm, 1);
    return;
  }
  if (!telemetryOn || ctrl.mode != bx::Mode::Balancing) return;
  out.print(F("pitch:"));     out.print(ctrl.pitch, 2);
  out.print(F(",target:"));   out.print(ctrl.targetDeg, 2);
  out.print(F(",rpm:"));      out.print(0.5f * (ctrl.leftRpm + ctrl.rightRpm), 1);
  out.print(F(",want:"));     out.print(ctrl.speedCmd, 1);
  out.print(F(",turn:"));     out.print(ctrl.turnCmd, 1);
  out.print(F(",L/10:"));     out.print(lastLeftPwm * 0.1f, 1);
  out.print(F(",R/10:"));     out.println(lastRightPwm * 0.1f, 1);
}

// =====================================================================================
//  setup() and loop()
// =====================================================================================
void setup() {
  MCUSR = 0;
  wdt_disable();

  // Motors off before anything else.
  const uint8_t motorPins[] = {PIN_LEFT_EN, PIN_LEFT_IN1, PIN_LEFT_IN2,
                               PIN_RIGHT_EN, PIN_RIGHT_IN1, PIN_RIGHT_IN2};
  for (uint8_t i = 0; i < sizeof(motorPins); i++) {
    pinMode(motorPins[i], OUTPUT);
    digitalWrite(motorPins[i], LOW);
  }

  pinMode(PIN_LEFT_ENC_A, INPUT_PULLUP);
  pinMode(PIN_LEFT_ENC_B, INPUT_PULLUP);
  pinMode(PIN_RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(PIN_RIGHT_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_LEFT_ENC_A), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_RIGHT_ENC_A), rightEncoderISR, CHANGE);

  Serial.begin(SERIAL_BAUD);
  Serial.println(F("\n=== BalanceX - Arduino Uno ==="));

  while (!imuBegin()) {
    Serial.println(F("MPU6050 not found: check VCC 3.3V, GND, SDA=A4, SCL=A5. Retrying..."));
    delay(1000);
  }
  Serial.println(F("MPU6050 ready."));

  calibrate();
  printHelp();
  Serial.println(F("Hold it upright and still: balancing starts in 0.8 s, driving 3 s later."));

  wdt_enable(WDTO_250MS);      // if the program ever hangs, the board resets (motors off)
  lastControlUs = micros();
}

void controlStep(float dt) {
  float ax, az, gy;
  if (!imuRead(ax, az, gy)) {
    ctrl.sensorFault();
    setMotors(0, 0);
    imuFailures++;
    if (imuFailures == 1) out.println(F("IMU read failed: motors off."));
    if (imuFailures % 40 == 0) imuBegin();   // every ~0.2 s: restart the sensor
    return;
  }
  if (imuFailures > 0) {
    out.println(F("IMU back. Stand the robot up to restart."));
    imuFailures = 0;
  }

  bx::Sample s;
  s.axG = ax;
  s.azG = az;
  s.gyDps = gy - gyroBiasDps;
  readEncoders(s.leftCount, s.rightCount);

  const bx::Event e = ctrl.step(s, dt, millis());

  if (ui == Ui::MotorTest) runMotorTest();
  else setMotors(ctrl.leftPwm, ctrl.rightPwm);

  reportEvent(e);
}

void loop() {
  wdt_reset();
  pollSerial();

  const uint32_t now = micros();
  const uint32_t elapsed = now - lastControlUs;
  if (elapsed >= CONTROL_PERIOD_US) {
    lastControlUs = now;
    if (elapsed > CONTROL_PERIOD_US + CONTROL_PERIOD_US / 2) lateSteps++;
    controlStep(elapsed * 1e-6f);
    const uint32_t work = micros() - now;
    if (work > maxWorkUs) maxWorkUs = work;
    printTelemetry();       // right after a step: formatting never delays the next one
  }
  out.service();
}
