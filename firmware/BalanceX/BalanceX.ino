/*
  =====================================================================================
   BalanceX - two-wheel self-balancing robot
   ESP32-WROOM-32 DevKit + MPU6050 + L298N + 2 DC gear motors + 3S LiPo
  =====================================================================================

   Files (keep them all in this folder - the Arduino IDE shows them as tabs):
     BalanceX.ino  main program: setup(), loop(), modes, serial / Bluetooth commands
     config.h      ALL pins and tunable values   <-- the file you will edit
     control.h     tilt angle maths, complementary filter, PID
     imu.h         MPU6050 driver (direct register access, no library needed)
     motors.h      L298N driver (works with ESP32 Arduino core 2.x and 3.x)

   One control step (runs 200 times per second):
     1. read accelerometer + gyroscope
     2. accelerometer -> tilt angle (no drift, but noisy);  gyro -> tilt rate (smooth)
     3. complementary filter blends them into one reliable tilt angle
     4. PID:  output = KP*error + KI*integral(error) + KD*tiltRate,  error = angle - target
     5. output -> both motors. Leaning forward -> drive forward, back under the robot.

   Safety: motors only run while the robot is within FALL_ANGLE_DEG of upright, and
   switch off instantly if the sensor stops answering.

   Type  help  in the Serial Monitor (115200 baud, line ending "Newline") for commands.
   Bring-up and tuning steps: docs/TUNING_GUIDE.md.  Problems + fixes: docs/ERROR_LOG.md
  =====================================================================================
*/
#include <Arduino.h>
#include <Wire.h>
#include <esp_system.h>   // esp_reset_reason()

#if !defined(ESP_ARDUINO_VERSION_MAJOR) || (ESP_ARDUINO_VERSION_MAJOR < 2)
#error "Update the ESP32 board package to version 2.x or 3.x (Tools > Board > Boards Manager > esp32)"
#endif

#include "config.h"
#include "control.h"
#include "imu.h"
#include "motors.h"

#if USE_BLUETOOTH
#include <BluetoothSerial.h>
BluetoothSerial SerialBT;
#endif

// =====================================================================================
//  Types and state (declared before any function, so the Arduino IDE's automatically
//  generated function prototypes can use them)
// =====================================================================================
enum class Mode : uint8_t { Balance, Stopped, SensorTest, MotorTest, Ramp };
enum class BalanceState : uint8_t { WaitUpright, Balancing };

// Sends text to USB Serial and, if a phone is connected, to Bluetooth as well.
class DualPrint : public Print {
 public:
  using Print::write;
  size_t write(uint8_t c) override { return write(&c, 1); }
  size_t write(const uint8_t* buf, size_t n) override {
    Serial.write(buf, n);
#if USE_BLUETOOTH
    if (SerialBT.hasClient()) SerialBT.write(buf, n);
#endif
    return n;
  }
};

// Collects characters into a line without ever waiting (Serial.readStringUntil() would
// pause the control loop for up to 1 s - long enough for the robot to fall over).
struct LineReader {
  char    buf[64];
  uint8_t len = 0;
  bool poll(Stream& s) {          // true when a complete line is in buf
    while (s.available() > 0) {
      const char c = (char)s.read();
      if (c == '\n' || c == '\r') {
        if (len == 0) continue;   // ignore empty lines (and the \n of a \r\n pair)
        buf[len] = '\0';
        len = 0;
        return true;
      }
      if (len < sizeof(buf) - 1) buf[len++] = c;
    }
    return false;
  }
};

struct MotorTestStep { float left, right; uint32_t ms; const char* text; };

// Each wheel alone, both directions, then both together. Wheels OFF the ground!
const MotorTestStep MOTOR_TEST_STEPS[] = {
  {+1,  0, 2000, "LEFT wheel  FORWARD  (should roll the robot toward the FRONT mark)"},
  { 0,  0,  800, "pause"},
  {-1,  0, 2000, "LEFT wheel  BACKWARD"},
  { 0,  0,  800, "pause"},
  { 0, +1, 2000, "RIGHT wheel FORWARD  (should roll the robot toward the FRONT mark)"},
  { 0,  0,  800, "pause"},
  { 0, -1, 2000, "RIGHT wheel BACKWARD"},
  { 0,  0,  800, "pause"},
  {+1, +1, 2000, "BOTH wheels FORWARD"},
};
constexpr uint8_t MOTOR_TEST_STEP_COUNT = sizeof(MOTOR_TEST_STEPS) / sizeof(MOTOR_TEST_STEPS[0]);

DualPrint out;
LineReader usbLine;
#if USE_BLUETOOTH
LineReader btLine;
#endif

Tuning       tune;
Mode         mode = START_BALANCING_ON_BOOT ? Mode::Balance : Mode::Stopped;
BalanceState balanceState = BalanceState::WaitUpright;

imu::Sample         sample;
ComplementaryFilter tiltFilter;
BalanceController   controller;

float accTiltDeg  = 0.0f;   // tilt from accelerometer only (noisy)
float tiltRateDps = 0.0f;   // tilt rate from gyro (deg/s, positive = tipping forward)
float tiltDeg     = 0.0f;   // filtered tilt angle (what the controller uses)
float sidewaysG   = 0.0f;   // gravity along the wheel axle, smoothed (~1 g = lying on its side)

bool     uprightTimerRunning = false;
uint32_t uprightSinceMs = 0;

uint8_t  telemetryLevel = TELEMETRY_AT_BOOT;
uint32_t lastTelemetryMs = 0;

uint32_t lastLoopUs = 0;
uint32_t maxLoopWorkUs = 0;    // longest time one control step took (should stay << LOOP_PERIOD_US)
uint32_t loopOverruns = 0;     // control steps that started late
uint32_t imuFailCount = 0;     // consecutive failed sensor reads

uint8_t  motorTestStep = 0;
uint32_t motorTestStepStartMs = 0;
float    rampPwm = 0.0f;
uint32_t rampStepStartMs = 0;

// sensor test: compares gyro and accelerometer to catch a wrong axis/sign
float    sensorGyroSumDeg = 0.0f;
float    sensorAccStartDeg = 0.0f;
uint32_t sensorWindowStartMs = 0;
uint32_t lastSensorPrintMs = 0;

// =====================================================================================
//  Helpers
// =====================================================================================
const char* resetReasonText(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON:   return "power on";
    case ESP_RST_EXT:       return "reset pin";
    case ESP_RST_SW:        return "software restart";
    case ESP_RST_PANIC:     return "CRASH (exception/panic)";
    case ESP_RST_INT_WDT:   return "CRASH (interrupt watchdog)";
    case ESP_RST_TASK_WDT:  return "CRASH (task watchdog)";
    case ESP_RST_WDT:       return "CRASH (other watchdog)";
    case ESP_RST_DEEPSLEEP: return "wake from deep sleep";
    case ESP_RST_BROWNOUT:  return "BROWNOUT (supply voltage dipped)";
    default:                return "unknown";
  }
}

const char* modeName(Mode m) {
  switch (m) {
    case Mode::Balance:    return "BALANCE";
    case Mode::Stopped:    return "STOPPED";
    case Mode::SensorTest: return "SENSOR TEST";
    case Mode::MotorTest:  return "MOTOR TEST";
    case Mode::Ramp:       return "RAMP TEST";
  }
  return "?";
}

char axisName(int axis) { return "XYZ"[axis]; }

// Motors off and controller memory (I term, speed estimate) cleared.
void stopMotors() {
  motors::stop();
  controller.reset();
}

void setStatusLed() {
  if (PIN_STATUS_LED < 0) return;
  const uint32_t t = millis();
  bool on;
  if (imuFailCount > 0)                              on = (t / 100) % 2;          // fast blink
  else if (mode == Mode::Balance &&
           balanceState == BalanceState::Balancing)  on = true;                   // solid
  else if (mode == Mode::Balance)                    on = (t / 500) % 2;          // slow blink
  else                                               on = (t % 2000) < 100;       // short flash
  digitalWrite(PIN_STATUS_LED, on ? HIGH : LOW);
}

void printHelp() {
  out.print(
      "\n------------------------------- COMMANDS -------------------------------\n"
      " help            this list\n"
      " show            all current values (ready to paste into config.h)\n"
      " kp|ki|kd <v>    set PID gains              e.g.  kp 18\n"
      " sp <v>          set balance setpoint (deg) e.g.  sp -1.5\n"
      " zero            setpoint = current angle (hold the robot at its balance point)\n"
      " alpha <v>       complementary filter alpha (0.95 .. 0.999)\n"
      " minpwm <v>      motor dead-band compensation (from the ramp test)\n"
      " maxpwm <v>      output limit (lower = gentler)\n"
      " skp <v>         stage-2 anti-drift gain (0 = off)\n"
      " plot <0|1|2>    telemetry: 0 off, 1 angle/target/output, 2 + P/I/D terms\n"
      " go              balance mode (starts when you hold the robot upright)\n"
      " stop            motors off\n"
      " sensor          sensor test - prints IMU data, motors off\n"
      " motor           motor direction test     (WHEELS OFF THE GROUND!)\n"
      " ramp            slowly raise PWM to find minpwm (WHEELS OFF THE GROUND!)\n"
      " calib           re-measure gyro offset (robot must be still)\n"
      " Typing a name without a value (e.g.  kp ) prints its current value.\n"
      "------------------------------------------------------------------------\n");
}

void printSettings() {
  out.printf("\n// ---- current values (copy into config.h) ----\n");
  out.printf("constexpr float KP = %.3ff;\n", tune.kp);
  out.printf("constexpr float KI = %.3ff;\n", tune.ki);
  out.printf("constexpr float KD = %.3ff;\n", tune.kd);
  out.printf("constexpr float BALANCE_SETPOINT_DEG = %.2ff;\n", tune.setpoint);
  out.printf("constexpr float COMP_FILTER_ALPHA = %.4ff;\n", tune.alpha);
  out.printf("constexpr float MOTOR_MIN_PWM = %.1ff;\n", tune.minPwm);
  out.printf("constexpr float MOTOR_MAX_PWM = %.1ff;\n", tune.maxPwm);
  out.printf("constexpr float SPEED_KP = %.4ff;\n", tune.speedKp);
  out.printf("// mode %s | angle %.2f deg | gyro offset X%+.2f Y%+.2f Z%+.2f deg/s\n",
             modeName(mode), tiltDeg, imu::gyroBias[0], imu::gyroBias[1], imu::gyroBias[2]);
  out.printf("// loop: longest step %lu us of %lu us, late steps %lu\n",
             (unsigned long)maxLoopWorkUs, (unsigned long)LOOP_PERIOD_US, (unsigned long)loopOverruns);
  maxLoopWorkUs = 0;   // so the next "show" reports the worst case since now
}

// =====================================================================================
//  Mode changes
// =====================================================================================
void enterMode(Mode m) {
  stopMotors();
  mode = m;
  balanceState = BalanceState::WaitUpright;
  uprightTimerRunning = false;

  switch (m) {
    case Mode::Balance:
      out.printf("BALANCE mode: stand the robot upright (within %.0f deg of setpoint %.1f) and hold it still.\n",
                 START_ANGLE_DEG, tune.setpoint);
      break;
    case Mode::Stopped:
      out.println("STOPPED: motors off.");
      break;
    case Mode::SensorTest:
      sensorWindowStartMs = millis();
      sensorGyroSumDeg = 0.0f;
      sensorAccStartDeg = accTiltDeg;
      out.println("SENSOR TEST: motors off. Tip the robot FORWARD: ANGLE must go POSITIVE.");
      break;
    case Mode::MotorTest:
      motorTestStep = 0;
      motorTestStepStartMs = millis();
      out.printf("MOTOR TEST (PWM %.0f) - wheels must be OFF the ground. Type 'stop' to abort.\n",
                 MOTOR_TEST_PWM);
      out.printf("  step 1/%u: %s\n", MOTOR_TEST_STEP_COUNT, MOTOR_TEST_STEPS[0].text);
      break;
    case Mode::Ramp:
      rampPwm = 0.0f;
      rampStepStartMs = millis();
      out.println("RAMP TEST - wheels OFF the ground. Both wheels speed up slowly.");
      out.println("  Note the PWM at which BOTH wheels keep turning, then type 'stop'.");
      out.println("  Use that number for  minpwm  (and MOTOR_MIN_PWM in config.h).");
      break;
  }
}

// Blocking (~1 s), so it is only ever called with the motors stopped.
void calibrateGyro() {
  stopMotors();
  for (int attempt = 1; attempt <= 5; attempt++) {
    out.printf("Gyro calibration %d/5: keep the robot completely STILL...\n", attempt);
    float spread = 0.0f;
    if (imu::calibrateGyro(GYRO_CALIB_SAMPLES, GYRO_CALIB_MAX_SPREAD_DPS, spread)) {
      out.printf("  done. offset X%+.2f Y%+.2f Z%+.2f deg/s (noise %.2f)\n",
                 imu::gyroBias[0], imu::gyroBias[1], imu::gyroBias[2], spread);
      tiltFilter.reset();
      lastLoopUs = micros();
      return;
    }
    out.printf("  robot moved or sensor error (spread %.2f deg/s) - retrying\n", spread);
    delay(500);
  }
  out.println("  WARNING: calibration failed, using previous offsets. Type 'calib' to retry.");
  lastLoopUs = micros();
}

// =====================================================================================
//  Commands (from USB Serial or Bluetooth)
// =====================================================================================
// "name"       -> print the current value
// "name value" -> check the range, set, print
void setValue(const char* name, float& target, bool hasValue, float value, float lo, float hi) {
  if (hasValue) {
    if (value < lo || value > hi) {
      out.printf("%s must be between %g and %g\n", name, lo, hi);
      return;
    }
    target = value;
  }
  out.printf("%s = %.4f\n", name, target);
}

void handleCommand(char* line) {
  // split "word number" into cmd + value
  char* p = line;
  while (*p == ' ') p++;
  char* cmd = p;
  while (*p != '\0' && *p != ' ') {
    *p = (char)tolower((unsigned char)*p);
    p++;
  }
  if (*p != '\0') *p++ = '\0';
  char* end = nullptr;
  const float value = strtof(p, &end);
  const bool hasValue = (end != p);

  if      (!strcmp(cmd, "help") || !strcmp(cmd, "?")) printHelp();
  else if (!strcmp(cmd, "show"))   printSettings();
  else if (!strcmp(cmd, "kp"))     setValue("kp", tune.kp, hasValue, value, 0.0f, 500.0f);
  else if (!strcmp(cmd, "ki"))     setValue("ki", tune.ki, hasValue, value, 0.0f, 2000.0f);
  else if (!strcmp(cmd, "kd"))     setValue("kd", tune.kd, hasValue, value, 0.0f, 50.0f);
  else if (!strcmp(cmd, "sp"))     setValue("setpoint", tune.setpoint, hasValue, value, -30.0f, 30.0f);
  else if (!strcmp(cmd, "alpha"))  setValue("alpha", tune.alpha, hasValue, value, 0.5f, 0.9999f);
  else if (!strcmp(cmd, "skp"))    setValue("skp", tune.speedKp, hasValue, value, 0.0f, 1.0f);
  else if (!strcmp(cmd, "minpwm")) setValue("minpwm", tune.minPwm, hasValue, value, 0.0f, tune.maxPwm - 1.0f);
  else if (!strcmp(cmd, "maxpwm")) setValue("maxpwm", tune.maxPwm, hasValue, value, tune.minPwm + 1.0f, (float)PWM_MAX);
  else if (!strcmp(cmd, "zero")) {
    tune.setpoint = tiltDeg;
    out.printf("setpoint = %.2f deg (current angle)\n", tune.setpoint);
  }
  else if (!strcmp(cmd, "plot")) {
    telemetryLevel = hasValue ? (uint8_t)clampf(value, 0.0f, 2.0f) : (telemetryLevel ? 0 : 1);
    out.printf("telemetry level %u\n", telemetryLevel);
  }
  else if (!strcmp(cmd, "go"))     enterMode(Mode::Balance);
  else if (!strcmp(cmd, "stop"))   enterMode(Mode::Stopped);
  else if (!strcmp(cmd, "sensor")) enterMode(Mode::SensorTest);
  else if (!strcmp(cmd, "motor"))  enterMode(Mode::MotorTest);
  else if (!strcmp(cmd, "ramp"))   enterMode(Mode::Ramp);
  else if (!strcmp(cmd, "calib")) {
    if (mode == Mode::Balance && balanceState == BalanceState::Balancing) {
      out.println("type 'stop' first - calibration needs the robot still");
    } else {
      calibrateGyro();
    }
  }
  else out.printf("unknown command '%s' - type help\n", cmd);
}

void pollCommands() {
  if (usbLine.poll(Serial)) handleCommand(usbLine.buf);
#if USE_BLUETOOTH
  if (btLine.poll(SerialBT)) handleCommand(btLine.buf);
#endif
}

// =====================================================================================
//  The control step and the modes
// =====================================================================================
bool readSensors(float dt) {
  if (!imu::read(sample)) return false;
  const float accFwd = IMU_FWD_SIGN * sample.acc[IMU_FWD_AXIS];
  const float accUp  = IMU_UP_SIGN * sample.acc[IMU_UP_AXIS];
  accTiltDeg  = accelTiltDeg(accFwd, accUp);
  tiltRateDps = IMU_LEFT_SIGN * sample.gyro[IMU_LEFT_AXIS];
  tiltDeg     = tiltFilter.update(accTiltDeg, tiltRateDps, dt, tune.alpha);
  // smoothed over ~0.1 s so a single vibration spike cannot trigger the sideways cut-off
  sidewaysG  += 0.05f * (fabsf(sample.acc[IMU_LEFT_AXIS]) - sidewaysG);
  return true;
}

void handleImuFailure() {
  stopMotors();
  balanceState = BalanceState::WaitUpright;   // must be stood up again after a sensor fault
  uprightTimerRunning = false;
  tiltFilter.reset();
  imuFailCount++;
  if (imuFailCount == 1 || imuFailCount % 200 == 0) {
    out.printf("ERROR: MPU6050 not answering (%lu failed reads) - motors OFF. "
               "Check SDA=21, SCL=22, 3V3, GND wires.\n", (unsigned long)imuFailCount);
  }
  if (imuFailCount % 50 == 0) {               // every ~0.25 s: try to restart the sensor
    if (imu::restart()) out.println("MPU6050 restarted.");
  }
}

void runBalance(float dt) {
  if (balanceState == BalanceState::WaitUpright) {
    stopMotors();
    controller.target = tune.setpoint;
    if (fabsf(tiltDeg - tune.setpoint) < START_ANGLE_DEG && sidewaysG < SIDEWAYS_LIMIT_G) {
      if (!uprightTimerRunning) {
        uprightTimerRunning = true;
        uprightSinceMs = millis();
      } else if (millis() - uprightSinceMs >= START_HOLD_MS) {
        balanceState = BalanceState::Balancing;
        out.println("Upright - balancing! (motors ON)");
      }
    } else {
      uprightTimerRunning = false;
    }
    return;
  }

  if (fabsf(tiltDeg - tune.setpoint) > FALL_ANGLE_DEG || sidewaysG > SIDEWAYS_LIMIT_G) {
    stopMotors();
    balanceState = BalanceState::WaitUpright;
    uprightTimerRunning = false;
    out.printf("Fell over (angle %.1f deg, sideways %.2f g) - motors OFF. Stand the robot up to restart.\n",
               tiltDeg, sidewaysG);
    return;
  }

  const float cmd = controller.step(tiltDeg, tiltRateDps, dt, tune);   // control.h
  motors::drive(cmd, tune.minPwm, tune.maxPwm);
}

// Prints IMU data 10x per second and checks that gyro and accelerometer agree.
void runSensorTest(float dt) {
  sensorGyroSumDeg += tiltRateDps * dt;

  const uint32_t now = millis();
  if (now - sensorWindowStartMs >= 500) {
    const float accChange = accTiltDeg - sensorAccStartDeg;
    if (fabsf(accChange) > 10.0f) {
      if (fabsf(sensorGyroSumDeg) < 0.3f * fabsf(accChange)) {
        out.printf("!! accelerometer says the tilt changed %+.0f deg but the gyro (%c axis) only %+.0f deg."
                   " Check IMU_UP_AXIS / IMU_FWD_AXIS.\n", accChange, axisName(IMU_LEFT_AXIS), sensorGyroSumDeg);
      } else if ((accChange > 0.0f) != (sensorGyroSumDeg > 0.0f)) {
        out.printf("!! gyro and accelerometer DISAGREE (gyro %+.0f deg, accel %+.0f deg)."
                   " Check IMU_UP_SIGN / IMU_FWD_SIGN.\n", sensorGyroSumDeg, accChange);
      } else {
        out.printf("   gyro and accelerometer agree (gyro %+.0f deg, accel %+.0f deg) - OK\n",
                   sensorGyroSumDeg, accChange);
      }
    }
    sensorWindowStartMs = now;
    sensorGyroSumDeg = 0.0f;
    sensorAccStartDeg = accTiltDeg;
  }

  if (now - lastSensorPrintMs < 100) return;
  lastSensorPrintMs = now;

  // which axis does gravity act on most? (+ = reads +1 g)
  int g = 0;
  for (int i = 1; i < 3; i++) if (fabsf(sample.acc[i]) > fabsf(sample.acc[g])) g = i;

  out.printf("acc g X%+.2f Y%+.2f Z%+.2f | gyro dps X%+6.1f Y%+6.1f Z%+6.1f | gravity: %c%c | "
             "accAngle %+6.1f  rate %+6.1f  ANGLE %+6.1f\n",
             sample.acc[0], sample.acc[1], sample.acc[2],
             sample.gyro[0], sample.gyro[1], sample.gyro[2],
             sample.acc[g] >= 0.0f ? '+' : '-', axisName(g),
             accTiltDeg, tiltRateDps, tiltDeg);
}

void runMotorTest() {
  const uint32_t now = millis();
  if (now - motorTestStepStartMs >= MOTOR_TEST_STEPS[motorTestStep].ms) {
    motorTestStep++;
    motorTestStepStartMs = now;
    if (motorTestStep >= MOTOR_TEST_STEP_COUNT) {
      enterMode(Mode::Stopped);
      out.println("Motor test finished. A wheel that turned the wrong way -> flip its *_MOTOR_INVERT.");
      return;
    }
    if (MOTOR_TEST_STEPS[motorTestStep].left != 0 || MOTOR_TEST_STEPS[motorTestStep].right != 0) {
      out.printf("  step %u/%u: %s\n", motorTestStep + 1, MOTOR_TEST_STEP_COUNT,
                 MOTOR_TEST_STEPS[motorTestStep].text);
    }
  }
  motors::driveRaw(motors::left,  MOTOR_TEST_STEPS[motorTestStep].left  * MOTOR_TEST_PWM);
  motors::driveRaw(motors::right, MOTOR_TEST_STEPS[motorTestStep].right * MOTOR_TEST_PWM);
}

void runRamp() {
  const uint32_t now = millis();
  if (now - rampStepStartMs >= RAMP_STEP_MS) {
    rampStepStartMs = now;
    rampPwm += RAMP_STEP_PWM;
    if (rampPwm > PWM_MAX) {
      enterMode(Mode::Stopped);
      out.println("Ramp reached maximum. If the wheels never turned, check motor power and wiring.");
      return;
    }
    out.printf("  ramp pwm = %.0f\n", rampPwm);
  }
  motors::driveRaw(motors::left, rampPwm);
  motors::driveRaw(motors::right, rampPwm);
}

void printTelemetry() {
  if (telemetryLevel == 0 || mode != Mode::Balance) return;
  const uint32_t now = millis();
  if (now - lastTelemetryMs < TELEMETRY_PERIOD_MS) return;
  lastTelemetryMs = now;
  // "name:value" pairs -> the Arduino IDE Serial Plotter draws one line per name.
  // The output is divided by 10 so it fits on the same scale as the angle.
  const BalanceController& c = controller;
  if (telemetryLevel == 1) {
    out.printf("angle:%.2f,target:%.2f,output_div10:%.1f\n", tiltDeg, c.target, c.output * 0.1f);
  } else {
    out.printf("angle:%.2f,target:%.2f,output_div10:%.1f,P_div10:%.1f,I_div10:%.1f,D_div10:%.1f\n",
               tiltDeg, c.target, c.output * 0.1f,
               c.pid.pTerm * 0.1f, c.pid.iTerm * 0.1f, c.pid.dTerm * 0.1f);
  }
}

// =====================================================================================
//  setup() and loop()
// =====================================================================================
void setup() {
  motors::forcePinsLow();                 // FIRST: motors must not move while booting
  if (PIN_STATUS_LED >= 0) pinMode(PIN_STATUS_LED, OUTPUT);

  Serial.setTxBufferSize(4096);           // big buffer: printing never stalls the control loop
  Serial.begin(SERIAL_BAUD);
  delay(200);
  out.printf("\n\n=== BalanceX self-balancing robot ===\n");
  out.printf("ESP32 Arduino core %d.%d.%d | last reset: %s\n",
             ESP_ARDUINO_VERSION_MAJOR, ESP_ARDUINO_VERSION_MINOR, ESP_ARDUINO_VERSION_PATCH,
             resetReasonText(esp_reset_reason()));
  if (esp_reset_reason() == ESP_RST_BROWNOUT) {
    out.println("!! The ESP32 restarted because its supply voltage dropped. Check the LM2596 output,"
                " battery charge and the ground wiring (see ERROR_LOG).");
  }

#if USE_BLUETOOTH
  if (SerialBT.begin(BT_DEVICE_NAME)) {
    out.printf("Bluetooth ready: pair your phone with \"%s\"\n", BT_DEVICE_NAME);
  } else {
    out.println("Bluetooth failed to start - continuing with USB only");
  }
#endif

  if (!motors::begin()) {
    out.println("ERROR: motor PWM setup failed. PWM_FREQ_HZ * 2^PWM_RES_BITS must be <= 80 MHz.");
  }

  out.printf("IMU mounting: up = %c%c, front = %c%c  ->  tilt rate from gyro %c%c\n",
             IMU_UP_SIGN > 0 ? '+' : '-', axisName(IMU_UP_AXIS),
             IMU_FWD_SIGN > 0 ? '+' : '-', axisName(IMU_FWD_AXIS),
             IMU_LEFT_SIGN > 0 ? '+' : '-', axisName(IMU_LEFT_AXIS));

  while (!imu::begin()) {                 // nothing is safe without the sensor: wait for it
    out.printf("MPU6050 not found at 0x%02X - check wiring (SDA=21, SCL=22, VCC=3V3, GND, AD0=GND)."
               " Retrying...\n", MPU_I2C_ADDR);
    for (int i = 0; i < 10; i++) {
      if (PIN_STATUS_LED >= 0) digitalWrite(PIN_STATUS_LED, i % 2);
      delay(100);
    }
    Wire.end();
  }
  out.printf("MPU6050 found (WHO_AM_I = 0x%02X)%s\n", imu::whoAmI,
             imu::whoAmI == 0x68 ? "" : " - not 0x68, probably a clone/compatible chip; usually fine");

  calibrateGyro();
  printHelp();
  enterMode(mode);
  lastLoopUs = micros();
}

void loop() {
  pollCommands();

  // Run the control step exactly every LOOP_PERIOD_US.
  const uint32_t nowUs = micros();
  const uint32_t elapsedUs = nowUs - lastLoopUs;   // unsigned maths: correct even when micros() wraps
  if (elapsedUs < LOOP_PERIOD_US) return;
  lastLoopUs = nowUs;
  if (elapsedUs > LOOP_PERIOD_US + LOOP_PERIOD_US / 2) loopOverruns++;
  float dt = elapsedUs * 1e-6f;
  if (dt > 0.05f) dt = LOOP_PERIOD_US * 1e-6f;     // after a long pause, do not integrate a huge step

  if (!readSensors(dt)) {
    handleImuFailure();
    setStatusLed();
    return;
  }
  if (imuFailCount > 0) {
    out.printf("MPU6050 reading again after %lu failed reads.\n", (unsigned long)imuFailCount);
    imuFailCount = 0;
  }

  switch (mode) {
    case Mode::Balance:    runBalance(dt);    break;
    case Mode::Stopped:    stopMotors();      break;
    case Mode::SensorTest: runSensorTest(dt); break;
    case Mode::MotorTest:  runMotorTest();    break;
    case Mode::Ramp:       runRamp();         break;
  }

  printTelemetry();
  setStatusLed();

  const uint32_t workUs = micros() - nowUs;
  if (workUs > maxLoopWorkUs) maxLoopWorkUs = workUs;
}
