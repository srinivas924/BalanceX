// =====================================================================================
//  uno_sim_test.cpp - runs the Arduino Uno controller (firmware/BalanceX_Uno/balance.h)
//  against a physics model of the robot on a PC. OPTIONAL, no robot needed.
//
//    g++ -std=c++17 -O2 -o uno_sim_test tests/uno_sim_test.cpp && ./uno_sim_test
//    ./uno_sim_test search      (also tries the gains 20-30 % up and down; a few minutes)
//
//  The model: body + two wheels (pitch, forward travel, turning), one DC gear motor per
//  wheel with back-EMF, starting friction calibrated from your measured LEFT/RIGHT_MIN_PWM
//  (80 / 100), gearbox play (backlash), L298N voltage drop, MPU6050 low-pass delay and
//  noise, encoder counts at 1325.1 per revolution, 200 Hz control with 1 ms latency.
//  Each run: held upright, let go, balances 3 s, drives forward, gets a push (body starts
//  tipping at 40 deg/s) after 13 s, keeps driving.
//
//  Test 1: sign conventions of the angle / rate formulas against your calibration.
//  Test 2: friction compensation is continuous (no jump from 0 to PWM 80/100).
//  Test 3: your previous sketch vs this one on the same simulated robot.
//  Test 4: config.h gains across the values nobody has measured yet (weight, centre of
//          mass height, motor speed/torque, gearbox play, battery, calibration error).
//  Test 5: safety: a reversed motor stops the robot; a fall switches the motors off.
//
//  Weight, centre-of-mass height, motor torque and gearbox play are GUESSES (swept in
//  Test 4). The simulation checks the logic and gives sensible starting gains; the real
//  robot sets the final ones.
// =====================================================================================
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <random>

#include "../firmware/BalanceX_Uno/config.h"
#include "../firmware/BalanceX_Uno/balance.h"

static int failures = 0;
#define CHECK(cond, ...)                 \
  do {                                   \
    if (!(cond)) {                       \
      std::printf("FAIL: " __VA_ARGS__); \
      std::printf("\n");                 \
      failures++;                        \
    }                                    \
  } while (0)

constexpr double PI = 3.14159265358979;
constexpr double G = 9.81;
constexpr double DEG = PI / 180.0;

// ------------------------------------------------------------------ the robot (guesses marked)
struct Plant {
  double bodyMass = 1.2;        // kg without wheels                          - guess
  double comHeight = 0.10;      // m, centre of mass above the axle           - guess
  double bodyHeight = 0.20;     // m, for the body's own inertia              - guess
  double wheelMass = 0.10;      // kg, both wheels                            - guess
  double wheelRadius = 0.035;   // m (70 mm wheels)
  double track = 0.17;          // m between the wheels                      - guess
  double noLoadRpm = 200.0;     // wheel rpm at 12 V                          - guess
  double stallTorque = 0.8;     // N m per motor at 12 V, at the wheel        - guess
  double rotorInertia = 0.0008; // kg m^2 per motor seen at the wheel (rotor x ratio^2) - guess
  double gearStiffness = 60.0;  // N m/rad                                    - guess
  double backlashDeg = 1.5;     // total gearbox play at the wheel            - guess
  double battery = 11.8;        // V (3S LiPo: 12.6 full, 11.1 nominal)
  double driverDrop = 2.0;      // V lost inside the L298N
  double leftStartPwm = LEFT_MIN_PWM, rightStartPwm = RIGHT_MIN_PWM;   // measured
  double movingFriction = 0.7;  // friction once turning / friction at start  - guess
  double sensorHeight = 0.08;   // m, MPU6050 above the axle                  - guess
  double zeroErrorDeg = 0.0;    // calibration done this far from the true balance point
  double gyroBiasDps = 0.3;     // gyro offset left after calibration
  int    leftMotorSignError = 1;   // -1 = this motor is wired the other way round
  int    rightMotorSignError = 1;
  int    imuSignError = 1;          // -1 = the sensor is mounted the other way round
};

// ------------------------------------------------------------------ physics + sensors
class Sim {
 public:
  explicit Sim(const Plant& pl, unsigned seed) : p(pl), rng(seed), accN(0.0, 0.01), gyroN(0.0, 0.1) {
    M = p.bodyMass;
    l = p.comHeight;
    Ib = M * p.bodyHeight * p.bodyHeight / 12.0;
    mw = p.wheelMass;
    r = p.wheelRadius;
    Jw = 0.5 * mw * r * r;
    Jz = M * (0.15 * 0.15 + 0.06 * 0.06) / 12.0;
    w0 = p.noLoadRpm * 2.0 * PI / 60.0;
    // starting friction = torque the motor makes at the measured start PWM (wheels lifted)
    tsL = p.stallTorque * (p.leftStartPwm / 255.0 * (p.battery - p.driverDrop)) / 12.0;
    tsR = p.stallTorque * (p.rightStartPwm / 255.0 * (p.battery - p.driverDrop)) / 12.0;
  }

  // state (public for the test scenarios)
  double phi = 0, phid = 0;   // average wheel angle (rad, + = forward)
  double th = 0, thd = 0;     // body pitch (rad, + = leaning forward)
  double del = 0, deld = 0;   // half the difference of the wheel angles (right - left)
  double mL = 0, mLd = 0, mR = 0, mRd = 0;   // motor output shafts relative to the body
  double pushForce = 0;       // N, horizontal, applied at 0.15 m above the axle
  double voltsL = 0, voltsR = 0;

  void setPwm(int left, int right) {   // forward-positive PWM, as the robot's wheels see it
    voltsL = p.leftMotorSignError * left / 255.0 * (p.battery - p.driverDrop);
    voltsR = p.rightMotorSignError * right / 255.0 * (p.battery - p.driverDrop);
  }

  // Hold the body still at a given angle (someone holding the robot before it starts).
  void hold(double angleRad) {
    th = angleRad;
    thd = phid = deld = mLd = mRd = 0;
    mL = (phi - del) - th;
    mR = (phi + del) - th;
    phidd = thdd = 0;
  }

  void advance(double h) {
    const double gap = 0.5 * p.backlashDeg * DEG;
    const double k = p.gearStiffness, c = 2.0 * 0.3 * std::sqrt(k * p.rotorInertia);
    const double phiL = phi - del, phiR = phi + del, phiLd = phid - deld, phiRd = phid + deld;
    const double ttL = gear(mL + th - phiL, mLd + thd - phiLd, gap, k, c);
    const double ttR = gear(mR + th - phiR, mRd + thd - phiRd, gap, k, c);

    // motor shafts: motor torque - gearbox friction - torque passed on to the wheel
    const double tmL = p.stallTorque * (voltsL / 12.0 - mLd / w0);
    const double tmR = p.stallTorque * (voltsR / 12.0 - mRd / w0);
    const double mLdd = (tmL - friction(mLd, tsL) - ttL) / p.rotorInertia;
    const double mRdd = (tmR - friction(mRd, tsR) - ttR) / p.rotorInertia;

    // body + wheels (rolling without slipping)
    const double hp = 0.15;
    const double a11 = (M + mw) * r * r + Jw, a12 = M * l * r * std::cos(th), a22 = Ib + M * l * l;
    const double b1 = ttL + ttR + M * l * r * std::sin(th) * thd * thd + pushForce * r;
    const double b2 = M * G * l * std::sin(th) - (ttL + ttR) + pushForce * hp * std::cos(th);
    const double D = a11 * a22 - a12 * a12;
    phidd = (b1 * a22 - a12 * b2) / D;
    thdd = (a11 * b2 - a12 * b1) / D;
    const double deldd = (ttR - ttL) / (Jw + mw * r * r + Jz * std::pow(2.0 * r / p.track, 2));

    phid += phidd * h;  phi += phid * h;
    thd += thdd * h;    th += thd * h;
    deld += deldd * h;  del += deld * h;
    mLd += mLdd * h;    mL += mLd * h;
    mRd += mRdd * h;    mR += mRd * h;

    // MPU6050 low-pass filter (DLPF 3: about 4.8 ms delay), applied to the true signals
    double accFwd, accUp;
    trueAccel(accFwd, accUp);
    const double a = h / (0.0048 + h);
    fAx += a * (p.imuSignError * -PITCH_SIGN * accFwd - fAx);
    fAz += a * (accUp - fAz);
    fGy += a * (p.imuSignError * thd / DEG / RATE_SIGN - fGy);
  }

  // What the firmware gets from the MPU6050 and the encoder interrupts.
  bx::Sample read() {
    bx::Sample s;
    s.axG = (float)(fAx + accN(rng));
    s.azG = (float)(fAz + accN(rng));
    s.gyDps = (float)(fGy + gyroN(rng) + p.gyroBiasDps);
    // The encoders sit on the motor shafts: they count motor turns relative to the body,
    // on the motor side of the gearbox play.
    s.leftCount = (long)std::floor(mL / (2 * PI) * LEFT_COUNTS_PER_WHEEL_REV) * LEFT_ENCODER_SIGN;
    s.rightCount = (long)std::floor(mR / (2 * PI) * RIGHT_COUNTS_PER_WHEEL_REV) * RIGHT_ENCODER_SIGN;
    return s;
  }

  void settleSensor() {   // start the low-pass filter at the current reading
    double accFwd, accUp;
    trueAccel(accFwd, accUp);
    fAx = p.imuSignError * -PITCH_SIGN * accFwd;
    fAz = accUp;
    fGy = p.imuSignError * thd / DEG / RATE_SIGN;
  }

  // Horizontal force for 0.1 s at 0.15 m that starts the body tipping at rateDps,
  // so light and heavy robot variants get a push of the same severity.
  double pushForTipRate(double rateDps) const { return rateDps * DEG * (Ib + M * l * l) / (0.15 * 0.1); }

  double yawDeg() const { return 2.0 * r * del / p.track / DEG; }
  double travel() const { return phi * r; }
  double rpm() const { return phid * 60.0 / (2 * PI); }

 private:
  Plant p;
  std::mt19937 rng;
  std::normal_distribution<double> accN, gyroN;
  double M, l, Ib, mw, r, Jw, Jz, w0, tsL, tsR;
  double phidd = 0, thdd = 0, fAx = 0, fAz = 1, fGy = 0;

  // Gearbox play: no torque inside the gap, a stiff spring outside it.
  static double gear(double defl, double rate, double gap, double k, double c) {
    if (defl > gap) return k * (defl - gap) + c * rate;
    if (defl < -gap) return k * (defl + gap) + c * rate;
    return 0.0;
  }
  // Starting friction falling to moving friction, smoothed around zero speed.
  double friction(double w, double ts) const {
    const double tk = p.movingFriction * ts;
    return (tk + (ts - tk) * std::exp(-(w / 0.5) * (w / 0.5))) * std::tanh(w / 0.1);
  }
  // Specific force along the body's forward and up axes, in g.
  void trueAccel(double& accFwd, double& accUp) const {
    const double hs = p.sensorHeight;
    const double ax = r * phidd + hs * (thdd * std::cos(th) - thd * thd * std::sin(th));
    const double az = -hs * (thdd * std::sin(th) + thd * thd * std::cos(th)) + G;
    accFwd = (ax * std::cos(th) - az * std::sin(th)) / G;
    accUp = (ax * std::sin(th) + az * std::cos(th)) / G;
  }
};

// ------------------------------------------------------------------ the two controllers
// Common interface: one control step -> forward-positive PWM per wheel.
struct Brain {
  virtual ~Brain() {}
  virtual void step(const bx::Sample& s, float dt, uint32_t nowMs, int& left, int& right) = 0;
  virtual bool armed() const = 0;
  virtual bool faulted() const { return false; }
  // Start-up calibration done while held zeroErrorDeg away from the true balance point.
  virtual void calibrate(double zeroErrorDeg) = 0;
};

// This firmware: balance.h, plus the same PWM rounding as BalanceX_Uno.ino.
struct NewBrain : Brain {
  bx::Controller c;
  bool dirFault = false, fell = false;
  void step(const bx::Sample& s, float dt, uint32_t nowMs, int& left, int& right) override {
    const bx::Event e = c.step(s, dt, nowMs);
    if (e == bx::EV_DIR_LEFT || e == bx::EV_DIR_RIGHT) dirFault = true;
    if (e == bx::EV_FELL) fell = true;
    left = (int)lroundf(c.leftPwm);
    right = (int)lroundf(c.rightPwm);
  }
  bool armed() const override { return c.mode == bx::Mode::Balancing; }
  bool faulted() const override { return dirFault; }
  void calibrate(double zeroErrorDeg) override { c.zeroDeg = (float)zeroErrorDeg; }
};

// Your previous sketch (ref_code), ported line by line: same signs, gains, filters,
// outputToPWM mapping, 25 Hz outer loop and RPM sync. Assumes, like this firmware, that a
// positive motor command turns the wheel forward.
struct OldBrain : Brain {
  bool outerLoop = true;   // false = its speed/position loop switched off (target angle 0)
  float zeroUser = 0;      // its calibrated angleOffsetDeg
  float angle = 0, rateF = 0;
  bool started = false, isArmed = false, everArmed = false, refReady = false;
  uint32_t uprightSince = 0;
  uint32_t lastOuterMs = 0;
  long prevL = 0, prevR = 0;
  float lRpm = 0, rRpm = 0, avgPos = 0, posTarget = 0, targetAngle = 0;

  static int toPwm(float out, int minPwm) {
    const float mag = std::fabs(out);
    if (mag <= 2.0f) return 0;
    const float ratio = std::min(1.0f, (mag - 2.0f) / (40.0f - 2.0f));
    int v = (int)lroundf(minPwm + ratio * (180 - minPwm));
    return out < 0 ? -v : v;
  }
  void step(const bx::Sample& s, float dt, uint32_t nowMs, int& left, int& right) override {
    const float accel = ANGLE_SIGN * std::atan2(s.axG, s.azG) * 57.29578f - zeroUser;
    const float rate = GYRO_SIGN * s.gyDps;
    rateF = 0.4f * rate + 0.6f * rateF;
    if (!started) { angle = accel; started = true; }
    angle = 0.985f * (angle + rateF * dt) + 0.015f * accel;

    // 25 Hz encoder / outer loop
    const long l = s.leftCount * LEFT_ENCODER_SIGN, r = s.rightCount * RIGHT_ENCODER_SIGN;
    if (nowMs - lastOuterMs >= 40) {
      const float odt = (nowMs - lastOuterMs) * 0.001f;
      lastOuterMs = nowMs;
      avgPos = 0.5f * (l / LEFT_COUNTS_PER_WHEEL_REV + r / RIGHT_COUNTS_PER_WHEEL_REV);
      if (!refReady) { prevL = l; prevR = r; refReady = true; }
      lRpm = (l - prevL) / LEFT_COUNTS_PER_WHEEL_REV * 60.0f / odt;
      rRpm = (r - prevR) / RIGHT_COUNTS_PER_WHEEL_REV * 60.0f / odt;
      prevL = l; prevR = r;
      if (isArmed && outerLoop) {
        posTarget += 2.0f * odt / 60.0f;
        const float tRpm = bx::clampf(2.0f + 1.0f * (posTarget - avgPos), -5.0f, 5.0f);
        targetAngle = bx::clampf(0.2f * (tRpm - 0.5f * (lRpm + rRpm)), -1.5f, 1.5f);
      } else {
        targetAngle = 0;
      }
    }
    if (isArmed && std::fabs(angle) >= 25.0f) isArmed = false;
    if (!isArmed && !everArmed) {   // auto-start (only once in the simulation)
      if (std::fabs(angle) <= 5.0f && std::fabs(rateF) <= 15.0f) {
        if (uprightSince == 0) uprightSince = nowMs ? nowMs : 1;
        if (nowMs - uprightSince >= 800) {
          isArmed = everArmed = true;
          posTarget = avgPos;
        }
      } else {
        uprightSince = 0;
      }
    }
    if (!isArmed) { left = right = 0; return; }
    const float out = BALANCE_SIGN * (8.0f * (angle - targetAngle) + 0.75f * rateF);
    const int lb = toPwm(out, 80), rb = toPwm(out, 100);
    const float trim = (lb || rb) ? bx::clampf(0.2f * (lRpm - rRpm), -8.0f, 8.0f) : 0.0f;
    left = std::max(-180, std::min(180, lb - (int)lroundf(trim)));
    right = std::max(-180, std::min(180, rb + (int)lroundf(trim)));
  }
  bool armed() const override { return isArmed; }
  // its angle is BALANCE_SIGN * (forward-positive pitch), see Test 1
  void calibrate(double zeroErrorDeg) override { zeroUser = (float)(BALANCE_SIGN * zeroErrorDeg); }
};

// ------------------------------------------------------------------ one complete run
struct RunResult {
  bool armed = false, fell = false, dirFault = false;
  double pitchRmsHold = 0;   // deg, balancing in place
  double pitchRmsCruise = 0; // deg, driving
  double flipsPerSec = 0;    // motor direction reversals per second while driving (vibration)
  double pwmStepMean = 0;    // average PWM change from one 5 ms step to the next
  double cruiseRpm = 0;      // average wheel speed while driving
  double yawDeg = 0;         // heading change over the whole run
  double maxPitchAfterPush = 0;
  double balancedSeconds = 0;  // from letting go until the end or the fall
  double flipsAllPerSec = 0;   // motor direction reversals per second, whole run (vibration)
};

struct Timing {
  float latencyS = 0.001f;         // sensor read -> motor update
  uint32_t stallEverySteps = 0;    // 0 = never; else every Nth step the loop is late ...
  float stallS = 0.0f;             // ... by this long (blocking Serial.print)
};

// Optional: called after every control step (used for printing traces while debugging).
static void (*traceHook)(double t, const Sim& sim, Brain& brain, int left, int right) = nullptr;

static RunResult run(const Plant& pl, Brain& brain, const Timing& tm, double seconds,
                     bool push, unsigned seed) {
  Sim sim(pl, seed);
  RunResult res;
  const double h = 0.0001;   // physics step 0.1 ms
  double t = 0, nextCtrl = 0, applyAt = -1;
  int pendL = 0, pendR = 0, lastL = 0, lastR = 0;
  uint32_t steps = 0;
  double lastCtrlT = -0.005;
  double armedAt = -1;
  // statistics
  double hs = 0, hss = 0, cs = 0, css = 0, rpmSum = 0;
  long hn = 0, cn = 0, flips = 0, pwmSteps = 0, flipsAll = 0, armedSteps = 0;
  double pwmStepSum = 0;

  brain.calibrate(pl.zeroErrorDeg);
  const double holdAngle = (pl.zeroErrorDeg + 1.0) * DEG;   // then held roughly upright
  sim.hold(holdAngle);
  sim.settleSensor();

  while (t < seconds) {
    if (t >= nextCtrl) {
      const float dt = (float)(t - lastCtrlT);
      lastCtrlT = t;
      int l, r;
      brain.step(sim.read(), dt, (uint32_t)(t * 1000.0), l, r);
      pendL = l; pendR = r; applyAt = t + tm.latencyS;
      if (traceHook) traceHook(t, sim, brain, l, r);
      steps++;
      nextCtrl += 0.005;
      if (tm.stallEverySteps && steps % tm.stallEverySteps == 0) nextCtrl += tm.stallS;

      const bool isArmed = brain.armed();
      if (isArmed && armedAt < 0) { armedAt = t; res.armed = true; }
      if (armedAt >= 0 && !isArmed) {
        res.fell = !brain.faulted();
        res.dirFault = brain.faulted();
        break;
      }
      if (armedAt >= 0) {
        const double since = t - armedAt;
        res.balancedSeconds = since;
        armedSteps++;
        if ((l > 0 && lastL < 0) || (l < 0 && lastL > 0)) flipsAll++;
        if ((r > 0 && lastR < 0) || (r < 0 && lastR > 0)) flipsAll++;
        const double pitch = sim.th / DEG;
        if (since > 0.5 && since < 2.5) { hs += pitch; hss += pitch * pitch; hn++; }
        const bool cruise = since > 12.0 && (!push || since < 13.0 || since > 17.0);
        if (cruise) {
          cs += pitch; css += pitch * pitch; cn++;
          rpmSum += sim.rpm();
          if ((l > 0 && lastL < 0) || (l < 0 && lastL > 0)) flips++;
          if ((r > 0 && lastR < 0) || (r < 0 && lastR > 0)) flips++;
          pwmStepSum += std::abs(l - lastL) + std::abs(r - lastR);
          pwmSteps += 2;
        }
        if (push && since > 13.0 && since < 17.0)
          res.maxPitchAfterPush = std::max(res.maxPitchAfterPush, std::fabs(pitch));
        sim.pushForce = (push && since > 13.0 && since < 13.1) ? sim.pushForTipRate(40.0) : 0.0;
      }
      lastL = l; lastR = r;
    }
    if (applyAt >= 0 && t >= applyAt) { sim.setPwm(pendL, pendR); applyAt = -1; }
    if (armedAt < 0) sim.hold(holdAngle); else sim.advance(h);
    t += h;
  }
  if (hn) res.pitchRmsHold = std::sqrt(std::max(0.0, hss / hn - (hs / hn) * (hs / hn)));
  if (cn) {
    res.pitchRmsCruise = std::sqrt(std::max(0.0, css / cn - (cs / cn) * (cs / cn)));
    res.cruiseRpm = rpmSum / cn;
    res.flipsPerSec = flips / (cn * 0.005) / 2.0;   // per motor
  }
  if (pwmSteps) res.pwmStepMean = pwmStepSum / pwmSteps;
  if (armedSteps) res.flipsAllPerSec = flipsAll / (armedSteps * 0.005) / 2.0;
  res.yawDeg = sim.yawDeg();
  return res;
}

static Timing newTiming() { return Timing(); }
static Timing oldTiming() {
  // previous sketch: Adafruit library at 100 kHz I2C (~3 ms from reading to motors), and a
  // 150-character debug line every 100 ms through a 64-byte serial buffer (~9 ms blocked)
  Timing t;
  t.latencyS = 0.003f;
  t.stallEverySteps = 20;
  t.stallS = 0.009f;
  return t;
}

static bool good(const RunResult& r, double wantRpm) {
  return r.armed && !r.fell && !r.dirFault && r.pitchRmsCruise < 1.0 &&
         std::fabs(r.cruiseRpm - wantRpm) < 0.25 * wantRpm + 1.0 && std::fabs(r.yawDeg) < 5.0;
}

static void print(const char* name, const RunResult& r) {
  const char* status = !r.armed ? "never started" : (r.fell ? "FELL" : (r.dirFault ? "STOPPED" : "ok"));
  std::printf("  %-33s %-5s upright %4.1f s | wheel reversals %5.1f /s | pitch rms %.2f deg",
              name, status, r.balancedSeconds, r.flipsAllPerSec, r.pitchRmsHold);
  if (r.armed && !r.fell && !r.dirFault)
    std::printf(" | speed %4.1f rpm | heading %+4.1f deg | push %4.1f deg", r.cruiseRpm, r.yawDeg, r.maxPitchAfterPush);
  std::printf("\n");
}

// ------------------------------------------------------------------ tests
static void testSigns() {
  std::printf("Test 1: sign conventions\n");
  // Robot leaning forward by 10 deg: forward axis reads -sin, up axis reads cos (see Sim).
  // With your calibration the raw sensor X reading is -PITCH_SIGN * that.
  const float t = 10.0f * (float)DEG;
  const float pitch = bx::accelPitchDeg(-PITCH_SIGN * -std::sin(t), std::cos(t));
  CHECK(std::fabs(pitch - 10.0f) < 0.01f, "accelPitchDeg gave %.3f for a 10 deg forward lean", pitch);
  CHECK(bx::gyroPitchRateDps(37.0f / RATE_SIGN) == 37.0f, "gyroPitchRateDps sign");
  // The forward-positive pitch must equal BALANCE_SIGN * your old angle, so that
  // KP * pitch reproduces your old (working) balance direction.
  const float ax = 0.17f, az = 0.98f;
  const float oldAngle = ANGLE_SIGN * std::atan2(ax, az) * 57.29578f;
  CHECK(std::fabs(bx::accelPitchDeg(ax, az) - BALANCE_SIGN * oldAngle) < 1e-4f, "pitch != BALANCE_SIGN * old angle");
  CHECK(bx::gyroPitchRateDps(12.0f) == BALANCE_SIGN * GYRO_SIGN * 12.0f, "rate != BALANCE_SIGN * old rate");
  std::printf("  ok\n");
}

static void testFrictionCompensation() {
  std::printf("Test 2: friction compensation\n");
  // Motor still: sweeping the command slowly must never make the PWM jump.
  float prev = 0.0f, maxJump = 0.0f;
  for (float u = -60.0f; u <= 60.0f; u += 0.01f) {
    const float v = bx::frictionCompensate(u, 0.0f, RIGHT_MIN_PWM, FRICTION_COMP_PERCENT, MAX_PWM);
    if (u > -60.0f) maxJump = std::max(maxJump, std::fabs(v - prev));
    prev = v;
  }
  CHECK(maxJump < 1.0f, "output jumps by %.1f PWM", maxJump);
  CHECK(bx::frictionCompensate(0.0f, 0.0f, LEFT_MIN_PWM, 70.0f, 255.0f) == 0.0f, "zero command must give zero PWM");
  const float big = bx::frictionCompensate(100.0f, 0.0f, RIGHT_MIN_PWM, 70.0f, 255.0f);
  CHECK(std::fabs(big - 170.0f) < 0.01f, "100 + 70%% of 100 should be 170, got %.2f", big);
  CHECK(bx::frictionCompensate(-1000.0f, 0.0f, LEFT_MIN_PWM, 70.0f, 200.0f) == -200.0f, "clamp to maxPwm");
  // Motor turning forward, command wobbling slightly negative: the PWM must stay forward
  // (it only brakes a little), instead of flipping to -56 like a sign-based compensation.
  const float wobble = bx::frictionCompensate(-10.0f, 20.0f, LEFT_MIN_PWM, 70.0f, 255.0f);
  CHECK(std::fabs(wobble - 46.0f) < 0.01f, "turning motor, small negative command: expected 46, got %.2f", wobble);
  std::printf("  largest step for a 0.01 change of command: %.2f PWM (previous sketch: 80 / 100)\n", maxJump);
}

// The values nobody has measured yet: 2 x 2 x 2 x 2 x 2 x 2 = 64 robot variants.
template <typename F>
static void forEachVariant(F f) {
  unsigned seed = 7;
  for (double mass : {0.8, 1.6})
    for (double com : {0.06, 0.14})
      for (int motor = 0; motor < 2; motor++)
        for (double backlash : {0.5, 3.0})
          for (double battery : {11.1, 12.4})
            for (double zeroErr : {-2.0, 2.0}) {
              Plant pl;
              pl.bodyMass = mass;
              pl.comHeight = com;
              pl.noLoadRpm = motor ? 300.0 : 150.0;     // fast + weak, or slow + strong
              pl.stallTorque = motor ? 0.5 : 1.0;
              pl.backlashDeg = backlash;
              pl.battery = battery;
              pl.zeroErrorDeg = zeroErr;
              f(pl, seed++);
            }
}

static void testUnknowns(const bx::Settings* gains, bool verbose, int& okOut, int& totalOut) {
  int ok = 0, total = 0;
  forEachVariant([&](const Plant& pl, unsigned seed) {
    NewBrain nb;
    if (gains) nb.c.set = *gains;
    const RunResult r = run(pl, nb, newTiming(), 22.0, true, seed);
    const bool g = good(r, nb.c.set.forwardRpm);
    ok += g;
    total++;
    if (!g && verbose) {
      char name[96];
      std::snprintf(name, sizeof(name), "%.1fkg com%.2f %s bl%.1f %.1fV z%+.0f", pl.bodyMass, pl.comHeight,
                    pl.noLoadRpm > 200.0 ? "fast" : "slow", pl.backlashDeg, pl.battery, pl.zeroErrorDeg);
      print(name, r);
    }
  });
  okOut = ok;
  totalOut = total;
}

static void testOldVsNew() {
  std::printf("Test 3: previous sketch vs this firmware, same simulated robot (nominal guesses)\n");
  std::printf("  wheel reversals = motor direction changes per second (vibration); pitch rms = while\n");
  std::printf("  balancing in place; push = largest lean after the push (body tipping at 40 deg/s)\n");
  Plant pl;
  OldBrain old;
  print("previous sketch", run(pl, old, oldTiming(), 25.0, true, 1));
  OldBrain oldNoStall;
  print("previous, no Serial stall", run(pl, oldNoStall, newTiming(), 25.0, true, 1));
  OldBrain oldNoOuter;
  oldNoOuter.outerLoop = false;
  print("previous, no stall, no outer loop", run(pl, oldNoOuter, newTiming(), 25.0, true, 1));
  NewBrain nb;
  const RunResult r = run(pl, nb, newTiming(), 25.0, true, 1);
  print("this firmware", r);
  CHECK(good(r, FORWARD_SPEED_RPM), "new firmware did not balance / drive straight on the nominal robot");

  // The previous sketch on all the Test 4 robot variants (no push).
  int up = 0, total = 0;
  double seconds = 0.0;
  forEachVariant([&](const Plant& v, unsigned seed) {
    OldBrain b;
    const RunResult o = run(v, b, oldTiming(), 22.0, false, seed);
    up += o.armed && !o.fell;
    seconds += o.balancedSeconds;
    total++;
  });
  std::printf("  previous sketch on the %d Test 4 variants: stayed up in %d, on average fell after %.1f s\n",
              total, up, seconds / total);
}

static void testSafety() {
  std::printf("Test 5: safety\n");
  // Left motor wired the other way round: the robot must end up with the motors off.
  Plant pl;
  pl.leftMotorSignError = -1;
  NewBrain nb;
  const RunResult r = run(pl, nb, newTiming(), 10.0, false, 3);
  std::printf("  left motor reversed           : %s\n",
              r.dirFault ? "stopped by the direction check" : (r.fell ? "fell, motors off" : "NOT STOPPED"));
  CHECK(r.dirFault || r.fell, "a reversed motor was not stopped");

  // Both motors AND the sensor the other way round: the old balance direction still works,
  // but the encoders disagree with the motors (the case the  t  test is for).
  Plant pl3;
  pl3.leftMotorSignError = pl3.rightMotorSignError = pl3.imuSignError = -1;
  NewBrain nb3;
  const RunResult r3 = run(pl3, nb3, newTiming(), 15.0, false, 4);
  std::printf("  both motors + sensor reversed : %s\n",
              r3.dirFault ? "stopped by the direction check" : (r3.fell ? "fell, motors off" : "NOT STOPPED"));
  CHECK(r3.dirFault || r3.fell, "both motors reversed was not stopped");

  // A big shove must end in "fell, motors off", not in the motors running on.
  Plant pl2;
  Sim sim(pl2, 5);
  bx::Controller c;
  sim.hold(1.0 * DEG);
  sim.settleSensor();
  uint32_t ms = 0;
  bool armed = false, fell = false;
  for (int i = 0; i < 4000 && !fell; i++, ms += 5) {
    const bx::Event e = c.step(sim.read(), 0.005f, ms);
    armed = armed || (e == bx::EV_ARMED);
    fell = (e == bx::EV_FELL);
    sim.setPwm((int)lroundf(c.leftPwm), (int)lroundf(c.rightPwm));
    sim.pushForce = (armed && i % 1000 > 900) ? 25.0 : 0.0;   // a hard kick
    for (int k = 0; k < 50; k++) armed ? sim.advance(0.0001) : sim.hold(1.0 * DEG);
  }
  std::printf("  hard kick                     : %s\n", fell && c.leftPwm == 0 && c.rightPwm == 0 ? "fell, motors off" : "NOT HANDLED");
  CHECK(fell && c.leftPwm == 0.0f && c.rightPwm == 0.0f, "fall did not switch the motors off");

  // A control step that comes very late must switch the motors off.
  bx::Controller c2;
  bx::Sample s{0.0f, 1.0f, 0.0f, 0, 0};
  for (uint32_t t = 0; t < 2000; t += 5) c2.step(s, 0.005f, t);
  CHECK(c2.mode == bx::Mode::Balancing, "did not arm on a still, upright reading");
  CHECK(c2.step(s, 0.2f, 2200) == bx::EV_TIMING && c2.leftPwm == 0.0f, "late step did not stop the motors");
  std::printf("  late control step             : motors off\n");
}

// Optional (./uno_sim_test search): how the result depends on the four main gains.
static void searchGains() {
  std::printf("Gain search around the config.h values (each line: variants that passed of 64)\n");
  for (float kp : {0.8f * KP, KP, 1.2f * KP})
    for (float kd : {0.8f * KD, KD, 1.2f * KD})
      for (float skp : {0.8f * SPEED_KP, SPEED_KP, 1.2f * SPEED_KP})
        for (float pkp : {0.67f * POS_KP, POS_KP, 1.33f * POS_KP}) {
          bx::Settings s;
          s.kp = kp; s.kd = kd; s.speedKp = skp; s.posKp = pkp;
          int ok, total;
          testUnknowns(&s, false, ok, total);
          std::printf("  KP %5.1f  KD %4.2f  SPEED_KP %5.3f  POS_KP %4.1f : %2d / %d\n", kp, kd, skp, pkp, ok, total);
        }
}

int main(int argc, char** argv) {
  testSigns();
  testFrictionCompensation();
  testOldVsNew();
  std::printf("Test 4: config.h gains across the unknowns\n");
  int ok, total;
  testUnknowns(nullptr, true, ok, total);
  std::printf("  balanced and drove straight in %d / %d variants\n", ok, total);
  CHECK(ok >= total * 9 / 10, "config.h gains failed in more than 10%% of the variants");
  testSafety();
  if (argc > 1 && std::strcmp(argv[1], "search") == 0) searchGains();
  std::printf(failures == 0 ? "\nALL TESTS PASSED\n" : "\n%d CHECK(S) FAILED\n", failures);
  return failures == 0 ? 0 : 1;
}
