// =====================================================================================
//  host_test.cpp - checks the firmware maths on a PC (no robot needed). OPTIONAL.
//
//    g++ -std=c++17 -O2 -o host_test tests/host_test.cpp && ./host_test
//
//  Test 1: for all 24 ways the MPU6050 board can be mounted (flat, upside down, vertical,
//          rotated...), simulate the sensor, find IMU_UP_* / IMU_FWD_* exactly the way
//          docs/TUNING_GUIDE.md tells you to, and check that the firmware formulas give
//          the true tilt angle and tilt rate with the correct sign.
//  Test 2: a rough physics simulation of a two-wheeled robot driven through the same
//          filter / balance controller / motor-shaping code as the firmware (control.h),
//          using the config.h values.
//          The robot parameters below are GUESSES - this proves the signs and the logic,
//          not the final gains. Real tuning happens on the real robot.
// =====================================================================================
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <random>

#include "../firmware/BalanceX/config.h"
#include "../firmware/BalanceX/control.h"

static int failures = 0;
#define CHECK(cond, ...)                \
  do {                                  \
    if (!(cond)) {                      \
      std::printf("FAIL: " __VA_ARGS__); \
      std::printf("\n");                \
      failures++;                       \
    }                                   \
  } while (0)

// ------------------------------------------------------------------ small 3x3 helpers
struct M3 { double m[3][3]; };
struct V3 { double v[3]; };

static V3 mul(const M3& a, const V3& x) {
  V3 r{};
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) r.v[i] += a.m[i][j] * x.v[j];
  return r;
}
static M3 mul(const M3& a, const M3& b) {
  M3 r{};
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      for (int k = 0; k < 3; k++) r.m[i][j] += a.m[i][k] * b.m[k][j];
  return r;
}
static M3 transpose(const M3& a) {
  M3 r{};
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) r.m[i][j] = a.m[j][i];
  return r;
}
static double det(const M3& a) {
  return a.m[0][0] * (a.m[1][1] * a.m[2][2] - a.m[1][2] * a.m[2][1]) -
         a.m[0][1] * (a.m[1][0] * a.m[2][2] - a.m[1][2] * a.m[2][0]) +
         a.m[0][2] * (a.m[1][0] * a.m[2][1] - a.m[1][1] * a.m[2][0]);
}
// World/body frame: x = FRONT, y = LEFT, z = UP. Tipping forward by t = rotation about +y.
static M3 tiltForward(double t) {
  return M3{{{std::cos(t), 0, std::sin(t)}, {0, 1, 0}, {-std::sin(t), 0, std::cos(t)}}};
}

// The firmware formulas, with the mounting constants as variables instead of constexpr.
static void firmwareAngleAndRate(const double acc[3], const double gyro[3], int upAxis, int upSign,
                                 int fwdAxis, int fwdSign, float& angle, float& rate) {
  const int leftAxis = 3 - upAxis - fwdAxis;                                         // control.h
  const int leftSign = upSign * fwdSign * cyclicSign(upAxis, fwdAxis, leftAxis);     // control.h
  angle = accelTiltDeg(fwdSign * (float)acc[fwdAxis], upSign * (float)acc[upAxis]);  // .ino
  rate = leftSign * (float)gyro[leftAxis];                                           // .ino
}

// Which sensor axis reads about +-1 g?  (what "gravity: +Z" in sensor mode tells you)
static void dominantAxis(const double acc[3], int& axis, int& sign) {
  axis = 0;
  for (int i = 1; i < 3; i++)
    if (std::fabs(acc[i]) > std::fabs(acc[axis])) axis = i;
  sign = acc[axis] > 0 ? 1 : -1;
}

static void testAllMountings() {
  std::printf("Test 1: sensor mounting / sign conventions\n");
  int mountings = 0;
  const int perms[6][3] = {{0, 1, 2}, {0, 2, 1}, {1, 0, 2}, {1, 2, 0}, {2, 0, 1}, {2, 1, 0}};
  for (auto& p : perms) {
    for (int s = 0; s < 8; s++) {
      // M: sensor axes expressed in body coordinates (a signed permutation matrix)
      M3 mount{};
      for (int c = 0; c < 3; c++) mount.m[p[c]][c] = (s >> c & 1) ? -1 : 1;
      if (det(mount) < 0) continue;   // a real sensor is right-handed: skip mirror images
      mountings++;

      auto simulate = [&](double tiltRad, double rateRad, double acc[3], double gyro[3]) {
        const M3 toSensor = transpose(mul(tiltForward(tiltRad), mount));
        const V3 a = mul(toSensor, V3{{0, 0, 1}});         // at rest: +1 g "up"
        const V3 w = mul(toSensor, V3{{0, rateRad, 0}});   // rotating about the axle
        for (int i = 0; i < 3; i++) {
          acc[i] = a.v[i];
          gyro[i] = w.v[i] * kRadToDeg;
        }
      };

      // Find the config exactly like the tuning guide: upright -> UP, lying on its back -> FWD.
      double acc[3], gyro[3];
      int upAxis, upSign, fwdAxis, fwdSign;
      simulate(0.0, 0.0, acc, gyro);                  // upright
      dominantAxis(acc, upAxis, upSign);
      simulate(-90.0 / kRadToDeg, 0.0, acc, gyro);   // lying on its back
      dominantAxis(acc, fwdAxis, fwdSign);

      for (double deg = -80; deg <= 80; deg += 10) {
        const double rateDps = 37.0;   // tipping forward at 37 deg/s
        simulate(deg / kRadToDeg, rateDps / kRadToDeg, acc, gyro);
        float angle, rate;
        firmwareAngleAndRate(acc, gyro, upAxis, upSign, fwdAxis, fwdSign, angle, rate);
        CHECK(std::fabs(angle - deg) < 0.01, "mounting %d: tilt %.0f deg read as %.2f", mountings, deg, angle);
        CHECK(std::fabs(rate - rateDps) < 0.01, "mounting %d: rate %.0f read as %.2f", mountings, rateDps, rate);
      }
    }
  }
  std::printf("  checked %d mountings x 17 angles\n", mountings);
}

// ------------------------------------------------------------------ physics simulation
struct Robot {                 // GUESSED parameters for a robot like the one in the photo
  double bodyMass = 1.6;       // kg (chassis + battery + electronics, without wheels)
  double comHeight = 0.12;     // m, centre of mass above the axle
  double bodyInertia = 0.010;  // kg m^2 about the centre of mass
  double wheelMass = 0.15;     // kg, both wheels
  double wheelRadius = 0.0425; // m (85 mm wheels)
  double stallTorque = 0.8;    // N m at the wheel, both motors together, at 12 V
  double noLoadSpeed = 31.4;   // rad/s at the wheel (300 rpm) at 12 V
  double battery = 12.0;       // V
  double driverDrop = 2.0;     // V lost inside the L298N
  double friction = 0.05;      // N m, gearbox friction (needs some minimum PWM)
  double sensorHeight = 0.10;  // m, MPU6050 above the axle
};

struct SimResult { bool fell; double maxLateAngle; };

// Runs the firmware's ComplementaryFilter + BalanceController + shapeCommand (control.h)
// against the physics model. 't' holds the gains, exactly like the live-tunable values.
static SimResult simulateBalance(const Robot& rb, const Tuning& t, double extraMassOnTop,
                                 double startDeg, double seconds, unsigned seed) {
  std::mt19937 rng(seed);
  std::normal_distribution<double> accNoise(0.0, 0.02), gyroNoise(0.0, 0.1);

  // add the payload as a point mass 0.25 m above the axle
  const double M = rb.bodyMass + extraMassOnTop;
  const double l = (rb.bodyMass * rb.comHeight + extraMassOnTop * 0.25) / M;
  const double I = rb.bodyInertia + rb.bodyMass * std::pow(rb.comHeight - l, 2) +
                   extraMassOnTop * std::pow(0.25 - l, 2);
  const double g = 9.81, r = rb.wheelRadius;
  const double mw = rb.wheelMass + 0.5 * rb.wheelMass;   // wheel mass + rolling inertia

  double x = 0, xd = 0, th = startDeg / kRadToDeg, thd = 0, xdd = 0, thdd = 0;
  const double h = 0.0005;                               // physics step 0.5 ms
  const float dt = LOOP_PERIOD_US * 1e-6f;
  const int stepsPerLoop = (int)(dt / h + 0.5);
  ComplementaryFilter filter;
  BalanceController controller;
  SimResult res{false, 0.0};

  const int loops = (int)(seconds / dt);
  for (int k = 0; k < loops; k++) {
    // ---- what the MPU6050 measures (gravity + the sensor's own acceleration)
    const double ax = xdd + rb.sensorHeight * (thdd * std::cos(th) - thd * thd * std::sin(th));
    const double az = -rb.sensorHeight * (thdd * std::sin(th) + thd * thd * std::cos(th));
    const double fx = ax / g, fz = az / g + 1.0;          // specific force in g, world frame
    const float accFwd = (float)(fx * std::cos(th) - fz * std::sin(th) + accNoise(rng));
    const float accUp = (float)(fx * std::sin(th) + fz * std::cos(th) + accNoise(rng));
    const float rate = (float)(thd * kRadToDeg + gyroNoise(rng) + 0.3);   // 0.3 deg/s leftover bias

    // ---- the firmware's control step
    const float angle = filter.update(accelTiltDeg(accFwd, accUp), rate, dt, t.alpha);
    if (std::fabs(angle - t.setpoint) > FALL_ANGLE_DEG) { res.fell = true; break; }
    const float cmd = controller.step(angle, rate, dt, t);
    const double duty = shapeCommand(cmd, 1.0f, t.minPwm, t.maxPwm) / PWM_MAX;

    // ---- physics until the next control step
    for (int s = 0; s < stepsPerLoop; s++) {
      const double volts = duty * (rb.battery - rb.driverDrop);
      const double wheelVsBody = xd / r - thd;
      double torque = rb.stallTorque * (volts / 12.0 - wheelVsBody / rb.noLoadSpeed);
      if (std::fabs(wheelVsBody) < 0.05 && std::fabs(torque) < rb.friction) torque = 0;   // stuck
      else torque -= std::copysign(rb.friction, wheelVsBody != 0 ? wheelVsBody : torque);
      // (M+mw) xdd + M l cos(th) thdd = torque/r + M l sin(th) thd^2
      // M l cos(th) xdd + (I + M l^2) thdd = M g l sin(th) - torque
      const double a11 = M + mw, a12 = M * l * std::cos(th), a21 = a12, a22 = I + M * l * l;
      const double b1 = torque / r + M * l * std::sin(th) * thd * thd;
      const double b2 = M * g * l * std::sin(th) - torque;
      const double D = a11 * a22 - a12 * a21;
      xdd = (b1 * a22 - a12 * b2) / D;
      thdd = (a11 * b2 - a21 * b1) / D;
      xd += xdd * h;  x += xd * h;
      thd += thdd * h; th += thd * h;
    }
    if (k * dt > seconds / 2) res.maxLateAngle = std::max(res.maxLateAngle, std::fabs(th * kRadToDeg));
  }
  return res;
}

// Balanced from +5 deg AND from -8 deg, and stayed within 3 deg for the second half?
static bool balances(const Robot& rb, const Tuning& t, double payload, double seconds = 10.0) {
  for (unsigned seed = 1; seed <= 2; seed++) {
    const SimResult r = simulateBalance(rb, t, payload, seed == 1 ? 5.0 : -8.0, seconds, seed);
    if (r.fell || r.maxLateAngle > 3.0) return false;
  }
  return true;
}

static void printGainMap(const Robot& rb, Tuning t, double payload) {
  const float kds[] = {0.25f, 0.5f, 0.75f, 1.0f, 1.5f, 2.0f};
  std::printf("      payload %.0f kg  KD:", payload);
  for (float kd : kds) std::printf(" %4.2f", kd);
  std::printf("\n");
  for (float kp : {10.0f, 20.0f, 30.0f, 40.0f, 60.0f}) {
    std::printf("               KP %3.0f:", kp);
    for (float kd : kds) {
      t.kp = kp;
      t.kd = kd;
      std::printf("    %c", balances(rb, t, payload, 8.0) ? '#' : '.');
    }
    std::printf("\n");
  }
}

static void testClosedLoop() {
  Tuning t;              // = the config.h values
  t.minPwm = 40.0f;      // as if MOTOR_MIN_PWM had been measured with the ramp test
  Robot rb;
  std::printf("Test 2: closed-loop simulation, config.h gains KP=%.1f KI=%.1f KD=%.2f SPEED_KP=%.3f\n",
              t.kp, t.ki, t.kd, t.speedKp);

  const bool ok = balances(rb, t, 0.0);
  std::printf("  default gains, no payload        : %s\n", ok ? "balanced" : "FELL / did not settle");
  CHECK(ok, "robot did not balance with the config.h gains");

  const bool okPayload = balances(rb, t, 2.0);
  std::printf("  default gains, 2 kg at 0.25 m    : %s\n", okPayload ? "balanced" : "FELL / did not settle");
  CHECK(okPayload, "robot did not balance with the 2 kg payload");

  // Motors driving the wrong way must fall quickly: proves the test can fail at all.
  Tuning wrong = t;
  wrong.kp = -t.kp; wrong.ki = -t.ki; wrong.kd = -t.kd;
  const SimResult w = simulateBalance(rb, wrong, 0.0, 5.0, 3.0, 1);
  std::printf("  motor direction reversed         : %s (expected FELL)\n", w.fell ? "FELL" : "balanced");
  CHECK(w.fell, "reversed motors did not fall - the simulation is not testing anything");

  Tuning noMinPwm = t;
  noMinPwm.minPwm = 0.0f;
  std::printf("  without dead-band compensation   : %s\n",
              balances(rb, noMinPwm, 0.0) ? "balanced" : "FELL / did not settle");

  // KP/KD maps: '#' = balanced, '.' = fell or kept wobbling. Guessed robot - shapes, not numbers.
  std::printf("  KP/KD map with KI and SPEED_KP from config.h:\n");
  printGainMap(rb, t, 0.0);
  printGainMap(rb, t, 2.0);
  Tuning pdOnly = t;
  pdOnly.ki = 0.0f;
  pdOnly.speedKp = 0.0f;
  std::printf("  KP/KD map with KI = 0 and SPEED_KP = 0 (plain PD):\n");
  printGainMap(rb, pdOnly, 0.0);
}

int main() {
  testAllMountings();
  testClosedLoop();
  std::printf(failures == 0 ? "\nALL TESTS PASSED\n" : "\n%d CHECK(S) FAILED\n", failures);
  return failures == 0 ? 0 : 1;
}
