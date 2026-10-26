// =====================================================================================
//  balance.h  -  the controller: tilt angle, wheel speed, balance loop, forward-driving
//  loop, straight-line loop, friction compensation and the start / fall / direction safety.
//
//  Plain C++ only (no Arduino functions), so tests/uno_sim_test.cpp runs this exact code
//  against a physics model of the robot on a PC. BalanceX_Uno.ino does the hardware part:
//  it reads the sensor and encoders, calls Controller::step() every 5 ms and writes the
//  two PWM values it returns to the L298N.
// =====================================================================================
#pragma once
#include <math.h>
#include <stdint.h>
#include "config.h"

static_assert(FRICTION_RAMP_PWM > 0.0f, "FRICTION_RAMP_PWM must be above 0");
static_assert(START_ANGLE_DEG < FALL_ANGLE_DEG, "START_ANGLE_DEG must be below FALL_ANGLE_DEG");

namespace bx {

constexpr float RAD_TO_DEG_F = 57.2957795f;
constexpr float MAX_STEP_S = 0.05f;   // a control step later than this = timing fault

inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

// Share of the way a first-order low-pass filter moves toward its input in one step.
inline float lowPassFactor(float dt, float tau) { return tau > 0.0f ? dt / (tau + dt) : 1.0f; }

// Forward-positive pitch (deg) from the accelerometer: your formula, your signs.
inline float accelPitchDeg(float axG, float azG) {
  return PITCH_SIGN * atan2f(axG, azG) * RAD_TO_DEG_F;
}

// Forward-positive pitch rate (deg/s) from the gyro Y axis, offset already removed.
inline float gyroPitchRateDps(float gyDps) { return RATE_SIGN * gyDps; }

// -------------------------------------------------------------------------------------
// Friction compensation: command (PWM, + = forward) -> PWM sent to one motor.
//
// The gearbox friction always pushes AGAINST the way the motor is turning. So:
//   * motor turning (encoder speed above FRICTION_SPEED_RPM): add the friction PWM in the
//     direction it turns. That cancels the friction the gearbox really produces, whatever
//     sign the command has, and the PWM does not flip back and forth when the command
//     wobbles around zero.
//   * motor (nearly) still: add it in the commanded direction to break it loose, blended
//     in over the first FRICTION_RAMP_PWM of command so the output never jumps.
// offset = percent% of the PWM this motor needs to start turning (your LEFT/RIGHT_MIN_PWM).
// -------------------------------------------------------------------------------------
inline float frictionCompensate(float cmd, float motorRpm, float minPwm, float percent, float maxPwm) {
  const float offset = minPwm * percent * 0.01f;
  const float moving = clampf(motorRpm / FRICTION_SPEED_RPM, -1.0f, 1.0f);
  const float starting = clampf(cmd / FRICTION_RAMP_PWM, -1.0f, 1.0f) * (1.0f - fabsf(moving));
  const float pwm = cmd + offset * (moving + starting);
  if (fabsf(pwm) < MOTOR_ZERO_BAND) return 0.0f;
  return clampf(pwm, -maxPwm, maxPwm);
}

// -------------------------------------------------------------------------------------
// Values that can be changed live over Serial. They start from config.h.
// -------------------------------------------------------------------------------------
struct Settings {
  float kp = KP, kd = KD;
  float speedKp = SPEED_KP, posKp = POS_KP;
  float headingKp = HEADING_KP;
  float forwardRpm = FORWARD_SPEED_RPM;
  float frictionPercent = FRICTION_COMP_PERCENT;
  float maxPwm = MAX_PWM;
  float alpha = COMP_FILTER_ALPHA;
};

// One sensor reading, as delivered by the hardware.
struct Sample {
  float axG, azG;               // accelerometer X and Z (g)
  float gyDps;                  // gyro Y (deg/s), start-up offset already removed
  long  leftCount, rightCount;  // raw encoder counts from the interrupts
};

enum class Mode : uint8_t { WaitUpright, Balancing, Stopped };

enum Event : uint8_t {
  EV_NONE = 0,
  EV_ARMED,          // held upright long enough: motors on
  EV_DRIVING,        // started driving forward
  EV_FELL,           // past FALL_ANGLE_DEG: motors off, waiting to be stood up again
  EV_TIMING,         // a control step came far too late: motors off, waiting
  EV_DIR_LEFT,       // left wheel turned against its command: motors off, stopped
  EV_DIR_RIGHT,      // same, right wheel
};

class Controller {
 public:
  Settings set;
  float zeroDeg = 0.0f;   // accelerometer pitch at the balance point (start-up calibration)

  // ---- estimates, readable for telemetry
  float accPitch = 0.0f;  // accelerometer only (noisy, deg)
  float pitch = 0.0f;     // complementary filter (deg)
  float rate = 0.0f;      // gyro (deg/s)
  float rateF = 0.0f;     // gyro, low-passed for the KD term (deg/s)
  float leftRev = 0.0f, rightRev = 0.0f;   // encoder: wheel turns relative to the body (+ = forward)
  float leftRpm = 0.0f, rightRpm = 0.0f;   // encoder: wheel speed relative to the body (RPM)

  // ---- controller internals, readable for telemetry
  float speedCmd = 0.0f;   // forward speed reference right now (RPM, ramps up)
  float refRev = 0.0f;     // moving position reference (revolutions)
  float posErr = 0.0f;     // refRev - position (revolutions)
  float headErr = 0.0f;    // (left - right) travel since start (revolutions)
  float targetDeg = 0.0f;  // lean angle requested by the outer loop (deg)
  float balanceCmd = 0.0f, turnCmd = 0.0f;   // PWM, before friction compensation
  float leftPwm = 0.0f, rightPwm = 0.0f;     // final PWM per wheel (+ = forward)
  Mode  mode = Mode::WaitUpright;

  // One control step. dt = seconds since the previous step, nowMs = a millisecond clock.
  Event step(const Sample& s, float dt, uint32_t nowMs) {
    if (dt <= 0.0f || dt > MAX_STEP_S) {
      // The angle filter cannot be trusted after a long gap: start over from the accelerometer.
      filterStarted = false;
      encStarted = false;
      const bool wasBalancing = (mode == Mode::Balancing);
      if (wasBalancing) mode = Mode::WaitUpright;
      motorsOff();
      uprightTiming = false;
      return wasBalancing ? EV_TIMING : EV_NONE;
    }

    estimate(s, dt);

    switch (mode) {
      case Mode::Stopped:
        motorsOff();
        return EV_NONE;

      case Mode::WaitUpright:
        motorsOff();
        if (fabsf(pitch) <= START_ANGLE_DEG && fabsf(rateF) <= START_RATE_DPS) {
          if (!uprightTiming) {
            uprightTiming = true;
            uprightSinceMs = nowMs;
          } else if ((uint32_t)(nowMs - uprightSinceMs) >= START_HOLD_MS) {
            arm(nowMs);
            return EV_ARMED;
          }
        } else {
          uprightTiming = false;
        }
        return EV_NONE;

      case Mode::Balancing:
        if (fabsf(pitch) >= FALL_ANGLE_DEG) {
          mode = Mode::WaitUpright;
          uprightTiming = false;
          motorsOff();
          return EV_FELL;
        }
        return control(dt, nowMs);
    }
    return EV_NONE;
  }

  // Motors off until go(). Keeps estimating so the telemetry stays live.
  void stop() {
    mode = Mode::Stopped;
    motorsOff();
  }

  // Wait for the robot to be held upright, then balance.
  void go() {
    if (mode == Mode::Stopped) mode = Mode::WaitUpright;
    uprightTiming = false;
  }

  // Sensor failure: motors off at once, and restart the angle filter on the next reading.
  void sensorFault() {
    if (mode == Mode::Balancing) mode = Mode::WaitUpright;
    uprightTiming = false;
    filterStarted = false;
    motorsOff();
  }

  // Start the angle filter and wheel speeds again from the next reading (after calibration).
  void restartEstimates() {
    filterStarted = false;
    encStarted = false;
  }

  // Use the current (filtered) angle as the balance point. Hold the robot at it, still.
  void zeroHere() {
    const float shift = pitch;
    zeroDeg += shift;
    pitch -= shift;
    accPitch -= shift;
  }

 private:
  bool     filterStarted = false, encStarted = false;
  long     lastLeft = 0, lastRight = 0;
  bool     uprightTiming = false;
  uint32_t uprightSinceMs = 0, armedMs = 0;
  bool     driving = false;
  float    headRef = 0.0f;
  float    leftCmdF = 0.0f, rightCmdF = 0.0f;
  bool     leftWrong = false, rightWrong = false;
  uint32_t leftWrongSinceMs = 0, rightWrongSinceMs = 0;

  void estimate(const Sample& s, float dt) {
    // Tilt: gyro for fast changes, a small share of accelerometer to cancel drift.
    // The gyro is integrated unfiltered: filtering it here would make the angle lag.
    accPitch = accelPitchDeg(s.axG, s.azG) - zeroDeg;
    rate = gyroPitchRateDps(s.gyDps);
    if (!filterStarted) {
      pitch = accPitch;
      rateF = rate;
      filterStarted = true;
    } else {
      pitch = set.alpha * (pitch + rate * dt) + (1.0f - set.alpha) * accPitch;
      const float rateTau = RATE_FILTER_HZ > 0.0f ? 1.0f / (6.2831853f * RATE_FILTER_HZ) : 0.0f;
      rateF += lowPassFactor(dt, rateTau) * (rate - rateF);
    }

    // Wheels: normalized counts -> revolutions, and a smoothed speed.
    const long l = s.leftCount * LEFT_ENCODER_SIGN;
    const long r = s.rightCount * RIGHT_ENCODER_SIGN;
    if (!encStarted) {
      lastLeft = l;
      lastRight = r;
      leftRpm = rightRpm = 0.0f;
      encStarted = true;
    }
    const float k = lowPassFactor(dt, SPEED_FILTER_TAU_S);
    const float toRpmL = 60.0f / (LEFT_COUNTS_PER_WHEEL_REV * dt);
    const float toRpmR = 60.0f / (RIGHT_COUNTS_PER_WHEEL_REV * dt);
    leftRpm  += k * ((float)(l - lastLeft)  * toRpmL - leftRpm);
    rightRpm += k * ((float)(r - lastRight) * toRpmR - rightRpm);
    lastLeft = l;
    lastRight = r;
    leftRev  = (float)l / LEFT_COUNTS_PER_WHEEL_REV;
    rightRev = (float)r / RIGHT_COUNTS_PER_WHEEL_REV;
  }

  // The encoders sit on the motors, so they count wheel turns RELATIVE TO THE BODY.
  // Distance over the ground = that + the body's own tilt (1 deg = 1/360 revolution),
  // speed over the ground = that + the tilt rate (1 deg/s = 1/6 RPM).
  float groundRev() const { return 0.5f * (leftRev + rightRev) + pitch / 360.0f; }
  float groundRpm() const { return 0.5f * (leftRpm + rightRpm) + rateF / 6.0f; }

  void arm(uint32_t nowMs) {
    mode = Mode::Balancing;
    armedMs = nowMs;
    driving = false;
    speedCmd = 0.0f;
    refRev = groundRev();                   // hold the position where it was let go
    headRef = leftRev - rightRev;           // and the direction it was facing
    leftCmdF = rightCmdF = 0.0f;
    leftWrong = rightWrong = false;
  }

  void motorsOff() {
    balanceCmd = turnCmd = 0.0f;
    leftPwm = rightPwm = 0.0f;
    targetDeg = 0.0f;
    posErr = 0.0f;
    speedCmd = 0.0f;
  }

  Event control(float dt, uint32_t nowMs) {
    Event ev = EV_NONE;

    // ---- forward speed reference: wait DRIVE_DELAY_MS, then ramp up to forwardRpm
    float wanted = 0.0f;
    if ((uint32_t)(nowMs - armedMs) >= DRIVE_DELAY_MS) {
      wanted = clampf(set.forwardRpm, -MAX_SPEED_RPM, MAX_SPEED_RPM);
      if (!driving && wanted != 0.0f) {
        driving = true;
        ev = EV_DRIVING;
      }
    }
    const float rampStep = SPEED_RAMP_RPM_PER_S * dt;
    speedCmd += clampf(wanted - speedCmd, -rampStep, rampStep);

    // ---- outer loop: position + speed error -> target lean angle (config.h section 7)
    const float position = groundRev();
    const float speed = groundRpm();
    refRev += speedCmd * dt / 60.0f;
    posErr = refRev - position;
    if (posErr > POS_ERR_LIMIT_REV) {          // pushed far away: stop pulling harder,
      refRev = position + POS_ERR_LIMIT_REV;   // let the reference come along instead
      posErr = POS_ERR_LIMIT_REV;
    } else if (posErr < -POS_ERR_LIMIT_REV) {
      refRev = position - POS_ERR_LIMIT_REV;
      posErr = -POS_ERR_LIMIT_REV;
    }
    targetDeg = clampf(set.posKp * posErr + set.speedKp * (speedCmd - speed),
                       -MAX_TARGET_DEG, MAX_TARGET_DEG);

    // ---- inner loop: balance
    balanceCmd = set.kp * (pitch - targetDeg) + set.kd * rateF;

    // ---- straight line: equal travel on both wheels
    headErr = (leftRev - rightRev) - headRef;
    turnCmd = clampf(set.headingKp * headErr + HEADING_KD * (leftRpm - rightRpm),
                     -MAX_TURN_PWM, MAX_TURN_PWM);

    leftPwm  = frictionCompensate(balanceCmd - turnCmd, leftRpm, LEFT_MIN_PWM, set.frictionPercent, set.maxPwm);
    rightPwm = frictionCompensate(balanceCmd + turnCmd, rightRpm, RIGHT_MIN_PWM, set.frictionPercent, set.maxPwm);

    // ---- safety: a wheel that keeps turning against its own command has a sign error
    const float kc = lowPassFactor(dt, 0.1f);
    leftCmdF  += kc * (leftPwm - leftCmdF);
    rightCmdF += kc * (rightPwm - rightCmdF);
    if (turnsWrongWay(leftCmdF, leftRpm, leftWrong, leftWrongSinceMs, nowMs)) {
      stop();
      return EV_DIR_LEFT;
    }
    if (turnsWrongWay(rightCmdF, rightRpm, rightWrong, rightWrongSinceMs, nowMs)) {
      stop();
      return EV_DIR_RIGHT;
    }
    return ev;
  }

  static bool turnsWrongWay(float cmdF, float rpm, bool& timing, uint32_t& sinceMs, uint32_t nowMs) {
    const bool wrong = (cmdF > DIR_FAULT_MIN_CMD && rpm < -DIR_FAULT_MIN_RPM) ||
                       (cmdF < -DIR_FAULT_MIN_CMD && rpm > DIR_FAULT_MIN_RPM);
    if (!wrong) {
      timing = false;
      return false;
    }
    if (!timing) {
      timing = true;
      sinceMs = nowMs;
      return false;
    }
    return (uint32_t)(nowMs - sinceMs) >= DIR_FAULT_MS;
  }
};

}  // namespace bx
