// =====================================================================================
//  control.h  -  the maths: tilt angle, complementary filter, PID, balance controller.
//  Plain C++ only (no Arduino functions), so the exact same code is also compiled and
//  tested on a PC by tests/host_test.cpp.
// =====================================================================================
#pragma once
#include <math.h>
#include "config.h"

constexpr float kRadToDeg = 57.2957795f;

inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

// -------------------------------------------------------------------------------------
// Which gyro axis measures the tipping rate?
// The robot tips around the wheel axle. The axle points LEFT, and LEFT = UP x FORWARD
// (right-hand rule), so it follows from the UP and FORWARD axes chosen in config.h.
// A positive rotation around the LEFT axis moves the top of the robot toward the FRONT,
// i.e. a positive gyro rate on that axis = tipping forward = angle increasing.
// -------------------------------------------------------------------------------------
// +1 if (i, j, k) is in cyclic order (X,Y,Z / Y,Z,X / Z,X,Y), else -1. i, j, k must differ.
constexpr int cyclicSign(int i, int j, int k) {
  return ((i + 1) % 3 == j && (j + 1) % 3 == k) ? 1 : -1;
}
constexpr int IMU_LEFT_AXIS = 3 - IMU_UP_AXIS - IMU_FWD_AXIS;
constexpr int IMU_LEFT_SIGN =
    IMU_UP_SIGN * IMU_FWD_SIGN * cyclicSign(IMU_UP_AXIS, IMU_FWD_AXIS, IMU_LEFT_AXIS);

static_assert(IMU_UP_AXIS >= 0 && IMU_UP_AXIS <= 2, "IMU_UP_AXIS must be 0 (X), 1 (Y) or 2 (Z)");
static_assert(IMU_FWD_AXIS >= 0 && IMU_FWD_AXIS <= 2, "IMU_FWD_AXIS must be 0 (X), 1 (Y) or 2 (Z)");
static_assert(IMU_UP_AXIS != IMU_FWD_AXIS, "IMU_UP_AXIS and IMU_FWD_AXIS cannot be the same axis");
static_assert(IMU_UP_SIGN == 1 || IMU_UP_SIGN == -1, "IMU_UP_SIGN must be +1 or -1");
static_assert(IMU_FWD_SIGN == 1 || IMU_FWD_SIGN == -1, "IMU_FWD_SIGN must be +1 or -1");

// -------------------------------------------------------------------------------------
// Tilt angle from the accelerometer alone.
// At rest an accelerometer reads +1 g on whichever axis points UP. When the robot tips
// forward by angle a, its FRONT axis starts pointing slightly down and reads -sin(a) g,
// while the UP axis reads cos(a) g. So  a = atan2(-front, up).
// Only valid when the robot is not accelerating - that is why we also need the gyro.
// -------------------------------------------------------------------------------------
inline float accelTiltDeg(float accFwdG, float accUpG) {
  return atan2f(-accFwdG, accUpG) * kRadToDeg;
}

// -------------------------------------------------------------------------------------
// Complementary filter
//   gyro  : smooth and fast, but integrating it slowly drifts (bias)
//   accel : never drifts, but very noisy and disturbed by every motor jerk
// Use the gyro for fast changes and let a small share of the accelerometer correct drift.
// -------------------------------------------------------------------------------------
struct ComplementaryFilter {
  float angle = 0.0f;
  bool  started = false;

  float update(float accAngleDeg, float gyroRateDps, float dt, float alpha) {
    if (!started) {               // first sample: start from the accelerometer angle,
      angle = accAngleDeg;        // otherwise the angle needs seconds to "settle in"
      started = true;
    } else {
      angle = alpha * (angle + gyroRateDps * dt) + (1.0f - alpha) * accAngleDeg;
    }
    return angle;
  }
  void reset() { started = false; }
};

// -------------------------------------------------------------------------------------
// PID controller
//   * D term uses the measured tilt RATE (gyro) instead of differentiating the error:
//     no noise amplification and no "kick" when the setpoint is changed.
//   * The I term stores KI*error*dt (not just error*dt), so changing KI live never makes
//     the output jump, and it is clamped to +-iLimit (anti wind-up).
// -------------------------------------------------------------------------------------
struct PidController {
  float pTerm = 0.0f, iTerm = 0.0f, dTerm = 0.0f;   // kept for telemetry / debugging

  float update(float error, float errorRate, float dt,
               float kp, float ki, float kd, float iLimit, float outLimit) {
    pTerm = kp * error;
    dTerm = kd * errorRate;
    iTerm = clampf(iTerm + ki * error * dt, -iLimit, iLimit);
    return clampf(pTerm + iTerm + dTerm, -outLimit, outLimit);
  }
  void reset() { pTerm = iTerm = dTerm = 0.0f; }
};

// -------------------------------------------------------------------------------------
// Motor command shaping (used by motors.h):
//   1. per-motor gain (to match a stronger / weaker motor)
//   2. dead-band compensation: map 0..maxPwm onto minPwm..maxPwm, so that even a small
//      correction is strong enough to overcome gearbox friction.
// -------------------------------------------------------------------------------------
inline float shapeCommand(float cmd, float gain, float minPwm, float maxPwm) {
  cmd *= gain;
  const float mag = fabsf(cmd);
  if (mag < MOTOR_ZERO_BAND || maxPwm <= 0.0f) return 0.0f;
  const float duty = clampf(minPwm + mag * (maxPwm - minPwm) / maxPwm, 0.0f, maxPwm);
  return cmd > 0.0f ? duty : -duty;
}

// -------------------------------------------------------------------------------------
// Values that can be changed live with serial commands. They start from config.h.
// -------------------------------------------------------------------------------------
struct Tuning {
  float kp = KP, ki = KI, kd = KD;
  float setpoint = BALANCE_SETPOINT_DEG;
  float alpha = COMP_FILTER_ALPHA;
  float minPwm = MOTOR_MIN_PWM, maxPwm = MOTOR_MAX_PWM;
  float speedKp = SPEED_KP;
};

// -------------------------------------------------------------------------------------
// The complete balance controller: one call per control step.
//
//  Stage 2 (anti-drift): a DC motor driven by PWM settles at a SPEED roughly proportional
//  to the PWM, so the averaged motor output is a rough stand-in for wheel speed. If the
//  robot keeps rolling one way, the target angle is tilted the other way so it leans
//  back and slows down. Without this (or wheel encoders) the robot tends to run away:
//  while accelerating, the accelerometer reads "upright" even though the robot leans.
// -------------------------------------------------------------------------------------
struct BalanceController {
  PidController pid;
  float speedEst = 0.0f;   // averaged motor output (PWM counts)
  float target = 0.0f;     // angle we are trying to hold this step (deg)
  float output = 0.0f;     // PID output (PWM counts, positive = drive forward)

  float step(float tiltDeg, float tiltRateDps, float dt, const Tuning& t) {
    target = t.setpoint - clampf(t.speedKp * speedEst, -MAX_TARGET_SHIFT_DEG, MAX_TARGET_SHIFT_DEG);
    output = pid.update(tiltDeg - target, tiltRateDps, dt, t.kp, t.ki, t.kd, I_TERM_LIMIT, t.maxPwm);
    speedEst += (dt / (SPEED_FILTER_TAU_S + dt)) * (output - speedEst);   // low-pass filter
    return output;
  }
  void reset() {
    pid.reset();
    speedEst = 0.0f;
    output = 0.0f;
  }
};
