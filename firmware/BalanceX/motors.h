// =====================================================================================
//  motors.h  -  L298N motor driver using the ESP32 hardware PWM (LEDC).
//
//  How the L298N is driven (per motor):
//      IN1 = HIGH, IN2 = LOW   -> forward           ENx = PWM duty -> speed
//      IN1 = LOW,  IN2 = HIGH  -> backward
//      IN1 = LOW,  IN2 = LOW   -> stop (and ENx = 0, so the motor coasts freely)
//
//  The ESP32 Arduino core changed the PWM functions in version 3.0:
//      core 2.x : ledcSetup(channel, freq, bits);  ledcAttachPin(pin, channel);  ledcWrite(channel, duty)
//      core 3.x : ledcAttach(pin, freq, bits);                                   ledcWrite(pin, duty)
//  The #if blocks below pick the right one automatically, so this compiles on both.
// =====================================================================================
#pragma once
#include <Arduino.h>
#include "config.h"
#include "control.h"

namespace motors {

struct Motor {
  int     pwmPin, in1Pin, in2Pin;
  uint8_t ledcChannel;   // only used by ESP32 core 2.x
  bool    invert;
  float   gain;
};

Motor left  = {PIN_LEFT_PWM,  PIN_LEFT_IN1,  PIN_LEFT_IN2,  0, LEFT_MOTOR_INVERT,  LEFT_MOTOR_GAIN};
Motor right = {PIN_RIGHT_PWM, PIN_RIGHT_IN1, PIN_RIGHT_IN2, 1, RIGHT_MOTOR_INVERT, RIGHT_MOTOR_GAIN};

bool attachPwm(const Motor& m) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  return ledcAttach(m.pwmPin, PWM_FREQ_HZ, PWM_RES_BITS);
#else
  if (ledcSetup(m.ledcChannel, PWM_FREQ_HZ, PWM_RES_BITS) == 0) return false;  // 0 = impossible freq/bits
  ledcAttachPin(m.pwmPin, m.ledcChannel);
  return true;
#endif
}

void writePwm(const Motor& m, uint32_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(m.pwmPin, duty);
#else
  ledcWrite(m.ledcChannel, duty);
#endif
}

// Call this FIRST in setup(): drives every motor pin LOW as early as possible.
// (Until our code runs, the pins are not under our control, and GPIO 14 = ENB is one of
//  the ESP32 pins that outputs a signal during boot. Keep the wheels lifted on first tests.)
void forcePinsLow() {
  const int pins[] = {PIN_LEFT_PWM, PIN_LEFT_IN1, PIN_LEFT_IN2,
                      PIN_RIGHT_PWM, PIN_RIGHT_IN1, PIN_RIGHT_IN2};
  for (int p : pins) {
    pinMode(p, OUTPUT);
    digitalWrite(p, LOW);
  }
}

// Returns false if the PWM could not be set up (usually PWM_FREQ_HZ * 2^PWM_RES_BITS > 80 MHz).
bool begin() {
  forcePinsLow();
  const bool ok = attachPwm(left) && attachPwm(right);
  if (ok) {
    writePwm(left, 0);
    writePwm(right, 0);
  }
  return ok;
}

// duty: -PWM_MAX .. +PWM_MAX, positive = roll FORWARD. No dead-band compensation.
void driveRaw(const Motor& m, float duty) {
  if (m.invert) duty = -duty;
  if (duty > 0.0f) {
    digitalWrite(m.in1Pin, HIGH);
    digitalWrite(m.in2Pin, LOW);
  } else if (duty < 0.0f) {
    digitalWrite(m.in1Pin, LOW);
    digitalWrite(m.in2Pin, HIGH);
  } else {
    digitalWrite(m.in1Pin, LOW);
    digitalWrite(m.in2Pin, LOW);
  }
  writePwm(m, (uint32_t)lroundf(clampf(fabsf(duty), 0.0f, (float)PWM_MAX)));
}

// Balance output -> both wheels (same direction: this robot only balances, it does not steer yet).
void drive(float cmd, float minPwm, float maxPwm) {
  driveRaw(left,  shapeCommand(cmd, left.gain,  minPwm, maxPwm));
  driveRaw(right, shapeCommand(cmd, right.gain, minPwm, maxPwm));
}

void stop() {
  driveRaw(left, 0.0f);
  driveRaw(right, 0.0f);
}

}  // namespace motors
