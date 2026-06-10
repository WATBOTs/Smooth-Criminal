#include "Arduino.h"
#include "melty_config.h"
#include "motor_driver.h"
#include <Servo.h>

// ESCs
Servo esc1;
Servo esc2;

// Constants for ESC signal range
const int ESC_FULL_REVERSE = 1000;
const int ESC_NEUTRAL = 1500;
const int ESC_FULL_FORWARD = 2000;

void init_motors() {
  esc1.attach(MOTOR_PIN1);
  esc2.attach(MOTOR_PIN2);

  esc1.writeMicroseconds(ESC_NEUTRAL);
  esc2.writeMicroseconds(ESC_NEUTRAL);
  delay(2000); // Allow ESCs to initialize
}

void motor_on(float throttle_percent, Servo &esc) {
  int pwm_us;

  if (THROTTLE_TYPE == BINARY_THROTTLE) {
    pwm_us = (throttle_percent >= 0) ? ESC_FULL_FORWARD : ESC_FULL_REVERSE;
  }

  else if (THROTTLE_TYPE == FIXED_PWM_THROTTLE) {
    pwm_us = (throttle_percent >= 0) ? ESC_NEUTRAL + (ESC_FULL_FORWARD - ESC_NEUTRAL) / 2
                                     : ESC_NEUTRAL - (ESC_NEUTRAL - ESC_FULL_REVERSE) / 2;
  }

  else if (THROTTLE_TYPE == DYNAMIC_PWM_THROTTLE) {
    throttle_percent = constrain(throttle_percent, -DYNAMIC_PWM_THROTTLE_PERCENT_MAX, DYNAMIC_PWM_THROTTLE_PERCENT_MAX);
    float percent = throttle_percent / DYNAMIC_PWM_THROTTLE_PERCENT_MAX;
    pwm_us = ESC_NEUTRAL + (ESC_FULL_FORWARD - ESC_NEUTRAL) * percent;
  }

  pwm_us = constrain(pwm_us, ESC_FULL_REVERSE, ESC_FULL_FORWARD);
  esc.writeMicroseconds(pwm_us);
}

void motor_1_on(float throttle_percent) {
  motor_on(throttle_percent, esc1);
}

void motor_2_on(float throttle_percent) {
  motor_on(-throttle_percent, esc2);
}

void motor_coast(Servo &esc) {
  esc.writeMicroseconds(ESC_NEUTRAL);
}

void motor_1_coast() {
  motor_coast(esc1);
}

void motor_2_coast() {
  motor_coast(esc2);
}

void motor_off(Servo &esc) {
  esc.writeMicroseconds(ESC_NEUTRAL);  // You could detach the servo here if you want to cut signal
}

void motor_1_off() {
  motor_off(esc1);
}

void motor_2_off() {
  motor_off(esc2);
}

void motors_off() {
  motor_1_off();
  motor_2_off();
}
