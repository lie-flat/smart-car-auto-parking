#pragma once

#include <driver/mcpwm.h>

inline __attribute__((always_inline)) void set_a(float duty) {
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty);
}
inline __attribute__((always_inline)) void set_b(float duty) {
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, duty);
}
inline __attribute__((always_inline)) void set_servo(float duty) {
  mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, duty);
}

inline __attribute__((always_inline)) void motor_start_pwm() {
  mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0);
}
inline __attribute__((always_inline)) void motor_stop_pwm() {
  mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
}
inline __attribute__((always_inline)) void servo_start_pwm() {
  mcpwm_start(MCPWM_UNIT_1, MCPWM_TIMER_1);
}
inline __attribute__((always_inline)) void servo_stop_pwm() {
  mcpwm_stop(MCPWM_UNIT_1, MCPWM_TIMER_1);
}