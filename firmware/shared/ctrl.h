#pragma once

#include <driver/mcpwm.h>

#define SET_A_DUTY(duty_cycle) \
  (mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, ((duty_cycle))))
#define SET_B_DUTY(duty_cycle) \
  (mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, ((duty_cycle))))
#define SET_SERVO_DUTY(duty_cycle) \
  (mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, ((duty_cycle))))

#define MOTOR_PWM_START() (mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0))
#define MOTOR_PWM_STOP() (mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_0))
#define SERVO_PWM_START() (mcpwm_start(MCPWM_UNIT_1, MCPWM_TIMER_1))
#define SERVO_PWM_STOP() (mcpwm_stop(MCPWM_UNIT_1, MCPWM_TIMER_1))