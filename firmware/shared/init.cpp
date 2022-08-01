#include <driver/mcpwm.h>
#include "pin.h"

inline __attribute__((always_inline)) void init_motor() {
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, motorA);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, motorB);

  mcpwm_config_t motor_pwm_cfg = {
      .frequency = 2000,
      .cmpr_a = 0,                       // A's init duty cycle, %
      .cmpr_b = 0,                       // B's init duty cycle, %
      .duty_mode = MCPWM_DUTY_MODE_0,    // Hi mode
      .counter_mode = MCPWM_UP_COUNTER,  // 上位计数
  };

  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &motor_pwm_cfg);
}

inline __attribute__((always_inline)) void init_servo() {
  mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1A, servo);

  mcpwm_config_t servo_pwm_cfg = {
      .frequency = 50,
      .cmpr_a = 0,                       // A‘s init duty cycle, %
      .duty_mode = MCPWM_DUTY_MODE_0,    // Hi mode
      .counter_mode = MCPWM_UP_COUNTER,  // 上位计数
  };

  mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_1, &servo_pwm_cfg);
  SET_SERVO_DUTY(7.5);  // mid pos
  SERVO_PWM_START();
}