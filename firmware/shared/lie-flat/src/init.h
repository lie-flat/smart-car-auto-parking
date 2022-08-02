#pragma once

#include <Adafruit_MPU6050.h>
#include <WiFi.h>
#include <Wire.h>
#include <driver/mcpwm.h>

#include "config.h"

inline __attribute__((always_inline)) void init_motor(int motorA, int motorB) {
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

inline __attribute__((always_inline)) void init_servo(int servo) {
  mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1A, servo);

  mcpwm_config_t servo_pwm_cfg = {
      .frequency = 50,
      .cmpr_a = 0,                       // A‘s init duty cycle, %
      .duty_mode = MCPWM_DUTY_MODE_0,    // Hi mode
      .counter_mode = MCPWM_UP_COUNTER,  // 上位计数
  };

  mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_1, &servo_pwm_cfg);
}

inline __attribute__((always_inline)) void init_mpu(Adafruit_MPU6050& mpu,
                                                    int sda,
                                                    int scl) {
  Wire.setPins(sda, scl);
  while (!mpu.begin()) {
    Serial.println(
        "ERROR: could not connect to a valid MPU6050 sensor! Retrying...");
    delay(500);
  }
  Serial.println("INFO: MPU6050 sensor ready!");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
}

inline __attribute__((always_inline)) void init_serial() {
  Serial.begin(115200);
  while (!Serial) {
    // Wait until serial connects
    delay(10);
  }
  Serial.println("INFO: Serial connected! :)");
}

inline __attribute__((always_inline)) void init_wifi() {
#ifdef LIE_FLAT_WIFI_STA
  WiFi.mode(WIFI_STA);  // station mode
  WiFi.begin(SSID, PASSWORD);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println(
        "ERROR: Failed to connect to WiFi!\nRetrying...");
    delay(500);
  }
  Serial.print("INFO: Connected to WiFi! My IP Address is ");
  Serial.println(WiFi.localIP());
#elif defined LIE_FLAT_WIFI_AP
  WiFi.mode(WIFI_AP);  // AP/STA mode
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  Serial.print("INFO: WiFi AP started! My IP Address is ");
  Serial.println(WiFi.softAPIP());
#else
#error "Please define LIE_FLAT_WIFI_STA or LIE_FLAT_WIFI_AP"
#endif
}