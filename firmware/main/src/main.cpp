#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <constants.h>

// Be sure to define the WiFi mode before include lie-flat.h
#define LIE_FLAT_WIFI_AP
#include <lie-flat.h>

AsyncWebServer server(80);
Adafruit_MPU6050 mpu;
IPAddress camAddr;
bool ready = false;
QueueHandle_t cmdQueue;

void notFound(AsyncWebServerRequest* request) {
  request->send(404, "text/plain", "Not found");
}

inline __attribute__((always_inline)) float parse_float_param(
    AsyncWebServerRequest* request,
    const char* param) {
  return request->getParam(param, true)->value().toFloat();
}

inline __attribute__((always_inline)) int parse_int_param(
    AsyncWebServerRequest* request,
    const char* param) {
  return request->getParam(param, true)->value().toInt();
}

volatile int optocoupler_state = 0;

void IRAM_ATTR optocoupler_interrupt() {
  // Serial.println("INFO: Optocoupler interrupt!");
  // Serial.printf("INFO: Analog read: %d\n", analogRead(optocoupler));
  optocoupler_state += digitalRead(optocoupler);
}

void act(float servo, float motor_a, float motor_b, int duration) {
  set_servo(servo);
  set_a(motor_a);
  set_b(motor_b);
  vTaskDelay(duration);
  set_a(0);
  set_b(0);
}

void setup() {
  // put your setup code here, to run once:
  // Motor & Servo
  init_motor(motorA, motorB);
  init_servo(servo);
  set_servo(7.5);
  servo_start_pwm();
  // Serial
  init_serial();
  // Connect to WiFi / Start an AP
  init_wifi();
  // MPU6050
  init_mpu(mpu, mpuSDA, mpuSCL);
  // Optocoupler
  attachInterrupt(digitalPinToInterrupt(optocoupler), optocoupler_interrupt,
                  RISING);
  // Command queue
  cmdQueue = xQueueCreate(1, sizeof(AsyncWebServerRequest*));
  // Web server
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send(200, "text/plain", "Hello, world");
  });
  server.on("/init", HTTP_POST, [](AsyncWebServerRequest* request) {
    if (ready) {
      request->send(200, "text/plain", "OK");
      Serial.println("INFO: Closing UDP server!");
      buzz(2000, 1500);
    } else
      request->send(425, "text/plain", "Too Early");
  });
  server.on("/act", HTTP_POST, [](AsyncWebServerRequest* request) {
    // Synchronous action web API
    // Put the request in the queue
    if (xQueueSend(cmdQueue, (void*)&request, (TickType_t)10) != pdPASS) {
      request->send(500);  // Send internal server error if we did not manage to
                           // put the request to the queue.
    }
  });
  server.on("/cmd", HTTP_POST, [](AsyncWebServerRequest* request) {
    if (request->hasParam(SERVO_PARAM, true))
      set_servo(parse_float_param(request, SERVO_PARAM));
    if (request->hasParam(MOTOR_A_PARAM, true))
      set_a(parse_float_param(request, MOTOR_A_PARAM));
    if (request->hasParam(MOTOR_B_PARAM, true))
      set_b(parse_float_param(request, MOTOR_B_PARAM));
    request->send(200, "text/plain", "OK");
  });
  server.on("/ping", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send(200, "text/plain", "Pong");
  });
  server.on("/read", HTTP_GET, [](AsyncWebServerRequest* request) {
    sensors_event_t acc, gyro, temp;
    // acc: m/s^2
    // gyro: rad/s
    // temp: Celsius
    mpu.getEvent(&acc, &gyro, &temp);
    request->send(200, "application/json",
                  "{\"acceleration\":{"
                  "\"x\":" +
                      String(acc.acceleration.x) +
                      ",\"y\":" + String(acc.acceleration.y) +
                      ",\"z\":" + String(acc.acceleration.z) +
                      "},\"gyro\":{"
                      "\"x\":" +
                      String(gyro.gyro.x) + ",\"y\":" + String(gyro.gyro.y) +
                      ",\"z\":" + String(gyro.gyro.z) +
                      "},\"temp\":" + String(temp.temperature) + "}");
  });
  server.on("/optocoupler", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send(200, "text/plain", String(optocoupler_state));
  });
  server.on("/buzz", HTTP_POST, [](AsyncWebServerRequest* request) {
    int freq = 3000;
    int duration = 1000;
    if (request->hasParam(FREQ_PARAM, true))
      freq = parse_int_param(request, FREQ_PARAM);
    if (request->hasParam(DURATION_PARAM, true))
      duration = parse_int_param(request, DURATION_PARAM);
    Serial.printf("INFO: Buzzing! Freq: %d, duration: %d\n", freq, duration);
    buzz(freq, duration);
    request->send(200, "text/plain", "OK");
  });
  server.onNotFound(notFound);
  server.begin();
  ready = true;
}

void loop() {
  if (uxQueueMessagesWaiting(cmdQueue)) {
    AsyncWebServerRequest* request;
    if (xQueueReceive(cmdQueue, &request, (TickType_t)10)) {
      auto duration = parse_int_param(request, "duration");
      auto servo = parse_float_param(request, SERVO_PARAM);
      auto motor_a = parse_float_param(request, MOTOR_A_PARAM);
      auto motor_b = parse_float_param(request, MOTOR_B_PARAM);
      act(servo, motor_a, motor_b, duration);
      request->send(200, "text/plain", "OK");
    }
  }
}