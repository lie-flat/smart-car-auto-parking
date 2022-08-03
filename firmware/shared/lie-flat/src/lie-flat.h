#include "Arduino.h"
#include "ctrl.h"
#include "init.h"
#include "pin.h"

// The shared library

inline __attribute__((always_inline)) void buzz(int freq, int duration) {
  tone(buzzer, freq, duration);
}