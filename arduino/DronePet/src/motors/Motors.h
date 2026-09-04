#pragma once

#include <Arduino.h>

namespace Motors {
  // Configure the four ESC signal pins and immediately command zero throttle.
  void begin();

  void arm();
  void disarm();
  bool armed();

  void setThrottlePercent(uint8_t motorNumber, float percent);

  float throttlePercent(uint8_t motorNumber);

  void stop(uint8_t motorNumber);
  void stopAll();
}
