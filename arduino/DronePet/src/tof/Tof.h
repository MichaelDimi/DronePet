#pragma once

#include <Arduino.h>

struct TofSample {
  uint16_t distanceMm = 0;
  uint32_t timestampUs = 0;
  uint32_t intervalUs = 0;
  uint8_t rangeStatus = 0;
  bool rangeValid = false;
};

namespace Tof {
  // Configure the VL53L1X and start continuous ranging.
  bool begin();

  // Read a finished measurement when one is available.
  // Returns true only when a new sample was stored.
  bool update();

  bool initialized();
  bool hasSample();
  bool communicationError();

  const TofSample& latestSample();

  const char* rangeStatusToString(uint8_t rangeStatus);
}