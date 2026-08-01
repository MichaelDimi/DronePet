#pragma once

#include "MathTypes.h"

struct ImuSample {
  Vector3 gyroDps;
  Vector3 accelG;
};

namespace Imu {
  bool begin();
  void calibrateGyro();
  Vector3 gyroBiasDps();
  uint8_t whoAmI();
  void readSample(ImuSample& sample);
}
