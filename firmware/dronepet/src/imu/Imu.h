#pragma once

#include "../common/MathTypes.h"

struct ImuSample {
  Vector3 gyroDps;
  Vector3 accelG;
};

namespace Imu {
  bool begin();
  bool calibrateGyro();
  Vector3 gyroBiasDps();
  uint8_t whoAmI();
  // Return false when the SPI result fails basic integrity checks.
  bool readSample(ImuSample& sample);
}
