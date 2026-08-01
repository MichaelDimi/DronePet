#pragma once

#include "Imu.h"

struct ImuHealthStatus {
  bool spiReadOk = false;
  bool accelerationValid = false;
  bool gyroSaturated = false;
  bool accelMagnitudeValid = false;

  float accelMagnitudeG = 0.0f;

  bool healthy() const {
    return spiReadOk
        && accelerationValid
        && !gyroSaturated
        && accelMagnitudeValid;
  }
};

namespace ImuHealth {
  ImuHealthStatus evaluate(
      bool spiReadOk,
      const ImuSample& sample
  );
}