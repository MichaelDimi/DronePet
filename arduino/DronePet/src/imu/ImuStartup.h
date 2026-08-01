#pragma once

#include "../common/MathTypes.h"

struct GyroCalibrationResult {
  Vector3 biasDps;
  Vector3 stationaryMeanDps;

  bool readSucceeded = false;
  bool passed = false;
};

namespace ImuStartup {
  // Initialize and configure the physical IMU.
  bool initialize();

  // Measure the stationary gyro bias, apply it, and verify that the
  // resulting stationary output is centered near zero.
  GyroCalibrationResult calibrateAndCheck();
}
