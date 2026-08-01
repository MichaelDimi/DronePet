#include <Arduino.h>
#include <math.h>

#include "Imu.h"
#include "ImuStartup.h"

namespace {
  constexpr size_t STATIONARY_CHECK_SAMPLES = 100;
  constexpr uint32_t STATIONARY_CHECK_INTERVAL_MS = 3;
  constexpr float STATIONARY_MEAN_LIMIT_DPS = 0.5f;

  bool measureStationaryGyroMean(Vector3& mean) {
    Vector3 sum;

    for (size_t i = 0; i < STATIONARY_CHECK_SAMPLES; i++) {
      ImuSample sample;

      if (!Imu::readSample(sample)) {
        return false;
      }

      sum.x += sample.gyroDps.x;
      sum.y += sample.gyroDps.y;
      sum.z += sample.gyroDps.z;

      delay(STATIONARY_CHECK_INTERVAL_MS);
    }

    const float sampleCount =
        static_cast<float>(STATIONARY_CHECK_SAMPLES);

    mean = {
      sum.x / sampleCount,
      sum.y / sampleCount,
      sum.z / sampleCount
    };

    return true;
  }

  bool isStationaryMeanNearZero(const Vector3& mean) {
    return fabsf(mean.x) <= STATIONARY_MEAN_LIMIT_DPS
        && fabsf(mean.y) <= STATIONARY_MEAN_LIMIT_DPS
        && fabsf(mean.z) <= STATIONARY_MEAN_LIMIT_DPS;
  }
}

bool ImuStartup::initialize() {
  return Imu::begin();
}

GyroCalibrationResult ImuStartup::calibrateAndCheck() {
  delay(1000);

  GyroCalibrationResult result;

  if (!Imu::calibrateGyro()) {
    return result;
  }

  result.biasDps = Imu::gyroBiasDps();

  if (!measureStationaryGyroMean(result.stationaryMeanDps)) {
    return result;
  }

  result.readSucceeded = true;
  result.passed =
      isStationaryMeanNearZero(result.stationaryMeanDps);

  return result;
}
