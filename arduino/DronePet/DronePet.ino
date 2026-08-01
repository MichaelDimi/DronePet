#include <Arduino.h>
#include <math.h>

#include "Imu.h"

namespace {
  constexpr size_t STATIONARY_CHECK_SAMPLES = 100;
  constexpr uint32_t STATIONARY_CHECK_INTERVAL_MS = 3;
  constexpr float STATIONARY_MEAN_LIMIT_DPS = 0.5f;

  void printVector(const Vector3& value) {
    Serial.print(value.x, 4);
    Serial.print(", ");
    Serial.print(value.y, 4);
    Serial.print(", ");
    Serial.println(value.z, 4);
  }

  Vector3 measureStationaryGyroMean() {
    Vector3 sum;

    for (size_t i = 0; i < STATIONARY_CHECK_SAMPLES; i++) {
      ImuSample sample;
      Imu::readSample(sample);
      sum.x += sample.gyroDps.x;
      sum.y += sample.gyroDps.y;
      sum.z += sample.gyroDps.z;
      delay(STATIONARY_CHECK_INTERVAL_MS);
    }

    const float sampleCount = static_cast<float>(STATIONARY_CHECK_SAMPLES);
    Vector3 mean;
    mean.x = sum.x / sampleCount;
    mean.y = sum.y / sampleCount;
    mean.z = sum.z / sampleCount;
    return mean;
  }

  bool isStationaryMeanNearZero(const Vector3& mean) {
    return fabsf(mean.x) <= STATIONARY_MEAN_LIMIT_DPS
        && fabsf(mean.y) <= STATIONARY_MEAN_LIMIT_DPS
        && fabsf(mean.z) <= STATIONARY_MEAN_LIMIT_DPS;
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  if (!Imu::begin()) {
    Serial.println("IMU initialization failed.");

    while (true) {
      delay(1000);
    }
  }

  Serial.println("ISM330DHCX initialized.");
  Serial.println("Keep the drone completely still. Calibrating gyro...");
  delay(1000);

  Imu::calibrateGyro();

  Serial.print("Gyro bias (dps): ");
  printVector(Imu::gyroBiasDps());

  const Vector3 stationaryMean = measureStationaryGyroMean();
  Serial.print("Stationary gyro mean after calibration (dps): ");
  printVector(stationaryMean);

  if (isStationaryMeanNearZero(stationaryMean)) {
    Serial.println("Gyro calibration check: PASS");
  } else {
    Serial.println("Gyro calibration check: FAIL - restart and keep the drone still.");
  }

  Serial.println();
}

void loop() {
  ImuSample sample;
  Imu::readSample(sample);

  Serial.print("Gyro: ");
  Serial.print(sample.gyroDps.x);
  Serial.print(", ");
  Serial.print(sample.gyroDps.y);
  Serial.print(", ");
  Serial.println(sample.gyroDps.z);

  Serial.print("Accel: ");
  Serial.print(sample.accelG.x);
  Serial.print(", ");
  Serial.print(sample.accelG.y);
  Serial.print(", ");
  Serial.println(sample.accelG.z);

  Serial.println();
  delay(100);
}
