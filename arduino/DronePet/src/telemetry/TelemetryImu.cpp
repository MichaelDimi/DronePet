#include <Arduino.h>

#include "Telemetry.h"
#include "TelemetrySections.h"
#include "../flight/FlightLoop.h"
#include "../imu/Imu.h"
#include "../imu/ImuHealth.h"
#include "../imu/ImuStartup.h"

namespace {
  void printVector(const Vector3& value, int decimalPlaces) {
    Serial.print(value.x, decimalPlaces);
    Serial.print(", ");
    Serial.print(value.y, decimalPlaces);
    Serial.print(", ");
    Serial.println(value.z, decimalPlaces);
  }
}

void Telemetry::printImuInitializationFailed() {
  Serial.println("IMU initialization failed.");
}

void Telemetry::printImuInitialized() {
  Serial.println("ISM330DHCX initialized.");
}

void Telemetry::printGyroCalibrationPrompt() {
  Serial.println(
      "Keep the drone completely still. Calibrating gyro..."
  );
}

void Telemetry::printGyroCalibrationResult(
    const GyroCalibrationResult& result
) {
  if (!result.readSucceeded) {
    Serial.println(
        "Gyro calibration check: FAIL - IMU read failed."
    );
    Serial.println();
    return;
  }

  Serial.print("Gyro bias (dps): ");
  printVector(result.biasDps, 4);

  Serial.print(
      "Stationary gyro mean after calibration (dps): "
  );
  printVector(result.stationaryMeanDps, 4);

  if (result.passed) {
    Serial.println("Gyro calibration check: PASS");
  } else {
    Serial.println(
        "Gyro calibration check: FAIL - restart and keep "
        "the drone still."
    );
  }

  Serial.println();
}

void TelemetrySections::printImuSample(
    const ImuSample& sample,
    const FlightLoopIteration& iteration
) {
  Serial.print("dt=");
  Serial.print(iteration.dtUs);

  Serial.print(" gyro=");
  Serial.print(sample.gyroDps.x, 2);
  Serial.print(",");
  Serial.print(sample.gyroDps.y, 2);
  Serial.print(",");
  Serial.print(sample.gyroDps.z, 2);

  Serial.print(" accel=");
  Serial.print(sample.accelG.x, 2);
  Serial.print(",");
  Serial.print(sample.accelG.y, 2);
  Serial.print(",");
  Serial.print(sample.accelG.z, 2);
}

void TelemetrySections::printImuHealth(
    const ImuHealthStatus& health
) {
  Serial.print("|a|=");
  Serial.print(health.accelMagnitudeG, 3);

  Serial.print(" imuHealth=");

  if (health.healthy()) {
    Serial.print("OK");
  } else if (!health.spiReadOk) {
    Serial.print("SPI_FAIL");
  } else {
    if (!health.accelerationValid) {
      Serial.print("ACCEL_RANGE,");
    }

    if (health.gyroSaturated) {
      Serial.print("GYRO_SAT,");
    }

    if (!health.accelMagnitudeValid) {
      Serial.print("ACCEL_MAG,");
    }
  }
}
