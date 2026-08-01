#include <Arduino.h>

#include "src/config/DronePetConfig.h"
#include "src/diagnostics/AxisTest.h"
#include "src/flight/FlightLoop.h"
#include "src/imu/Imu.h"
#include "src/imu/ImuStartup.h"
#include "src/telemetry/Telemetry.h"
#include "src/imu/ImuHealth.h"

void setup() {
  Serial.begin(DronePetConfig::SERIAL_BAUD);
  delay(1000);

  if (!ImuStartup::initialize()) {
    Telemetry::printImuInitializationFailed();

    while (true) {
      delay(1000);
    }
  }

  if constexpr (DronePetConfig::TELEMETRY_STARTUP) {
    Telemetry::printImuInitialized();
    Telemetry::printGyroCalibrationPrompt();
  }

  const GyroCalibrationResult calibration =
      ImuStartup::calibrateAndCheck();

  if constexpr (DronePetConfig::TELEMETRY_STARTUP) {
    Telemetry::printGyroCalibrationResult(calibration);
  }

  // A failed stationary calibration must prevent the future flight
  // controller from starting with an invalid gyro bias.
  if (!calibration.passed) {
    while (true) {
      delay(1000);
    }
  }

  if (DronePetConfig::RUN_AXIS_TEST) {
    AxisTest::start();
  } else {
    FlightLoop::begin();

    if constexpr (DronePetConfig::TELEMETRY_STARTUP) {
      Telemetry::printFlightLoopStarted();
    }
  }
}

void loop() {
  if (DronePetConfig::RUN_AXIS_TEST) {
    ImuSample sample{};

    if (!Imu::readSample(sample)) {
      delay(20);
      return;
    }

    AxisTest::update(sample.gyroDps);

    delay(20);
    return;
  }

  FlightLoopIteration iteration;

  // Arduino repeatedly calls loop(), but flight work begins only when
  // the next scheduled 5 ms period has arrived.
  if (!FlightLoop::beginIteration(iteration)) {
    return;
  }

  ImuSample sample{};

  const bool spiReadOk = Imu::readSample(sample);

  const ImuHealthStatus health = ImuHealth::evaluate(spiReadOk, sample);

  // TODO: Prevent arming or enter a safe state when health is not acceptable.
  // For now, record and report the condition through telemetry.

  // TODO: Pass sample and iteration.dtSeconds into the attitude
  // estimator and flight controllers.

  if constexpr (DronePetConfig::TELEMETRY_ENABLED) {
    if (FlightLoop::telemetryDue()) {
      Telemetry::printFlightSample(
          sample,
          iteration,
          health
      );
    }
  }

  // This is deliberately called after telemetry so serial-output delays
  // are included in the measured execution time and overrun detection.
  FlightLoop::endIteration(iteration);
}
