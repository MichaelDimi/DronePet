#include <Arduino.h>

#include "src/config/DronePetConfig.h"

#include "src/flight/FlightSupervisor.h"

#include "src/control/AttitudeEstimator.h"

#include "src/flight/FlightLoop.h"

#include "src/motors/Motors.h"

#include "src/imu/Imu.h"
#include "src/imu/ImuHealth.h"
#include "src/imu/ImuStartup.h"

#include "src/telemetry/Telemetry.h"

#include "src/tof/Tof.h"
#include "src/tof/TofHealth.h"
#include "src/tof/TofStartup.h"

#include "src/board/BoardPins.h"

void setup() {

  rgbLedWrite(STATUS_LED, 0, 0, 0);

  Serial.begin(DronePetConfig::SERIAL_BAUD);
  delay(1000);

  Motors::begin();

  // IMU startup
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

  if (!calibration.passed) {
    while (true) {
      delay(1000);
    }
  }

  // ToF startup
  const bool tofInitialized = TofStartup::initialize();
  if constexpr (DronePetConfig::TELEMETRY_STARTUP) {

    if (tofInitialized) {
      Telemetry::printTofInitialized();
    }
    else {
      Telemetry::printTofInitializationFailed();
    }
  }

  AttitudeEstimator::begin();
  FlightSupervisor::begin();
  FlightLoop::begin();

  if constexpr (DronePetConfig::TELEMETRY_STARTUP) {
    Telemetry::printFlightLoopStarted();
  }
}

void loop() {

  // 1. Wait for the next 200 Hz iteration.
  FlightLoopIteration iteration;

  if (!FlightLoop::beginIteration(iteration)) {
    return;
  }


  // 2. Read sensors.
  ImuSample imuSample{};
  const bool imuReadOk = Imu::readSample(imuSample);

  Tof::update();
  const TofSample& tofSample = Tof::latestSample();


  // 3. Check sensor health.
  const ImuHealthStatus imuHealth =
      ImuHealth::evaluate(imuReadOk, imuSample);

  const TofHealthStatus tofHealth =
      TofHealth::evaluate(
          Tof::initialized(),
          Tof::hasSample(),
          Tof::communicationError(),
          tofSample
      );


  // 4. Estimate attitude.
  if (imuReadOk) {
    AttitudeEstimator::update(
        imuSample,
        iteration.dtSeconds
    );
  }

  const AttitudeState& attitude =
      AttitudeEstimator::state();


  // 5. Run flight supervision + control.

  FlightSupervisor::update(
      attitude,
      imuHealth,
      tofSample,
      tofHealth,
      iteration.dtSeconds
  );


  // 6. Telemetry.
  if constexpr (DronePetConfig::TELEMETRY_ENABLED) {

    if (FlightLoop::telemetryDue()) {

      Telemetry::printFlightSample(
          imuSample,
          iteration,
          imuHealth,
          tofSample,
          tofHealth,
          attitude
      );
    }
  }


  // 7. Measure complete iteration time.
  FlightLoop::endIteration(iteration);
}
