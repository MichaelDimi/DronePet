#include <Arduino.h>

#include "Telemetry.h"
#include "TelemetrySections.h"
#include "../config/DronePetConfig.h"
#include "../flight/FlightLoop.h"
#include "../control/AttitudeEstimator.h"

namespace {
  void printSectionSeparator(bool wroteSection) {
    if (wroteSection) {
      Serial.print(" ");
    }
  }

  void printLoopTiming() {
    const FlightLoopStats& stats = FlightLoop::stats();

    Serial.print("timing=");
    Serial.print(stats.minDtUs);
    Serial.print("/");
    Serial.print(FlightLoop::averageDtUs(), 1);
    Serial.print("/");
    Serial.print(stats.maxDtUs);

    Serial.print(" execMax=");
    Serial.print(stats.maxExecutionUs);

    Serial.print(" overruns=");
    Serial.print(stats.executionOverruns);

    Serial.print(" missed=");
    Serial.print(stats.missedDeadlines);
  }
}

void Telemetry::printFlightLoopStarted() {
  Serial.println("Fixed-rate flight loop started.");

  Serial.print("Target rate: ");
  Serial.print(FlightLoop::rateHz());
  Serial.println(" Hz");

  Serial.print("Target period: ");
  Serial.print(FlightLoop::periodUs());
  Serial.println(" us");

  Serial.println();
}

void Telemetry::printFlightSample(
    const ImuSample& imuSample,
    const FlightLoopIteration& iteration,
    const ImuHealthStatus& imuHealth,
    const TofSample& tofSample,
    const TofHealthStatus& tofHealth,
    const AttitudeState& attitude
) {
  bool wroteSection = false;

  if constexpr (DronePetConfig::TELEMETRY_ATTITUDE) {
    TelemetrySections::printAttitude(attitude);
    wroteSection = true;
  }

  if constexpr (DronePetConfig::TELEMETRY_IMU_SAMPLE) {
    TelemetrySections::printImuSample(imuSample, iteration);
    wroteSection = true;
  }

  if constexpr (DronePetConfig::TELEMETRY_IMU_HEALTH) {
    printSectionSeparator(wroteSection);
    TelemetrySections::printImuHealth(imuHealth);
    wroteSection = true;
  }

  if constexpr (DronePetConfig::TELEMETRY_TOF_SAMPLE) {
    printSectionSeparator(wroteSection);
    TelemetrySections::printTofSample(tofSample, tofHealth);
    wroteSection = true;
  }

  if constexpr (DronePetConfig::TELEMETRY_TOF_HEALTH) {
    printSectionSeparator(wroteSection);
    TelemetrySections::printTofHealth(tofHealth);
    wroteSection = true;
  }

  if constexpr (DronePetConfig::TELEMETRY_MOTORS) {
    printSectionSeparator(wroteSection);
    TelemetrySections::printMotors();
    wroteSection = true;
  }

  if constexpr (DronePetConfig::TELEMETRY_LOOP_TIMING) {
    printSectionSeparator(wroteSection);
    printLoopTiming();
    wroteSection = true;
  }

  if (wroteSection) {
    Serial.println();
  }
}
