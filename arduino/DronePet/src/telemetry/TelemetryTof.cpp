#include <Arduino.h>

#include "Telemetry.h"
#include "TelemetrySections.h"
#include "../tof/Tof.h"
#include "../tof/TofHealth.h"

namespace {
  void printHealthState(const TofHealthStatus& health) {
    if (health.healthy()) {
      Serial.print("OK");
    } else if (!health.initialized) {
      Serial.print("NOT_INITIALIZED");
    } else if (health.communicationError) {
      Serial.print("I2C_ERROR");
    } else if (!health.sampleReceived) {
      Serial.print("NO_SAMPLE");
    } else if (!health.sampleFresh) {
      Serial.print("STALE");
    } else if (!health.rangeValid) {
      Serial.print("RANGE_INVALID");
    } else if (!health.distanceAboveMinimum) {
      Serial.print("TOO_CLOSE");
    } else {
      Serial.print("UNKNOWN");
    }
  }
}

void Telemetry::printTofInitializationFailed() {
  Serial.println("VL53L1X initialization failed.");
}

void Telemetry::printTofInitialized() {
  Serial.println("VL53L1X initialized.");
}

void TelemetrySections::printTofSample(
    const TofSample& sample,
    const TofHealthStatus& health
) {
  if (!health.sampleReceived) {
    Serial.print("tof=NA tofStatus=NA tofDt=NA");
    return;
  }

  Serial.print("tof=");
  Serial.print(sample.distanceMm);
  Serial.print("mm");

  Serial.print(" tofStatus=\"");
  Serial.print(
      Tof::rangeStatusToString(
          sample.rangeStatus
      )
  );
  Serial.print("\"");

  Serial.print(" tofDt=");

  if (sample.intervalUs == 0) {
    Serial.print("NA");
  } else {
    Serial.print(sample.intervalUs / 1000);
    Serial.print("ms");
  }
}

void TelemetrySections::printTofHealth(
    const TofHealthStatus& health
) {
  Serial.print("tofAge=");

  if (health.sampleReceived) {
    Serial.print(health.sampleAgeMs);
    Serial.print("ms");
  } else {
    Serial.print("NA");
  }

  Serial.print(" tofHealth=");
  printHealthState(health);
}
