#include <Arduino.h>

#include "src/board/BoardPins.h"
#include "src/config/DronePetConfig.h"

#include "src/tof/Tof.h"
#include "src/tof/TofHealth.h"

namespace {

  uint32_t lastTofTelemetryMs = 0;

  void printTofTelemetry() {

    const TofSample& sample = Tof::latestSample();

    const TofHealthStatus health = TofHealth::evaluate(
        Tof::initialized(),
        Tof::hasSample(),
        Tof::communicationError(),
        sample
    );

    Serial.print("tof=");

    if (!health.sampleReceived) {
      Serial.print("NA");
    }
    else {
      Serial.print(sample.distanceMm);
      Serial.print("mm");
    }

    Serial.print(" status=");

    if (!health.sampleReceived) {
      Serial.print("NO_SAMPLE");
    }
    else {
      Serial.print(Tof::rangeStatusToString(sample.rangeStatus));
    }

    Serial.print(" health=");

    if (health.healthy()) {
      Serial.print("OK");
    }
    else if (!health.initialized) {
      Serial.print("NOT_INITIALIZED");
    }
    else if (health.communicationError) {
      Serial.print("I2C_ERROR");
    }
    else if (!health.sampleReceived) {
      Serial.print("NO_SAMPLE");
    }
    else if (!health.sampleFresh) {
      Serial.print("STALE");
    }
    else if (!health.rangeValid) {
      Serial.print("RANGE_INVALID");
    }
    else if (!health.distanceAboveMinimum) {
      Serial.print("TOO_CLOSE");
    }
    else {
      Serial.print("UNKNOWN");
    }

    Serial.println();
  }

}

void setup() {

  rgbLedWrite(STATUS_LED, 0, 0, 0);

  Serial.begin(DronePetConfig::SERIAL_BAUD);
  delay(1000);

  if (Tof::begin()) {
    Serial.println("VL53L1X initialized.");
  }
  else {
    Serial.println("VL53L1X initialization failed.");
  }

  Serial.println("Drone Pet hardware skeleton ready.");
  Serial.println("Custom flight controller removed; motors are not driven.");
}

void loop() {

  Tof::update();

  const uint32_t nowMs = millis();

  if (
      nowMs - lastTofTelemetryMs
      < DronePetConfig::TOF_TELEMETRY_INTERVAL_MS
  ) {
    return;
  }

  lastTofTelemetryMs = nowMs;
  printTofTelemetry();
}
