#include <Arduino.h>

#include "TelemetrySections.h"
#include "../control/AttitudeEstimator.h"

void TelemetrySections::printAttitude(
    const AttitudeState& attitude
) {
  if (!attitude.ready) {
    Serial.print("attitude=CALIBRATING");
    return;
  }

  Serial.print("roll=");
  Serial.print(attitude.rollDeg, 1);

  Serial.print(" pitch=");
  Serial.print(attitude.pitchDeg, 1);
}
