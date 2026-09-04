#include <Arduino.h>

#include "TelemetrySections.h"
#include "../motors/Motors.h"

void TelemetrySections::printMotors() {
  Serial.print("motors=");
  Serial.print(Motors::armed() ? "ARMED" : "DISARMED");

  for (uint8_t motor = 1; motor <= 4; ++motor) {
    Serial.print(" m");
    Serial.print(motor);
    Serial.print("=");
    Serial.print(Motors::throttlePercent(motor), 1);
    Serial.print("%");
  }
}