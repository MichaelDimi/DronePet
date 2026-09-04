#include <Arduino.h>

#include "Motors.h"
#include "../board/BoardPins.h"

namespace {
  constexpr uint32_t PWM_FREQUENCY_HZ = 50;
  constexpr uint8_t PWM_RESOLUTION_BITS = 14;

  // Standard RC-style PWM range. This is a conservative starting point for
  // bench testing the ESC; we can switch the implementation to DShot later
  // without changing the rest of the firmware's motor API.
  constexpr uint16_t MIN_PULSE_US = 1000;
  constexpr uint16_t MAX_PULSE_US = 2000;

  constexpr uint32_t PWM_PERIOD_US = 1'000'000 / PWM_FREQUENCY_HZ;
  constexpr uint32_t PWM_MAX_DUTY = (1UL << PWM_RESOLUTION_BITS) - 1UL;

  bool motorsArmed = false;

  float commandedThrottlePercent[4] = {
      0.0f,
      0.0f,
      0.0f,
      0.0f
  };

  constexpr int MOTOR_PINS[4] = {
      ESC_1,
      ESC_2,
      ESC_3,
      ESC_4
  };

  uint32_t pulseUsToDuty(uint16_t pulseUs) {
    return static_cast<uint32_t>(
        (static_cast<uint64_t>(pulseUs) * PWM_MAX_DUTY)
        / PWM_PERIOD_US
    );
  }

  void writePulseUs(uint8_t motorNumber, uint16_t pulseUs) {
    if (motorNumber < 1 || motorNumber > 4) {
      return;
    }

    const int pin = MOTOR_PINS[motorNumber - 1];
    ledcWrite(pin, pulseUsToDuty(pulseUs));
  }
}

void Motors::begin() {
  motorsArmed = false;

  for (const int pin : MOTOR_PINS) {
    const bool attached =
        ledcAttach(pin, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS);

    Serial.print("ESC GPIO ");
    Serial.print(pin);
    Serial.print(" PWM attach: ");
    Serial.println(attached ? "OK" : "FAILED");

    if (attached) {
      const bool written =
          ledcWrite(pin, pulseUsToDuty(MIN_PULSE_US));

      Serial.print("  idle write: ");
      Serial.println(written ? "OK" : "FAILED");
    }
  }

  for (float& throttle : commandedThrottlePercent) {
    throttle = 0.0f;
  }
}

void Motors::arm() {
  motorsArmed = true;
}

void Motors::disarm() {
  motorsArmed = false;
  stopAll();
}

bool Motors::armed() {
  return motorsArmed;
}

void Motors::setThrottlePercent(
    uint8_t motorNumber,
    float percent
) {
  if (motorNumber < 1 || motorNumber > 4) {
    return;
  }

  if (!motorsArmed) {
    stop(motorNumber);
    return;
  }

  percent = constrain(percent, 0.0f, 100.0f);

  commandedThrottlePercent[motorNumber - 1] = percent;

  const uint16_t pulseUs = static_cast<uint16_t>(
      MIN_PULSE_US
      + ((MAX_PULSE_US - MIN_PULSE_US) * percent / 100.0f)
  );

  writePulseUs(motorNumber, pulseUs);
}

float Motors::throttlePercent(uint8_t motorNumber) {
  if (motorNumber < 1 || motorNumber > 4) {
    return 0.0f;
  }

  return commandedThrottlePercent[motorNumber - 1];
}

void Motors::stop(uint8_t motorNumber) {
  if (motorNumber < 1 || motorNumber > 4) {
    return;
  }

  commandedThrottlePercent[motorNumber - 1] = 0.0f;
  writePulseUs(motorNumber, MIN_PULSE_US);
}

void Motors::stopAll() {
  for (uint8_t motor = 1; motor <= 4; ++motor) {
    stop(motor);
  }
}
