#pragma once

#include <Arduino.h>

namespace DronePetConfig {
  constexpr uint32_t SERIAL_BAUD = 921600;

  // Master switch for periodic flight telemetry.
  constexpr bool TELEMETRY_ENABLED = true;

  // Individual sections of the periodic telemetry line.
  constexpr bool TELEMETRY_IMU_SAMPLE = false;
  constexpr bool TELEMETRY_IMU_HEALTH = false;
  constexpr bool TELEMETRY_TOF_SAMPLE = true;
  constexpr bool TELEMETRY_TOF_HEALTH = true;
  constexpr bool TELEMETRY_MOTORS = true;
  constexpr bool TELEMETRY_LOOP_TIMING = false;
  constexpr bool TELEMETRY_ATTITUDE = true;

  // Startup messages, including IMU initialization and gyro calibration.
  constexpr bool TELEMETRY_STARTUP = true;

  // Safety limits.
  // First low-altitude flight-test limits.
  constexpr float MAX_MOTOR_THROTTLE_PERCENT = 45.0f;

  // Altitude target is the downward ToF distance to the floor.
  constexpr float HOVER_ALTITUDE_MM = 120.0f;

  // Initial estimate only. We will adjust this after seeing
  // where this specific drone actually hovers.
  constexpr float HOVER_THROTTLE_PERCENT = 30.0f;

  constexpr float MIN_COLLECTIVE_THROTTLE_PERCENT = 20.0f;
  constexpr float MAX_COLLECTIVE_THROTTLE_PERCENT = 40.0f;

  // Temporary autonomous first-flight envelope.
  constexpr uint32_t ARM_WARNING_MS = 5000;
  constexpr uint32_t FLIGHT_TEST_DURATION_MS = 2000;

  constexpr float MAX_SAFE_TILT_DEG = 35.0f;

  // Controlled landing.
  constexpr float TAKEOFF_RATE_MM_PER_S = 60.0f;
  constexpr float LANDING_TARGET_ALTITUDE_MM = 50.0f;
  constexpr float LANDING_RATE_MM_PER_S = 100.0f;

  // Once the ToF reports we're this close to the floor,
  // hold that condition briefly before shutting the motors off.
  constexpr float LANDING_DISARM_HEIGHT_MM = 45.0f;
  constexpr uint32_t LANDING_CONFIRM_MS = 300;

  // Backup only: prevents a failed landing from running forever.
  // At our 300 mm test altitude, this is still a small fall if triggered.
  constexpr uint32_t LANDING_TIMEOUT_MS = 8000;
}
