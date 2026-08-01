#pragma once

#include <Arduino.h>

namespace DronePetConfig {
  constexpr bool RUN_AXIS_TEST = false;

  constexpr uint32_t SERIAL_BAUD = 921600;

  // Master switch for periodic flight telemetry.
  constexpr bool TELEMETRY_ENABLED = true;

  // Individual sections of the periodic telemetry line.
  constexpr bool TELEMETRY_IMU_SAMPLE = true;
  constexpr bool TELEMETRY_IMU_HEALTH = true;
  constexpr bool TELEMETRY_LOOP_TIMING = true;

  // Startup messages, including IMU initialization and gyro calibration.
  constexpr bool TELEMETRY_STARTUP = true;
}
