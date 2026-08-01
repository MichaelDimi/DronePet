#pragma once

#include <Arduino.h>

struct FlightLoopIteration {
  uint32_t startUs = 0;
  uint32_t dtUs = 0;
  float dtSeconds = 0.0f;
};

struct FlightLoopStats {
  uint32_t minDtUs = UINT32_MAX;
  uint32_t maxDtUs = 0;
  uint64_t totalDtUs = 0;
  uint32_t sampleCount = 0;

  uint32_t maxExecutionUs = 0;
  uint32_t executionOverruns = 0;
  uint32_t missedDeadlines = 0;
};

namespace FlightLoop {
  void begin();

  // Return true only when the next scheduled flight-loop period has arrived.
  bool beginIteration(FlightLoopIteration& iteration);

  // Record how long the complete scheduled iteration took to execute.
  void endIteration(const FlightLoopIteration& iteration);

  // Return true once every configured telemetry interval.
  bool telemetryDue();

  const FlightLoopStats& stats();
  float averageDtUs();

  uint32_t rateHz();
  uint32_t periodUs();
}
