#include <Arduino.h>

#include "FlightLoop.h"

namespace {
  constexpr uint32_t LOOP_RATE_HZ = 200;
  constexpr uint32_t LOOP_PERIOD_US =
      1'000'000 / LOOP_RATE_HZ;

  constexpr uint32_t TELEMETRY_RATE_HZ = 10;
  constexpr uint32_t TELEMETRY_INTERVAL_LOOPS =
      LOOP_RATE_HZ / TELEMETRY_RATE_HZ;

  FlightLoopStats timingStats;

  uint32_t previousLoopStartUs = 0;
  uint32_t nextLoopStartUs = 0;
  uint32_t telemetryLoopCounter = 0;
  bool telemetryPending = false;

  void recordDt(uint32_t dtUs) {
    if (dtUs < timingStats.minDtUs) {
      timingStats.minDtUs = dtUs;
    }

    if (dtUs > timingStats.maxDtUs) {
      timingStats.maxDtUs = dtUs;
    }

    timingStats.totalDtUs += dtUs;
    timingStats.sampleCount++;
  }
}

void FlightLoop::begin() {
  const uint32_t nowUs = micros();

  previousLoopStartUs = nowUs;
  nextLoopStartUs = nowUs + LOOP_PERIOD_US;

  timingStats = FlightLoopStats{};
  telemetryLoopCounter = 0;
  telemetryPending = false;
}

bool FlightLoop::beginIteration(
    FlightLoopIteration& iteration
) {
  const uint32_t nowUs = micros();

  // A negative signed difference means the next scheduled start time
  // has not arrived yet.
  if (static_cast<int32_t>(nowUs - nextLoopStartUs) < 0) {
    return false;
  }

  iteration.startUs = nowUs;
  iteration.dtUs = nowUs - previousLoopStartUs;
  iteration.dtSeconds =
      static_cast<float>(iteration.dtUs) * 0.000001f;

  previousLoopStartUs = nowUs;

  // Advance from the previous deadline rather than from the actual start
  // time so small scheduling errors do not accumulate into long-term drift.
  nextLoopStartUs += LOOP_PERIOD_US;

  // Skip any deadlines that have already expired instead of immediately
  // running multiple catch-up iterations with almost no time between them.
  while (
      static_cast<int32_t>(nowUs - nextLoopStartUs) >= 0
  ) {
    nextLoopStartUs += LOOP_PERIOD_US;
    timingStats.missedDeadlines++;
  }

  recordDt(iteration.dtUs);

  telemetryLoopCounter++;

  if (telemetryLoopCounter >= TELEMETRY_INTERVAL_LOOPS) {
    telemetryLoopCounter = 0;
    telemetryPending = true;
  }

  return true;
}

void FlightLoop::endIteration(
    const FlightLoopIteration& iteration
) {
  const uint32_t executionUs =
      micros() - iteration.startUs;

  if (executionUs > timingStats.maxExecutionUs) {
    timingStats.maxExecutionUs = executionUs;
  }

  // An iteration longer than the complete loop period cannot finish
  // before the next iteration is scheduled to begin.
  if (executionUs > LOOP_PERIOD_US) {
    timingStats.executionOverruns++;
  }
}

bool FlightLoop::telemetryDue() {
  if (!telemetryPending) {
    return false;
  }

  telemetryPending = false;
  return true;
}

const FlightLoopStats& FlightLoop::stats() {
  return timingStats;
}

float FlightLoop::averageDtUs() {
  if (timingStats.sampleCount == 0) {
    return 0.0f;
  }

  return static_cast<float>(timingStats.totalDtUs)
      / static_cast<float>(timingStats.sampleCount);
}

uint32_t FlightLoop::rateHz() {
  return LOOP_RATE_HZ;
}

uint32_t FlightLoop::periodUs() {
  return LOOP_PERIOD_US;
}
