#include <Arduino.h>

#include "AltitudeController.h"
#include "PidController.h"

#include "../config/DronePetConfig.h"
#include "../tof/Tof.h"
#include "../tof/TofHealth.h"


namespace {

  // Very conservative first-flight altitude controller.
  //
  // 100 mm below target -> +3 percentage points throttle.
  constexpr PidGains ALTITUDE_GAINS{
      .kp = 0.03f,
      .ki = 0.0f,
      .kd = 0.0f
  };

  constexpr float MAX_CORRECTION_PERCENT = 12.0f;

  constexpr float INTEGRAL_LIMIT = 1000.0f;

  constexpr float DEFAULT_SAMPLE_DT_S = 0.05f;

    float requestedTargetHeightMm = DronePetConfig::HOVER_ALTITUDE_MM;

    // NAN means we haven't received our first valid ToF
    // measurement yet.
    float activeTargetHeightMm = NAN;

  PidController altitudePid(
      ALTITUDE_GAINS,
      MAX_CORRECTION_PERCENT,
      INTEGRAL_LIMIT
  );


  uint32_t lastProcessedTimestampUs = 0;

  AltitudeControlOutput currentOutput;


  bool usable(
      const TofHealthStatus& health
  ) {

    // We deliberately don't require distanceAboveMinimum here.
    //
    // The sensor may be physically closer than 40 mm to the
    // floor while the drone is landed.
    return health.responsive()
        && health.rangeValid;
  }
}


void AltitudeController::begin() {
  reset();
}


void AltitudeController::reset() {

  altitudePid.reset();

  lastProcessedTimestampUs = 0;

  currentOutput = {};

  requestedTargetHeightMm =
    DronePetConfig::HOVER_ALTITUDE_MM;

    activeTargetHeightMm = NAN;
}

void AltitudeController::setTargetHeightMm(
    float targetHeightMm
) {

  if (!isfinite(targetHeightMm)) {
    return;
  }

  requestedTargetHeightMm =
    max(0.0f, targetHeightMm);
}


float AltitudeController::targetHeightMm() {

  if (isfinite(activeTargetHeightMm)) {
    return activeTargetHeightMm;
  }

  return requestedTargetHeightMm;
}

AltitudeControlOutput AltitudeController::update(
    const TofSample& sample,
    const TofHealthStatus& health
) {

  if (!usable(health)) {

    currentOutput.valid = false;

    return currentOutput;
  }


  // The flight loop is 200 Hz but ToF measurements arrive
  // approximately every 50 ms. Don't run the altitude PID
  // repeatedly on the same measurement.
  if (sample.timestampUs == lastProcessedTimestampUs
      && currentOutput.valid) {

    return currentOutput;
  }


  lastProcessedTimestampUs = sample.timestampUs;


  float sampleDtSeconds = DEFAULT_SAMPLE_DT_S;

  if (sample.intervalUs > 0) {

    sampleDtSeconds =
        static_cast<float>(sample.intervalUs)
        * 0.000001f;
  }


  currentOutput.measuredHeightMm =
    static_cast<float>(sample.distanceMm);


    // On the first valid measurement after arming,
    // begin the altitude command exactly where the
    // drone currently is.
    if (!isfinite(activeTargetHeightMm)) {

    activeTargetHeightMm =
        currentOutput.measuredHeightMm;
    }


    // Smooth requested altitude changes. Use the gentler
    // takeoff rate when climbing and the configured landing
    // rate when descending.
    if (
        requestedTargetHeightMm
        > activeTargetHeightMm
    ) {

    const float maxStepMm =
        DronePetConfig::TAKEOFF_RATE_MM_PER_S
        * sampleDtSeconds;

    const float remainingMm =
        requestedTargetHeightMm
        - activeTargetHeightMm;

    activeTargetHeightMm +=
        min(remainingMm, maxStepMm);
    }
    else if (
        requestedTargetHeightMm
        < activeTargetHeightMm
    ) {

    const float maxStepMm =
        DronePetConfig::LANDING_RATE_MM_PER_S
        * sampleDtSeconds;

    const float remainingMm =
        activeTargetHeightMm
        - requestedTargetHeightMm;

    activeTargetHeightMm -=
        min(remainingMm, maxStepMm);
    }


    currentOutput.targetHeightMm =
        activeTargetHeightMm;


    currentOutput.errorMm =
        currentOutput.targetHeightMm
        - currentOutput.measuredHeightMm;


  currentOutput.correctionPercent =
      altitudePid.update(
          currentOutput.errorMm,
          sampleDtSeconds
      );


  currentOutput.collectiveThrottlePercent =
      constrain(
          DronePetConfig::HOVER_THROTTLE_PERCENT
              + currentOutput.correctionPercent,

          DronePetConfig::MIN_COLLECTIVE_THROTTLE_PERCENT,

          DronePetConfig::MAX_COLLECTIVE_THROTTLE_PERCENT
      );


  currentOutput.valid = true;

  return currentOutput;
}
