#include <Arduino.h>

#include "FlightController.h"

#include "AltitudeController.h"
#include "AttitudeController.h"
#include "AttitudeEstimator.h"
#include "MotorMixer.h"

#include "../config/DronePetConfig.h"
#include "../imu/ImuHealth.h"
#include "../motors/Motors.h"

#include "../tof/Tof.h"
#include "../tof/TofHealth.h"


namespace {

  bool controllerArmed = false;


  void writeMotorOutputs(
      const MotorMixOutput& mix
  ) {

    const float maxThrottle =
        DronePetConfig::MAX_MOTOR_THROTTLE_PERCENT;


    Motors::setThrottlePercent(
        1,
        constrain(
            mix.motor1Percent,
            0.0f,
            maxThrottle
        )
    );

    Motors::setThrottlePercent(
        2,
        constrain(
            mix.motor2Percent,
            0.0f,
            maxThrottle
        )
    );

    Motors::setThrottlePercent(
        3,
        constrain(
            mix.motor3Percent,
            0.0f,
            maxThrottle
        )
    );

    Motors::setThrottlePercent(
        4,
        constrain(
            mix.motor4Percent,
            0.0f,
            maxThrottle
        )
    );
  }

}


void FlightController::begin() {

  controllerArmed = false;

  AttitudeController::begin();
  AltitudeController::begin();

  Motors::disarm();
}


void FlightController::arm() {

  AttitudeController::begin();
  AltitudeController::begin();

  Motors::arm();

  controllerArmed = true;
}


void FlightController::disarm() {

  controllerArmed = false;

  Motors::disarm();

  AttitudeController::begin();
  AltitudeController::reset();
}


bool FlightController::armed() {
  return controllerArmed;
}

void FlightController::setTargetAltitudeMm(
    float targetHeightMm
) {

  AltitudeController::setTargetHeightMm(
      targetHeightMm
  );
}

void FlightController::update(
    const AttitudeState& attitude,
    const ImuHealthStatus& imuHealth,
    const TofSample& tofSample,
    const TofHealthStatus& tofHealth,
    float dtSeconds
) {

  if (!controllerArmed) {
    return;
  }


  const bool tofUsable =
      tofHealth.responsive()
      && tofHealth.rangeValid;


  if (!attitude.ready
      || !imuHealth.healthy()
      || !tofUsable
      || !isfinite(dtSeconds)
      || dtSeconds <= 0.0f) {

    disarm();

    return;
  }


  const AttitudeControlOutput attitudeControl =
      AttitudeController::update(
          attitude,
          dtSeconds
      );


  const AltitudeControlOutput altitudeControl =
      AltitudeController::update(
          tofSample,
          tofHealth
      );


  if (!attitudeControl.valid
      || !altitudeControl.valid) {

    disarm();

    return;
  }


  const MotorMixOutput motorMix =
      MotorMixer::mix(
          altitudeControl.collectiveThrottlePercent,
          attitudeControl
      );


  if (!motorMix.valid) {

    disarm();

    return;
  }


  writeMotorOutputs(motorMix);
}