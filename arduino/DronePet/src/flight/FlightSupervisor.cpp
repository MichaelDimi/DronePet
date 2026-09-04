#include <Arduino.h>
#include <math.h>
#include <Preferences.h>

#include "FlightSupervisor.h"

#include "../board/BoardPins.h"
#include "../config/DronePetConfig.h"

#include "../control/AttitudeEstimator.h"
#include "../control/FlightController.h"

#include "../imu/ImuHealth.h"

#include "../tof/Tof.h"
#include "../tof/TofHealth.h"


namespace {

  enum class FaultReason {
    Tilt,
    Imu,
    Tof,
    Controller,
    LandingTimeout
  };

  FlightState currentState = FlightState::WaitingForReady;

  uint32_t stateStartedMs = 0;
  uint32_t landingNearGroundStartedMs = 0;
  bool landingNearGroundTimerRunning = false;

  AttitudeState latestAttitude;
  ImuHealthStatus latestImuHealth;
  bool latestFaultSnapshotValid = false;

  float peakAbsRollRateDps = 0.0f;
  float peakAbsPitchRateDps = 0.0f;
  float peakAccelMagnitudeG = 0.0f;

  void setLed(
      uint8_t red,
      uint8_t green,
      uint8_t blue
  ) {

    rgbLedWrite(
        STATUS_LED,
        red,
        green,
        blue
    );
  }

  bool tiltSafe(
      const AttitudeState& attitude
  ) {

    return fabsf(attitude.rollDeg)
            <= DronePetConfig::MAX_SAFE_TILT_DEG

        && fabsf(attitude.pitchDeg)
            <= DronePetConfig::MAX_SAFE_TILT_DEG;
  }


  bool sensorsReady(
      const AttitudeState& attitude,
      const ImuHealthStatus& imuHealth,
      const TofHealthStatus& tofHealth
  ) {

    return attitude.ready
        && imuHealth.healthy()
        && tofHealth.responsive()
        && tofHealth.rangeValid
        && tiltSafe(attitude);
  }

  const char* faultReasonName(
      FaultReason reason
  ) {

    switch (reason) {

      case FaultReason::Tilt:
        return "TILT";

      case FaultReason::Imu:
        return "IMU";

      case FaultReason::Tof:
        return "TOF";

      case FaultReason::Controller:
        return "CONTROLLER";

      case FaultReason::LandingTimeout:
        return "LANDING_TIMEOUT";
    }

    return "UNKNOWN";
  }


  void saveFaultSnapshot(
      FaultReason reason
  ) {

    if (!latestFaultSnapshotValid) {
      return;
    }

    Preferences prefs;

    if (!prefs.begin("flightFault", false)) {
      return;
    }

    prefs.putBool("valid", true);

    prefs.putUChar(
        "reason",
        static_cast<uint8_t>(reason)
    );

    prefs.putFloat(
        "roll",
        latestAttitude.rollDeg
    );

    prefs.putFloat(
        "pitch",
        latestAttitude.pitchDeg
    );

    prefs.putFloat(
        "rollRate",
        latestAttitude.rollRateDps
    );

    prefs.putFloat(
        "pitchRate",
        latestAttitude.pitchRateDps
    );

    prefs.putBool(
        "spi",
        latestImuHealth.spiReadOk
    );

    prefs.putBool(
        "accelOk",
        latestImuHealth.accelerationValid
    );

    prefs.putBool(
        "gyroSat",
        latestImuHealth.gyroSaturated
    );

    prefs.putBool(
        "magOk",
        latestImuHealth.accelMagnitudeValid
    );

    prefs.putFloat(
        "accelMag",
        latestImuHealth.accelMagnitudeG
    );

    prefs.putFloat(
        "peakRoll",
        peakAbsRollRateDps
    );

    prefs.putFloat(
        "peakPitch",
        peakAbsPitchRateDps
    );

    prefs.putFloat(
        "peakAccel",
        peakAccelMagnitudeG
    );

    prefs.putUChar(
        "state",
        static_cast<uint8_t>(currentState)
    );

    prefs.putUInt(
        "stateMs",
        millis() - stateStartedMs
    );

    prefs.end();
  }

  void printStoredFault() {

    Preferences prefs;

    if (!prefs.begin("flightFault", true)) {
      return;
    }

    if (!prefs.getBool("valid", false)) {
      prefs.end();
      return;
    }

    const FaultReason reason =
        static_cast<FaultReason>(
            prefs.getUChar("reason", 0)
        );

    Serial.println();
    Serial.println(
        "=== LAST STORED FLIGHT FAULT ==="
    );

    Serial.print("reason: ");
    Serial.println(
        faultReasonName(reason)
    );

    Serial.print("roll: ");
    Serial.println(
        prefs.getFloat("roll", 0.0f),
        2
    );

    Serial.print("pitch: ");
    Serial.println(
        prefs.getFloat("pitch", 0.0f),
        2
    );

    Serial.print("rollRate: ");
    Serial.println(
        prefs.getFloat("rollRate", 0.0f),
        2
    );

    Serial.print("pitchRate: ");
    Serial.println(
        prefs.getFloat("pitchRate", 0.0f),
        2
    );

    Serial.print("imu.spiReadOk: ");
    Serial.println(
        prefs.getBool("spi", false)
    );

    Serial.print("imu.accelerationValid: ");
    Serial.println(
        prefs.getBool("accelOk", false)
    );

    Serial.print("imu.gyroSaturated: ");
    Serial.println(
        prefs.getBool("gyroSat", false)
    );

    Serial.print("imu.accelMagnitudeValid: ");
    Serial.println(
        prefs.getBool("magOk", false)
    );

    Serial.print("imu.accelMagnitudeG: ");
    Serial.println(
        prefs.getFloat("accelMag", 0.0f),
        3
    );

    Serial.print("peakAbsRollRate: ");
    Serial.println(
        prefs.getFloat("peakRoll", 0.0f),
        2
    );

    Serial.print("peakAbsPitchRate: ");
    Serial.println(
        prefs.getFloat("peakPitch", 0.0f),
        2
    );

    Serial.print("peakAccelMagnitudeG: ");
    Serial.println(
        prefs.getFloat("peakAccel", 0.0f),
        3
    );

    Serial.print("stateAtFault: ");
    Serial.println(
        prefs.getUChar("state", 0)
    );

    Serial.print("stateElapsedMs: ");
    Serial.println(
        prefs.getUInt("stateMs", 0)
    );

    Serial.println(
        "================================"
    );

    Serial.println();

    prefs.end();
  }


  void enterFault(FaultReason reason) {
    // Safety first.
    FlightController::disarm();

    // Motors are already stopped. Now preserve
    // the sensor state that caused the fault.
    saveFaultSnapshot(reason);

    currentState =
      FlightState::Fault;

    switch (reason) {
      case FaultReason::Tilt:
        setLed(30, 0, 0);
        return;

      case FaultReason::Imu:
        setLed(0, 30, 0);
        return;

      case FaultReason::Tof:
        setLed(0, 20, 20);
        return;

      case FaultReason::Controller:
        setLed(30, 0, 15);
        return;

      case FaultReason::LandingTimeout:
        setLed(20, 20, 20);
        return;
    }
  }


  void complete() {

    FlightController::disarm();

    currentState = FlightState::Complete;

    setLed(0, 30, 0);
  }

}


void FlightSupervisor::begin() {

  FlightController::begin();

  printStoredFault();

  peakAbsRollRateDps = 0.0f;
  peakAbsPitchRateDps = 0.0f;
  peakAccelMagnitudeG = 0.0f;

  currentState = FlightState::WaitingForReady;

  stateStartedMs = millis();

  setLed(0, 0, 0);
}


void FlightSupervisor::update(
    const AttitudeState& attitude,
    const ImuHealthStatus& imuHealth,
    const TofSample& tofSample,
    const TofHealthStatus& tofHealth,
    float dtSeconds
) {

  latestAttitude = attitude;
  latestImuHealth = imuHealth;
  latestFaultSnapshotValid = true;

  peakAbsRollRateDps =
      max(
          peakAbsRollRateDps,
          fabsf(attitude.rollRateDps)
      );

  peakAbsPitchRateDps =
      max(
          peakAbsPitchRateDps,
          fabsf(attitude.pitchRateDps)
      );

  peakAccelMagnitudeG =
      max(
          peakAccelMagnitudeG,
          imuHealth.accelMagnitudeG
      );

  const bool ready =
      sensorsReady(
          attitude,
          imuHealth,
          tofHealth
      );


  switch (currentState) {

    case FlightState::WaitingForReady:

      if (!ready) {
        return;
      }

      currentState =
          FlightState::ArmWarning;

      stateStartedMs = millis();

      // Yellow: motors will arm after warning period.
      setLed(30, 20, 0);

      return;


    case FlightState::ArmWarning:

      // If readiness disappears, restart the waiting process
      // rather than arming anyway.
      if (!ready) {

        currentState =
            FlightState::WaitingForReady;

        setLed(0, 0, 0);

        return;
      }


      if (
          millis() - stateStartedMs
          < DronePetConfig::ARM_WARNING_MS
      ) {
        return;
      }

      peakAbsRollRateDps = 0.0f;
      peakAbsPitchRateDps = 0.0f;
      peakAccelMagnitudeG = 0.0f;

      FlightController::arm();
      FlightController::setTargetAltitudeMm(
          DronePetConfig::HOVER_ALTITUDE_MM
      );
      currentState = FlightState::Flying;

      stateStartedMs = millis();

      // Blue: flight controller is actively driving motors.
      setLed(0, 0, 30);

      return;


    case FlightState::Flying:

      if (!imuHealth.healthy()) {
        enterFault(FaultReason::Imu);
        return;
      }

      if (!tofHealth.responsive()
          || !tofHealth.rangeValid) {

        enterFault(FaultReason::Tof);
        return;
      }

      if (!tiltSafe(attitude)) {
        enterFault(FaultReason::Tilt);
        return;
      }


      if (
          millis() - stateStartedMs
          >= DronePetConfig::FLIGHT_TEST_DURATION_MS
      ) {

        currentState = FlightState::Landing;

        FlightController::setTargetAltitudeMm(
            DronePetConfig::LANDING_TARGET_ALTITUDE_MM
        );

        stateStartedMs = millis();

        landingNearGroundTimerRunning = false;

        // Purple = controlled landing.
        setLed(20, 0, 20);

        return;
      }


      FlightController::update(
          attitude,
          imuHealth,
          tofSample,
          tofHealth,
          dtSeconds
      );

      // FlightController can also disarm itself if something
      // internally becomes invalid.
      if (!FlightController::armed()) {
        enterFault(FaultReason::Controller);
        return;
      }

      return;

    case FlightState::Landing: {

      if (!imuHealth.healthy()) {
        enterFault(FaultReason::Imu);
        return;
      }

      if (!tofHealth.responsive()
          || !tofHealth.rangeValid) {

        enterFault(FaultReason::Tof);
        return;
      }

      if (!tiltSafe(attitude)) {
        enterFault(FaultReason::Tilt);
        return;
      }

      const uint32_t landingElapsedMs =
          millis() - stateStartedMs;

      // Keep running the exact same real flight controller.
      FlightController::update(
          attitude,
          imuHealth,
          tofSample,
          tofHealth,
          dtSeconds
      );


      if (!FlightController::armed()) {
        enterFault(FaultReason::Controller);
        return;
      }


      // Detect that we're actually near the ground.
      if (
          tofSample.distanceMm
          <= DronePetConfig::LANDING_DISARM_HEIGHT_MM
      ) {

        if (!landingNearGroundTimerRunning) {

          landingNearGroundTimerRunning = true;

          landingNearGroundStartedMs = millis();
        }


        if (
            millis() - landingNearGroundStartedMs
            >= DronePetConfig::LANDING_CONFIRM_MS
        ) {

          complete();

          return;
        }

      }
      else {

        landingNearGroundTimerRunning = false;
      }


      // Backup safety cutoff.
      if (
          landingElapsedMs
          >= DronePetConfig::LANDING_TIMEOUT_MS
      ) {

        enterFault(FaultReason::LandingTimeout);

        return;
      }


      return;
    }

    case FlightState::Complete:
    case FlightState::Fault:

      // One-shot first-flight profile.
      // Requires a power cycle to fly again.
      return;
  }
}


FlightState FlightSupervisor::state() {
  return currentState;
}
