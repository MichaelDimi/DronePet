#include <Arduino.h>
#include <math.h>

#include "AttitudeTest.h"
#include "../control/AttitudeEstimator.h"

namespace {
  enum class TestStage {
    WaitingForEstimator,
    CheckLevel,
    CheckPositiveRoll,
    ReturnToLevel,
    CheckPositivePitch,
    Complete
  };

  constexpr float LEVEL_LIMIT_DEG = 5.0f;
  constexpr float TEST_ANGLE_DEG = 12.0f;
  constexpr uint32_t LEVEL_HOLD_MS = 500;

  TestStage stage = TestStage::WaitingForEstimator;
  bool levelTimerRunning = false;
  uint32_t levelStartMs = 0;

  bool nearLevel(const AttitudeState& attitude) {
    return fabsf(attitude.rollDeg) <= LEVEL_LIMIT_DEG
        && fabsf(attitude.pitchDeg) <= LEVEL_LIMIT_DEG;
  }

  bool heldLevelLongEnough(const AttitudeState& attitude) {
    if (!nearLevel(attitude)) {
      levelTimerRunning = false;
      return false;
    }

    if (!levelTimerRunning) {
      levelTimerRunning = true;
      levelStartMs = millis();
      return false;
    }

    return millis() - levelStartMs >= LEVEL_HOLD_MS;
  }

  void printRollInstruction() {
    Serial.println();
    Serial.println("ATTITUDE TEST 1/2: Roll");
    Serial.println("Slowly lower the RIGHT side of the drone.");
    Serial.println("Expected: roll becomes POSITIVE and exceeds +12 deg.");
  }

  void printPitchInstruction() {
    Serial.println();
    Serial.println("ATTITUDE TEST 2/2: Pitch");
    Serial.println("Slowly lower the NOSE of the drone.");
    Serial.println("Expected: pitch becomes POSITIVE and exceeds +12 deg.");
  }
}

void AttitudeTest::start() {
  stage = TestStage::WaitingForEstimator;
  levelTimerRunning = false;

  Serial.println();
  Serial.println("===== ROLL / PITCH ATTITUDE TEST =====");
  Serial.println("Motors remain disarmed during this test.");
  Serial.println("Place the drone flat and keep it still while the level reference is learned.");
}

void AttitudeTest::update(const AttitudeState& attitude) {
  switch (stage) {
    case TestStage::WaitingForEstimator:
      if (!attitude.ready) {
        return;
      }

      Serial.println("Attitude estimator ready.");
      Serial.println("Checking that the resting drone is near roll=0, pitch=0...");
      stage = TestStage::CheckLevel;
      levelTimerRunning = false;
      return;

    case TestStage::CheckLevel:
      if (!heldLevelLongEnough(attitude)) {
        return;
      }

      Serial.println("Level check: PASS");
      printRollInstruction();
      stage = TestStage::CheckPositiveRoll;
      levelTimerRunning = false;
      return;

    case TestStage::CheckPositiveRoll:
      if (attitude.rollDeg < TEST_ANGLE_DEG) {
        return;
      }

      Serial.print("Roll check: PASS at ");
      Serial.print(attitude.rollDeg, 1);
      Serial.println(" deg");
      Serial.println("Return the drone to level and hold it still.");

      stage = TestStage::ReturnToLevel;
      levelTimerRunning = false;
      return;

    case TestStage::ReturnToLevel:
      if (!heldLevelLongEnough(attitude)) {
        return;
      }

      printPitchInstruction();
      stage = TestStage::CheckPositivePitch;
      levelTimerRunning = false;
      return;

    case TestStage::CheckPositivePitch:
      if (attitude.pitchDeg < TEST_ANGLE_DEG) {
        return;
      }

      Serial.print("Pitch check: PASS at ");
      Serial.print(attitude.pitchDeg, 1);
      Serial.println(" deg");
      Serial.println();
      Serial.println("===== ATTITUDE TEST COMPLETE =====");
      Serial.println("Roll and pitch signs are ready for controller work.");

      stage = TestStage::Complete;
      return;

    case TestStage::Complete:
      return;
  }
}

bool AttitudeTest::isComplete() {
  return stage == TestStage::Complete;
}
