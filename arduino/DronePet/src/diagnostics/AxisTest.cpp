#include <Arduino.h>
#include <math.h>

#include "AxisTest.h"

/*
 * Drone body frame: right-handed X-forward, Y-left, Z-up.
 *
 * Translation axes:
 *   +X = forward
 *   +Y = left
 *   +Z = up
 *
 * Positive rotations follow the right-hand rule:
 *   +X roll  = right side moves down
 *   +Y pitch = nose moves down
 *   +Z yaw   = nose turns left while the drone remains level
 *
 * Guided verification movements:
 *   Test 1: Move the right side down        -> expected body +X
 *   Test 2: Move the nose down              -> expected body +Y
 *   Test 3: Turn the nose left while level  -> expected body +Z
 *
 * This test reports the IMU's native sensor axis for each movement.
 * The results will define the sensor-frame-to-body-frame conversion.
 */

 namespace {
    constexpr size_t TEST_COUNT = 3;

    // Ignore slow handling and stationary sensor noise.
    constexpr float MIN_ROTATION_RATE_DPS = 20.0f;

    // The strongest axis must be clearly larger than the second-strongest
    // axis before the movement is accepted as rotation around one axis.
    constexpr float AXIS_DOMINANCE_RATIO = 1.5f;

    // After recording a movement, require the board to remain nearly still
    // before advancing to the next test.
    constexpr float STILL_RATE_LIMIT_DPS = 5.0f;
    constexpr uint32_t STILL_TIME_MS = 500;

    struct AxisResult {
        char sensorAxis = '?';
        char sign = '?';
    };

    AxisResult results[TEST_COUNT];

    size_t currentTest = 0;
    bool complete = false;
    bool waitingForStillness = false;
    bool stillnessTimerRunning = false;
    uint32_t stillnessStartMs = 0;

    void printCurrentInstruction() {
        Serial.println();

        switch (currentTest) {
        case 0:
            Serial.println("TEST 1/3: Move the RIGHT SIDE DOWN.");
            Serial.println(
                "This represents positive body X rotation (positive roll)."
            );
            break;

        case 1:
            Serial.println("TEST 2/3: Move the NOSE DOWN.");
            Serial.println(
                "This represents positive body Y rotation (positive pitch)."
            );
            break;

        case 2:
            Serial.println(
                "TEST 3/3: Turn the NOSE LEFT while keeping the board level."
            );
            Serial.println(
                "This represents positive body Z rotation (positive yaw)."
            );
            break;
        }

        Serial.println("Rotate smoothly, then hold the board still.");
    }

    void printResults() {
        Serial.println();
        Serial.println("===== AXIS TEST RESULTS =====");
        Serial.println("Drone body frame: +X forward, +Y left, +Z up");
        Serial.println();

        Serial.print("Body +X: right side down -> sensor ");
        Serial.print(results[0].sign);
        Serial.println(results[0].sensorAxis);

        Serial.print("Body +Y: nose down       -> sensor ");
        Serial.print(results[1].sign);
        Serial.println(results[1].sensorAxis);

        Serial.print("Body +Z: nose turns left -> sensor ");
        Serial.print(results[2].sign);
        Serial.println(results[2].sensorAxis);

        Serial.println("============================");
    }

    void findDominantAxis(
        const Vector3& gyroDps,
        size_t& dominantIndex,
        float& dominantMagnitude,
        float& secondLargestMagnitude
    ) {
        const float values[] = {
            gyroDps.x,
            gyroDps.y,
            gyroDps.z
        };

        dominantIndex = 0;
        dominantMagnitude = 0.0f;
        secondLargestMagnitude = 0.0f;

        for (size_t i = 0; i < 3; i++) {
            const float magnitude = fabsf(values[i]);

            if (magnitude > dominantMagnitude) {
                secondLargestMagnitude = dominantMagnitude;
                dominantMagnitude = magnitude;
                dominantIndex = i;
            } else if (magnitude > secondLargestMagnitude) {
                secondLargestMagnitude = magnitude;
            }
        }
    }

    float largestMagnitude(const Vector3& value) {
        return fmaxf(
            fabsf(value.x),
            fmaxf(fabsf(value.y), fabsf(value.z))
        );
    }
 }

 void AxisTest::start() {
    currentTest = 0;
    complete = false;
    waitingForStillness = false;
    stillnessTimerRunning = false;

    for (AxisResult& result : results) {
        result = {};
    }

    Serial.println();
    Serial.println("===== IMU AXIS VERIFICATION =====");
    Serial.println("Right-handed body frame:");
    Serial.println("  +X = forward");
    Serial.println("  +Y = left");
    Serial.println("  +Z = up");
    Serial.println();
    Serial.println(
        "Keep the marked front of the board pointed toward the drone's nose."
    );

    printCurrentInstruction();
}
    

void AxisTest::update(const Vector3& gyroDps) {
    if (complete) {
        return;
    }

    if (waitingForStillness) {
        if (largestMagnitude(gyroDps) <= STILL_RATE_LIMIT_DPS) {
        if (!stillnessTimerRunning) {
            stillnessTimerRunning = true;
            stillnessStartMs = millis();
        } else if (millis() - stillnessStartMs >= STILL_TIME_MS) {
            waitingForStillness = false;
            stillnessTimerRunning = false;
            currentTest++;

            if (currentTest == TEST_COUNT) {
            complete = true;
            printResults();
            } else {
            printCurrentInstruction();
            }
        }
        } else {
        // Any renewed movement restarts the required stillness period.
        stillnessTimerRunning = false;
        }

        return;
    }

    size_t dominantIndex;
    float dominantMagnitude;
    float secondLargestMagnitude;

    findDominantAxis(
        gyroDps,
        dominantIndex,
        dominantMagnitude,
        secondLargestMagnitude
    );
    
    if (dominantMagnitude < MIN_ROTATION_RATE_DPS) {
        return;
    }

    if (
        dominantMagnitude
        < secondLargestMagnitude * AXIS_DOMINANCE_RATIO
    ) {
        return;
    }

    const float values[] = {
        gyroDps.x,
        gyroDps.y,
        gyroDps.z
    };

    const char axisNames[] = {
        'X',
        'Y',
        'Z'
    };

    results[currentTest].sensorAxis = axisNames[dominantIndex];
    results[currentTest].sign =
        values[dominantIndex] >= 0.0f ? '+' : '-';

    Serial.print("Detected: sensor ");
    Serial.print(results[currentTest].sign);
    Serial.print(results[currentTest].sensorAxis);
    Serial.print(" at ");
    Serial.print(values[dominantIndex], 1);
    Serial.println(" dps");

    Serial.println("Return to the starting orientation and hold still.");

    waitingForStillness = true;
    stillnessTimerRunning = false;
}

bool AxisTest::isComplete() {
  return complete;
}
