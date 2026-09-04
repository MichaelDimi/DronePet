#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>

#include "../board/BoardPins.h"
#include "Tof.h"

namespace {
    VL53L1X sensor;

    // VL53L1X supports I2C fast mode at up to 400 kHz.
    constexpr uint32_t I2C_CLOCK_HZ = 100'000;

    // Used by library operations that are allowed to wait.
    constexpr uint16_t I2C_TIMEOUT_MS = 100;

    // Limit a failed I2C transaction so it cannot hold the
    // 5 ms flight loop for the ESP32 Wire default of 50 ms.
    constexpr uint16_t I2C_TRANSACTION_TIMEOUT_MS = 20;

    // Long mode requires at least a 33 ms timing budget.
    // A 40 ms budget leaves enough margin for a 50 ms
    // start-to-start measurement period.
    constexpr uint32_t MEASUREMENT_TIMING_BUDGET_US = 40'000;

    // Keep the intermeasurement period more than 4 ms longer
    // than the timing budget so the sensor does not miss a
    // measurement-start window.
    constexpr uint32_t INTERMEASUREMENT_PERIOD_MS = 50;

    TofSample currentSample;

    bool sensorInitialized = false;
    bool sampleAvailable = false;
    bool lastCommunicationError = false;
}

bool Tof::begin() {
    sensorInitialized = false;
    sampleAvailable = false;
    lastCommunicationError = false;
    currentSample = {};

    pinMode(TOF_XSHUT, OUTPUT);
    digitalWrite(TOF_XSHUT, LOW);

    delay(10);

    // Arguments are SDA pin, SCL pin, and I2C clock rate.
    if (!Wire.begin(
        TOF_SDA,
        TOF_SCL,
        I2C_CLOCK_HZ
    )) {
    return false;
    }

    // This controls ESP32 Wire transaction blocking, separately
    // from the Pololu driver's measurement-wait timeout.
    Wire.setTimeOut(I2C_TRANSACTION_TIMEOUT_MS);
    Wire.setClock(I2C_CLOCK_HZ);

    sensor.setBus(&Wire);
    sensor.setTimeout(I2C_TIMEOUT_MS);

    digitalWrite(TOF_XSHUT, HIGH);

    // The sensor's maximum documented boot time is 1.2 ms.
    delay(5);

    if (!sensor.init()) { return false; }

    if (!sensor.setDistanceMode(VL53L1X::Long) || 
        sensor.last_status != 0) {
        return false;
    }

    if (!sensor.setMeasurementTimingBudget(MEASUREMENT_TIMING_BUDGET_US) ||
        sensor.last_status != 0) {
        return false;
    }

    sensor.startContinuous(INTERMEASUREMENT_PERIOD_MS);

    if (sensor.last_status != 0) {
        return false;
    }

    sensorInitialized = true;
    return true;
}

bool Tof::update() {
    if (!sensorInitialized) {
        return false;
    }

    const bool ready = sensor.dataReady();

    // Pololu exposes the result of the most recent I2C
    // address/write transaction through last_status.
    if (sensor.last_status != 0) {
        lastCommunicationError = true;
        return false;
    }

    if (!ready) {
        return false;
    }

    // dataReady() confirmed that a completed result is available,
    // so read(false) retrieves it without waiting.
    const uint16_t distanceMm =
        sensor.read(false);

    if (sensor.last_status != 0) {
        lastCommunicationError = true;
        return false;
    }

    const uint32_t nowUs = micros();

    currentSample.intervalUs =
        sampleAvailable
            ? nowUs - currentSample.timestampUs
            : 0;

    currentSample.distanceMm = distanceMm;
    currentSample.timestampUs = nowUs;

    currentSample.rangeStatus =
        static_cast<uint8_t>(
            sensor.ranging_data.range_status
        );

    currentSample.rangeValid =
        sensor.ranging_data.range_status
        == VL53L1X::RangeValid;

    sampleAvailable = true;
    lastCommunicationError = false;

    return true;
}

bool Tof::initialized() {
  return sensorInitialized;
}

bool Tof::hasSample() {
  return sampleAvailable;
}

bool Tof::communicationError() {
  return lastCommunicationError;
}

const TofSample& Tof::latestSample() {
  return currentSample;
}

const char* Tof::rangeStatusToString(uint8_t rangeStatus) {
    return VL53L1X::rangeStatusToString(
        static_cast<VL53L1X::RangeStatus>(rangeStatus)
    );
}