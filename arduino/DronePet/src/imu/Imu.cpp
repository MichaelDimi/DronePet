#include <Arduino.h>
#include <SPI.h>

#include "../board/BoardPins.h"
#include "Imu.h"

// TODO: Later update gyro bias only during confidently stationary periods.

namespace {
    SPIClass imuSPI(VSPI);

    constexpr uint32_t SPI_CLOCK_HZ = 1'000'000;
    constexpr size_t SPI_IDENTITY_CHECK_INTERVAL_SAMPLES = 200;

    constexpr size_t   GYRO_CALIBRATION_SETTLING_SAMPLES = 100;
    constexpr size_t   GYRO_CALIBRATION_SAMPLES = 500;
    constexpr uint32_t GYRO_SAMPLE_INTERVAL_MS = 3;

    constexpr uint8_t REG_WHO_AM_I = 0x0F;
    constexpr uint8_t REG_CTRL1_XL = 0x10;
    constexpr uint8_t REG_CTRL2_G  = 0x11;
    constexpr uint8_t REG_CTRL3_C  = 0x12;
    constexpr uint8_t REG_CTRL4_C  = 0x13;
    constexpr uint8_t REG_CTRL9_XL = 0x18;
    constexpr uint8_t REG_GYRO_X_L = 0x22;

    constexpr uint8_t EXPECTED_WHO_AM_I = 0x6B;

    SPISettings imuSettings(
        SPI_CLOCK_HZ,  // 1 MHz SPI clock
        MSBFIRST,      // Transmit the most-significant bit first
        SPI_MODE3      // Clock idles HIGH; data sampled on rising edge
    );

    Vector3 gyroBias;

    size_t samplesSinceIdentityCheck = 0;
    bool communicationHealthy = true;

    uint8_t readRegister(uint8_t address) {
        imuSPI.beginTransaction(imuSettings);
        digitalWrite(IMU_CS, LOW); // Select the IMU for this SPI transaction

        imuSPI.transfer(address | 0x80);  // Set the SPI read bit
        uint8_t value = imuSPI.transfer(0x00); // Send a dummy byte to generate 8 clock pulses while reading the IMU's reply on MISO

        digitalWrite(IMU_CS, HIGH); // Select the IMU for this SPI transaction
        imuSPI.endTransaction();

        return value;
    }

    void writeRegister(uint8_t address, uint8_t value) {
        imuSPI.beginTransaction(imuSettings);
        digitalWrite(IMU_CS, LOW); // Select the IMU for this SPI transaction

        imuSPI.transfer(address & 0x7F);  // Clear the SPI read bit for a write
        imuSPI.transfer(value);

        digitalWrite(IMU_CS, HIGH); // Deselect the IMU and end the SPI transaction
        imuSPI.endTransaction();
    }

    bool readRegisters(
        uint8_t startAddress,
        uint8_t* data,
        size_t length
    ) {
        imuSPI.beginTransaction(imuSettings);
        digitalWrite(IMU_CS, LOW);

        imuSPI.transfer(
            startAddress | 0x80
        );  // Begin a sequential register read

        bool allBytesZero = true;
        bool allBytesOne = true;

        for (size_t i = 0; i < length; i++) {
            // SPI must transmit a dummy byte to generate the clock pulses that
            // allow the IMU to shift the next response byte back through MISO.
            data[i] = imuSPI.transfer(0x00);

            allBytesZero &= data[i] == 0x00;
            allBytesOne &= data[i] == 0xFF;
        }

        digitalWrite(IMU_CS, HIGH);
        imuSPI.endTransaction();

        // A disconnected or failed SPI bus commonly returns only zeros or ones.
        // SPI has no acknowledgement bit, so this is a basic integrity heuristic.
        return !allBytesZero && !allBytesOne;
    }

    int16_t combineBytes(uint8_t lowByte, uint8_t highByte) {
        return static_cast<int16_t>(
            static_cast<uint16_t>(highByte) << 8 | lowByte
        );
    }

    bool readUncalibratedSample(ImuSample& sample) {
        uint8_t data[12];

        if (!readRegisters(REG_GYRO_X_L, data, sizeof(data))) {
            return false;
        }

        int16_t gyroX = combineBytes(data[0], data[1]);
        int16_t gyroY = combineBytes(data[2], data[3]);
        int16_t gyroZ = combineBytes(data[4], data[5]);

        int16_t accelX = combineBytes(data[6], data[7]);
        int16_t accelY = combineBytes(data[8], data[9]);
        int16_t accelZ = combineBytes(data[10], data[11]);

        sample.gyroDps.x = gyroX * 0.0175f;
        sample.gyroDps.y = gyroY * 0.0175f;
        sample.gyroDps.z = gyroZ * 0.0175f;

        sample.accelG.x = accelX * 0.000122f;
        sample.accelG.y = accelY * 0.000122f;
        sample.accelG.z = accelZ * 0.000122f;

        return true;
    }
}

bool Imu::begin() {
    pinMode(IMU_CS, OUTPUT);
    digitalWrite(IMU_CS, HIGH); // Switch to I2C communication (disabled) while idle.

    imuSPI.begin(
        IMU_SCK,
        IMU_MISO,
        IMU_MOSI,
        IMU_CS
    );  // Route this SPI bus through our selected ESP32 pins

    delay(50);  // Datasheet specifies time for the sensor to start

    if (whoAmI() != EXPECTED_WHO_AM_I) {
        return false;
    }

    writeRegister(REG_CTRL3_C, 0x44);  // Enable block-data update and sequential reads
    writeRegister(REG_CTRL4_C, 0x04);  // Disable the unused I²C interface
    writeRegister(REG_CTRL9_XL, 0xE2); // Enable the recommended device configuration
    writeRegister(REG_CTRL1_XL, 0x68); // Accelerometer: 416 Hz, ±4 g
    writeRegister(REG_CTRL2_G, 0x64);  // Gyroscope: 416 Hz, ±500 degrees/second

    delay(20);

    samplesSinceIdentityCheck = 0;
    communicationHealthy = true;

    return true;
}

bool Imu::calibrateGyro() {
    gyroBias = {};

    ImuSample sample;

    for (
        size_t i = 0;
        i < GYRO_CALIBRATION_SETTLING_SAMPLES;
        i++
    ) {
        if (!readUncalibratedSample(sample)) {
        return false;
        }

        delay(GYRO_SAMPLE_INTERVAL_MS);
    }

    Vector3 sum;

    for (size_t i = 0; i < GYRO_CALIBRATION_SAMPLES; i++) {
        if (!readUncalibratedSample(sample)) {
        return false;
        }

        sum.x += sample.gyroDps.x;
        sum.y += sample.gyroDps.y;
        sum.z += sample.gyroDps.z;

        delay(GYRO_SAMPLE_INTERVAL_MS);
    }

    const float sampleCount =
        static_cast<float>(GYRO_CALIBRATION_SAMPLES);

    gyroBias.x = sum.x / sampleCount;
    gyroBias.y = sum.y / sampleCount;
    gyroBias.z = sum.z / sampleCount;

    return true;
}

Vector3 Imu::gyroBiasDps() {
    return gyroBias;
}

uint8_t Imu::whoAmI() {
  return readRegister(REG_WHO_AM_I);
}

bool Imu::readSample(ImuSample& sample) {
  if (!readUncalibratedSample(sample)) {
    return false;
  }

  samplesSinceIdentityCheck++;

  if (
      samplesSinceIdentityCheck
      >= SPI_IDENTITY_CHECK_INTERVAL_SAMPLES
  ) {
    samplesSinceIdentityCheck = 0;

    // Periodically verify that the device responding on SPI is still
    // the expected ISM330DHCX, not merely returning plausible bytes.
    communicationHealthy =
        whoAmI() == EXPECTED_WHO_AM_I;
  }

  if (!communicationHealthy) {
    return false;
  }

  sample.gyroDps.x -= gyroBias.x;
  sample.gyroDps.y -= gyroBias.y;
  sample.gyroDps.z -= gyroBias.z;

  return true;
}
