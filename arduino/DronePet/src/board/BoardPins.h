#pragma once

constexpr int STATUS_LED = 21;

// IMU — SPI
constexpr int IMU_SCK  = 10;
constexpr int IMU_MOSI = 9;
constexpr int IMU_MISO = 8;
constexpr int IMU_CS   = 7;

// ToF — I2C
constexpr int TOF_SDA   = 12;
constexpr int TOF_SCL   = 11;
constexpr int TOF_XSHUT = 13;

// ESC motor signals
constexpr int ESC_1 = 5;
constexpr int ESC_2 = 4;
constexpr int ESC_3 = 2;
constexpr int ESC_4 = 1;
