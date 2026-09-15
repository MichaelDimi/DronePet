#pragma once

constexpr int STATUS_LED = 21;

// IMU — SPI
constexpr int IMU_SCK  = 10;
constexpr int IMU_MOSI = 9;
constexpr int IMU_MISO = 8;
constexpr int IMU_CS   = 7;

// MTF01 — UART
constexpr int MTF_RX = 12;  // connects to MTF Tx
constexpr int MTF_TX = 11;  // connects to MTF Rx

// ESC motor signals
constexpr int ESC_1 = 5;
constexpr int ESC_2 = 4;
constexpr int ESC_3 = 2;
constexpr int ESC_4 = 1;
