#include <Arduino.h>
#include <SPIFFS.h>

#include "app/DronePetTask.h"
#include "flight/EspFcRuntime.h"


void setup() {
  SPIFFS.begin(true, "/spiffs", 10, "dplogs");

  EspFcRuntime::start();
  DronePetTask::start();

  vTaskDelete(nullptr);
}

void loop() {}

// #include <Arduino.h>

// #include "board/BoardPins.h"

// void setup() {
//     // USB CDC -> computer / MicoAssistant
//     Serial.begin(115200);

//     // Hardware UART -> MTF01
//     Serial1.begin(
//         115200,
//         SERIAL_8N1,
//         MTF_RX,
//         MTF_TX
//     );

//     delay(500);
// }

// void loop() {
//     // Computer -> MTF01
//     while (Serial.available() > 0) {
//         Serial1.write(
//             static_cast<uint8_t>(
//                 Serial.read()
//             )
//         );
//     }

//     // MTF01 -> Computer
//     while (Serial1.available() > 0) {
//         Serial.write(
//             static_cast<uint8_t>(
//                 Serial1.read()
//             )
//         );
//     }
// }