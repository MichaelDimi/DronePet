#include <Arduino.h>

#include "app/DronePetTask.h"
#include "flight/EspFcRuntime.h"


void setup() {

  EspFcRuntime::start();
  DronePetTask::start();

  vTaskDelete(nullptr);
}

void loop() {}