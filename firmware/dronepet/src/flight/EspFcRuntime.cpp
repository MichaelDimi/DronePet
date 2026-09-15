#include "EspFcRuntime.h"
#include "DronePetInputDevice.h"

#include <Arduino.h>
#include <Espfc.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/timer.h>


namespace {

Espfc::Espfc espfc;

DronePet::DronePetInputDevice dronePetInput;

TaskHandle_t gyroTaskHandle = nullptr;
TaskHandle_t flightTaskHandle = nullptr;

constexpr timer_group_t TIMER_GROUP =
    TIMER_GROUP_0;

constexpr timer_idx_t TIMER_INDEX =
    TIMER_0;


bool IRAM_ATTR gyroTimerIsr(
    void* args
) {

  BaseType_t higherPriorityTaskWoken =
      pdFALSE;

  vTaskNotifyGiveFromISR(
      gyroTaskHandle,
      &higherPriorityTaskWoken
  );

  return higherPriorityTaskWoken
      == pdTRUE;
}


void gyroTimerInit(
    bool (*isrCallback)(void*),
    int intervalUs
) {

  timer_config_t config = {
      .alarm_en = TIMER_ALARM_EN,
      .counter_en = TIMER_PAUSE,
      .intr_type = TIMER_INTR_LEVEL,
      .counter_dir = TIMER_COUNT_UP,
      .auto_reload = TIMER_AUTORELOAD_EN,
      .divider = 80,
  };

  timer_init(
      TIMER_GROUP,
      TIMER_INDEX,
      &config
  );

  timer_set_counter_value(
      TIMER_GROUP,
      TIMER_INDEX,
      0
  );

  timer_set_alarm_value(
      TIMER_GROUP,
      TIMER_INDEX,
      intervalUs
  );

  timer_isr_callback_add(
      TIMER_GROUP,
      TIMER_INDEX,
      isrCallback,
      nullptr,
      ESP_INTR_FLAG_IRAM
  );

  timer_enable_intr(
      TIMER_GROUP,
      TIMER_INDEX
  );

  timer_start(
      TIMER_GROUP,
      TIMER_INDEX
  );
}


void gyroTask(
    void* parameter
) {

  espfc.begin();

  gyroTimerInit(
      gyroTimerIsr,
      espfc.getGyroInterval()
  );

  while (true) {

    ulTaskNotifyTake(
        pdTRUE,
        portMAX_DELAY
    );

    espfc.update(true);
  }
}


void flightTask(
    void* parameter
) {

  while (true) {
    espfc.updateOther();
  }
}

}


void EspFcRuntime::start() {

  disableCore0WDT();

  // Loads persisted ESP-FC configuration:
  // pins, alignment, motor protocol, etc.
  espfc.load();

  espfc.setInputDevice(
    &dronePetInput
  );

  xTaskCreateUniversal(
      gyroTask,
      "EspFcGyro",
      8192,
      nullptr,
      24,
      &gyroTaskHandle,
      1
  );

  xTaskCreateUniversal(
      flightTask,
      "EspFcFlight",
      8192,
      nullptr,
      1,
      &flightTaskHandle,
      0
  );
}

Espfc::RuntimeStatus EspFcRuntime::status()
{
  return espfc.getRuntimeStatus();
}