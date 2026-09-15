#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "Blackbox/Blackbox.h"
#include "Connect/Buzzer.hpp"
#include "Control/Actuator.h"
#include "Control/Controller.h"
#include "Hardware.h"
#include "Input.h"
#include "Model.h"
#include "Output/Mixer.h"
#include "SensorManager.h"
#include "SerialManager.h"
#include "TelemetryManager.h"
#include "RuntimeStatus.h"

namespace Espfc {

class Espfc
{
public:
  Espfc();

  int load();
  int begin();
  int update(bool externalTrigger = false);
  int updateOther();

  RuntimeStatus getRuntimeStatus();

  void setInputDevice(Device::InputDevice* device) {
    _input.setExternalDevice(device);
  }

  int getGyroInterval() const
  {
    return _model.state.gyro.timer.interval;
  }

private:
  Model _model;
  Hardware _hardware;
  Control::Controller _controller;
  TelemetryManager _telemetry;
  Input _input;
  Control::Actuator _actuator;
  SensorManager _sensor;
  Output::Mixer _mixer;
  Blackbox::Blackbox _blackbox;
  Connect::Buzzer _buzzer;
  SerialManager _serial;
  uint32_t _loop_next;

  enum StatusPart {
    STATUS_MODES,
    STATUS_CONTROL,
    STATUS_ATTITUDE
  };

  void updateRuntimeStatus(StatusPart part);

  uint32_t _statusTimesUs[3]{};
  uint8_t _statusSeen = 0;

  bool _statusSensorsReady = false;
  bool _statusGyroCalibrated = false;

  RuntimeStatus _runtimeStatus{};
  portMUX_TYPE _statusMux = portMUX_INITIALIZER_UNLOCKED;
};

} // namespace Espfc
