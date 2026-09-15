#pragma once

namespace DronePet {

struct PilotCommand {

  // Normalized virtual-stick commands.
  // -1.0 to +1.0
  float roll = 0.0f;
  float pitch = 0.0f;
  float yaw = 0.0f;

  // 0.0 to 1.0
  float throttle = 0.0f;

  bool armed = false;
  bool angleMode = true;
};

}