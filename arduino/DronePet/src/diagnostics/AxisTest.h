#pragma once

#include "../common/MathTypes.h"

namespace AxisTest {
  // Restart the guided three-axis verification procedure.
  void start();

  // Feed each calibrated gyroscope sample into the active test.
  void update(const Vector3& gyroDps);

  bool isComplete();
}
