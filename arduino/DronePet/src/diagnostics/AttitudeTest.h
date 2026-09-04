#pragma once

struct AttitudeState;

namespace AttitudeTest {
  void start();
  void update(const AttitudeState& attitude);
  bool isComplete();
}
