#pragma once

namespace DronePet {

struct AltitudeEstimate {
    float altitudeM = 0.0f;
};

class AltitudeEstimator {
public:
    const AltitudeEstimate& update(
        float rangeM,
        float rollRad,
        float pitchRad
    );

    const AltitudeEstimate& state() const;

private:
    static float compensateTilt(
        float rangeM,
        float rollRad,
        float pitchRad
    );

    AltitudeEstimate _state{};
};

}