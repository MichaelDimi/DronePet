#pragma once

#include <Control/Pid.h>
#include <Utils/Filter.h>

namespace DronePet {

struct AltitudeControllerDebug {
    float filteredAltitudeM = 0.0f;
    float errorM = 0.0f;

    float pTerm = 0.0f;
    float iTerm = 0.0f;
    float dTerm = 0.0f;

    float correction = 0.0f;
};

class AltitudeController {

public:
    void begin();
    void reset();

    float update(
        float targetAltitudeM,
        float altitudeM
    );

    const AltitudeControllerDebug& debug() const;

private:
    Espfc::Control::Pid _pid;
    Espfc::Utils::FilterStatePt1 _altitudeFilter;

    AltitudeControllerDebug _debug;

    bool _filterInitialized = false;
};

}