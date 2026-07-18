#pragma once

#include <config.h>

struct RotorChannel
{
    int pin;
    float lastReading = 0;
    float troughRef = 9999;
    float currentPeak = -9999;
    bool rising = true;

    int dipCounter = 0;                                      // counts completed peaks
    float facePeak[FACES_PER_ROTOR] = {-9999, -9999, -9999}; // worst of the observed peaks per face
    bool faceHasData[FACES_PER_ROTOR] = {false, false, false};

    unsigned long lastDipTime = 0;
    unsigned long intervalSumMs = 0;
    int intervalCount = 0;

    explicit RotorChannel(int pin) : pin(pin) {}

    void reset()
    {
        lastReading = 0;
        troughRef = 9999;
        currentPeak = -9999;
        rising = true;
        dipCounter = 0;
        lastDipTime = 0;
        intervalSumMs = 0;
        intervalCount = 0;
        for (int i = 0; i < FACES_PER_ROTOR; i++)
        {
            facePeak[i] = -9999;
            faceHasData[i] = false;
        }
    };

    int steadyPulses() const
    {
        return dipCounter > WARMUP_DIPS ? dipCounter - WARMUP_DIPS : 0;
    }
};
