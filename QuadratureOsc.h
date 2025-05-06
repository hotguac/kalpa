#ifndef QUADOSC_H
#define QUADOSC_H

#include <math.h>
#include <stdio.h>

// based on https://www.dsprelated.com/showarticle/1467.php

class QuadratureOscillator {
public:
    QuadratureOscillator() {}

    void Init(float frequency, float sampleRate)
    {
        this->frequency = frequency;
        this->sampleRate = sampleRate;

        omega = (2.0f * 3.14159265358979323846f * frequency) / sampleRate; // Angular frequency
        k1 = tanf(omega / 2.0f); // phase increment for cosine
        k2 = sinf(omega); // phase increment for sine

        u0 = 1.0f; // previous cosine output
        v0 = 0.0f; // previous sine output
    }

    void update()
    {
        float w1 = u0 - k1 * v0;
        float v1 = v0 + k2 * w1;
        float u1 = w1 - k1 * v1;

        u0 = u1; // update previous cosine output
        v0 = v1; // update previous sine output
    }

    float oscOut1() { return u0; }

    float oscOut2() { return v0; }

private:
    float frequency; // Frequency in Hz (e.g., A4 note)
    float sampleRate; // Sample rate in Hz
    float omega;
    float k1; // phase increment for cosine
    float k2; // phase increment for sine

    float u0; // previous cosine output
    float v0; // previous sine output

};

#endif // QUADOSC_H