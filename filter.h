// kalpa for Hothouse DIY DSP Platform
// Copyright (C) 2024 Joe Kokosa <jkokosa@gmail.com>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

// ### Uncomment if IntelliSense can't resolve DaisySP-LGPL classes ###
// #include "daisysp-lgpl.h"

#pragma once

#include "Biquad.h"
#include "Filters/biquad.h"
#include "Filters/onepole.h"
#include "Utility/dsp.h"
#include "daisysp-lgpl.h"
#include "daisysp.h"
#include <cmath>

namespace jkoDSP {

using daisysp::OnePole;

/*
This file includes a set of audio filters
- Low Pass
- Low Shelf
- Peak
- Notch
- High Shelf
- High Pass
- All Pass

For each of the filter types, multiple algorithms may be provided.
This implementation provides a standard interface for each type
of filter hiding details of each algorithm.

*/

const int ft_undefined = 0;

class Filter {
public:
    virtual void Init(float sampleRate, int ftype) = 0;
    virtual float render(float in) = 0;
    virtual void setFreq(float freq) = 0;
    virtual void updateCoefficients() = 0;

protected:
    virtual void reset() = 0;

    float sampleRate = 0.f; // undefined
    int ftype = ft_undefined;
    float freq = 0.f; // undefined
};

class LowPass : Filter {
public:
    enum filterType : int { ft_cytomic_svf = 1,
        ft_daisy_onepole = 2,
        ft_daisy_biquad = 3,
        ft_daisy_svf = 4 };

    void Init(float sampleRate, int ftype) override;
    float render(float in) override;
    void setFreq(float freq) override;
    void updateCoefficients() override;

protected:
    void reset() override;

private:
    jkoDSP::Biquad biquad;
    OnePole onepole;
};

void LowPass::Init(float sampleRate, int ftype)
{
    this->sampleRate = sampleRate;

    if (ftype < ft_cytomic_svf || ftype > ft_daisy_svf)
        ftype = ft_undefined;

    this->ftype = ftype;

    switch (ftype) {
    case ft_daisy_onepole:
        onepole.Init();
        onepole.SetFilterMode(OnePole::FILTER_MODE_LOW_PASS);
        /* code */
        break;

    default:
        break;
    }
}

float LowPass::render(float in)
{
    switch (ftype) {
    case ft_cytomic_svf:
        return in;
        break;

    case ft_daisy_onepole:
        return in;
        break;

    case ft_daisy_biquad:
        return in;
        break;

    case ft_daisy_svf:
        return in;
        break;

    default:
        return in;
        break;
    }

    return in;
}

void LowPass::setFreq(float freq)
{
    this->freq = freq;
    updateCoefficients();
}

void LowPass::updateCoefficients()
{

    switch (ftype) {
    case ft_cytomic_svf:
        /* code */
        break;

    case ft_daisy_onepole:
        /* code */
        break;

    case ft_daisy_biquad:
        /* code */
        break;

    case ft_daisy_svf:
        /* code */
        break;

    default:
        break;
    }
}

void LowPass::reset()
{
    switch (ftype) {
    case ft_cytomic_svf:
        /* code */
        break;

    case ft_daisy_onepole:
        /* code */
        break;

    case ft_daisy_biquad:
        /* code */
        break;

    case ft_daisy_svf:
        /* code */
        break;

    default:
        break;
    }
}

// Below this point should be removed after reimplementing

// Resonant low-pass filter based on Cytomic SVF.
class Cytomic_SVF {
public:
    float sampleRate;

    void updateCoefficients(float cutoff, float Q)
    {
        g = std::tan(PI * cutoff / sampleRate);
        k = 1.0f / Q;
        a1 = 1.0f / (1.0f + g * (g + k));
        a2 = g * a1;
        a3 = g * a2;
    }

    void reset()
    {
        g = 0.0f;
        k = 0.0f;
        a1 = 0.0f;
        a2 = 0.0f;
        a3 = 0.0f;

        ic1eq = 0.0f;
        ic2eq = 0.0f;
    }

    float render(float x)
    {
        float v3 = x - ic2eq;
        float v1 = a1 * ic1eq + a2 * v3;
        float v2 = ic2eq + a2 * ic1eq + a3 * v3;
        ic1eq = 2.0f * v1 - ic1eq;
        ic2eq = 2.0f * v2 - ic2eq;
        return v2;
    }

private:
    const float PI = 3.1415926535897932f;

    float g, k, a1, a2, a3; // filter coefficients
    float ic1eq, ic2eq; // internal state
};

class LowShelf {
public:
    float sampleRate;

    void Init(float samplerate)
    {
        sampleRate = samplerate;
        reset();
        updateCoefficients(400.f, 0.7f, 0.f);
    }

    void updateCoefficients(float cutoff, float Q, float gain)
    {
        // jassert(sampleRate > 0.0);
        // jassert(cutOffFrequency > 0.0 && cutOffFrequency <= sampleRate * 0.5);
        // jassert(Q > 0.0);

        // 0 < gain <= 1;

        const auto A = std::sqrt(gain);
        const auto aminus1 = A - 1;
        const auto aplus1 = A + 1;
        const auto omega = (2 * PI * fmax(cutoff, 2.0f)) / sampleRate;
        const auto coso = std::cos(omega);
        const auto beta = std::sin(omega) * std::sqrt(A) / Q;
        const auto aminus1TimesCoso = aminus1 * coso;

        b0 = A * (aplus1 - aminus1TimesCoso + beta);
        b1 = A * 2 * (aminus1 - aplus1 * coso);
        b2 = A * (aplus1 - aminus1TimesCoso - beta);
        a0 = aplus1 + aminus1TimesCoso + beta;
        a1 = -2 * (aminus1 + aplus1 * coso);
        a2 = aplus1 + aminus1TimesCoso - beta;
    }

    void reset()
    {
        for (size_t i = 0; i < state.size(); ++i) {
            state[i] = 0.f;
        }
    }

    float render(float sample)
    {
        std::array<float, 6> c = { b0, b1, b2, a0, a1, a2 };

        float output = (c[0] * sample) + state[0];

        for (int j = 0; j < order - 1; ++j)
            state[j] = (c[j + 1] * sample) - (c[order + j + 1] * output) + state[j + 1];

        state[order - 1] = (c[order] * sample) - (c[order * 2] * output);

        return output;
    }

private:
    const float PI = 3.1415926535897932f;
    const float minimumDecibels = -300.0f;

    float g, k, a0, a1, a2, a3, b0, b1, b2; // filter coefficients
    float ic1eq, ic2eq; // internal state
    std::array<float, 6> state;
    int order = 2;
};

class LowShelf2 {

public:
    void init(float sample_rate)
    {
        sampleRate = sample_rate;
        z1 = 0.f;
        z2 = 0.f;
    }

    // Function to calculate biquad filter coefficients for a low shelf filter
    void calculateLowShelfCoefficients(float fc, float G, float Q)
    {
        // G is the logrithmic gain (in dB)
        // FC is the center frequency
        // Q adjusts the slope be replacing the sqrt(2) term

        float K = tan((PI_F * fc) / sampleRate);
        float K2 = K * K;
        float V0 = daisysp::pow10f(G / 20.f);
        float root2 = 1.f / Q;

        if (V0 < 1.f) {
            V0 = 1.f / V0;
        }

        float V0K2 = V0 * K2;
        float r2sqrV0K = root2 * sqrtf(V0) * K;
        float r2K = root2 * K;

        if (G > 0.f) {
            b0 = (1.f + r2sqrV0K + V0K2) / (1 + r2K + 2);
            b1 = (2.f * (V0K2 - 1.f)) / (1 + r2K + 2);
            b2 = (1.f - r2sqrV0K + V0K2) / (1 + r2K + 2);
            a1 = (2.f * (K2 - 1.f)) / (1 + r2K + 2);
            ;
            a2 = (1.f - r2K + K2) / (1 + r2K + 2);
            ;
        } else {
            b0 = (1.f + r2K + K2) / (1 + r2sqrV0K + V0K2);
            b1 = (2.f * (K2 - 1.f)) / (1 + r2sqrV0K + V0K2);
            b2 = (1.f - r2K + K2) / (1 + r2sqrV0K + V0K2);
            a1 = (2.f * (V0K2 - 1.f)) / (1 + r2sqrV0K + V0K2);
            a2 = (1.f - r2sqrV0K + V0K2) / (1 + r2sqrV0K + V0K2);
        }

        a0 = 1.f;
    }

    // Apply the low shelf filter to a single sample
    float applyLowShelfFilter(float in)
    {
        float out = in * a0 + z1;
        z1 = in * a1 + z2 - b1 * out;
        z2 = in * a2 - b2 * out;
        return out;
    }

private:
    float a0, a1, a2;
    float b0, b1, b2;

    float z1, z2;

    float sampleRate;
};

}
