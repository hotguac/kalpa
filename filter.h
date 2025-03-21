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

using daisysp::OnePole;

/*

*/

const int ft_undefined = 0;

class Filter {
public:
    // enum filterType : integer { ft_undefined };
    virtual void Init(float sampleRate, int ftype) = 0;
    virtual void reset() = 0;
    virtual float render(float in) = 0;
    virtual void setFreq(float freq) = 0;
    virtual void updateCoefficients() = 0;

protected:
    float sampleRate = 48000.f;
    int ftype = ft_undefined;
    float freq;
};

class LowPass : Filter {
public:
    enum filterType : int { ft_cytomic_svf = 1,
        ft_daisy_onepole = 2,
        ft_daisy_biquad = 3,
        ft_daisy_svf = 4 };

    void Init(float sampleRate, int ftype) override
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

    void reset()
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

    float render(float in)
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

    void setFreq(float freq)
    {
        this->freq = freq;
        updateCoefficients();
    }

    void updateCoefficients()
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

private:
    jkoDSP::Biquad biquad;
    OnePole onepole;
};

// Provides a set of audio filters
class xLowpass : Filter {
public:
    enum filterType {
        ft_cytomic_svf,
        ft_onepole_lowpass1, // daisy onepole
        ft_onepole_highpass1, // daisy onepole
        ft_lowshelf1, // daisy biquad
        ft_lowshelf2, // daisy svf
        ft_lowpass1, // daisy biquad
        ft_lowpass2, // daisy svf
        ft_highpass1, // daisy biquad
        ft_highpass2,
        ft_peak1, // daisy biquad
        ft_notch1, // daisy biquad
        ft_bandpass1, // daisy biquad
        ft_allpass
    };

    // sample_rate - samples per second
    // filter_type - enum
    void Init(float sampleRate, filterType filter_type)
    {
        fs = sampleRate;
        fType = filter_type;
        fc = 8000.f;
        Q = 2.f;
        gain = 1.f;
        resonance = 0.5f;
        bandwidth = 400.f;

        reset();
        updateCoefficients();
    }

    // sets the internal state variables to known values
    void reset()
    {
        state1 = 0.f;
        state2 = 0.f;

        g = 0.f;
        k = 0.f;

        a0 = a1 = a2 = a3 = 0.f;
        b0 = b1 = b2 = b3 = 0.f;
    }

    // in is the next sample to process, returns a filtered sample
    float render(float in)
    {
        float out = in;

        return out;
    }

    // Filter cutoff in hz
    void setCutoff(float cutoff)
    {
        fc = daisysp::fclamp(cutoff, 10.f, 20000.f);
        ;
        updateCoefficients();
    }

    // Calculate the filter coefficients
    void updateCoefficients()
    {
        switch (fType) {
        case ft_allpass:
            /* code */
            break;

        case ft_bandpass1:
            /* code */
            break;

        case ft_cytomic_svf:
            /* code */
            break;

        case ft_highpass1:
            /* code */
            break;

        case ft_highpass2:
            /* code */
            break;

        case ft_lowpass1:
            /* code */
            break;

        case ft_lowpass2:
            /* code */
            break;

        case ft_peak1:
            /* code */
            break;

        case ft_lowshelf1:
            /* code */
            break;

        case ft_lowshelf2:
            /* code */
            break;

        case ft_onepole_highpass1:
            /* code */
            break;

        case ft_onepole_lowpass1:
            /* code */
            break;

        default:
            break;
        }
    }

    void setQ(float Q)
    {
        this->Q = daisysp::fclamp(Q, 0.707f, 20.f);
    }

    void setGain(float gain)
    {
        this->gain = gain;
    }

    void setResonance(float resonance)
    {
        this->resonance = resonance;
    }

    void setBandwidth(float bandwidth)
    {
        this->bandwidth = bandwidth;
    }

    float getQ()
    {
        return Q;
    }

    float getGain()
    {
        return gain;
    }

    float getResonance()
    {
        return resonance;
    }

    // Filter cutoff in hz
    float getCutoff()
    {
        return fc;
    }

private:
    float fs;

    filterType fType;

    float fc;
    float Q;
    float gain;
    float resonance;
    float bandwidth;

    float state1;
    float state2;

    float a0, a1, a2, a3;
    float b0, b1, b2, b3;

    float g, k;
};

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
    void init(float sample_rate) {
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
            b0 = (1.f + r2sqrV0K + V0K2)  / (1 + r2K + 2);
            b1 = (2.f * (V0K2 - 1.f))  / (1 + r2K + 2);
            b2 = (1.f - r2sqrV0K + V0K2)  / (1 + r2K + 2);
            a1 = (2.f * (K2 - 1.f))  / (1 + r2K + 2);;
            a2 = (1.f - r2K + K2) / (1 + r2K + 2);;
        } else {
            b0 = (1.f + r2K + K2)  / (1 + r2sqrV0K + V0K2);
            b1 = (2.f * (K2 - 1.f))  / (1 + r2sqrV0K + V0K2);
            b2 = (1.f - r2K + K2)  / (1 + r2sqrV0K + V0K2);
            a1 = (2.f * (V0K2 - 1.f))  / (1 + r2sqrV0K + V0K2);
            a2 = (1.f - r2sqrV0K + V0K2) / (1 + r2sqrV0K + V0K2);
        }

        a0 = 1.f;
    }

    // Apply the low shelf filter to a single sample
    float applyLowShelfFilter(float in) {
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




/*

function [b, a]  = shelving(G, fc, fs, Q, type)

%
% Derive coefficients for a shelving filter with a given amplitude and
% cutoff frequency.  All coefficients are calculated as described in 
% Zolzer's DAFX book (p. 50 -55).  
%
% Usage:     [B,A] = shelving(G, Fc, Fs, Q, type);
%
%            G is the logrithmic gain (in dB)
%            FC is the center frequency
%            Fs is the sampling rate
%            Q adjusts the slope be replacing the sqrt(2) term
%            type is a character string defining filter type
%                 Choices are: 'Base_Shelf' or 'Treble_Shelf'
%
% Author:    sparafucile17 08/22/05
%

%Error Check
if((strcmp(type,'Base_Shelf') ~= 1) && (strcmp(type,'Treble_Shelf') ~= 1))
    error(['Unsupported Filter Type: ' type]);
end

K = tan((pi * fc)/fs);
V0 = 10^(G/20);
root2 = 1/Q; %sqrt(2)

%Invert gain if a cut
if(V0 < 1)
    V0 = 1/V0;
end

%%%%%%%%%%%%%%%%%%%%
%    BASE BOOST
%%%%%%%%%%%%%%%%%%%%
if(( G > 0 ) & (strcmp(type,'Base_Shelf')))
   
    b0 = (1 + sqrt(V0)*root2*K + V0*K^2) / (1 + root2*K + K^2);
    b1 =             (2 * (V0*K^2 - 1) ) / (1 + root2*K + K^2);
    b2 = (1 - sqrt(V0)*root2*K + V0*K^2) / (1 + root2*K + K^2);
    a1 =                (2 * (K^2 - 1) ) / (1 + root2*K + K^2);
    a2 =             (1 - root2*K + K^2) / (1 + root2*K + K^2);

%%%%%%%%%%%%%%%%%%%%
%    BASE CUT
%%%%%%%%%%%%%%%%%%%%
elseif (( G < 0 ) & (strcmp(type,'Base_Shelf')))
    
    b0 =             (1 + root2*K + K^2) / (1 + root2*sqrt(V0)*K + V0*K^2);
    b1 =                (2 * (K^2 - 1) ) / (1 + root2*sqrt(V0)*K + V0*K^2);
    b2 =             (1 - root2*K + K^2) / (1 + root2*sqrt(V0)*K + V0*K^2);
    a1 =             (2 * (V0*K^2 - 1) ) / (1 + root2*sqrt(V0)*K + V0*K^2);
    a2 = (1 - root2*sqrt(V0)*K + V0*K^2) / (1 + root2*sqrt(V0)*K + V0*K^2);

%%%%%%%%%%%%%%%%%%%%
%   TREBLE BOOST
%%%%%%%%%%%%%%%%%%%%
elseif (( G > 0 ) & (strcmp(type,'Treble_Shelf')))

    b0 = (V0 + root2*sqrt(V0)*K + K^2) / (1 + root2*K + K^2);
    b1 =             (2 * (K^2 - V0) ) / (1 + root2*K + K^2);
    b2 = (V0 - root2*sqrt(V0)*K + K^2) / (1 + root2*K + K^2);
    a1 =              (2 * (K^2 - 1) ) / (1 + root2*K + K^2);
    a2 =           (1 - root2*K + K^2) / (1 + root2*K + K^2);

%%%%%%%%%%%%%%%%%%%%
%   TREBLE CUT
%%%%%%%%%%%%%%%%%%%%

elseif (( G < 0 ) & (strcmp(type,'Treble_Shelf')))

    b0 =               (1 + root2*K + K^2) / (V0 + root2*sqrt(V0)*K + K^2);
    b1 =                  (2 * (K^2 - 1) ) / (V0 + root2*sqrt(V0)*K + K^2);
    b2 =               (1 - root2*K + K^2) / (V0 + root2*sqrt(V0)*K + K^2);
    a1 =             (2 * ((K^2)/V0 - 1) ) / (1 + root2/sqrt(V0)*K + (K^2)/V0);
    a2 = (1 - root2/sqrt(V0)*K + (K^2)/V0) / (1 + root2/sqrt(V0)*K + (K^2)/V0);

%%%%%%%%%%%%%%%%%%%%
%   All-Pass
%%%%%%%%%%%%%%%%%%%%
else
    b0 = V0;
    b1 = 0;
    b2 = 0;
    a1 = 0;
    a2 = 0;
end

%return values
a = [  1, a1, a2];
b = [ b0, b1, b2];


*/