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

#include "Biquad.h"
#include "daisysp.h"

using daisysp::fmap;
using daisysp::Mapping;
using daisysp::Svf;

static inline float fast_tanh(float x)
{
    if (x > 3.0f)
        return 1.0f;
    if (x < -3.0f)
        return -1.0f;
    float x2 = x * x;
    return x * (27.0f + x2) / (27.0f + 9.0f * x2);
}

class Distortion {
public:
    const float lp_pre_hz = 2400.0f;
    const float hp_pre_hz = 60.0f;

    const float ls_pre_hz = 300.0f;
    const float ls_pre_gain = -6.0f;

    const float ls_post_hz = ls_pre_hz;
    const float ls_post_gain = ls_pre_gain * -0.9f;

    const float lp_post_hz = 8000.0f;

    //    const float pk_post_hz = 400.f;
    //    const float pk_post_q = 2.f;
    //    const float pk_post_gain = 3.f;

    void init(float samplerate)
    {
        sample_rate = samplerate * 2.f;

        lp_pre.setType(jkoDSP::bq_type_lowpass);
        lp_pre.setFc(lp_pre_hz / sample_rate);
        lp_pre.setQ(0.707f);

        hp_pre.setType(jkoDSP::bq_type_highpass);
        hp_pre.setFc(hp_pre_hz / sample_rate);
        hp_pre.setQ(0.707f);

        lp_tone.setType(jkoDSP::bq_type_lowpass);
        lp_tone.setFc(lp_pre_hz / sample_rate);
        lp_tone.setQ(0.707f);

        lp_anti.setType(jkoDSP::bq_type_lowpass);
        lp_anti.setFc(6000.f / sample_rate);
        lp_anti.setQ(0.707f);
    }

    float softClip(float in, float drive, float tone, float volume)
    {
        float out = in;

        // Pre-clip EQ
        out = lp_pre.process(out);
        out = hp_pre.process(out);

        // add drive to increase distortion
        // add offset to increase assymetry
        const float offset = -0.1f;
        float preGain = fmap(drive, 4.f, 100.f, Mapping::EXP);

        out = out * preGain + offset;

        // Our clipping function
        out = fast_tanh(out);
        // float xdrive = 2.8; -- good range 2 to 3 ish
        // out = fast_tanh(out * xdrive) / fast_tanh(xdrive);

        // Anti-aliasing filter
        out = lp_anti.process(out);

        // User tone control
        lp_tone.setFc(get_freq(tone) / sample_rate);
        out = lp_tone.process(out);

        // Adjust output gain
        float gain = volume / (3.f + drive * 10.f);

        return (out * gain);
    }

private:
    float sample_rate;

    float get_freq(float tone)
    {
        return fmap(tone, 1800, 6000, Mapping::LOG);
    }

    float get_q(float tone)
    {
        return fmap(tone, 1.0f, 2.0f, Mapping::LINEAR);
    }

    jkoDSP::Biquad lp_pre;
    jkoDSP::Biquad hp_pre;

    jkoDSP::Biquad lp_tone;
    jkoDSP::Biquad lp_anti;
};
