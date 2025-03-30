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
    void init(float samplerate)
    {
        this->samplerate = samplerate * 2.f;

        init_pre();
        init_nolinear();
        init_post();
    }

    void setPrePeak(float freq, float Q, float gain)
    {
        peak_pre.setBiquad(jkoDSP::bq_type_peak, freq / samplerate, Q, gain);
    }

    void setPostPeak(float freq, float Q, float gain)
    {
        peak_post.setBiquad(jkoDSP::bq_type_peak, freq / samplerate, Q, gain);
    }

    float softClip(float in, float drive, float tone, float volume)
    {
        float out = in;

        out = pre_process(out);
        out = nonlinear_process(out, drive);
        out = post_process(out, drive, tone, volume);

        return out;
    }

private:
    float samplerate;

    const float lp_pre_hz = 3600.0f;
    const float hp_pre_hz = 40.0f;

    const float ls_pre_hz = 300.0f;
    const float ls_pre_gain = -6.0f;

    const float ls_post_hz = ls_pre_hz;
    const float ls_post_gain = ls_pre_gain * -0.9f;

    const float lp_post_hz = 8000.0f;

    void init_pre()
    {
        lp_pre.setType(jkoDSP::bq_type_lowpass);
        lp_pre.setFc(lp_pre_hz / samplerate);
        lp_pre.setQ(0.707f);

        hp_pre.setType(jkoDSP::bq_type_highpass);
        hp_pre.setFc(hp_pre_hz / samplerate);
        hp_pre.setQ(0.707f);

        peak_pre.setType(jkoDSP::bq_type_peak);
        peak_pre.setFc(500.f / samplerate);
        peak_pre.setQ(1.f);
        peak_pre.setPeakGain(-3.f);
    }

    void init_nolinear()
    {
        lp_anti_pre.setType(jkoDSP::bq_type_lowpass);
        lp_anti_pre.setFc(6000.f / samplerate * 2.f);
        lp_anti_pre.setQ(0.707f);

        lp_anti_post.setType(jkoDSP::bq_type_lowpass);
        lp_anti_post.setFc(6000.f / samplerate * 2.f);
        lp_anti_post.setQ(0.707f);
    }

    void init_post()
    {
        lp_tone.setType(jkoDSP::bq_type_lowpass);
        lp_tone.setFc(lp_pre_hz / samplerate);
        lp_tone.setQ(0.707f);

        peak_post.setType(jkoDSP::bq_type_peak);
        peak_post.setFc(1000.f / samplerate);
        peak_post.setQ(1.f);
        peak_post.setPeakGain(6.f);
    }

    // Pre-clip EQ
    float pre_process(float in)
    {
        float out = in;

        out = lp_pre.process(out);
        out = hp_pre.process(out);

        out = peak_pre.process(out);

        return out;
    }

    float nonlinear_process(float in, float drive)
    {
        float out;

        // add drive to increase distortion
        // add offset to increase assymetry
        const float offset = -0.1f;
        float preGain = fmap(drive, 12.f, 140.f, Mapping::EXP);

        for (int i = 0; i < 2; ++i) {
            if (i == 0) {
                out = 0.f;
            } else {
                out = in * preGain + offset;
            }
            // Anti-aliasing filter
            out = lp_anti_pre.process(out);

            // Our clipping function
            out = fast_tanh(out);
            // float xdrive = 2.8; -- good range 2 to 3 ish
            // out = fast_tanh(out * xdrive) / fast_tanh(xdrive);

            // Anti-aliasing filter
            out = lp_anti_post.process(out);
        }

        return out;
    }

    float post_process(float in, float drive, float tone, float volume)
    {
        float out = in;

        out = peak_post.process(out);

        // User tone control
        lp_tone.setFc(get_freq(tone) / samplerate);
        out = lp_tone.process(out);

        // Adjust output gain
        float gain = volume / (1.f + drive * 4.f);

        return (out * gain);
    }

    float get_freq(float tone)
    {
        return fmap(tone, 1200, 12000, Mapping::LOG);
    }

    float get_q(float tone)
    {
        return fmap(tone, 1.0f, 2.0f, Mapping::LINEAR);
    }

    jkoDSP::Biquad lp_pre;
    jkoDSP::Biquad hp_pre;

    jkoDSP::Biquad lp_tone;
    jkoDSP::Biquad lp_anti_pre;
    jkoDSP::Biquad lp_anti_post;

    jkoDSP::Biquad peak_pre;
    jkoDSP::Biquad peak_post;
};
