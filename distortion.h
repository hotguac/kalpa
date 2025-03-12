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

#include "daisysp.h"
// #include "filter.h"
#include "Biquad.h"

using daisysp::fmap;
using daisysp::Mapping;
using daisysp::Svf;

class Distortion {
public:
    const float lp_pre_hz = 6000.0f;
    const float hp_pre_hz = 80.0f;

    const float ls_pre_hz = 300.0f;
    const float ls_pre_gain = -6.0f;

    const float lp_post_hz = 8000.0f;

    const float ls_post_hz = ls_pre_hz;
    const float ls_post_gain = ls_pre_gain * -0.8f;

    const float pk_post_hz = 400.f;
    const float pk_post_q = 2.f;
    const float pk_post_gain = 3.f;

    void init(float samplerate)
    {
        sample_rate = samplerate;

        lp_pre.setType(bq_type_lowpass);
        lp_pre.setFc(lp_pre_hz / samplerate);
        lp_pre.setQ(0.707f);

        hp_pre.setType(bq_type_highpass);
        hp_pre.setFc(hp_pre_hz / samplerate);
        hp_pre.setQ(0.707f);

        ls_pre.setType(bq_type_lowshelf);
        ls_pre.setFc(ls_pre_hz / samplerate);
        ls_pre.setQ(0.707f);
        ls_pre.setPeakGain(ls_pre_gain);

        lp_post.setType(bq_type_lowpass);
        lp_post.setFc(lp_post_hz / samplerate);
        lp_post.setQ(0.707f);

        ls_post.setType(bq_type_lowshelf);
        ls_post.setFc(ls_post_hz / samplerate);
        ls_post.setQ(0.707f);
        ls_post.setPeakGain(ls_post_gain);

        pk_post.setType(bq_type_peak);
        pk_post.setFc(pk_post_hz / samplerate);
        pk_post.setQ(pk_post_q);
        pk_post.setPeakGain(pk_post_gain);

        lp_tone.setType(bq_type_lowpass);
        lp_tone.setFc(lp_pre_hz / samplerate);
        lp_tone.setQ(0.707f);

    }

    float softClip(float in, float drive, float tone, float volume, float drive2, float tone2)
    {
        float out;

        // Bump up the signal level to drive more clipping
        out = in * (2.f + drive * 36.f);

        // Pre-clipping EQ
        out = lp_pre.process(out);
        out = hp_pre.process(out);
        out = ls_pre.process(out);

        // Our clipping function
        out = tanh(out);

        // Post clip EQ
        out = ls_post.process(out);
        out = pk_post.process(out);
        
        // User tone control
        lp_tone.setFc(get_freq(tone) / sample_rate);
        out = lp_tone.process(out);

        // Adjust output gain
        float gain = volume / (2.f + drive * 4.f);

        return (out * gain);
    }

private:
    float sample_rate;

    float get_freq(float tone)
    {
        return fmap(tone, 1200, 8000, Mapping::LOG);
    }
    float get_q(float tone)
    {
        return fmap(tone, 1.0f, 2.0f, Mapping::LINEAR);
    }

    Biquad lp_pre;
    Biquad hp_pre;
    Biquad ls_pre;

    Biquad lp_post;
    Biquad ls_post;
    Biquad pk_post;

    Biquad lp_tone;
};
