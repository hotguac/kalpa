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

#include "daisysp-lgpl.h"
#include "daisysp.h"
#include "distortion.h"
#include "hothouse.h"

using clevelandmusicco::Hothouse;
using daisy::AudioHandle;
using daisy::Led;
using daisy::SaiHandle;
using daisysp::DcBlock;

Hothouse hw;

Led led_1, led_2;
bool led1_on = false, led2_on = false;
bool bypassA = true, bypassB = true;

Distortion distA;
daisysp::ChorusEngine chorus;
bool use_chorus = false;

float current_sample = 0.f;

float driveA = 0.f;
float toneA = 0.f;
float volumeA = 0.f;

float freq = 100.f;
float Q = 1.f;
float gain = 1.f;

float samplerate;

float smooth_drive = 0.f;
float smooth_tone = 0.f;
float smooth_volume = 0.f;

float smooth_freq = 100.f;
float smooth_Q = 1.f;
float smooth_gain = 1.f;

DcBlock blocker;

float coeff = 0.0001f;

float processA(float in)
{
    float out = 0.0f;

    if (bypassA) {
        return in;
    }

    out = distA.softClip(in, smooth_drive, smooth_tone, smooth_volume);

    return out;
}

void ProcessControls()
{
    driveA = hw.GetKnobValue(Hothouse::KNOB_1);
    driveA = fmap(driveA, 12.f, 140.f, Mapping::EXP);

    Hothouse::ToggleswitchPosition sw1 = hw.GetToggleswitchPosition(Hothouse::TOGGLESWITCH_1);
    if (sw1 == Hothouse::ToggleswitchPosition::TOGGLESWITCH_MIDDLE) {
        driveA *= 2.f; // +6db
    } else if (sw1 == Hothouse::ToggleswitchPosition::TOGGLESWITCH_DOWN) {
        driveA *= 4.f; // +12db
    }

    Hothouse::ToggleswitchPosition sw2 = hw.GetToggleswitchPosition(Hothouse::TOGGLESWITCH_2);
    switch (sw2) {
    case Hothouse::TOGGLESWITCH_UP:
        chorus.SetFeedback(0.2f);
        chorus.SetLfoDepth(0.1f);
        chorus.SetLfoFreq(2.f);
        use_chorus = true;
        break;

    case Hothouse::TOGGLESWITCH_MIDDLE:
        chorus.SetFeedback(0.3f);
        chorus.SetLfoDepth(0.2f);
        chorus.SetLfoFreq(1.5f);
        use_chorus = true;
        break;

    default:
        use_chorus = false;
        break;
    }

    toneA = hw.GetKnobValue(Hothouse::KNOB_2);
    volumeA = hw.GetKnobValue(Hothouse::KNOB_3);

    freq = hw.GetKnobValue(Hothouse::KNOB_4);
    freq = fmap(freq, 160, 3000, Mapping::LOG);

    Q = hw.GetKnobValue(Hothouse::KNOB_5);
    Q = fmap(Q, 0.5f, 1.5f, Mapping::LINEAR);

    gain = hw.GetKnobValue(Hothouse::KNOB_6);
    gain = fmap(gain, -12.f, 12.0f, Mapping::LINEAR);

    daisysp::fonepole(smooth_drive, driveA, coeff);
    daisysp::fonepole(smooth_tone, toneA, coeff);
    daisysp::fonepole(smooth_volume, volumeA, coeff);

    daisysp::fonepole(smooth_freq, freq, coeff / 2.f);
    daisysp::fonepole(smooth_Q, Q, coeff / 2.f);
    daisysp::fonepole(smooth_gain, gain, coeff / 2.f);

    // Toggle LEDs based on footswitches
    bypassA ^= hw.switches[Hothouse::FOOTSWITCH_1].RisingEdge();
    bypassB ^= hw.switches[Hothouse::FOOTSWITCH_2].RisingEdge();

    led1_on ^= hw.switches[Hothouse::FOOTSWITCH_1].RisingEdge();
    led2_on ^= hw.switches[Hothouse::FOOTSWITCH_2].RisingEdge();
}

void AudioCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out,
    size_t size)
{
    hw.ProcessAllControls();

    ProcessControls();
    distA.setPrePeak(smooth_freq, smooth_Q, smooth_gain);

    float output = 0.0f;
    float input = 0.0f;

    for (size_t i = 0; i < size; ++i) {
        input = in[0][i]; // + denormal_guard;

        if (bypassA) {
            output = input;
        } else {
            output = processA(input);
        }

        output = blocker.Process(output);
        if (use_chorus) {
            float wet = chorus.Process(output);
            output = (output + wet) * 0.5f;
        }

        out[0][i] = out[1][i] = DSY_CLAMP(output, -0.90f, 0.92f);
    }
}

int main()
{
    hw.Init();
    hw.SetAudioBlockSize(96); // Number of samples handled per callback
    hw.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_96KHZ);

    // DaisySeed::StartLog();

    samplerate = hw.AudioSampleRate();
    coeff = 1.f / (0.01f * samplerate); // 10ms ramp to target setting

    distA.init(samplerate);
    blocker.Init(samplerate);

    chorus.Init(samplerate);

    led_1.Init(hw.seed.GetPin(Hothouse::LED_1), false);
    led_2.Init(hw.seed.GetPin(Hothouse::LED_2), false);

    hw.StartAdc();
    hw.StartAudio(AudioCallback);

    // DaisySeed::PrintLine("starting");

    while (true) {
        hw.DelayMs(10);

        // DaisySeed::PrintLine("looping");

        // Toggle effect bypass LED when footswitch is pressed
        // Toggle LEDs
        led_1.Set(led1_on ? 1.0f : 0.0f);
        led_1.Update();
        led_2.Set(led2_on ? 1.0f : 0.0f);
        led_2.Update();

        // Call System::ResetToBootloader() if FOOTSWITCH_1 is pressed for 2 seconds
        hw.CheckResetToBootloader();
    }
    return 0;
}