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

Distortion distA, distB;

float current_sample = 0.0f;

float driveA;
float toneA;
float volumeA;

float driveB;
float toneB;
float volumeB;

float denormal_guard = 10e-15f;

float samplerate;
float coeff = 0.01f;
float smooth_drive = 0.f;
float smooth_tone = 0.f;
float smooth_volume = 0.f;

DcBlock blocker;

float processA(float in)
{
    float out = 0.0f;

    if (bypassA) {
        return in;
    }

    float coeff = 1.f / (0.01f * samplerate);
    daisysp::fonepole(smooth_drive, driveA, coeff);
    daisysp::fonepole(smooth_tone, toneA, coeff);
    daisysp::fonepole(smooth_volume, volumeA, coeff);

    out = distA.softClip(in, smooth_drive, smooth_tone, smooth_volume);

    return out;
}

void AudioCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out,
    size_t size)
{
    denormal_guard = -denormal_guard;

    hw.ProcessAllControls();

    bypassA ^= hw.switches[Hothouse::FOOTSWITCH_1].RisingEdge();
    bypassB ^= hw.switches[Hothouse::FOOTSWITCH_2].RisingEdge();

    // Toggle LEDs based on footswitches
    led1_on ^= hw.switches[Hothouse::FOOTSWITCH_1].RisingEdge();
    led2_on ^= hw.switches[Hothouse::FOOTSWITCH_2].RisingEdge();

    driveA = hw.GetKnobValue(Hothouse::KNOB_1);
    toneA = hw.GetKnobValue(Hothouse::KNOB_2);
    volumeA = hw.GetKnobValue(Hothouse::KNOB_3);

    driveB = hw.GetKnobValue(Hothouse::KNOB_4);
    toneB = hw.GetKnobValue(Hothouse::KNOB_5);
    volumeB = hw.GetKnobValue(Hothouse::KNOB_6);

    float output = 0.0f;
    float input = 0.0f;

    for (size_t i = 0; i < size; ++i) {
        input = in[0][i] + denormal_guard;

        if (bypassA) {
            output = input;
        } else {
            output = processA(input);
        }

        out[0][i] = out[1][i] = blocker.Process(output);
    }
}

int main()
{
    hw.Init();
    hw.SetAudioBlockSize(48); // Number of samples handled per callback
    hw.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_48KHZ);

    samplerate = hw.AudioSampleRate();

    distA.init(samplerate);
    distB.init(samplerate);
    blocker.Init(samplerate);

    led_1.Init(hw.seed.GetPin(Hothouse::LED_1), false);
    led_2.Init(hw.seed.GetPin(Hothouse::LED_2), false);

    hw.StartAdc();
    hw.StartAudio(AudioCallback);

    while (true) {
        hw.DelayMs(10);

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