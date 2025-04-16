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

#include "controller.h"

using clevelandmusicco::Hothouse;
using daisy::Led;
using daisy::SaiHandle;

Hothouse hw;
DaisySeed hw2;

Led led_1, led_2;
bool led1_on = false, led2_on = false;
bool bypassA = true, bypassB = true;

bool toggle_state = false;

jkoDSP::Controller controller;

void PrintLoad()
{
    // get the current load (smoothed value and peak values)
    // const float avgLoad = loadMeter.GetAvgCpuLoad();
    // const float maxLoad = loadMeter.GetMaxCpuLoad();
    // const float minLoad = loadMeter.GetMinCpuLoad();
    // // print it to the serial connection (as percentages)
    // hw2.Print("Processing Load %:  ");
    // hw2.Print("Max: " FLT_FMT3, FLT_VAR3(maxLoad * 100.0f));
    // hw2.Print("  Avg: " FLT_FMT3, FLT_VAR3(avgLoad * 100.0f));
    // hw2.PrintLine("  Min: " FLT_FMT3, FLT_VAR3(minLoad * 100.0f));
}

void AudioCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out,
    size_t size)
{
    // loadMeter.OnBlockStart();
    hw.ProcessAllControls();

    //
    // Toggle LEDs based on footswitches
    //
    bypassA ^= hw.switches[Hothouse::FOOTSWITCH_1].RisingEdge();
    bypassB ^= hw.switches[Hothouse::FOOTSWITCH_2].RisingEdge();

    led1_on ^= hw.switches[Hothouse::FOOTSWITCH_1].RisingEdge();
    led2_on ^= hw.switches[Hothouse::FOOTSWITCH_2].RisingEdge();

    controller.ProcessControls(
        hw.knobs[Hothouse::KNOB_1].Value(),
        hw.knobs[Hothouse::KNOB_2].Value(),
        hw.knobs[Hothouse::KNOB_3].Value(),
        hw.knobs[Hothouse::KNOB_4].Value(),
        hw.knobs[Hothouse::KNOB_5].Value(),
        hw.knobs[Hothouse::KNOB_6].Value(),
        hw.GetToggleswitchPosition(Hothouse::TOGGLESWITCH_1),
        hw.GetToggleswitchPosition(Hothouse::TOGGLESWITCH_2),
        hw.GetToggleswitchPosition(Hothouse::TOGGLESWITCH_3),
        !bypassA,
        !bypassB);

    controller.process(in, out, size);

    // loadMeter.OnBlockEnd();
}

void Init()
{
    hw.Init(true);
    hw.SetAudioBlockSize(96); // Number of samples handled per callback
    hw.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_96KHZ);

    led_1.Init(hw.seed.GetPin(Hothouse::LED_1), false);
    led_2.Init(hw.seed.GetPin(Hothouse::LED_2), false);

    controller.init(hw.AudioSampleRate());

    hw.StartAdc();
    hw.StartAudio(AudioCallback);
}

int main()
{
    // hw2.StartLog(false);
    Init();

    while (true) {
        hw.DelayMs(10);

        if (led2_on && (toggle_state == false)) {
            PrintLoad();
            // hw2.PrintLine("Toggle state: %d", hw.GetToggleswitchPosition(Hothouse::TOGGLESWITCH_2));
        }

        if (!led2_on) {
            toggle_state = false;
        }

        // Toggle effect bypass LED when footswitch is pressed
        led_1.Set(led1_on ? 1.0f : 0.0f);
        led_1.Update();
        led_2.Set(led2_on ? 1.0f : 0.0f);
        led_2.Update();

        // Call System::ResetToBootloader() if FOOTSWITCH_1 is pressed for 2 seconds
        hw.CheckResetToBootloader();
    }

    return 0;
}