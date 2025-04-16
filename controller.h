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

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "daisysp.h"

#include "hothouse.h"

#include "NonLinear.h"
#include "PostProcess.h"
#include "PreProcess.h"

using clevelandmusicco::Hothouse;
using daisysp::Mapping;

namespace jkoDSP {

class Controller {
private:
    float driveA = 0.f;
    float toneA = 0.f;
    float volumeA = 0.f;

    float freq = 100.f;
    float Q = 1.f;

    bool bypassA = true, bypassB = true;

    Hothouse::ToggleswitchPosition sw2;

    jkoDSP::PreProcessor preprocess;
    jkoDSP::NonLinearProcessor nonlinear;
    jkoDSP::PostProcessor postprocess;

public:
    Controller(/* args */);
    ~Controller();

    void init(float samplerate);

    void process(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out,
        size_t size);

    void ProcessControls(float kb1, float kb2, float kb3, float kb4,
        float kb5, float kb6, Hothouse::ToggleswitchPosition sw1,
        Hothouse::ToggleswitchPosition sw2, Hothouse::ToggleswitchPosition sw3, bool fs1,
        bool fs2);
};

Controller::Controller() { }
Controller::~Controller() { }

void Controller::init(float samplerate)
{
    // Initialize processors before starting the audio
    preprocess.init(samplerate);
    nonlinear.init(samplerate);
    postprocess.init(samplerate);
}

void Controller::process(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out,
    size_t size)
{
    preprocess.process(in, out, size);
    nonlinear.process(out, size);
    postprocess.process(out, size);
}

void Controller::ProcessControls(float drive, float tone, float volume, float freq,
    float gain, float fx_intensity, Hothouse::ToggleswitchPosition sw1,
    Hothouse::ToggleswitchPosition sw2, Hothouse::ToggleswitchPosition sw3, bool fs1,
    bool fs2)
{
    //
    // Pre-Process controls
    //
    switch (sw1) {
    case Hothouse::TOGGLESWITCH_UP:
        preprocess.setBoost(1.f);
        break;

    case Hothouse::TOGGLESWITCH_MIDDLE:
        preprocess.setBoost(2.f);
        break;
    case Hothouse::TOGGLESWITCH_DOWN:
        preprocess.setBoost(4.f);
        break;

    default:
        // should never get here!!
        break;
    }

    preprocess.setSemiParametric(freq, gain);

    //
    // Non-linear controls
    //
    nonlinear.setDrive(drive);

    //
    // Post-Process controls
    //
    switch (sw2) {
    case Hothouse::TOGGLESWITCH_UP:
        postprocess.setFX(fx_intensity, jkoDSP::EffectType::SLAP);
        break;
    case Hothouse::TOGGLESWITCH_MIDDLE:
        postprocess.setFX(fx_intensity, jkoDSP::EffectType::REVERB);
        break;
    case Hothouse::TOGGLESWITCH_DOWN:
        postprocess.setFX(fx_intensity, jkoDSP::EffectType::CHORUS);
        break;
    default:
        // should never get here!!
        break;
    }

    postprocess.setTone(tone);
    postprocess.setVolume(volume, drive, fs1);

    //
    // Bypass controls
    //
    preprocess.enable(fs1);
    nonlinear.enable(fs1);
    postprocess.enable(fs1);

    switch (sw3) {
    case Hothouse::TOGGLESWITCH_UP:
        preprocess.enablePreEQ(fs2);
        break;

    case Hothouse::TOGGLESWITCH_MIDDLE:
        postprocess.enableFX(fs2);
        break;

    case Hothouse::TOGGLESWITCH_DOWN:
        preprocess.enableBoost(fs2);
        break;

    default:
        // should never get here!!
        break;
    }
}
}

#endif // CONTROLLER_H