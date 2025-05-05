#ifndef POSTPROCESS_H
#define POSTPROCESS_H

#include "Biquad.h"
#include "daisysp.h"
#include "reverb.h"

using daisysp::Mapping;

namespace jkoDSP {

enum class EffectType {
    SLAP,
    AMBIENCE,
    REVERB
};

class PostProcessor {
public:
    PostProcessor() {};
    ~PostProcessor() {};

    void init(float samplerate)
    {
        this->samplerate = samplerate;
        reverb.Init(samplerate);
        blocker.Init(samplerate);
        reverb2.Init();

        lp_tone.setType(jkoDSP::bq_type_lowpass);
        lp_tone.setFc(100.f / samplerate);
        lp_tone.setQ(0.707f);
    };

    void setTone(float tone) {
        lp_tone.setFc(get_freq(tone) / samplerate);
    }

    void enableFX(bool enable)
    {
        useFX = enable;
    }

    void enable(bool enable)
    {
        useFX = enable;
        useToneVol = enable;
    }

    void setFX(float intensity, EffectType fxtyp)
    {
        float feedback;
        float depth;
        float freq;

        float delaytime = 0.1f;

        fonepole(fx_intensity, intensity, coeff);

        fx_type = fxtyp;

        switch (fx_type) {
        case EffectType::SLAP:
            delaytime = fmap(fx_intensity, 0.06f, 0.120f, Mapping::EXP);
            delay.SetDelay(delaytime * samplerate);
            // delay.SetDelay(46000.f);
            break;
        case EffectType::AMBIENCE:
            feedback = fmap(fx_intensity, 0.1f, 0.2f, Mapping::EXP);
            depth = fmap(fx_intensity, 0.1f, 0.3f, Mapping::EXP);
            freq = fmap(fx_intensity, 1.f, 2.5f, Mapping::EXP);

            fonepole(reverbFeedback, feedback, coeff);
            fonepole(reverbDepth, depth, coeff);
            fonepole(reverbFreq, freq, coeff);

            reverb.SetDelayMs(40.f);
            reverb.SetFeedback(reverbFeedback);
            reverb.SetLfoDepth(reverbDepth);
            reverb.SetLfoFreq(reverbFreq);
            break;
        case EffectType::REVERB:
            feedback = fmap(fx_intensity, 0.2f, 0.6f, Mapping::EXP);
            depth = fmap(fx_intensity, 0.3f, 0.5f, Mapping::EXP);
            freq = fmap(fx_intensity, 0.5f, 1.f, Mapping::EXP);

            fonepole(reverbFeedback, feedback, coeff);
            fonepole(reverbDepth, depth, coeff);
            fonepole(reverbFreq, freq, coeff);

            reverb.SetDelayMs(80.f);
            reverb.SetFeedback(reverbFeedback);
            reverb.SetLfoDepth(reverbDepth);
            reverb.SetLfoFreq(reverbFreq);
            break;
        default:
            // shouldn't get here
            break;
        }
    }

    void setVolume(float level, float drive, bool fs1)
    {
        // TODO: take boost into account
        if (fs1) {
            fonepole(volume, (level / (drive * 4.f + 4.f)), 0.1f);
        } else {
            fonepole(volume, 1.f, 0.1f);
        }
    }

    void process(AudioHandle::OutputBuffer out, size_t size)
    {
        float output = 0.f;
        float mix = 0.5f;

        for (size_t i = 0; i < size; ++i) {
            output = out[0][i];

            if (useToneVol) {
                output = lp_tone.process(output);
            }

            if (useFX) {
                switch (fx_type) {
                case EffectType::SLAP:
                    mix = fmap(fx_intensity, 0.1f, 0.4f, Mapping::LINEAR);
                    delay.Write(output);
                    output = output * (1.2f - mix) + delay.Read() * mix;
                    break;
                case EffectType::AMBIENCE:
                    mix = fmap(fx_intensity, 0.4f, 1.2f, Mapping::LINEAR);
                    output = output * (1.2f - mix) + reverb.Process(output) * mix;
                    break;
                case EffectType::REVERB:
                    mix = fmap(fx_intensity, 0.125f, 0.50f, Mapping::LINEAR);
                    reverb2.setBandwidth(daisysp::fmap(fx_intensity, 0.25f, 0.65f, Mapping::EXP));
                    reverb2.setDamping(1.0f - daisysp::fmap(fx_intensity, 0.4f, 0.6f, Mapping::EXP));
                    reverb2.setDecay(daisysp::fmap(fx_intensity, 0.125f, 0.375f));
                    output = output * (1.2f - mix) + reverb2.Process(output) * mix;
                    break;
                default:
                    // shouldn't get here
                    break;
                }
            }

            output = blocker.Process(output);
            if (useToneVol) {
                out[0][i] = out[1][i] = DSY_CLAMP(output * volume, -0.94f, 0.92f);
            } else {
                out[0][i] = out[1][i] = DSY_CLAMP(output, -0.94f, 0.92f);
            }
        }
    };

    void reset();

private:
    float samplerate;
    float coeff = 0.01f;

    float volume = 0.f;

    bool useToneVol = false;
    float tone = 0.f;

    bool useFX = false;
    float fx_intensity;
    EffectType fx_type;

    daisysp::ChorusEngine reverb;
    Reverb reverb2;

    // daisysp::Chorus chorus;
    daisysp::DelayLine<float, 48000> delay;

    // float chorusFeedback = 0.2f;
    // float chorusDepth = 0.1f;
    // float chorusFreq = 2.f;

    float reverbFeedback = 0.2f;
    float reverbDepth = 0.1f;
    float reverbFreq = 2.f;

    jkoDSP::Biquad lp_tone;

    daisysp::DcBlock blocker;

    float get_freq(float tone)
    {
        return fmap(tone, 200, 12000, Mapping::LOG);
    }

    // float get_gain(float gain)
    // {
    //     return fmap(gain, -12.f, 12.f, Mapping::LINEAR);
    // }

    float get_q(float tone)
    {
        return fmap(tone, 1.0f, 2.0f, Mapping::LINEAR);
    }

    // float get_volume(float volume)
    // {
    //     return fmap(volume, -12.f, 12.f, Mapping::LINEAR);
    // }
};

} // namespace jkoDSP

#endif // POSTPROCESS_H