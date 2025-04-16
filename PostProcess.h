#ifndef POSTPROCESS_H
#define POSTPROCESS_H

#include "Biquad.h"
#include "daisysp.h"

using daisysp::Mapping;

namespace jkoDSP {

enum class EffectType {
    SLAP,
    REVERB,
    CHORUS
};

class PostProcessor {
public:
    PostProcessor() {};
    ~PostProcessor() {};

    void init(float samplerate)
    {
        this->samplerate = samplerate;
        chorus.Init(samplerate);
        blocker.Init(samplerate);

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
        // fx_intensity = intensity;
        fx_type = fxtyp;

        switch (fx_type) {
        case EffectType::SLAP:
            delaytime = fmap(fx_intensity, 0.01f, 0.05f, Mapping::EXP);
            delay.SetDelay(delaytime * samplerate);
            break;
        case EffectType::REVERB:
            feedback = fmap(fx_intensity, 0.1f, 0.3f, Mapping::EXP);
            depth = fmap(fx_intensity, 0.1f, 0.2f, Mapping::EXP);
            freq = fmap(fx_intensity, 2.f, 1.5f, Mapping::EXP);

            // fonepole(reverbFeedback, feedback, coeff);
            // fonepole(reverbDepth, depth, coeff);
            // fonepole(reverbFreq, freq, coeff);

            reverbFeedback = feedback;
            reverbDepth = depth;
            reverbFreq = freq;

            reverb.SetFeedback(chorusFeedback);
            reverb.SetLfoDepth(chorusDepth);
            reverb.SetLfoFreq(chorusFreq);

            break;
        case EffectType::CHORUS:
            feedback = fmap(fx_intensity, 0.1f, 0.9f, Mapping::EXP);
            depth = fmap(fx_intensity, 0.1f, 0.9f, Mapping::EXP);
            freq = fmap(fx_intensity, 1.f, 3.5f, Mapping::EXP);

            chorusFeedback = feedback;
            chorusDepth = depth;
            chorusFreq = freq;

            // fonepole(chorusFeedback, feedback, coeff);
            // fonepole(chorusDepth, depth, coeff);
            // fonepole(chorusFreq, freq, coeff);

            chorus.SetFeedback(chorusFeedback);
            chorus.SetLfoDepth(chorusDepth);
            chorus.SetLfoFreq(chorusFreq);
            break;
        default:
            // shouldn't get here
            break;
        }
    }

    void setVolume(float level, float drive, bool fs1)
    {
        if (fs1) {
            fonepole(volume, (level / (drive * 4.f + 1.f)), coeff);
        } else {
            volume = 1.f;
        }
    }

    void process(AudioHandle::OutputBuffer out, size_t size)
    {
        float output = 0.f;
        float mix = fmap(fx_intensity, 0.f, 0.9f, Mapping::LOG);

        mix = 0.5f;

        for (size_t i = 0; i < size; ++i) {
            output = out[0][i];
            output = lp_tone.process(output);

            // TODO: add a connection to the fx control for mix level
            if (useFX) {
                switch (fx_type) {
                case EffectType::SLAP:
                    output = output * (1.f - mix) + reverb.Process(output) * mix;
                    break;
                case EffectType::REVERB:
                    delay.Write(output);
                    output = output * (1.f - mix) + delay.Read() * mix;
                    break;
                case EffectType::CHORUS:
                    output = output * (1.f - mix) + chorus.Process(output) * mix;
                    break;
                default:
                    // shouldn't get here
                    break;
                }
            }

            output = blocker.Process(output);
            out[0][i] = out[1][i] = DSY_CLAMP(output * volume, -0.94f, 0.92f);
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
    daisysp::Chorus chorus;
    daisysp::DelayLine<float, 48000> delay;

    float chorusFeedback = 0.2f;
    float chorusDepth = 0.1f;
    float chorusFreq = 2.f;

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