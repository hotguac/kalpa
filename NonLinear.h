#ifndef JKO_DSP_NONLINEAR_H
#define JKO_DSP_NONLINEAR_H

#include "Biquad.h"
#include "daisysp-lgpl.h"
#include "daisysp.h"

using daisy::AudioHandle;
using daisysp::fonepole;
using daisysp::Mapping;

namespace jkoDSP {

class NonLinearProcessor {
public:
    NonLinearProcessor() {};
    ~NonLinearProcessor() {};

    void init(float hw_samplerate)
    {
        samplerate = hw_samplerate;

        lp_anti_pre.setType(jkoDSP::bq_type_lowpass);
        lp_anti_pre.setFc(6000.f / samplerate * 2.f);
        lp_anti_pre.setQ(0.707f);

        lp_anti_post.setType(jkoDSP::bq_type_lowpass);
        lp_anti_post.setFc(6000.f / samplerate * 2.f);
        lp_anti_post.setQ(0.707f);
    }

    void enable(bool enable)
    {
        useDistortion = enable;
    }

    void setDrive(float target_drive)
    {
        target_drive = fmap(target_drive, 12.f, 140.f, Mapping::EXP);
        fonepole(drive, target_drive, coeff);
    }

    void process(AudioHandle::OutputBuffer out, size_t size)
    {
        const float offset = -0.1f;
        float output = 0.f;

        if (useDistortion) {
            for (size_t i = 0; i < size; ++i) {
                for (int j = 0; j < 2; ++j) {
                    // Upsample x2
                    if (j == 0) {
                        output = 0.f;
                    } else {
                        output = out[0][i] * drive + offset;
                    }

                    // Anti-aliasing filter
                    output = lp_anti_pre.process(output);

                    // Our clipping function
                    output = fast_tanh(output);
                    // float xdrive = 2.8; -- good range 2 to 3 ish
                    // out = fast_tanh(out * xdrive) / fast_tanh(xdrive);

                    // Anti-aliasing filter
                    output = lp_anti_post.process(output);
                }

                out[0][i] = out[1][i] = output;
            }
        }
    };

    // void reset();

private:
    float samplerate;
    float coeff = 0.01f;
    float drive = 0.f;

    bool useDistortion = false;

    jkoDSP::Biquad lp_anti_pre;
    jkoDSP::Biquad lp_anti_post;

    static inline float fast_tanh(float x)
    {
        if (x > 3.0f)
            return 1.0f;
        if (x < -3.0f)
            return -1.0f;
        float x2 = x * x;

        return x * (27.0f + x2) / (27.0f + 9.0f * x2);
    }
};

} // namespace jkoDSP

#endif // JKO_DSP_NONLINEAR_H