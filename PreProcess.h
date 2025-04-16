#ifndef PREPROCESS_H
#define PREPROCESS_H

#include "Biquad.h"
#include "daisysp.h"

using daisysp::fonepole;

namespace jkoDSP {

class PreProcessor {
public:
    PreProcessor() {};
    ~PreProcessor() {};

    void init(float hw_samplerate)
    {
        samplerate = hw_samplerate;

        hp_pre.setType(jkoDSP::bq_type_highpass);
        hp_pre.setFc(hp_freq / samplerate);
        hp_pre.setQ(hp_Q);

        lp_pre.setType(jkoDSP::bq_type_lowpass);
        lp_pre.setFc(lp_freq / samplerate);
        lp_pre.setQ(lp_Q);

        pre_peak.setType(jkoDSP::bq_type_peak);
        pre_peak.setFc(peak_freq / samplerate);
        pre_peak.setQ(peak_Q);
        pre_peak.setPeakGain(peak_gain_db);
    };

    void setBoost(float boost_level)
    {
        fonepole(boost, boost_level, coeff);
    }

    void setSemiParametric(float target_freq, float target_gain)
    {
        // TODO: make this the miniumum in the middle and maximum at the edges
        float target_Q = daisysp::fmap(target_gain, 0.5f, 1.5f, Mapping::LINEAR);
        target_gain = fmap(target_gain, -12.f, 12.f, Mapping::LINEAR);
        target_freq = fmap(target_freq, 160.f, 3000.f, Mapping::LOG);

        fonepole(peak_freq, target_freq, coeff);
        fonepole(peak_Q, target_Q, coeff);
        fonepole(peak_gain_db, target_gain, coeff);

        pre_peak.setFc(peak_freq / samplerate);
        pre_peak.setQ(peak_Q);
        pre_peak.setPeakGain(peak_gain_db);
    }

    void enableSemiParametric(bool enable)
    {
        use_semi_parametric = enable;
    }

    void enableBoost(bool enable)
    {
        use_boost = enable;
    }

    void enablePreEQ(bool enable)
    {
        use_fixed_eq = enable;
    }

    void enable(bool enable)
    {
        use_fixed_eq = enable;
        use_boost = enable;
        use_semi_parametric = enable;
    }

    void process(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out,
        size_t size) {

        for (size_t i = 0; i < size; ++i) {
            float output = in[0][i];

            // Pre EQ
            if (use_fixed_eq) {
                output = hp_pre.process(output);
                output = lp_pre.process(output);
            }
            // Apply the pre-peak filter
            if (use_semi_parametric) {
                output = pre_peak.process(output);
            }

            // Set the output
            if (use_boost) {
                out[0][i] = out[1][i] = output * boost;
            } else {
                out[0][i] = out[1][i] = output;
            }
        };
    };

    void reset();

private:
    float samplerate;

    bool use_fixed_eq = false;
    bool use_boost = false;
    bool use_semi_parametric = false;

    float boost = 1.f;

    float hp_freq = 40.f;
    float hp_Q = 0.707f;

    float lp_freq = 3600.f;
    float lp_Q = 0.707f;

    float peak_freq = 1000.f;
    float peak_Q = 1.f;
    float peak_gain_db = 0.f;

    Biquad hp_pre;
    Biquad lp_pre;
    Biquad pre_peak;

    float coeff = 0.01f;
};

} // namespace jkoDSP

#endif // PREPROCESS_H