/** Simplified slew limiter algorithm inspired from Befaco modules in VCV Rack. */
#include <cmath>

class Slew {
public:
    void init(float samplerate)
    {
        sample_rate = samplerate;
        Ts = 1.0f / sample_rate;
    }

    void setRates(float rise, float fall)
    {
        rise_rate = rise;
        fall_rate = fall;
        slewRise = slewMax * Ts * std::pow(slewMin / slewMax, rise_rate);
        slewFall = slewMax * Ts * std::pow(slewMin / slewMax, fall_rate);
    }

    float Process(float in)
    {
        // Rise limiting
        if (in > out)
            out = fmin(in, out + slewRise);

        // Fall limiting
        else
            out = fmax(in, out - slewFall);

        return out;
    }

private:
    float slewMin = 0.1f;
    float slewMax = 10000.0f;

    // User parameters
    float rise_rate = 0.2f;
    float fall_rate = 0.2f;
    float sample_rate = 48000.0f;

    // Internal variables
    float Ts = 1.0f / sample_rate;
    float slewRise = slewMax * Ts * std::pow(slewMin / slewMax, rise_rate);
    float slewFall = slewMax * Ts * std::pow(slewMin / slewMax, fall_rate);

    // State variables
    float out = 0.f;
};