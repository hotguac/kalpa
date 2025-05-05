#ifndef REVERB_H
#define REVERB_H

#include "daisysp.h"
#include "diffusor.h"

static constexpr int sMaxExcursion = 60;

static constexpr int sPreDelayLength = 4800;
static constexpr int sDiffuserDelay1 = 461;
static constexpr int sDiffuserDelay2 = 347;
static constexpr int sDiffuserDelay3 = 1223;
static constexpr int sDiffuserDelay4 = 907;
static constexpr int sDiffuserDelay5 = 2237; // +/- Excursion
static constexpr int sDiffuserDelay6 = 5807;
static constexpr int sDiffuserDelay7 = 2999; // +/- Excursion
static constexpr int sDiffuserDelay8 = 8573;

static constexpr int sTankDelay5 = 14369;
static constexpr int sTankDelay6 = 12007;
static constexpr int sTankDelay7 = 13613;
static constexpr int sTankDelay8 = 10211;

// Stored in SDRAM because not enough room in base RAM
daisysp::DelayLine<float, sDiffuserDelay1 + 2> DSY_SDRAM_BSS lineDiff1;
daisysp::DelayLine<float, sDiffuserDelay2 + 2> DSY_SDRAM_BSS lineDiff2;
daisysp::DelayLine<float, sDiffuserDelay3 + 2> DSY_SDRAM_BSS lineDiff3;
daisysp::DelayLine<float, sDiffuserDelay4 + 2> DSY_SDRAM_BSS lineDiff4;
daisysp::DelayLine<float, sDiffuserDelay5 + 2> DSY_SDRAM_BSS lineDiff5;
daisysp::DelayLine<float, sDiffuserDelay6 + 2> DSY_SDRAM_BSS lineDiff6;
daisysp::DelayLine<float, sDiffuserDelay7 + 2> DSY_SDRAM_BSS lineDiff7;
daisysp::DelayLine<float, sDiffuserDelay8 + 2> DSY_SDRAM_BSS lineDiff8;

class Damper {
public:
    Damper() {};
    ~Damper() {};

    float Process(float inputSample, float damping)
    {
        float outputSample = inputSample + (mPrevious * damping);
        mPrevious = outputSample;
        return outputSample;
    }

private:
    float mPrevious = 0.0f;
};

class Reverb {
public:
    Reverb() {};
    ~Reverb() {};

    void Init()
    {
        // Control parameters all have a range of 0 to 1
        mDecay = 0.15f;

        mPreDelay = 0.0f;
        mBandwidth = 0.5f;
        mExcursion = 0.5f;
        
        mInputDiffusion1 = 0.650f;
        mInputDiffusion2 = 0.500f;

        mDecayDiffusion1 = 0.600f;
        mDecayDiffusion2 = 0.625f;
        
        mBandwidth = 0.89995f;
        mDamping = 0.04f;

        preDelay.Init();
        preDelay.SetDelay(mPreDelay);

        diff1.Init(&lineDiff1);
        diff2.Init(&lineDiff2);
        diff3.Init(&lineDiff3);
        diff4.Init(&lineDiff4);
        diff5.Init(&lineDiff5);
        diff6.Init(&lineDiff6);
        diff7.Init(&lineDiff7);
        diff8.Init(&lineDiff8);

        tank5.Init();
        tank5.SetDelay(static_cast<float>(sTankDelay5 - 2));

        tank6.Init();
        tank6.SetDelay(static_cast<float>(sTankDelay6 - 2));

        tank7.Init();
        tank7.SetDelay(static_cast<float>(sTankDelay7 - 2));

        tank8.Init();
        tank8.SetDelay(static_cast<float>(sTankDelay8 - 2));
    }

   // Process a single sample
    float Process(float inputSample)
    {
        // TODO: fix this auto generated code
        float outputSample = ProcessEarly(inputSample);

        // Send to tank loops 5,6,7,8
        float leftTank = outputSample + mLastTank8 * mDecay;
        float rightTank = outputSample + mLastTank6 * mDecay;

        mLastTank6 = ProcessLeftTank(leftTank);
        mLastTank8 = ProcessRightTank(rightTank);

        // return monophonic output
        return (GetOutputL() + GetOutputR()) * 0.5f;
    }

    // Setters for reverb parameters
    void setPreDelay(float preDelay)
    {
        mPreDelay = daisysp::fmap(preDelay, 0.0f,
            static_cast<float>(sPreDelayLength - 1.0f));
    }

    void setBandwidth(float bandwidth)
    {
        mBandwidth = daisysp::fmap(bandwidth, 0.25f, 0.999999f);
    };

    void setExcursion(float excursion)
    {
        mExcursion = daisysp::fmap(excursion, 0.0f, static_cast<float>(sMaxExcursion));
    };

    void setDamping(float damping)
    {
        mDamping = daisysp::fmap(damping, 0.0f, 0.01f);
    };

    void setDecay(float decay)
    {
        mDecay = daisysp::fmap(decay, 0.1f, 0.9f);

        mDecayDiffusion1 = daisysp::fmap(decay, 0.25f, 0.90f, daisysp::Mapping::EXP);
        mDecayDiffusion2 = daisysp::fmap(decay, 0.25f, 0.50f, daisysp::Mapping::EXP);
    };

private:
    daisysp::DelayLine<float, sPreDelayLength> preDelay;

    // Input diffusion delay lines
    Diffusor<float, sDiffuserDelay1 + 2> diff1;
    Diffusor<float, sDiffuserDelay2 + 2> diff2;
    Diffusor<float, sDiffuserDelay3 + 2> diff3;
    Diffusor<float, sDiffuserDelay4 + 2> diff4;

    // Tank diffusion delay lines 5,6 left, 7,8 right
    Diffusor<float, sDiffuserDelay5 + 2, sMaxExcursion, true> diff5;
    daisysp::DelayLine<float, sTankDelay5> tank5;

    Diffusor<float, sDiffuserDelay6 + 2, 0, false> diff6;
    daisysp::DelayLine<float, sTankDelay6> tank6;

    Diffusor<float, sDiffuserDelay7 + 2, sMaxExcursion, true> diff7;
    daisysp::DelayLine<float, sTankDelay7> tank7;

    Diffusor<float, sDiffuserDelay8 + 2, 0, false> diff8;
    daisysp::DelayLine<float, sTankDelay8> tank8;

    void updateCoefficients();

    float mDecay;
    float mPreDelay;
    float mBandwidth;
    float mExcursion;
    float mDamping;

    float mInputDiffusion1;
    float mInputDiffusion2;

    float mDecayDiffusion1;
    float mDecayDiffusion2;

    float mLastAccum1 = 0.0f;
    float mLastAccum2 = 0.0f;
    float mLastAccum3 = 0.0f;

    float mLastTank6 = 0.0f;
    float mLastTank8 = 0.0f;

    // Damper preDelayDamper;
    Damper leftTankDamper;
    Damper rightTankDamper;

    float ProcessEarly(float inputSample)
    {
        // Apply pre-delay
        preDelay.Write(inputSample);
        float outputSample = preDelay.Read();

        // Apply bandwidth control
        outputSample *= mBandwidth;
        outputSample += mLastAccum1 * (1.0f - mBandwidth);
        mLastAccum1 = outputSample;

        // Apply input diffusion
        outputSample = diff1.Process(outputSample, sDiffuserDelay1, mInputDiffusion1);
        outputSample = diff2.Process(outputSample, sDiffuserDelay2, mInputDiffusion1);
        outputSample = diff3.Process(outputSample, sDiffuserDelay3, mInputDiffusion2);
        outputSample = diff4.Process(outputSample, sDiffuserDelay4, mInputDiffusion2);

        return outputSample;
    }

    float ProcessLeftTank(float inputSample)
    {
        // Process the left tank (5,6)
        float outputSample = diff5.Process(inputSample, sDiffuserDelay5, -mDecayDiffusion1);
        tank5.Write(outputSample);

        outputSample = tank5.Read() * (1 - mDamping);
        outputSample = leftTankDamper.Process(outputSample, mDamping);

        outputSample *= mDecay;
        outputSample = diff6.Process(outputSample, sDiffuserDelay6, mDecayDiffusion1);
        tank6.Write(outputSample);

        return tank6.Read();
    }

    float ProcessRightTank(float inputSample)
    {
        // Process the right tank (7,8)
        float outputSample = diff7.Process(inputSample, sDiffuserDelay7, -mDecayDiffusion2);
        tank7.Write(outputSample);

        outputSample = tank7.Read() * (1 - mDamping);
        outputSample = rightTankDamper.Process(outputSample, mDamping);

        outputSample *= mDecay;
        outputSample = diff8.Process(outputSample, sDiffuserDelay8, mDecayDiffusion2);
        tank8.Write(outputSample);

        return tank8.Read();
    }

    float GetOutputL()
    {
        float leftOutput = tank7.Read(858) * 0.6f;
        leftOutput += tank7.Read(9593) * 0.6f;
        leftOutput -= diff8.Read(6171) * 0.6f;
        leftOutput += tank8.Read(6438) * 0.6f;
        leftOutput -= tank5.Read(6419) * 0.6f;
        leftOutput -= diff6.Read(603) * 0.6f;
        leftOutput -= tank6.Read(3439) * 0.6f;

        return leftOutput;
    }

    float GetOutputR()
    {
        float RightOutput = tank5.Read(1139) * 0.6f;
        RightOutput += tank5.Read(11700) * 0.6f;
        RightOutput -= diff6.Read(3961) * 0.6f;
        RightOutput += tank6.Read(8622) * 0.6f;
        RightOutput -= tank7.Read(6809) * 0.6f;
        RightOutput -= diff8.Read(1081) * 0.6f;
        RightOutput -= tank8.Read(390) * 0.6f;

        return RightOutput;
    }

 
};

#endif // REVERB_H