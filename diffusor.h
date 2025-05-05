#ifndef DIFFUSOR_H
#define DIFFUSOR_H

#include "daisysp.h"

template <typename T, size_t MaxSize, size_t MaxExcursion = 0, bool invert = false>
class Diffusor {
public:
    Diffusor() {};
    ~Diffusor() {};

    void Init(daisysp::DelayLine<T, MaxSize> *delayLine)
    {
        mDelayLine = delayLine;
        mDelayLine->Init();
        mDelayLine->SetDelay(MaxSize);
    }

    void setExcursion(float excursion)
    {
        //TODO: implement modulation using https://www.dsprelated.com/showarticle/1467.php
        if (MaxExcursion == 0) {
            mExcursion = 0.0f;
        } else {
            mExcursion = daisysp::fmap(excursion, 0.0f, static_cast<float>(MaxExcursion));
        }
    }

    void setDiffusion(float diffusion)
    {
        mDiffusion = diffusion;
    }

    float Process(float inputSample)
    {
        // TODO: implement modulation
        float accum1 = inputSample;

        if (invert) {
            accum1 += mLastRead * mDiffusion;
        } else {
            accum1 -= mLastRead * mDiffusion;
        }

        mDelayLine->Write(accum1);
        mLastRead = mDelayLine->Read();

        float accum2 = accum1;

        if (invert) {
            accum2 -= mLastRead * mDiffusion;
        } else {
            accum2 += mLastRead * mDiffusion;
        }

        return accum2;
    }

    float Read(size_t position)
    {
        return mDelayLine->Read(position);
    }

private:
    float mExcursion = 0.0f;
    float mDiffusion = 0.0f;

    float mLastRead = 0.0f;

    daisysp::DelayLine<T, MaxSize> *mDelayLine;
};

#endif // DIFFUSOR_H