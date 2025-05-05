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
        mDelayLine->SetDelay(MaxSize - 2);
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

    float Process(float inputSample, size_t delay, float diffusion)
    {
        // TODO: implement modulation
        return mDelayLine->Allpass(inputSample, delay - static_cast<size_t>(mExcursion), diffusion);
    }

    float Read(size_t position)
    {
        return mDelayLine->Read(position);
    }

private:
    float mExcursion = 0.0f;
    float mLastRead = 0.0f;

    daisysp::DelayLine<T, MaxSize> *mDelayLine;
};

#endif // DIFFUSOR_H