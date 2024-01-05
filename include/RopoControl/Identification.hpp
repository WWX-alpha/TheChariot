// Code : UTF - 8
#ifndef IDENTIFICATION_HPP
#define IDENTIFICATION_HPP

#include "RopoMath/Header.hpp"

#include "pros/rtos.hpp"

namespace RopoControl
{
    class Identifier
    {
    public:
        Identifier():
            startTime(0),
            period(10* 1000),
            startFreq(0.5f),
            stopFreq(10.0f),
            maxAmp(5.0f),
            amp(exp(1000.0f * log(stopFreq / startFreq) / (period))),
            phase(2.0f * RopoMath::Pi * startFreq / log(amp))
        {}

        Identifier(uint32_t _deltaTime, float _startFreq, float _stopFreq, float _maxAmp):
            startTime(pros::millis()),
            period(_deltaTime),
            startFreq(_startFreq),
            stopFreq(_stopFreq),
            maxAmp(_maxAmp),
            amp(exp(1000.0f * log(stopFreq / startFreq) / (period))),
            phase(2.0f * RopoMath::Pi * startFreq / log(amp))
        {}

        ~Identifier()
        {}

        uint32_t startTime; 
        uint32_t period; 
        float startFreq;
        float stopFreq; 
        float amp;
        float phase;
        float maxAmp;
        float t;

        float sweepOutput()
        {
            uint32_t nowTime = pros::millis();
            
            t = (nowTime - startTime) % period * 0.001f;
            float output = maxAmp * sin(phase * (pow(amp, t) - 1));

            pros::lcd::print(6,"%.1f %.1f",t,output);
            
            return output;
        }

    private:
    };
}

#endif // IDENTIFICATION_HPP
