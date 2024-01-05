// Code : UTF - 8
#ifndef REGULATOR_HPP
#define REGULATOR_HPP

#include "pros/rtos.hpp"

#ifdef _PROS_RTOS_HPP_
    float GetSystemTimeInSec(){return pros::millis() / 1000.0;}
#else
    float GetSystemTimeInSec(){return 0;}
#endif

namespace RopoControl{
    template<class T>inline T Limit(T Input, T HighLimit, T LowLimit)
    {
        return (Input > HighLimit)?(HighLimit):( (Input < LowLimit)?(LowLimit):(Input) );
    }

    class Regulator
    {
        protected:
            bool Arrived;
        public:
            Regulator(){}
            ~Regulator(){}
            virtual float Update(float Error) = 0;
            virtual void Reset() = 0;
            bool IfArrived(){return Arrived;};
    };

    class PRegulator : public Regulator
    {
        protected:
            float Kp;
            float OutputLimitHigh;
            float OutputLimitLow;
            float ErrorTol;
        public:
            PRegulator(float _Kp,float _OutputLimitHigh,float _OutputLimitLow):
                Kp(_Kp),OutputLimitHigh(_OutputLimitHigh),OutputLimitLow(_OutputLimitLow){}
            
            virtual float Update(float Error)
            {
                return Limit( Error * Kp, OutputLimitHigh, OutputLimitLow );
            }
            virtual void Reset(){Arrived = false;}
    };

    class PIDRegulator : public Regulator
    {
        protected:
            float Kp,Ki,Kd;
            float OutputLimitHigh;
            float OutputLimitLow;
            bool First;
        public:
            PIDRegulator(float _Kp,float _Ki,float _Kd,float _OutputLimitHigh,float _OutputLimitLow):
                Kp(_Kp),Ki(_Ki),Kd(_Kd),OutputLimitHigh(_OutputLimitHigh),OutputLimitLow(_OutputLimitLow),First(true){}
            virtual float Update(float Error){
                static float PreError;
                static float IntError;
                static float DevError;
                static float Time;
                if(First){
                    PreError = Error;
                    IntError = 0;
                    Time = GetSystemTimeInSec();
                    First = false;
                }
                IntError += Error * (GetSystemTimeInSec() - Time);
                DevError = (Error - PreError) / (GetSystemTimeInSec() - Time);

                PreError = Error;

                Time = GetSystemTimeInSec();

                return Limit((Kp * Error + Ki * IntError + Kd * DevError) , OutputLimitHigh, OutputLimitLow);
            }
            virtual void Reset(){First = true,Arrived = false;}
    };

    class antiWindblowPIRegulator : public Regulator
    {
        protected:
            float Kp,Ki,Kc;
            float OutputLimitHigh;
            float OutputLimitLow;
            bool First;
        public:
            antiWindblowPIRegulator(float _Kp,float _Ki,float _Kc,float _OutputLimitHigh,float _OutputLimitLow):
                Kp(_Kp),
                Ki(_Ki),
                Kc(_Kc),
                OutputLimitHigh(_OutputLimitHigh),
                OutputLimitLow(_OutputLimitLow),
                First(true){}
            virtual float Update(float Error){
                static float Sum;
                static float Output;
                static float Exc;
                static float Time;
                if(First){
                    Sum = 0;
                    First = false;
                    Output = 0;

                    Time = GetSystemTimeInSec();
                }
                float U = Sum + Kp * Error;

                Output = Limit(U, OutputLimitHigh, OutputLimitLow);

                Exc = U - Output;
                Sum = Sum + (Ki * Error - Kc * Exc )* (GetSystemTimeInSec() - Time);

                Time = GetSystemTimeInSec();
                return Output;
            }
            virtual void Reset(){First = true,Arrived = false;}
    };
};

#endif // REGULATOR_HPP