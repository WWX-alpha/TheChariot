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
            PRegulator(float _Kp,float _OutputLimitHigh,float _OutputLimitLow,float _ErrorTol):
                Kp(_Kp),OutputLimitHigh(_OutputLimitHigh),OutputLimitLow(_OutputLimitLow),ErrorTol(_ErrorTol){}
            virtual float Update(float Error){
                if(Error < ErrorTol)Arrived = true;
                else Arrived = false;
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
            float ErrorTol;
            float JumpTime;
            bool First;
        public:
            PIDRegulator(float _Kp,float _Ki,float _Kd,float _OutputLimitHigh,float _OutputLimitLow,float _ErrorTol,float _JumpTime = 0.05):
                Kp(_Kp),Ki(_Ki),Kd(_Kd),OutputLimitHigh(_OutputLimitHigh),OutputLimitLow(_OutputLimitLow),ErrorTol(_ErrorTol),JumpTime(_JumpTime),First(true){}
            virtual float Update(float Error){
                static float PreError;
                static float IntError;
                static float DevError;
                static float Time;
                static float ArrivedTime;
                if(First){
                    PreError = Error;
                    IntError = 0;
                    Time = GetSystemTimeInSec();
                    First = false;
                    ArrivedTime = -1;
                }
                IntError += Error * ( GetSystemTimeInSec() - Time );
                if(IntError * Error < 0.0)
                    IntError = 0;
                Time = GetSystemTimeInSec();
                DevError = Error - PreError;
                PreError = Error;
                if( fabs(Error) < ErrorTol ){
                    if(ArrivedTime < 0.0) 
                        ArrivedTime = GetSystemTimeInSec();
                    else if(GetSystemTimeInSec() - ArrivedTime > JumpTime)
                        Arrived = true;
                }
                else ArrivedTime = -1 , Arrived = false;
                return Arrived?0:Limit( Kp * Error + Ki * IntError + Kd * DevError , OutputLimitHigh, OutputLimitLow) ;
            }
            virtual void Reset(){First = true,Arrived = false;}
    };
};

#endif // REGULATOR_HPP