# pragma once
/*
The Sweep.hpp is used to creat a chirp signal.
Generally, it can be used for system identification and system performance testing.
Example usage:
    const int SampleTime = 10; // ms
    float Period = 10.0; // seconds
    float f0 = 0.1; // Hz
    float f1 = 10.0; // Hz
    float A = 6000.0;
    float start_time = pros::c::millis() / 1000.0;
	Sweep Sweeper(start_time, Period, f0, f1, A);

    while(True){
	    float time_now = pros::c::millis() / 1000.0;
        Sweep Sweeper(start_time, Period, f0, f1, A);
        flaot Input = Sweeper.GetValue(time_now);
		pros::delay(SampleTime);
    }

Input will be a wave signal from 0.1Hz to 10Hz, the amplitude of which is 6000.
And it will repeat after 10s.
*/

#include <iostream>
#include <cmath>

namespace RopoMath {

    class Sweep{
        // Sweep Signal from f0 Hz to f1 Hz with Amplitude of A
        private:
            // time unit: s
            // frequency unit: Hz
            float t_0; /* start time */
            float t_01; /* delta_time from t_0 to t_1 */
            float f0; /* frequency at t_0 */
            float f1; /* frequency at t_1 */
            float k; /* k for log */
            float p; /* parameter: p */
            float A; /* A for signal*/

        public:
            Sweep(float _t_0, float _t_01, float _f0, float _f1, float _A)
            :t_0(_t_0), t_01(_t_01), f0(_f0), f1(_f1), A(_A){
                if ((t_01 <= 0.0) || (f0 <= 0.0) || (f1 <= 0.0) || (f0 == f1) || (A == 0)){
                    /* Wrong parameters!*/
                    k = p = 0;
                    return;
                }
                else{
                    k = exp(log(f1 / f0) / t_01);
                    float pi = acos(-1);
                    p = 2 * pi * f0 / log(k);
                }
            }

            ~Sweep(){}

            float GetValue(float t_now){
                if (t_now < t_0)
                {
                    /* time is not statisfied*/
                    return 0;
                }

                float t = fmod(t_now - t_0, t_01); // delta time, Restart after t_01
                
                float y = A * sin(p * (pow(k, t) - 1));
                
                return y;
            }
    };
    
}