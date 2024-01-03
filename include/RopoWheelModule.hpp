#ifndef ROPO_WHEEL_MODULE_HPP
#define ROPO_WHEEL_MODULE_HPP

#include "RopoParameter.hpp"
#include "RopoMath/Misc.hpp"
#include "pros/motors.hpp"

namespace RopoWheelModule {
	class WheelModule {
	public:
		WheelModule(pros::Motor &mtr0, pros::Motor &mtr1)
		: motor0(mtr0), motor1(mtr1) { }
		virtual void MoveVelocity(float velocity)
		{
			// m/s -> rpm
			float omega = velocity * 60 * RopoParameter::CHASSIS_WHEEL_GEAR_RATIO / (2 * RopoMath::Pi * RopoParameter::CHASSIS_WHEEL_R);
			motor0.move_velocity(omega);
			motor1.move_velocity(omega);
		}
	protected:
		pros::Motor& motor0;
		pros::Motor& motor1;
	};
}

#endif // ROPO_WHEEL_MODULE_HPP