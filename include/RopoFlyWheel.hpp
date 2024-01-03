#ifndef ROPO_FLY_WHEEL_HPP
#define ROPO_FLY_WHEEL_HPP

#include "RopoParameter.hpp"
#include "RopoMath/Misc.hpp"
#include "pros/motors.hpp"

namespace RopoFlyWheel {
	class FlyWheel {
	public:
		FlyWheel(pros::Motor &mtr0)
		: motor0(mtr0) { }

		float targetVelocity;
		bool flyWheelMode;

		virtual void MoveVelocity(float inputVelocity, bool inputMode)
		{
			float omega = 0;
			targetVelocity = inputVelocity;
			flyWheelMode = inputMode;

			if(flyWheelMode)
			{
				omega = targetVelocity / 6;
			}
			motor0.move_velocity(omega);
			pros::lcd::print(2,"FW:%.1f %.1f %0.1f",targetVelocity, omega, (float)flyWheelMode);
		}

		RopoControl::Regulator *FlywheelRegulator;

		pros::Task *BackgroundTask;
		static void BackgroundTaskFunction(void *Parameter)
		{
			if(Parameter == nullptr)return;
			FlyWheel *This = static_cast<FlyWheel *>(Parameter);
		}

	protected:
		pros::Motor& motor0;
		
	};
}

#endif // ROPO_FLY_WHEEL_HPP