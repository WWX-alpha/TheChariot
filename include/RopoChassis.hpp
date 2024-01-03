#ifndef ROPO_CHASSIS_HPP
#define ROPO_CHASSIS_HPP

#include "RopoWheelModule.hpp"
#include "RopoParameter.hpp"
#include "RopoMath/Header.hpp"

namespace RopoChassis {
	class ChassisModule {
	public:
		ChassisModule(RopoWheelModule::WheelModule &lfWheelModule,
				RopoWheelModule::WheelModule &lbWheelModule,
				RopoWheelModule::WheelModule &rbWheelModule,
				RopoWheelModule::WheelModule &rfWheelModule)
		: leftFrontWheelModule(lfWheelModule), leftBackWheelModule(lbWheelModule),
		rightBackWheelModule(rbWheelModule), rightFrontWheelModule(rfWheelModule)
		{ }
		virtual void MoveVelocity(RopoMath::Vector<float> velocity)
		{
			static RopoMath::Matrix<float> parameterMatrix = {
				{(float)(-1.0/sin(RopoParameter::CHASSIS_WHEEL_ANGLE)), (float)(1.0/cos(RopoParameter::CHASSIS_WHEEL_ANGLE)), RopoParameter::CHASSIS_PARA_L},
				{(float)(-1.0/sin(RopoParameter::CHASSIS_WHEEL_ANGLE)), (float)(-1.0/cos(RopoParameter::CHASSIS_WHEEL_ANGLE)), RopoParameter::CHASSIS_PARA_L},
				{(float)(1.0/sin(RopoParameter::CHASSIS_WHEEL_ANGLE)), (float)(-1.0/cos(RopoParameter::CHASSIS_WHEEL_ANGLE)), RopoParameter::CHASSIS_PARA_L},
				{(float)(1.0/sin(RopoParameter::CHASSIS_WHEEL_ANGLE)), (float)(1.0/cos(RopoParameter::CHASSIS_WHEEL_ANGLE)), RopoParameter::CHASSIS_PARA_L}
			};
			RopoMath::Vector<float> motorVelocity = parameterMatrix * velocity;
			leftFrontWheelModule	.MoveVelocity(motorVelocity[1]);
			rightFrontWheelModule	.MoveVelocity(motorVelocity[4]);
			rightBackWheelModule	.MoveVelocity(motorVelocity[3]);
			leftBackWheelModule		.MoveVelocity(motorVelocity[2]);
		}
	private:
		RopoWheelModule::WheelModule &leftFrontWheelModule;
		RopoWheelModule::WheelModule &leftBackWheelModule;
		RopoWheelModule::WheelModule &rightBackWheelModule;
		RopoWheelModule::WheelModule &rightFrontWheelModule;
	};
}

#endif // ROPO_CHASSIS_HPP