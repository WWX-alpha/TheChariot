#ifndef ROPO_FLY_WHEEL_HPP
#define ROPO_FLY_WHEEL_HPP

#include "RopoParameter.hpp"
#include "RopoMath/Misc.hpp"
#include "pros/motors.hpp"
#include "RopoControl/Header.hpp"
#include "Roposensor/Header.hpp"
#include "okapi/api/filter/ekfFilter.hpp"

namespace RopoFlyWheel {
	class FlyWheel {
	public:
		FlyWheel(pros::Motor &mtr0,
				RopoControl::Regulator *_FlywheelRegulator = nullptr,
				float (*_GetFlywheelVelocity)(void) = nullptr): 
					FlywheelMotor(mtr0) ,
					FlywheelRegulator(_FlywheelRegulator),
					GetFlywheelVelocity(_GetFlywheelVelocity),
					flyWheelMode(RopoParameter::initFlyWheelMode),
					currentVelocity(0.0f),
					targetVelocity(RopoParameter::initFlyWheelSpeed),
					SumVoltage(0.0f)
		{ 
			BackgroundTask = new pros::Task(BackgroundTaskFunction,this);
		}

		float targetVelocity;
		float currentVelocity;
		float SumVoltage;
		bool flyWheelMode;

		// RopoControl::Identifier identifier = RopoControl::Identifier(10 * 1000, 0.5f, 10.0f, 12000.0f);

	protected:
		pros::Motor& FlywheelMotor;

		RopoControl::Regulator *FlywheelRegulator;
		pros::Task *BackgroundTask;
		float (*GetFlywheelVelocity)(void);
		
		okapi::EKFFilter vFilter = okapi::EKFFilter(0.001, 0.04);

		static void BackgroundTaskFunction(void *Parameter)
		{
			if(Parameter == nullptr)return;
			FlyWheel *This = static_cast<FlyWheel *>(Parameter);
			while(true)
			{
				This->vFilter.filter(This->FlywheelMotor.get_actual_velocity() * 6);
				This->currentVelocity = This->vFilter.getOutput();

				if(This->GetFlywheelVelocity != nullptr)
				{
					This->targetVelocity = This->GetFlywheelVelocity();
				}
				if(This->FlywheelRegulator != nullptr)
				{
					if(This->flyWheelMode)
					{
						This->SumVoltage = This->FlywheelRegulator->Update(This->targetVelocity - This->currentVelocity);
						// This->SumVoltage = This->identifier.sweepOutput();

						This->FlywheelMotor.move_voltage(This->SumVoltage);

						// std::cout << " " << This->targetVelocity << " " << currentVelocity << std::endl;
						// pros::lcd::print(2,"FW:%.1f %.1f %.0f %.1f",This->targetVelocity, This->currentVelocity, (float)This->flyWheelMode, This->SumVoltage);
					}
					else 
					{
						This->FlywheelMotor.move_voltage(This->SumVoltage = 0);
					}
				}
				else if(This->flyWheelMode)
				{
					This->FlywheelMotor.move_velocity(This->targetVelocity);
				}
				else
				{
					This->FlywheelMotor.move_velocity(0);
				}
				pros::delay(5);
			}
		}
	};
}

#endif // ROPO_FLY_WHEEL_HPP