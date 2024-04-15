#ifndef ROPO_TURRET_HPP
#define ROPO_TURRET_HPP

#include "RopoParameter.hpp"
#include "RopoMath/Misc.hpp"
#include "pros/motors.hpp"

namespace RopoTurret {
	class TurretModule {
	public:
		float directPos = 0;
		float elevatePos = 0;

		float mode = 0.0f;

		bool elevateStableFlag = false;
		bool directStableFlag = false;

		float elevateVoltage;
		float directVoltage;

		float currentElevateAngle;
		float targetElevateAngle;

		float currentDirectAngle;
		float targetDirectAngle;

		bool yoloFindFlag = false;
		int modeFlag = 1; //0reset 1ready 2shoot

		pros::Motor& directMotor;
		pros::Motor& elevateMotor;

		RopoControl::Identifier identifier = RopoControl::Identifier(10 * 1000, 0.5f, 10.0f, 6000.0f);

		// TurretModule(pros::Motor &mtr0, pros::Motor &mtr1, const float* Range): 
		// directMotor(mtr0), elevateMotor(mtr1), turretRange{Range[0], Range[1], Range[2], Range[3]}{ }

		TurretModule(pros::Motor &mtr0, 
					pros::Motor &mtr1, 
					const float* Range, 
					pros::Imu &imu0, 
					RopoControl::Regulator *_elevateRegulator = nullptr,
					RopoControl::Regulator *_directRegulator = nullptr): 
			directMotor	(mtr0),
			elevateMotor(mtr1),
			turretRange{Range[0], Range[1], Range[2], Range[3]},
			turretImu	(imu0),
			currentElevateAngle(0.0f),
			targetElevateAngle	(0.0f),
			currentDirectAngle	(0.0f),
			targetDirectAngle	(0.0f),
			elevateVoltage		(0.0f),
			directVoltage		(0.0f),
			elevateRegulator(_elevateRegulator), 
			directRegulator	(_directRegulator)
		{
			BackgroundTask = new pros::Task(BackgroundTaskFunction,this); 
		}

		virtual void MoveVelocity(RopoMath::Vector<float> velocity){
			float directOmega = 0.0f;
			float elevateOmega = 0.0f;
			if(elevatePos <= turretRange[2]){
				elevateOmega = 50.0f;
				mode = 1.0f;
			}
			else if(elevatePos >= turretRange[3]){
				elevateOmega = -50.0f;
				mode = 2.0f;
			}
			else{
				// deg/s to rpm
				directOmega    = ((velocity[1] < 0 && directPos >= turretRange[0]) || (velocity[1] > 0 && directPos <= turretRange[1]))?(velocity[1] * RopoParameter::DIRECT_MOTOR_RATIO / (6.0f)):(0);
            	elevateOmega   = ((velocity[2] < 0 && elevatePos >= turretRange[2]) || (velocity[2] > 0 && elevatePos <= turretRange[3]))?(velocity[2] * RopoParameter::ELEVATE_MOTOR_RATIO / (6.0f)):(0);
				mode = 3.0f;
			}
			
			directMotor .move_velocity(directOmega);
			elevateMotor.move_velocity(elevateOmega);
		}

		virtual void MoveVoltage(RopoMath::Vector<float> voltage){
			directVoltage = ((voltage[1] < 0 && directPos >= turretRange[0]) || (voltage[1] > 0 && directPos <= turretRange[1]))?(voltage[1]):(0);
			elevateVoltage = ((voltage[2] < 0 && elevatePos >= turretRange[2]) || (voltage[2] > 0 && elevatePos <= turretRange[3]))?(voltage[2]):(0);

			if(elevatePos <= turretRange[2]){
				elevateVoltage = 3000.0f;
				mode = 1.0f;
			}
			else if(elevatePos >= turretRange[3]){
				elevateVoltage = -3000.0f;
				mode = 2.0f;
			}
			directMotor .move_voltage(directVoltage);
			// elevateMotor.move_voltage(elevateVoltage);
		}

		virtual void Update(){
			directPos  = directMotor.get_position() / RopoParameter::DIRECT_MOTOR_RATIO + RopoParameter::TURRET_INIT_ANGLE[0];
            elevatePos = elevateMotor.get_position() / RopoParameter::ELEVATE_MOTOR_RATIO + RopoParameter::TURRET_INIT_ANGLE[1];

			// if(abs(directPos) <= 30)
			// {
			// 	turretRange[3] = 40.0f;
			// }
			// else
			// {
			// 	turretRange[3] = 30.0f;
			// }

			// if(directPos >= -160.0f && directPos <= -10.0f){
			// 	turretRange[2] = -10.0f;
			// }
			// else{
			// 	turretRange[2] = -25.0f;
			// }

			currentElevateAngle = turretImu.get_pitch();
			currentDirectAngle = turretImu.get_yaw();
		}

	protected:
		pros::Imu& turretImu;

		std::vector<float> turretRange;

		RopoControl::Regulator *elevateRegulator;
		RopoControl::Regulator *directRegulator;
		pros::Task *BackgroundTask;

		static void BackgroundTaskFunction(void *Parameter){
			if(Parameter == nullptr)return;
			TurretModule *This = static_cast<TurretModule *>(Parameter);

			RopoMath::Vector<float> voltage(RopoMath::ColumnVector, 2);
			voltage[1] = 0.0f;
			voltage[2] = 0.0f;
			while(true){
				This->Update();

				if(This->modeFlag == 0)
				{
					// This->elevateStableFlag = false;
					// This->directStableFlag = false;
					This->elevateRegulator->Reset();
					This->directRegulator->Reset();
				}

				if(This->elevateRegulator != nullptr) {
					if(This->elevateStableFlag) {
						if(This->yoloFindFlag) {
							This->elevateVoltage = This->elevateRegulator->Update(This->targetElevateAngle);
							voltage[2] = This->elevateVoltage;
						}
						else {
							voltage[2] = 0;
						}
					}
				}

				if(This->directRegulator != nullptr){
					if(This->directStableFlag){
						if(This->yoloFindFlag){
							This->directVoltage = This->directRegulator->Update(This->targetDirectAngle);
							voltage[1] = -This->directVoltage;
						}
						else{
							voltage[1] = 0;
						}
					}
				}
				
				if((This->directStableFlag || This->elevateStableFlag) && This->modeFlag == 1){
					This->MoveVoltage(voltage);

					float elevateOmega = 0.0;
					if(This->elevatePos <= This->turretRange[2]){
						elevateOmega = 50.0f;
					}
					else if(This->elevatePos >= This->turretRange[3]){
						elevateOmega = -50.0f;
					}
					else{
						elevateOmega   = ((This->targetElevateAngle < 0 && This->elevatePos >= This->turretRange[2]) 
												|| (This->targetElevateAngle > 0 && This->elevatePos <= This->turretRange[3]))
												?(This->targetElevateAngle *RopoParameter::ELEVATE_SCALE * RopoParameter::ELEVATE_MOTOR_RATIO / (6.0f)):(0);
					}
					This->elevateMotor.move_velocity(elevateOmega);
				}
				else if(This->modeFlag == 0){
					This->directMotor.move_absolute(0,50);
					This->elevateMotor.move_absolute(0,50);
				}
				else{
					voltage[1] = 0;
					voltage[2] = 0;
					This->MoveVoltage(voltage);

					This->elevateMotor.move_velocity(0);
				}
				pros::delay(5);
			}
		}
	};
}

#endif // ROPO_TURRET_HPP