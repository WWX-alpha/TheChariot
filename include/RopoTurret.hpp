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

		TurretModule(pros::Motor &mtr0, pros::Motor &mtr1, const float* Range)
		: directMotor(mtr0), elevateMotor(mtr1), turretRange{Range[0], Range[1], Range[2], Range[3]} { }

		virtual void MoveVelocity(RopoMath::Vector<float> velocity)
		{
            
			float directOmega = 0.0f;
			float elevateOmega = 0.0f;
			if(elevatePos <= turretRange[2])
			{
				elevateOmega = 50.0f;
				mode = 1.0f;
			}
			else if(elevatePos >= turretRange[3])
			{
				elevateOmega = -50.0f;
				mode = 2.0f;
			}
			else
			{
				// deg/s to rpm
				directOmega    = ((velocity[1] < 0 && directPos >= turretRange[0]) || (velocity[1] > 0 && directPos <= turretRange[1]))?(velocity[1] * RopoParameter::DIRECT_MOTOR_RATIO / (6.0f)):(0);
            	elevateOmega   = ((velocity[2] < 0 && elevatePos >= turretRange[2]) || (velocity[2] > 0 && elevatePos <= turretRange[3]))?(velocity[2] * RopoParameter::ELEVATE_MOTOR_RATIO / (6.0f)):(0);
				mode = 3.0f;
			}
			
			directMotor .move_velocity(directOmega);
			elevateMotor.move_velocity(elevateOmega);

            pros::lcd::print(3,"TR:%.1f %.1f",directOmega, elevateOmega);
			pros::lcd::print(4,"%.1f %.1f",directPos, elevatePos);
			pros::lcd::print(5,"%.1f",mode);
		}

		virtual void Update()
		{
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

			if(directPos >= -160.0f && directPos <= -10.0f)
			{
				turretRange[2] = -10.0f;
			}
			else
			{
				turretRange[2] = -25.0f;
			}
		}

	private:
		pros::Motor& directMotor;
		pros::Motor& elevateMotor;
		std::vector<float> turretRange;
	};
}

#endif // ROPO_TURRET_HPP