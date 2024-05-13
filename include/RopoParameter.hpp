#ifndef ROPO_PARAMETER_HPP
#define ROPO_PARAMETER_HPP

#include "pros/motors.hpp"
#include "RopoMath/Misc.hpp"
#include "RopoControl/Header.hpp"

namespace RopoParameter {
	// Chassis Motor Parameter
	static constexpr pros::motor_gearset_e CHASSIS_MOTOR_GEARSET = pros::E_MOTOR_GEAR_BLUE;
	static constexpr pros::motor_gearset_e DIRECT_MOTOR_GEARSET = pros::E_MOTOR_GEAR_BLUE;
	static constexpr pros::motor_gearset_e ELEVATE_MOTOR_GEARSET = pros::E_MOTOR_GEAR_GREEN;
	static constexpr pros::motor_gearset_e FLY_WHEEL_MOTOR_GEARSET = pros::E_MOTOR_GEAR_BLUE;

	static constexpr pros::motor_brake_mode_e_t ELEVATE_MOTOR_MODE = pros::E_MOTOR_BRAKE_BRAKE;
	static constexpr pros::motor_brake_mode_e_t DIRECT_MOTOR_MODE = pros::E_MOTOR_BRAKE_BRAKE;

	static constexpr pros::motor_encoder_units_e_t ELEVATE_MOTOR_ENCODER = pros::E_MOTOR_ENCODER_DEGREES;
	static constexpr pros::motor_encoder_units_e_t DIRECT_MOTOR_ENCODER = pros::E_MOTOR_ENCODER_DEGREES;

	static constexpr int LEFT_FRONT_MOTOR_PORT[] 	= {18, 17};
	static constexpr int LEFT_BACK_MOTOR_PORT[] 	= {10, 9};
	static constexpr int RIGHT_BACK_MOTOR_PORT[] 	= {2, 1};
	static constexpr int RIGHT_FRONT_MOTOR_PORT[] 	= {11, 12};
	static constexpr int DIRECT_MOTOR_PORT[] 		= {14};
	static constexpr int ELEVATE_MOTOR_PORT[] 		= {16};
	static constexpr int FLY_WHEEL_MOTOR_PORT[] 	= {7};
	static constexpr int TURRET_IMU_PORT[] 			= {5};
	static constexpr int DEBUGGER_PORT[] 			= {4};
	static constexpr int UPLOAD_PORT[] 				= {3};
	static constexpr int DOWNLOAD_PORT[] 			= {6};
	
	// Chassis Shape Parameter
	static constexpr float CHASSIS_WHEEL_ANGLE 		= (float)RopoMath::Pi / 3.0; 		// (rad)
	static constexpr float CHASSIS_PARA_L 			= 205.76f / 1000.0f; 				// (m)
	static constexpr float CHASSIS_WHEEL_R 			= 3.25f * 25.4f / 2.0f / 1000.0f; 	// (m)
	static constexpr float CHASSIS_WHEEL_GEAR_RATIO = 3.0f / 2.0f;
	static constexpr float DIRECT_MOTOR_RATIO 		= 55.0f / 4.0f;
	static constexpr float ELEVATE_MOTOR_RATIO 		= 24.0f / 4.0f;
	static constexpr float TURRET_RANGE[]			= {-180.0f, 180.0f, -10.0f, 45.0f};	//deg
	static constexpr float TURRET_INIT_ANGLE[]		= {0.0f, 0.0f};						//deg

	// Chassis Control Parameter
	static constexpr float CHASSIS_X_SCALE 			= 1.496f;							//(m/s)
	static constexpr float CHASSIS_Y_SCALE 			= 0.864f;							//(m/s)
	static constexpr float CHASSIS_W_SCALE 			= 8.429f * 0.5;

	static constexpr float DIRECT_SCALE 			= 150.0f;							//(deg/s)
	static constexpr float ELEVATE_SCALE 			= 60.0f;							//(deg/s)

	static constexpr float DeltaFlywheelVelocity 	= 200.0f;
	static constexpr float initFlyWheelSpeed 		= 1800.0f;
	static constexpr float maxFlyWheelSpeed 		= 3600.0f;
	static constexpr float minFlyWheelSpeed 		= 0.0f;

	static constexpr bool initFlyWheelMode 			= false;

	// Three Wire
	static constexpr char ShootPneumaticPort  = 'A';

	RopoControl::antiWindblowPIRegulator FlywheelRegulator(0.25f, 3.0f, 8.0f, 12000,-12000); //0.25 5 5

	RopoControl::PIDRegulator ElevateRegulator(200.0f, 75.0f, 0.0f, 12000,-12000);
	
	RopoControl::PIDRegulator DirectRegulator(10.0f, 7.0f, 10.0f, 12000,-12000);
};

#endif // ROPO_PARAMETER_HPP