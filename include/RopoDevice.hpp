#ifndef ROPO_DEVICE_HPP
#define ROPO_DEVICE_HPP

#include "pros/motors.hpp"
#include "pros/misc.hpp"
#include "RopoParameter.hpp"
#include "RopoWheelModule.hpp"
#include "RopoChassis.hpp"
#include "RopoTurret.hpp"
#include "RopoController.hpp"
#include "RopoFlyWheel.hpp"
#include "RopoControl/Header.hpp"
#include "Roposensor/Header.hpp"

namespace RopoDevice {
	// Controller
	static pros::Controller masterController(pros::E_CONTROLLER_MASTER);
	static pros::Controller partnerController(pros::E_CONTROLLER_PARTNER);

	// Chassis Motors
	static pros::Motor leftFrontMotor0(	RopoParameter::LEFT_FRONT_MOTOR_PORT[0] ,
										RopoParameter::CHASSIS_MOTOR_GEARSET, true);
	static pros::Motor leftFrontMotor1(	RopoParameter::LEFT_FRONT_MOTOR_PORT[1] ,
										RopoParameter::CHASSIS_MOTOR_GEARSET, false);

	static pros::Motor leftBackMotor0(	RopoParameter::LEFT_BACK_MOTOR_PORT[0] ,
										RopoParameter::CHASSIS_MOTOR_GEARSET, true);
	static pros::Motor leftBackMotor1(	RopoParameter::LEFT_BACK_MOTOR_PORT[1] ,
										RopoParameter::CHASSIS_MOTOR_GEARSET, false);

	static pros::Motor rightBackMotor0(	RopoParameter::RIGHT_BACK_MOTOR_PORT[0] ,
										RopoParameter::CHASSIS_MOTOR_GEARSET, true);
	static pros::Motor rightBackMotor1(	RopoParameter::RIGHT_BACK_MOTOR_PORT[1] ,
										RopoParameter::CHASSIS_MOTOR_GEARSET, false);

	static pros::Motor rightFrontMotor0(RopoParameter::RIGHT_FRONT_MOTOR_PORT[0],
										RopoParameter::CHASSIS_MOTOR_GEARSET, true);
	static pros::Motor rightFrontMotor1(RopoParameter::RIGHT_FRONT_MOTOR_PORT[1],
										RopoParameter::CHASSIS_MOTOR_GEARSET, false);

	static pros::Motor directMotor0(RopoParameter::DIRECT_MOTOR_PORT[0],
										RopoParameter::DIRECT_MOTOR_GEARSET, true);
	static pros::Motor elevateMotor0(RopoParameter::ELEVATE_MOTOR_PORT[0],
										RopoParameter::ELEVATE_MOTOR_GEARSET, false);

	static pros::Motor flyWheelMotor0(RopoParameter::FLY_WHEEL_MOTOR_PORT[0],
										RopoParameter::FLY_WHEEL_MOTOR_GEARSET, false);

	static pros::IMU turretImu(RopoParameter::TURRET_IMU_PORT[0]);

	static RopoWheelModule::WheelModule leftFrontMotorModule(leftFrontMotor0, leftFrontMotor1);
	static RopoWheelModule::WheelModule leftBackMotorModule(leftBackMotor0, leftBackMotor1);
	static RopoWheelModule::WheelModule rightBackMotorModule(rightBackMotor0, rightBackMotor1);
	static RopoWheelModule::WheelModule rightFrontMotorModule(rightFrontMotor0, rightFrontMotor1);

	static RopoChassis::ChassisModule chassisModule(leftFrontMotorModule,
													leftBackMotorModule,
													rightBackMotorModule,
													rightFrontMotorModule);

	static RopoTurret::TurretModule turretModule(directMotor0,
												elevateMotor0,
												RopoParameter::TURRET_RANGE,
												turretImu,
												&RopoParameter::ElevateRegulator,
												&RopoParameter::DirectRegulator);

	static RopoFlyWheel::FlyWheel flyWheel(flyWheelMotor0, &RopoParameter::FlywheelRegulator);

	
	static pros::ADIDigitalOut ShootPneumatic(RopoParameter::ShootPneumaticPort,false);

	static RopoSensor::Debugger Debugger(RopoParameter::DEBUGGER_PORT[0],115200);

	void DeviceInit()
	{
		RopoDevice::directMotor0.set_brake_mode(RopoParameter::DIRECT_MOTOR_MODE);
		RopoDevice::elevateMotor0.set_brake_mode(RopoParameter::ELEVATE_MOTOR_MODE);
		RopoDevice::directMotor0.set_encoder_units(RopoParameter::DIRECT_MOTOR_ENCODER);
		RopoDevice::elevateMotor0.set_encoder_units(RopoParameter::ELEVATE_MOTOR_ENCODER);

		RopoDevice::turretImu.tare();
	}
}

#endif // ROPO_DEVICE_HPP