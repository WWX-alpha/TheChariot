#include "main.h"
#include "RopoDevice.hpp"
#include "RopoController.hpp"

void initialize() {
	pros::lcd::initialize();
	RopoDevice::DeviceInit();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

float FlyWheelSpeed = RopoParameter::initFlyWheelSpeed;
bool flyWheelMode = RopoParameter::initFlyWheelMode;

void FlywheelValueUp()
{
	FlyWheelSpeed += RopoParameter::DeltaFlywheelVelocity;
	if(FlyWheelSpeed >= RopoParameter::maxFlyWheelSpeed)
	{
		FlyWheelSpeed = RopoParameter::maxFlyWheelSpeed;
	}
	RopoDevice::flyWheel.MoveVelocity(FlyWheelSpeed, flyWheelMode);
}

void FlywheelValueDown()
{
	FlyWheelSpeed -= RopoParameter::DeltaFlywheelVelocity;
	if(FlyWheelSpeed <= RopoParameter::minFlyWheelSpeed)
	{
		FlyWheelSpeed = RopoParameter::minFlyWheelSpeed;
	}
	RopoDevice::flyWheel.MoveVelocity(FlyWheelSpeed, flyWheelMode);
}

void FlywheelShoot()
{
	RopoDevice::ShootPneumatic.set_value(true);
	pros::delay(50);
	RopoDevice::ShootPneumatic.set_value(false);
}

void FlywheelSwitch()
{
	flyWheelMode ^= 1;
	RopoDevice::flyWheel.MoveVelocity(FlyWheelSpeed, flyWheelMode);
}

void opcontrol() {
	RopoController::AxisValueCast xVelocityInput(RopoDevice::masterController, pros::E_CONTROLLER_ANALOG_LEFT_Y , RopoController::Exp);
	RopoController::AxisValueCast yVelocityInput(RopoDevice::masterController, pros::E_CONTROLLER_ANALOG_LEFT_X , RopoController::Exp);
	RopoController::AxisValueCast wVelocityInput(RopoDevice::masterController, pros::E_CONTROLLER_ANALOG_RIGHT_X, RopoController::Exp);

	RopoController::AxisValueCast directionInput(RopoDevice::partnerController, pros::E_CONTROLLER_ANALOG_LEFT_X , RopoController::Exp);
	RopoController::AxisValueCast elevationInput(RopoDevice::partnerController, pros::E_CONTROLLER_ANALOG_LEFT_Y , RopoController::Exp);

	float xInput, yInput, wInput, dInput, eInput;

	RopoMath::Vector<float> inputVelocity(RopoMath::ColumnVector, 3);
	RopoMath::Vector<float> turretVelocity(RopoMath::ColumnVector, 2);

	RopoController::ButtonTaskLine ButtonDetectLine1(RopoDevice::masterController);
	RopoDevice::masterController.clear();
	ButtonDetectLine1.Enable();

	RopoController::ButtonTaskLine ButtonDetectLine2(RopoDevice::partnerController);
	RopoDevice::partnerController.clear();
	ButtonDetectLine2.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_UP , RopoController::Rising, FlywheelValueUp);
	ButtonDetectLine2.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_DOWN , RopoController::Rising, FlywheelValueDown);
	ButtonDetectLine2.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_R1 , RopoController::Rising, FlywheelShoot);
	ButtonDetectLine2.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_X , RopoController::DoubleClick, FlywheelSwitch);
	ButtonDetectLine2.Enable();

	while (true) {
		xInput =   xVelocityInput.GetAxisValue();
		yInput = - yVelocityInput.GetAxisValue();
		wInput = - wVelocityInput.GetAxisValue();

		dInput = - directionInput.GetAxisValue();
		eInput =   elevationInput.GetAxisValue();

		inputVelocity[1] = (fabs(xInput) < 0.08) ? (0) : (xInput  * RopoParameter::CHASSIS_X_SCALE);
		inputVelocity[2] = (fabs(yInput) < 0.08) ? (0) : (yInput  * RopoParameter::CHASSIS_Y_SCALE);
		inputVelocity[3] = (fabs(wInput) < 0.08) ? (0) : (wInput  * RopoParameter::CHASSIS_W_SCALE);

		turretVelocity[1] = (fabs(dInput) < 0.08) ? (0) : (dInput  * RopoParameter::DIRECT_SCALE);
		turretVelocity[2] = (fabs(eInput) < 0.08) ? (0) : (eInput  * RopoParameter::ELEVATE_SCALE);

		RopoDevice::chassisModule.MoveVelocity(inputVelocity);
		RopoDevice::turretModule.Update();
		RopoDevice::turretModule.MoveVelocity(turretVelocity);

		pros::lcd::print(1,"%.1f %.1f %.1f %.1f %.1f",xInput,yInput,wInput,dInput,eInput);
		
		pros::delay(10);
	}
}
