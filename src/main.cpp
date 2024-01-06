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
	RopoDevice::flyWheel.targetVelocity += RopoParameter::DeltaFlywheelVelocity;
	if(RopoDevice::flyWheel.targetVelocity >= RopoParameter::maxFlyWheelSpeed)
	{
		RopoDevice::flyWheel.targetVelocity = RopoParameter::maxFlyWheelSpeed;
	}

}

void FlywheelValueDown()
{
	RopoDevice::flyWheel.targetVelocity -= RopoParameter::DeltaFlywheelVelocity;
	if(RopoDevice::flyWheel.targetVelocity <= RopoParameter::minFlyWheelSpeed)
	{
		RopoDevice::flyWheel.targetVelocity = RopoParameter::minFlyWheelSpeed;
	}
}

void FlywheelShoot()
{
	RopoDevice::ShootPneumatic.set_value(true);
	pros::delay(50);
	RopoDevice::ShootPneumatic.set_value(false);
}

void FlywheelSwitch()
{
	RopoDevice::flyWheel.flyWheelMode ^= 1;
}

void ElevateStableSwitch()
{
	RopoDevice::turretModule.elevateStableFlag ^= 1;
}

void DirectStableSwitch()
{
	RopoDevice::turretModule.directStableFlag ^= 1;
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
	ButtonDetectLine2.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_A , RopoController::DoubleClick, ElevateStableSwitch);
	ButtonDetectLine2.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_B , RopoController::DoubleClick, DirectStableSwitch);
	ButtonDetectLine2.Enable();

	okapi::EKFFilter vFilter(0.001, 0.04);

	while (true)
	{
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

		if(turretVelocity[1] !=0 || turretVelocity[2] !=0)
		{
			RopoDevice::turretModule.elevateStableFlag = false;
			RopoDevice::turretModule.directStableFlag = false;
		}
		// RopoDevice::turretModule.MoveVelocity(turretVelocity);
		RopoDevice::turretModule.MoveVoltage(turretVelocity);

		pros::lcd::print(1,"%.1f %.1f %.1f %.1f %.1f",xInput,yInput,wInput,dInput,eInput);
		pros::lcd::print(2,"FW:%.1f %.1f %.0f %.1f",RopoDevice::flyWheel.targetVelocity, RopoDevice::flyWheel.currentVelocity, (float)RopoDevice::flyWheel.flyWheelMode, RopoDevice::flyWheel.SumVoltage);
		pros::lcd::print(3,"TR:%.1f %.1f %.1f %.1f %.1f",RopoDevice::turretModule.directPos, RopoDevice::turretModule.elevatePos, RopoDevice::turretModule.mode, RopoDevice::turretModule.directMotor.get_voltage(), RopoDevice::turretModule.elevateMotor.get_voltage());
		pros::lcd::print(4,"EV:%.1f %.1f %.0f %.1f",RopoDevice::turretModule.targetElecvateAngle, RopoDevice::turretModule.currentElecvateAngle, (float)RopoDevice::turretModule.elevateStableFlag, RopoDevice::turretModule.elevateVoltage);
		pros::lcd::print(5,"DC:%.1f %.1f %.0f %.1f",RopoDevice::turretModule.targetDirectAngle, RopoDevice::turretModule.currentDirectAngle, (float)RopoDevice::turretModule.directStableFlag, RopoDevice::turretModule.directVoltage);


		// RopoDevice::Debugger.Print("%.1f,%.1f,%.1f\r\n",
		// 							RopoDevice::flyWheel.targetVelocity, 
		// 							RopoDevice::flyWheel.currentVelocity, 
		// 							RopoDevice::flyWheel.SumVoltage);

		// RopoDevice::Debugger.Print("%.1f,%.1f,%.1f\r\n",
		// 							RopoDevice::turretModule.targetElecvateAngle,
		// 							RopoDevice::turretModule.currentElecvateAngle, 
		// 							RopoDevice::turretModule.elevateVoltage);

		RopoDevice::Debugger.Print("%.1f,%.1f,%.1f\r\n",
									RopoDevice::turretModule.targetDirectAngle * 400,
									RopoDevice::turretModule.currentDirectAngle * 400, 
									RopoDevice::turretModule.directVoltage);

		pros::delay(10);
	}
}
