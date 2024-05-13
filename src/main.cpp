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
bool manualReset = false;

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
	pros::delay(200);
	RopoDevice::ShootPneumatic.set_value(false);
}

void FlywheelSwitch()
{
	RopoDevice::flyWheel.flyWheelMode ^= 1;
}

void TurretTrackingSwitch()
{
	RopoDevice::turretModule.directStableFlag ^= 1;
	RopoDevice::turretModule.elevateStableFlag ^= 1;
	RopoDevice::flyWheel.flyWheelMode ^= 1;
}

void ResetTurret()
{
	manualReset ^= 1;
}

void opcontrol() {
	RopoController::AxisValueCast xVelocityInput(RopoDevice::masterController, pros::E_CONTROLLER_ANALOG_LEFT_Y , RopoController::Exp);
	RopoController::AxisValueCast yVelocityInput(RopoDevice::masterController, pros::E_CONTROLLER_ANALOG_LEFT_X , RopoController::Exp);
	RopoController::AxisValueCast wVelocityInput(RopoDevice::masterController, pros::E_CONTROLLER_ANALOG_RIGHT_X, RopoController::Exp);

	float xInput, yInput, wInput, dInput, eInput;

	RopoMath::Vector<float> inputVelocity(RopoMath::ColumnVector, 3);
	RopoMath::Vector<float> turretVelocity(RopoMath::ColumnVector, 2);

	RopoMath::Vector<float> odomPos(RopoMath::ColumnVector, 3);

	RopoController::ButtonTaskLine ButtonDetectLine1(RopoDevice::masterController);
	RopoDevice::masterController.clear();
	ButtonDetectLine1.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_L1 , RopoController::Rising, FlywheelValueUp);
	ButtonDetectLine1.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_L2 , RopoController::Rising, FlywheelValueDown);
	ButtonDetectLine1.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_R1 , RopoController::Rising, FlywheelShoot);
	ButtonDetectLine1.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_X , RopoController::DoubleClick, FlywheelSwitch);
	ButtonDetectLine1.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_A , RopoController::DoubleClick, TurretTrackingSwitch);
	ButtonDetectLine1.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_Y , RopoController::DoubleClick, ResetTurret);
	ButtonDetectLine1.Enable();

	bool last_shoot_value = false;

	while (true)
	{
		xInput =   xVelocityInput.GetAxisValue();
		yInput = - yVelocityInput.GetAxisValue();
		wInput = - wVelocityInput.GetAxisValue();

		inputVelocity[1] = (fabs(xInput) < 0.08) ? (0) : ((xInput-0.08)  * RopoParameter::CHASSIS_X_SCALE / 0.92);
		inputVelocity[2] = (fabs(yInput) < 0.08) ? (0) : ((yInput-0.08)  * RopoParameter::CHASSIS_Y_SCALE / 0.92);
		inputVelocity[3] = (fabs(wInput) < 0.08) ? (0) : ((wInput-0.08)  * RopoParameter::CHASSIS_W_SCALE / 0.92);

		RopoDevice::chassisModule.MoveVelocity(inputVelocity);

		odomPos = RopoDevice::xDrivePositionModule.GetPosition();

		//data update
		RopoDevice::turretModule.targetDirectAngle = RopoDevice::Downloader.data.errorX;
		RopoDevice::turretModule.targetElevateAngle = RopoDevice::Downloader.data.elevate_order;
		// RopoDevice::flyWheel.targetVelocity = RopoDevice::Downloader.data.speed_order;
		RopoDevice::turretModule.yoloFindFlag = RopoDevice::Downloader.data.find_flag;

		if(RopoDevice::Downloader.data.shoot_flag != last_shoot_value)
		{
			last_shoot_value = RopoDevice::Downloader.data.shoot_flag;
			RopoDevice::turretModule.modeFlag = 2;
			FlywheelShoot();
		}
		
		if(RopoDevice::Downloader.data.reset_flag)
		{
			RopoDevice::turretModule.modeFlag = 0;
		}
		else
		{
			RopoDevice::turretModule.modeFlag = 1;
		}

		if(manualReset){
			RopoDevice::turretModule.modeFlag = 0;
		}

		//data upload
		RopoDevice::Uploader.data.odomXpos = RopoDevice::xDrivePositionModule.GetPosX();
		RopoDevice::Uploader.data.odomYpos = RopoDevice::xDrivePositionModule.GetPosY();
		RopoDevice::Uploader.data.flyWheelSpeed = RopoDevice::flyWheel.currentVelocity;
		RopoDevice::Uploader.data.elevatePos = RopoDevice::turretModule.elevatePos;
		RopoDevice::Uploader.data.directPos = RopoDevice::turretModule.directPos;
		RopoDevice::xDrivePositionModule.updateYaw(RopoDevice::Downloader.data.yaw);

		pros::lcd::print(1,"%.1f %.1f %.1f",
						xInput,
						yInput,
						wInput);
		pros::lcd::print(2,"FW:%.1f %.1f %.0f %.1f",
						RopoDevice::flyWheel.targetVelocity, 
						RopoDevice::flyWheel.currentVelocity, 
						(float)RopoDevice::flyWheel.flyWheelMode, 
						RopoDevice::flyWheel.SumVoltage);
		pros::lcd::print(3,"TR:%.1f %.1f %d",
						RopoDevice::turretModule.directPos, 
						RopoDevice::turretModule.elevatePos, 
						RopoDevice::turretModule.modeFlag);
		pros::lcd::print(4,"EV:%.1f %.1f %.0f %.1f",
						RopoDevice::turretModule.targetElevateAngle, 
						RopoDevice::turretModule.currentElevateAngle, 
						(float)RopoDevice::turretModule.elevateStableFlag, 
						RopoDevice::turretModule.elevateVoltage);
		pros::lcd::print(5,"DC:%.1f %.1f %.0f %.1f",
						RopoDevice::turretModule.targetDirectAngle, 
						RopoDevice::turretModule.currentDirectAngle, 
						(float)RopoDevice::turretModule.directStableFlag, 
						RopoDevice::turretModule.directVoltage);
		pros::lcd::print(6,"DL:%.1f %.1f %.1f %d RE%d ST%d",
						RopoDevice::Downloader.data.errorX,
						RopoDevice::Downloader.data.elevate_order,
						RopoDevice::Downloader.data.speed_order,
						RopoDevice::Downloader.data.find_flag,
						RopoDevice::Downloader.data.reset_flag,
						RopoDevice::Downloader.data.shoot_flag);
		pros::lcd::print(7,"OD:%.2f %.2f %.2f",
						odomPos[1], 
						odomPos[2], 
						odomPos[3]);

		RopoDevice::Debugger.Print("%f,%f\r\n",RopoDevice::flyWheel.targetVelocity, 
										   	   RopoDevice::flyWheel.currentVelocity);

		pros::delay(5);
	}
}
