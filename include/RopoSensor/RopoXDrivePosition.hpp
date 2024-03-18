#ifndef ROPO_XDRIVE_POSITION
#define ROPO_XDRIVE_POSITION

#include "RopoWheelModule.hpp"
#include "RopoParameter.hpp"
#include "RopoMath/Header.hpp"
#include "pros/motors.hpp"
#include "pros/misc.hpp"

namespace RopoXDrivePosition {
	class XPositionModule{
    private:
        pros::Motor& leftFrontMotor;
        pros::Motor& leftBackMotor;
        pros::Motor& rightBackMotor;
        pros::Motor& rightFrontMotor;
        pros::Task *BackgroundTask;

        double posX,posY,deltaXToCar,deltaYToCar,angle;
        double last_LFEncoder,last_LBEncoder,last_RFEncoder,last_RBEncoder,LFEncoder,LBEncoder,RFEncoder,RBEncoder;
        float sampleTime;
        
        virtual void UpdateEncoder(){
            last_LFEncoder = LFEncoder;
            last_LBEncoder = LBEncoder;
            last_RBEncoder = RBEncoder;
            last_RFEncoder = RFEncoder;

            LFEncoder = leftFrontMotor.get_position();
            LBEncoder = leftBackMotor .get_position();
            RBEncoder = rightBackMotor .get_position();
            RFEncoder = rightFrontMotor.get_position();

        }

        static void BackgroundTaskFunction(void *Parameter){
            if(Parameter == nullptr) return;
            XPositionModule *This = static_cast<XPositionModule*>(Parameter);
            
            This -> posX = 0;
            This -> posY = 0;
            This -> leftFrontMotor .tare_position();
            This -> leftBackMotor  .tare_position();
            This -> rightBackMotor .tare_position();
            This -> rightFrontMotor.tare_position();
            while(1){
                This -> UpdateEncoder();
                This -> deltaXToCar = (-(This -> LFEncoder - This -> last_LFEncoder) - (This -> LBEncoder - This -> last_LBEncoder) + (This -> RBEncoder - This -> last_RBEncoder) + (This -> RFEncoder - This -> last_RFEncoder)) / 4.0;
                This -> deltaXToCar = This -> deltaXToCar * (float)RopoMath::Pi / 180.0 * RopoParameter::CHASSIS_WHEEL_GEAR_RATIO * RopoParameter::CHASSIS_WHEEL_R * ((float)cos(RopoParameter::CHASSIS_WHEEL_ANGLE));
                This -> deltaYToCar =  ((This -> LFEncoder - This -> last_LFEncoder) - (This -> LBEncoder - This -> last_LBEncoder) - (This -> RBEncoder - This -> last_RBEncoder) + (This -> RFEncoder - This -> last_RFEncoder)) / 4.0;
                This -> deltaYToCar = This -> deltaYToCar * (float)RopoMath::Pi / 180.0 * RopoParameter::CHASSIS_WHEEL_GEAR_RATIO * RopoParameter::CHASSIS_WHEEL_R * ((float)sin(RopoParameter::CHASSIS_WHEEL_ANGLE));
                if(This->angle <= 180.0 && This -> angle >= -180.0){    
                    This -> posX += This -> deltaXToCar *  cos( This->angle / 180.0 * RopoMath::Pi);
                    This -> posX += This -> deltaYToCar * (-sin( This->angle / 180.0 * RopoMath::Pi));
                    This -> posY += This -> deltaYToCar *  cos( This->angle / 180.0 * RopoMath::Pi);
                    This -> posY += This -> deltaXToCar *  sin( This->angle / 180.0 * RopoMath::Pi);
                }
                pros::delay(This -> sampleTime);
            }
        }

    public:   
        XPositionModule( pros::Motor& _leftFrontMotor, pros::Motor& _leftBackMotor,pros::Motor& _rightBackMotor, pros::Motor& _rightFrontMotor)
        : leftFrontMotor( _leftFrontMotor), rightFrontMotor( _rightFrontMotor), rightBackMotor( _rightBackMotor ), leftBackMotor( _leftBackMotor ),  
        posX(0), posY(0), last_LBEncoder(0), last_LFEncoder(0), last_RBEncoder(0), last_RFEncoder(0), sampleTime(10)
        {
            leftFrontMotor .set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES); 
            leftBackMotor .set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES); 
            rightBackMotor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
            rightFrontMotor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
            
            BackgroundTask = new pros::Task(BackgroundTaskFunction,this);    
        }

        float GetPosX(){return posX;}
        float GetPosY(){return posY;}
        float GetDeltaX(){return LFEncoder;}
        float GetDeltaY(){return last_LFEncoder;}
        void  SetPosXY(float _posX,float _posY){
            posX = _posX;
            posY = _posY;
        }
        void ResetPosition(){
            posX = 0;
            posY = 0;
        }
        void updateYaw(float value){
            angle = value * 180 / 3.14;
        }

        RopoMath::Vector<float> GetPosition()
        {
            RopoMath::Vector<float> PositionVector(RopoMath::ColumnVector,3);
            PositionVector[1] =  posX;
            PositionVector[2] =  posY;
            PositionVector[3] =  angle;

            return PositionVector;
	    }
    };
}

#endif // ROPO_XDRIVE_POSITION