#include "MotorControl.hpp"
#include "pinout.hpp"
#include <Arduino.h>


//DC Motor

    // Global motor instances
namespace MotorControl {
    DCMotor LeftWheel;
    DCMotor RightWheel;
    ServoMotor FrontWheel;
}

    void MotorControl::DCMotorControl::Init() {
        //Setup the Motor ID to the struct 
        LeftWheel.MotorID = 1; 
        LeftWheel.PWMChannelIN1 = 1;
        LeftWheel.PWMChannelIN2 = 2;

        RightWheel.MotorID = 2;
        RightWheel.PWMChannelIN1 = 3;
        RightWheel.PWMChannelIN2 = 4;

        // Setup PWM channels for DC Motors (Side Wheels)
        ledcAttachChannel(Pinout::LeftMotorIn1, LeftWheel.PWMFrequency, LeftWheel.PWMResolution, LeftWheel.PWMChannelIN1);
        ledcAttachChannel(Pinout::LeftMotorIn2, LeftWheel.PWMFrequency, LeftWheel.PWMResolution, LeftWheel.PWMChannelIN2);
        ledcAttachChannel(Pinout::RightMotorIn1, RightWheel.PWMFrequency, RightWheel.PWMResolution, RightWheel.PWMChannelIN1);
        ledcAttachChannel(Pinout::RightMotorIn2, RightWheel.PWMFrequency, RightWheel.PWMResolution, RightWheel.PWMChannelIN2);



        // Set all the PWM Channels' Dutycycle to 0 
        ledcWriteChannel(LeftWheel.PWMChannelIN1, 0);
        ledcWriteChannel(LeftWheel.PWMChannelIN2, 0);
        ledcWriteChannel(RightWheel.PWMChannelIN1, 0);
        ledcWriteChannel(RightWheel.PWMChannelIN2, 0);
                
    };

    void MotorControl::DCMotorControl::TurnClockwise(MotorControl::DCMotor& Motor){
        ledcWriteChannel(Motor.PWMChannelIN1, Motor.Speed);
        ledcWriteChannel(Motor.PWMChannelIN2, 0); 

    };

    void MotorControl::DCMotorControl::TurnAntiClockwise(MotorControl::DCMotor& Motor){
        
        ledcWriteChannel(Motor.PWMChannelIN1, 0);
        ledcWriteChannel(Motor.PWMChannelIN2, Motor.Speed); 

    };

    void MotorControl::DCMotorControl::Stop(MotorControl::DCMotor& Motor){
        ledcWriteChannel(Motor.PWMChannelIN1, 4096);
        ledcWriteChannel(Motor.PWMChannelIN2, 4096); 

    };

    void MotorControl::ServoMotorControl::Init(){
        // Setup PWM channel for Servo Motor (Front Wheel)
        ledcAttachChannel(Pinout::ServoPin, FrontWheel.PWMFrequency, FrontWheel.PWMResolution, FrontWheel.PWMChannel);
        // Set all the PWM Channels' Dutycycle to 0 
        ledcWriteChannel(FrontWheel.PWMChannel, 0);
    };

    /*For SG90 Servo Motor
  PWM         --> 50Hz  (20ms)
  Dutycycle   --> 1-2ms (5-10%)*/
void MotorControl::ServoMotorControl::TrunDeg(MotorControl::ServoMotor& Motor){
  Motor.PWMDuty = (float(Motor.TargetAngle) / 90.0f) * 51.2f + 25.0f ;
  ledcWriteChannel(Motor.PWMChannel, Motor.PWMDuty);
  /*For Debug*/
  // Serial.print("Servo Degree: ");
  // Serial.println(Degree);
  // Serial.print("Dutycycle: ");
  // Serial.println(Dutycycle);
};
