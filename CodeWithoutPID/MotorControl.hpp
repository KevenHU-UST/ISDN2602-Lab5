#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdint.h>

namespace MotorControl {

    // Configuration of DC Motor (Side Wheels)    
    struct DCMotor {
    // PWM Configuration
    const uint16_t PWMFrequency = 2000;
    const  uint8_t PWMResolution = 12;
          uint16_t PWMDuty = 0;
           uint8_t MotorID = 0; //ID = 1 (Left), = 2 (Right)
           uint8_t PWMChannelIN1 = 0; //ensure the PWM channel for different motor is not the same
           uint8_t PWMChannelIN2 = 0;
    //Adjustable Parameter
          uint16_t Speed = 0; //Init set to 0 
    }; 

    // Configuration of Servo Motor (Front Wheel)
    struct ServoMotor { 
    const uint8_t PWMFrequency = 50; //PWM must be in 50Hz 
    const uint8_t PWMResolution = 12; 
          uint16_t PWMDuty = 0;
    const uint8_t PWMChannel = 8; //Ideally select between 5-10
          float TargetAngle = 0.0f;
    };

    // Global motor instances

    extern DCMotor LeftWheel;
    extern DCMotor RightWheel;
    extern ServoMotor FrontWheel;

    namespace DCMotorControl{
    /*Initialization of PWM channels for DC Motors*/ 
    void Init(); 
    void TurnClockwise(DCMotor& Motor);
    void TurnAntiClockwise(DCMotor& Motor);
    void Stop(DCMotor& Motor);
    };

    namespace ServoMotorControl{
    /*Initialization of PWM Channel for Servo Motor*/
    void Init();
    void TrunDeg(ServoMotor& Motor); //in deg
    }
}

#endif // MOTOR_CONTROL_H