#include "Movement.hpp"
#include "MotorControl.hpp"
#include "pinout.hpp"
#include <Arduino.h>

/*------------The following config is for differential driving------------*/
    void Movement::RotateRight(){
        /*The following config is for differential driving, i.e. Front Wheel always 90 deg facing forward */
        /*Config. of DC Motor (Side Wheel)*/
        MotorControl::DCMotorControl::TurnAntiClockwise(MotorControl::LeftWheel);
        MotorControl::DCMotorControl::TurnClockwise(MotorControl::RightWheel);

        /*Setting the servo motor to 90 deg */
        MotorControl::FrontWheel.TargetAngle = servoCenter;
        MotorControl::ServoMotorControl::TrunDeg(MotorControl::FrontWheel);

    };
    void Movement::RotateLeft(){
        /*The following config is for differential driving, i.e. Front Wheel always 90 deg facing forward */
        /*Config. of DC Motor (Side Wheel)*/
        MotorControl::DCMotorControl::TurnClockwise(MotorControl::LeftWheel);
        MotorControl::DCMotorControl::TurnAntiClockwise(MotorControl::RightWheel);

        /*Setting the servo motor to 90 deg */
        MotorControl::FrontWheel.TargetAngle = servoCenter;
        MotorControl::ServoMotorControl::TrunDeg(MotorControl::FrontWheel);
    };

    void Movement::MoveForward(){
        /*The following config is for differential driving, i.e. Front Wheel always 90 deg facing forward */
        /*Config. of DC Motor (Side Wheel)*/
        MotorControl::DCMotorControl::TurnAntiClockwise(MotorControl::LeftWheel);
        MotorControl::DCMotorControl::TurnAntiClockwise(MotorControl::RightWheel);


        /*Setting the servo motor to 90 deg */
        MotorControl::FrontWheel.TargetAngle = servoCenter;
        MotorControl::ServoMotorControl::TrunDeg(MotorControl::FrontWheel);
    };
    void Movement::MoveBackward(){
        /*The following config is for differential driving, i.e. Front Wheel always 90 deg facing forward */
        /*Config. of DC Motor (Side Wheel)*/
        MotorControl::DCMotorControl::TurnClockwise(MotorControl::LeftWheel);
        MotorControl::DCMotorControl::TurnClockwise(MotorControl::RightWheel);

        /*Setting the servo motor to 90 deg */
        MotorControl::FrontWheel.TargetAngle = servoCenter;
        MotorControl::ServoMotorControl::TrunDeg(MotorControl::FrontWheel);
    };
    void Movement::Stop(){
        MotorControl::DCMotorControl::Stop(MotorControl::LeftWheel);
        MotorControl::DCMotorControl::Stop(MotorControl::RightWheel);
    }
