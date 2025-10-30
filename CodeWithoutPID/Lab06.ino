#include <Arduino.h>
#include "pinout.hpp"
#include "MotorControl.hpp"
#include "IRSensors.hpp"
#include "Movement.hpp"

/*Creating User Task in FreeRTOS*/
/*-------------LED Blinking Task-------------*/
TaskHandle_t LEDBlinkingTaskHandle = NULL;
StaticTask_t xLEDBlinkingTCB;
void LEDBlinkingTask(void* pvPara) {
/*The code before entering while loop will be only run once*/
    pinMode(Pinout::Led, OUTPUT);
while (true) {
/* The LED will start blinking if the task is running */
    digitalWrite(Pinout::Led, HIGH);
    vTaskDelay(500);
    digitalWrite(Pinout::Led, LOW);
    vTaskDelay(500);
}
};

TaskHandle_t MovementTaskHandle = NULL;
StaticTask_t xMovementTCB;
void MovementTask(void* pvPara){
// IrSensorData IRData;
while(true){
// Get the status of the IR sensor and store in IRData
    IRSensors::IRData.state = IRSensors::ReadSensorState(IRSensors::IRData);
/*-------For Debug Use------*/
// Serial.print( "IR Status: ");
// Serial.print(IRSensors::IRData.Read_IR_L);
// Serial.print(IRSensors::IRData.Read_IR_M);
// Serial.println(IRSensors::IRData.Read_IR_R);

// Serial.print( "Condition: ");
// Serial.println(IRSensors::IRData.state);


// Depend on the condition, do line tracking
// Using switch case for the movement control
switch (IRSensors::IRData.state) {
case IRSensors::Middle_ON_Track:
    MotorControl::LeftWheel.Speed = 1000;
    MotorControl::RightWheel.Speed = 1000;
    Movement::RotateRight();
    vTaskDelay(100);
break; 

case IRSensors::Left_Middle_ON_Track:
    MotorControl::LeftWheel.Speed = 3000;
    MotorControl::RightWheel.Speed = 3000;
    Movement::RotateRight();
    vTaskDelay(100);
break;

case IRSensors::ALL_ON_Track:
    MotorControl::LeftWheel.Speed = 1000;
    MotorControl::RightWheel.Speed = 1000;
    Movement::MoveForward();
    vTaskDelay(100);
break;

case IRSensors::Left_Right_ON_Track:
    Movement::Stop();
    vTaskDelay(100);
break;

case IRSensors::Middle_Right_ON_Track:
    MotorControl::LeftWheel.Speed = 3000;
    MotorControl::RightWheel.Speed = 3000;
    Movement::RotateRight();
    vTaskDelay(100);
break;
case IRSensors::Right_ON_Track:
    MotorControl::LeftWheel.Speed = 3000;
    MotorControl::RightWheel.Speed = 3000;
    Movement::RotateRight();
    vTaskDelay(100);
break;

case IRSensors::Left_ON_Track:
    MotorControl::LeftWheel.Speed = 3000;
    MotorControl::RightWheel.Speed = 3000;
    Movement::RotateRight();
    vTaskDelay(100);
break;

case IRSensors::All_OFF_Track:
    Movement::Stop();
    vTaskDelay(100);
break;
}
    /*-------For Debug Use------*/
// Serial.print( "LeftWheel.Speed: ");
// Serial.println(MotorControl::LeftWheel.Speed);
// Serial.print("RightWheel.Speed: ");
// Serial.println(MotorControl::RightWheel.Speed);

/*-------For Debug Use------*/
// Serial.print( "LeftWheel.PWMIN1: ");
// Serial.print(MotorControl::LeftWheel.PWMChannelIN1);
// Serial.println(MotorControl::LeftWheel.PWMChannelIN2);

// Serial.print("RightWheel.PWMIN2: ");
// Serial.print(MotorControl::RightWheel.PWMChannelIN1);
// Serial.println(MotorControl::RightWheel.PWMChannelIN2);

    vTaskDelay(10);
}
};



void setup() {
    // Initialize Serial communication
    Serial.begin(115200);
    IRSensors::Init();
    MotorControl::DCMotorControl::Init();
    MotorControl::ServoMotorControl::Init();
    Serial.println("Motors Pin Initialized.");
    // Create FreeRTOS tasks with proper namespaced functions
    // Task creation function:
    // TaskHandle_t xTaskCreateStaticPinnedToCore(
    //     TaskFunction_t pxTaskCode,
    //     const char *const pcName,
    //     const uint32_t ulStackDepth,
    //     void *const pvParameters,
    //     UBaseType_t uxPriority,
    //     StackType_t *const puxStackBuffer,
    //     StaticTask_t *const pxTaskBuffer,
    //     const BaseType_t xCoreID
    // );

    xTaskCreatePinnedToCore(
        LEDBlinkingTask,
        "Blinking",
        2000, 
        NULL,
        1,
        &LEDBlinkingTaskHandle,
        1 );

    xTaskCreatePinnedToCore(
        MovementTask,
        "Movement",
        12000, 
        NULL,
        2,
        &MovementTaskHandle,
        1 );

    Serial.println("Robot initialized and running");
    vTaskDelay(10); 
}

void loop() {}
