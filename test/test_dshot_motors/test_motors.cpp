#include <Arduino.h>

#include "MotorController.h"

DshotMotorController motor_controller;
// PwmMotorController motor_controller;
void setup() {
    Serial.begin(115200);
}

void loop() {

    const unsigned long time = millis();
    double command = 0.0;
    if (time < 2000) {
        motor_controller.setCommand(MotorInterface::MotorCommand(0.0, 0.0, 0.0, 0.0));
    } else if (time < 4000) {
        motor_controller.setCommand(MotorInterface::MotorCommand(0.5, 0.5, 0.5, 0.5));
        command = 0.5;
    } else if (time < 6000) {
        motor_controller.setCommand(MotorInterface::MotorCommand(0.2, 0.2, 0.2, 0.2));
        command = 0.2;
    } else if (time < 8000) {
        motor_controller.setCommand(MotorInterface::MotorCommand(0.8, 0.8, 0.8, 0.8));
        command = 0.8;
    } else if (time < 12000) {
        motor_controller.setCommand(MotorInterface::MotorCommand(1.0, 1.0, 1.0, 1.0));
        command = 1.0;
    } else {
        motor_controller.setCommand(MotorInterface::MotorCommand(0.0, 0.0, 0.0, 0.0));
    }

    motor_controller.loop();
    Serial.printf("commanding: %f\n", command);

    delay(10);
}
