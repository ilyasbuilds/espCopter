#include <Arduino.h>
#include <unity.h>
#include <ESP32Servo.h>
#include "config.h"

Servo testMotor;

void test_basic_motor_d(void) {
  Serial.println(">>> Starting simple motor test");
  
  // Initialize motor
  testMotor.setPeriodHertz(MOTOR_PWM_FREQUENCY);
  testMotor.attach(LOWER_RIGHT_MOTOR, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  
  // Set to min pulse (off)
  Serial.println(">>> Setting minimum pulse");
  testMotor.writeMicroseconds(MIN_PULSE_LENGTH);
  delay(5000);
  
  // Small pulse just to confirm connection
  Serial.println(">>> Small pulse test");
  testMotor.writeMicroseconds(MIN_PULSE_LENGTH + 50);
  delay(1000);
  
  // Back to min pulse
  Serial.println(">>> Back to minimum");
  testMotor.writeMicroseconds(MIN_PULSE_LENGTH);
  
  // Detach
  testMotor.detach();
  Serial.println(">>> Test complete");
  
  TEST_ASSERT_TRUE(true);
}

void setup() {
  Serial.begin(115200);
  delay(3000);
  Serial.println("\n\n>>> SIMPLE MOTOR D TEST <<<");
  
  UNITY_BEGIN();
  RUN_TEST(test_basic_motor_d);
  UNITY_END();
}

void loop() {} 