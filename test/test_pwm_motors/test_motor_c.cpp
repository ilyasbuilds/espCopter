#include <Arduino.h>
#include <unity.h>
#include <ESP32Servo.h>
#include "config.h"

Servo testMotor;

void setUp(void) {
  Serial.println(">>> SETUP: Attaching motor C...");
  testMotor.setPeriodHertz(MOTOR_PWM_FREQUENCY);
  testMotor.attach(LOWER_LEFT_MOTOR, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  Serial.println(">>> Motor C attached!");
}

void tearDown(void) {
  Serial.println(">>> TEARDOWN: Stopping and detaching motor...");
  testMotor.writeMicroseconds(MIN_PULSE_LENGTH);
  delay(500);
  testMotor.detach();
  Serial.println(">>> Motor C detached!");
}

void test_arm_and_run_motor_c(void) {
  Serial.println(">>> TEST START: Arming ESC...");
  
  // ESC arming sequence
  Serial.print(">>> Setting to MAX_PULSE_LENGTH: ");
  Serial.println(MAX_PULSE_LENGTH);
  testMotor.writeMicroseconds(MAX_PULSE_LENGTH);
  delay(3000);
  
  Serial.print(">>> Setting to MIN_PULSE_LENGTH: ");
  Serial.println(MIN_PULSE_LENGTH);
  testMotor.writeMicroseconds(MIN_PULSE_LENGTH);
  delay(3000);
  
  Serial.println(">>> Running motor at 20% throttle...");
  int throttle = MIN_PULSE_LENGTH + (MAX_PULSE_LENGTH - MIN_PULSE_LENGTH) * 0.2;
  Serial.print(">>> Throttle value: ");
  Serial.println(throttle);
  testMotor.writeMicroseconds(throttle);
  delay(3000);
  
  Serial.println(">>> Stopping motor...");
  testMotor.writeMicroseconds(MIN_PULSE_LENGTH);
  
  Serial.println(">>> TEST COMPLETE!");
  TEST_ASSERT_TRUE(true);
}

void setup() {
  // Initialize serial and wait for connection
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("\n\n>>> MOTOR C TEST STARTING <<<");
  
  UNITY_BEGIN();
  RUN_TEST(test_arm_and_run_motor_c);
  UNITY_END();
  
  Serial.println(">>> TEST SEQUENCE FINISHED <<<");
}

void loop() {
  // Nothing to do here
} 