#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <Arduino.h>
#include <DShotRMT.h>
#include <ESP32Servo.h>

class MotorInterface {
public:
  // Contains commanded power to each motor. Commands are in the range [-1,1]
  // with +/- 1 being max power in either direction
  struct MotorCommand {
    // C++ 11 defaults disqualify from brace initialization so this needs
    // constructor
    MotorCommand(double front_left_, double front_right_, double back_left_,
                 double back_right_)
        : front_left(front_left_), front_right(front_right_),
          back_left(back_left_), back_right(back_right_) {}
    MotorCommand(){};
    double front_left = 0.0;
    double front_right = 0.0;
    double back_left = 0.0;
    double back_right = 0.0;
  };
  virtual ~MotorInterface() = default;
  // Run the motor loop and apply commands
  virtual void loop() = 0;
  virtual void setCommand(const MotorCommand &command) = 0;
  // Blocking function (up to several seconds) to arm motors. Returns false if
  // the arm failed
  virtual bool armControllers() = 0;
  virtual bool isArmed() = 0;
};

class PwmMotorController : public MotorInterface {
public:
  PwmMotorController();
  void loop() override;
  void setCommand(const MotorCommand &command) override;
  bool armControllers() override;
  bool isArmed() override { return armed; }

private:
  struct {
    Servo front_left;
    Servo front_right;
    Servo back_left;
    Servo back_right;
  } servos{};
  MotorCommand last_command{};
  bool armed = false;
};

class DshotMotorController : public MotorInterface {
public:
  DshotMotorController();
  void loop() override;
  void setCommand(const MotorCommand &command) override;
  bool armControllers() override;
  bool isArmed() override { return armed; }

private:
  struct MotorContainer {
    MotorContainer(gpio_num_t front_left_pin, gpio_num_t front_right_pin,
                   gpio_num_t back_left_pin, gpio_num_t back_right_pin)
        : front_left(front_left_pin, 0), front_right(front_right_pin, 1),
          back_left(back_left_pin, 2), back_right(back_right_pin, 3) {}
    DShotRMT front_left;
    DShotRMT front_right;
    DShotRMT back_left;
    DShotRMT back_right;
  } motors;
  MotorCommand last_command{};
  bool armed = false;
};

#endif
