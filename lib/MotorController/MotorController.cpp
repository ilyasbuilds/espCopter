#include "MotorController.h"

#include "config.h"

PwmMotorController::PwmMotorController() {
  servos.front_left.setPeriodHertz(MOTOR_PWM_FREQUENCY);
  servos.front_right.setPeriodHertz(MOTOR_PWM_FREQUENCY);
  servos.back_left.setPeriodHertz(MOTOR_PWM_FREQUENCY);
  servos.back_right.setPeriodHertz(MOTOR_PWM_FREQUENCY);

  servos.front_left.attach(UPPER_LEFT_MOTOR, MIN_PULSE_LENGTH,
                           MAX_PULSE_LENGTH);
  servos.front_right.attach(UPPER_RIGHT_MOTOR, MIN_PULSE_LENGTH,
                            MAX_PULSE_LENGTH);
  servos.back_left.attach(LOWER_LEFT_MOTOR, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  servos.back_right.attach(LOWER_RIGHT_MOTOR, MIN_PULSE_LENGTH,
                           MAX_PULSE_LENGTH);
}

void PwmMotorController::loop() {
  auto convert_command = [](double command) {
    if (command > 1.0)
      command = 1.0;
    // PWM motor controller only supports forward operation
    if (command < 0.0)
      command = 0.0;
    return static_cast<int>((command * 1000) + 1000);
  };
  servos.front_left.writeMicroseconds(convert_command(last_command.front_left));
  servos.front_right.writeMicroseconds(
      convert_command(last_command.front_right));
  servos.back_left.writeMicroseconds(convert_command(last_command.back_left));
  servos.back_right.writeMicroseconds(convert_command(last_command.back_right));
}

void PwmMotorController::setCommand(const MotorCommand &command) {
  last_command = command;
}

bool PwmMotorController::armControllers() {
  setCommand(MotorCommand(1.0, 1.0, 1.0, 1.0));
  loop();
  delay(3000);
  setCommand(MotorCommand{0.0, 0.0, 0.0, 0.0});
  loop();
  delay(3000);
  armed = true;
  return true;
}

DshotMotorController::DshotMotorController()
    : motors(static_cast<gpio_num_t>(UPPER_LEFT_MOTOR),
             static_cast<gpio_num_t>(UPPER_RIGHT_MOTOR),
             static_cast<gpio_num_t>(LOWER_LEFT_MOTOR),
             static_cast<gpio_num_t>(LOWER_RIGHT_MOTOR)) {
  motors.front_left.begin(DSHOT300);
  motors.front_right.begin(DSHOT300);
  motors.back_left.begin(DSHOT300);
  motors.back_right.begin(DSHOT300);
};

void DshotMotorController::loop() {
  auto convert_command = [](double command) {
    if (command > 1.0)
      command = 1.0;
    // Dshot motor controller only supports forward operation (reprogramming
    // needed for bidirectional)
    if (command < 0.0)
      command = 0.0;
    // min throttle is DSHOT_THROTTLE_MIN and max is DSHOT_THROTTLE_MAX
    const uint16_t throttle_out =
        static_cast<uint16_t>(command * DSHOT_THROTTLE_MAX);
    if (throttle_out < DSHOT_THROTTLE_MIN)
      return static_cast<uint16_t>(0);
    return throttle_out;
  };

  motors.front_left.sendThrottleValue(convert_command(last_command.front_left));
  motors.front_right.sendThrottleValue(
      convert_command(last_command.front_right));
  motors.back_left.sendThrottleValue(convert_command(last_command.back_left));
  motors.back_right.sendThrottleValue(convert_command(last_command.back_right));
}

void DshotMotorController::setCommand(const MotorCommand &command) {
  last_command = command;
}

bool DshotMotorController::armControllers() {
  armed = true;
  return true;
}