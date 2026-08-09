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
    return static_cast<int>((command * 500) + 1500);
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
