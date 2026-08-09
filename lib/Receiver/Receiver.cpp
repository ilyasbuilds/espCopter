#include "Receiver.h"

#include "Arduino.h"

namespace {

// Receiver variables
double pastRh = 0;
double valueRh = 0;
double pastRv = 0;
double valueRv = 0;
double pastLv = 0;
double valueLv = 0;
double pastLh = 0;
double valueLh = 0;
double pastKl = 0;
double valueKl = 0;
double pastKr = 0;
double valueKr = 0;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

void changeRh() {
  portENTER_CRITICAL_ISR(&mux);
  unsigned long currentTime = esp_timer_get_time();
  if (digitalRead(25) == LOW) {
    double temp = currentTime - pastRh;
    if (temp > 900 && temp < 2100) { // Slightly wider range
      valueRh = temp;
    }
  } else {
    pastRh = currentTime;
  }
  portEXIT_CRITICAL_ISR(&mux);
}
void changeRv() {
  portENTER_CRITICAL_ISR(&mux);
  if (digitalRead(26) == LOW) {
    double temp = micros() - pastRv;
    if (temp > 950 && temp < 2100) {
      valueRv = temp;
    } else {
      valueRv = temp;
    }
  } else {
    pastRv = micros();
  }
  portEXIT_CRITICAL_ISR(&mux);
}
void changeLh() {
  portENTER_CRITICAL_ISR(&mux);
  double temp = esp_timer_get_time() - pastLh;
  if (digitalRead(27) == LOW) {
    if (temp > 950 && temp < 2100) {
      valueLh = temp;
    }
  } else {
    pastLh = esp_timer_get_time();
  }
  portEXIT_CRITICAL_ISR(&mux);
}
void changeLv() {
  portENTER_CRITICAL_ISR(&mux);
  double temp = esp_timer_get_time() - pastLv;
  if (digitalRead(14) == LOW) {
    if (temp > 950 && temp < 2100) {
      valueLv = temp;
    }
  } else {
    pastLv = esp_timer_get_time();
  }
  portEXIT_CRITICAL_ISR(&mux);
}
void changeKl() {
  portENTER_CRITICAL_ISR(&mux);
  double temp = esp_timer_get_time() - pastKl;
  if (digitalRead(34) == LOW) {
    if (temp > 950 && temp < 2100) {
      valueKl = temp;
    }
  } else {
    pastKl = esp_timer_get_time();
  }
  portEXIT_CRITICAL_ISR(&mux);
}
} // namespace

Receiver::Receiver() {
  Serial.println("Receiver setup");

  // Attach interrupts for the receiver
  pinMode(25, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(25), changeRh, CHANGE);
  pinMode(26, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(26), changeRv, CHANGE);
  pinMode(27, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(27), changeLh, CHANGE);
  pinMode(14, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(14), changeLv, CHANGE);
  pinMode(34, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(34), changeKl, CHANGE);
}

Receiver::values Receiver::getData() {
  auto map_value = [](unsigned long value) {
    // map invalid range to no command
    if (value < 900 || value > 2100)
      return 0.0;
    return (value - 1500.0) / 500.0;
  };
  Receiver::values values;
  values.roll = map_value(valueRh);
  values.pitch = map_value(valueRv);
  values.yaw = map_value(valueLh);
  // Throttle is special because it is [0,1]
  values.throttle = (1.0 + map_value(valueLv)) / 2.0;
  values.aux[0] = map_value(valueKl);
  values.aux[1] = map_value(valueKr);
  return values;
}

void Receiver::print() {
  Receiver::values values = getData();
  Serial.print("Pitch: ");
  Serial.println(values.pitch);
  Serial.print("Roll: ");
  Serial.println(values.roll);
  Serial.print("Yaw: ");
  Serial.println(values.yaw);
  Serial.print("Throttle: ");
  Serial.println(values.throttle);
  Serial.print("Aux 0: ");
  Serial.println(values.aux[0]);
  Serial.print("Aux 1: ");
  Serial.println(values.aux[1]);
}
