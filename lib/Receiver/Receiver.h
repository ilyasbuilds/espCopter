#ifndef RECEIVER_H
#define RECEIVER_H

class Receiver {
public:
  // Struct to store the receiver values. Bidirectional values are in the range
  // [-1,1], Throttle is [0,1]
  struct values {
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    double throttle = 0.0;
    double aux[2] = {0.0, 0.0};
  };
  Receiver();
  values getData();
  void print();
};

#endif
