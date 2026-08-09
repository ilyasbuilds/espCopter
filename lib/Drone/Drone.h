#ifndef DRONE_H
#define DRONE_H
#if defined(Arduino_h)
#include <Arduino.h>
#endif
#include "config.h"
#include <ESP32Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include "MotorController.h"
#include "Imu.h"
#include "Pid.h"
#include "Receiver.h"
#include <Preferences.h>

class Drone {
private:
	Imu* sensor;
    Adafruit_BNO055 bno;
    MotorController* controller;	
	Receiver* rc; 
    bool started = false;

	//clock trackers 
	int lastImuUpdate = 0;

	double rcValues[6];
	double imuValues[12];
	double velControlX, velControlY, velControlZ;
	double output[4] = {0, 0, 0, 0};
	double xK[3] = {0.0, 0.0, 0.0}; //Yaw
	double yK[3] = {24.0, 0.0, 0.0}; //Roll
	double zK[3] = {24.0, 0.0, 0.0}; //Pitch
    double velSetpoints[3] = {0, 0, 0};	
	DPID* velControllers[3];
	DPID* posControllers[3];

	Preferences preferences;
	void loadGainsFromStorage();
	void saveGainsToStorage();

	void printIO();
	// This should be run every loop and is internally gated for fast opperation
	void fastLoop();
	// Internal setup function. set up all objects
	void setup();
	//This function is to update xK, yK, zK gains

    unsigned long lastCalibrationPrint = 0;
    unsigned long droneLoopStartTime = 0;
    unsigned long droneLoopEndTime = 0;
    unsigned long droneLoopCount = 0;
    unsigned long droneTotalTime = 0;

    const int BNO055_RESET_PIN = 15;  // Add this constant

public:
	Drone(MotorController* mc, Receiver* r);
	// Pulse the loop function from the main thread
	void loop();
	void updateGain(double* gains);
	void printGains();

	// New methods for WebSocket telemetry
	double getRoll() { return imuValues[10]; }  // Roll from IMU
	double getPitch() { return imuValues[11]; } // Pitch from IMU
	double getYaw() { return imuValues[9]; }    // Yaw from IMU
	
	// Velocity getters (now using IMU gyroscope data)
	double getRollVelocity() { return imuValues[3]; }   // Roll velocity from gyro
	double getPitchVelocity() { return imuValues[4]; }  // Pitch velocity from gyro
	double getYawVelocity() { return imuValues[5]; }    // Yaw velocity from gyro
	
	// PID output getters
	double getRollPIDOutput() { return velControlY; }
	double getPitchPIDOutput() { return velControlZ; }
	double getYawPIDOutput() { return velControlX; }
	
	// Setpoint getters
	double getRollSetpoint() { return velSetpoints[1]; }  // Roll setpoint
	double getPitchSetpoint() { return velSetpoints[2]; } // Pitch setpoint
	double getYawSetpoint() { return velSetpoints[0]; }   // Yaw setpoint

	// New methods for telemetry
	void getGains(double* gains);  // Fills array with current PID gains
	double* getMotorValues();      // Returns array of current motor values

    void printCalibrationStatus();

    uint8_t getSystemCalibration() { return sensor ? sensor->getSystemCalibration() : 0; }
    uint8_t getGyroCalibration() { return sensor ? sensor->getGyroCalibration() : 0; }
    uint8_t getAccelCalibration() { return sensor ? sensor->getAccelCalibration() : 0; }
    uint8_t getMagCalibration() { return sensor ? sensor->getMagCalibration() : 0; }
};

#endif
