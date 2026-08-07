#include "Drone.h"

struct DroneLoopTiming {
    unsigned long imuTime;
    unsigned long rcTime;
    unsigned long fastLoopTime;
    unsigned long totalTime;
    unsigned long count;
} droneTiming = {0, 0, 0, 0, 0};

Drone::Drone(MotorController* mc, Reciever* r) { 
	sensor = NULL;
	controller = mc;
	rc = r;
	loadGainsFromStorage();  // Load gains before setup
	setup();
}

void Drone::setup() {
	Serial.println("Starting drone setup...");
	
	// Configure BNO055 reset pin
	const int BNO055_RESET_PIN = 15;
	pinMode(BNO055_RESET_PIN, OUTPUT);
	digitalWrite(BNO055_RESET_PIN, HIGH);  // Keep BNO055 enabled
	
	if(USE_IMU == true){ 
		Serial.println("Initializing IMU...");
		sensor = new Imu();
		/* Initialise the sensor */
		Serial.println("Setting up drone and sensors");	
		if (!bno.begin()) {
			/* There was a problem detecting the BNO055 ... check your connections */
			Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
			delay(1000);
			while (1)
				;
		} else {
			Serial.println("BNO STARTED");
			bno.setMode(adafruit_bno055_opmode_t::OPERATION_MODE_NDOF);
		}

		delay(1000);
		bno.setExtCrystalUse(true);
		sensor->startSensor(&bno);
		Serial.println("IMU initialization complete");
	}
	
	Serial.println("Initializing controllers...");
	started = true;

	velControlX = 0;
	velControlY = 0;
	velControlZ = 0;

	//Red is front
	//Roll = 2 Pitch = 1 Yaw = 0
	velControllers[0] = new DPID(&imuValues[9], &velControlX, &xK[0], &xK[1], &xK[2]);//Yaw
	velControllers[1] = new DPID(&imuValues[11], &velControlY, &yK[0], &yK[1], &yK[2]);//Pitch
	velControllers[2] = new DPID(&imuValues[10], &velControlZ, &zK[0], &zK[1], &zK[2]);//Roll
	

	controller->attachControllers(output);

	Serial.println("Drone setup complete!");
}

void Drone::loop() {
	if(!started) return;

	unsigned long loopStart = micros();
	unsigned long stepStart;

	if(USE_IMU) {
		stepStart = micros();
		unsigned long currentMicros = micros();
		
		if(currentMicros - lastImuUpdate > 2500) {
			lastImuUpdate = currentMicros;
			sensor->loop();
			sensor->getData(imuValues);
		}
		droneTiming.imuTime += (micros() - stepStart);
	}   

	stepStart = micros();
	fastLoop();
	droneTiming.fastLoopTime += (micros() - stepStart);

	droneTiming.totalTime += (micros() - loopStart);
	droneTiming.count++;
	
	if(droneTiming.count >= 1000 && LOG_TIMING) {
		Serial.println("\n=== Drone Loop Timing Breakdown ===");
		Serial.print("IMU avg (us): "); 
		Serial.println(droneTiming.imuTime / droneTiming.count);
		Serial.print("FastLoop avg (us): "); 
		Serial.println(droneTiming.fastLoopTime / droneTiming.count);
		Serial.print("Total avg (us): "); 
		Serial.println(droneTiming.totalTime / droneTiming.count);
		Serial.print("Drone Frequency (Hz): "); 
		Serial.println(1000000.0 / (droneTiming.totalTime / droneTiming.count));
		Serial.println("================================\n");
		
		// Reset counters
		droneTiming = {0, 0, 0, 0, 0};
	}
}


void Drone::fastLoop() { 
	//RC Values Yaw = 0 - Roll = 2 - Pitch = 3  
	rc->getData(rcValues);
	rcValues[0] -= 1500.0;
	rcValues[2] -= 1500.0;
	rcValues[3] -= 1500.0;

	rcValues[0] *= 10.0/500.0;
	rcValues[2] *= 10.0/500.0;
	rcValues[3] *= 10.0/500.0;

	//Vel Controller = Roll = 1 Pitch = 2 Yaw = 0
	//RC Values Yaw = 0 - Roll = 2 - Pitch = 3  
	//Set the setpoints for the controlllers
	velSetpoints[0] = rcValues[0]; //Yaw
	velSetpoints[1] = rcValues[2]*-1; //Roll
	velSetpoints[2] = rcValues[3]*-1; //Pitch

	//Set the required velocity
	velControllers[0]->setSetpoint(velSetpoints[0]); //Yaw
	velControllers[1]->setSetpoint(velSetpoints[1]); //Roll
	velControllers[2]->setSetpoint(velSetpoints[2]); //Pitch

	//Calculating required output signal
	velControllers[0]->calculate(); //Yaw
	velControllers[1]->calculate(); //Roll
	velControllers[2]->calculate(); //Pitch

	if(rcValues[1] < 1030){
		output[0] = 1000;
		output[1] = 1000;
		output[2] = 1000;
		output[3] = 1000;
	} else {
		//Current Output and Direction of spin
		output[0] = rcValues[1] + velControlY - velControlZ - velControlX; //Front left - CCW
		output[1] = rcValues[1] - velControlY - velControlZ + velControlX; //Front right - CW
		output[2] = rcValues[1] + velControlY + velControlZ + velControlX; //Rear Left - CW
		output[3] = rcValues[1] - velControlY + velControlZ - velControlX; //Rear right - CCW
	}
	controller->loop();
}

void Drone::printIO(){
	delay(100);
	Serial.println("---------- RC Values ---------");
	Serial.print(rcValues[0]);
	Serial.print(", ");
	Serial.print(rcValues[1]);
	Serial.print(", ");
	Serial.print(rcValues[2]);
	Serial.print(", ");
	Serial.print(rcValues[3]);
	Serial.println(".");

	Serial.println("---------- IMU Values ---------");
	Serial.print(imuValues[9]);
	Serial.print(", ");
	Serial.print(imuValues[10]);
	Serial.print(", ");
	Serial.print(imuValues[11]);
	Serial.println(".");

	Serial.println("---------- Vel Control ---------");
	Serial.print(velControlX);
	Serial.print(", ");
	Serial.print(velControlY);
	Serial.print(", ");
	Serial.print(velControlZ);
	Serial.println(".");
}

void Drone::updateGain(double* gains) {
    yK[0] = gains[0];
    yK[1] = gains[1];
    yK[2] = gains[2];

    zK[0] = gains[3];
    zK[1] = gains[4];
    zK[2] = gains[5];

    xK[0] = gains[6];
    xK[1] = gains[7];
    xK[2] = gains[8];
    
    // Save to storage after updating
    saveGainsToStorage();
}

void Drone::printGains(){
	Serial.println("------PID GAINS------");
	Serial.print("Xk: ");
	Serial.print(xK[0]);
	Serial.print(",");
	Serial.print(xK[1]);
	Serial.print(",");
	Serial.println(xK[2]);

	Serial.print("Yk: ");
	Serial.print(yK[0]);
	Serial.print(",");
	Serial.print(yK[1]);
	Serial.print(",");
	Serial.println(yK[2]);

	Serial.print("Zk: ");
	Serial.print(zK[0]);
	Serial.print(",");
	Serial.print(zK[1]);
	Serial.print(",");
	Serial.println(zK[2]);
}

void Drone::getGains(double* gains) {
    // Copy roll gains
    gains[0] = yK[0];  // P
    gains[1] = yK[1];  // I
    gains[2] = yK[2];  // D
    
    // Copy pitch gains
    gains[3] = zK[0];  // P
    gains[4] = zK[1];  // I
    gains[5] = zK[2];  // D
    
    // Copy yaw gains
    gains[6] = xK[0];  // P
    gains[7] = xK[1];  // I
    gains[8] = xK[2];  // D
}

double* Drone::getMotorValues() {
    return output;  // Return the array of motor output values
}

void Drone::loadGainsFromStorage() {
    preferences.begin("drone", false);  // false = read/write mode
    
    // Load gains with defaults if not found
    yK[0] = preferences.getDouble("yKp", 24.0);
    yK[1] = preferences.getDouble("yKi", 0.0);
    yK[2] = preferences.getDouble("yKd", 0.0);
    
    zK[0] = preferences.getDouble("zKp", 24.0);
    zK[1] = preferences.getDouble("zKi", 0.0);
    zK[2] = preferences.getDouble("zKd", 0.0);
    
    xK[0] = preferences.getDouble("xKp", 0.0);
    xK[1] = preferences.getDouble("xKi", 0.0);
    xK[2] = preferences.getDouble("xKd", 0.0);

    preferences.end();

	yK[0] = 40.0;
    yK[1] = 0.0;
    yK[2] = 0.0;
    
    zK[0] = 0.0;
    zK[1] = 0.0;
    zK[2] = 0.0;
    
    xK[0] = 0.0;
    xK[1] = 0.0;
    xK[2] = 0.0;
}

void Drone::saveGainsToStorage() {
    preferences.begin("drone", false);
    
    preferences.putDouble("yKp", yK[0]);
    preferences.putDouble("yKi", yK[1]);
    preferences.putDouble("yKd", yK[2]);
    
    preferences.putDouble("zKp", zK[0]);
    preferences.putDouble("zKi", zK[1]);
    preferences.putDouble("zKd", zK[2]);
    
    preferences.putDouble("xKp", xK[0]);
    preferences.putDouble("xKi", xK[1]);
    preferences.putDouble("xKd", xK[2]);
    
    preferences.end();
}

void Drone::printCalibrationStatus() {
    if (!sensor || !USE_IMU) return;
    
    uint8_t system, gyro, accel, mag;
    sensor->getCalibrationStatus(system, gyro, accel, mag);
    
    Serial.println("=== Calibration Status ===");
    Serial.print("System: "); Serial.print(system, DEC);
    Serial.print(" Gyro: "); Serial.print(gyro, DEC);
    Serial.print(" Accel: "); Serial.print(accel, DEC);
    Serial.print(" Mag: "); Serial.println(mag, DEC);
}