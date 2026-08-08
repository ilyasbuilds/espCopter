#include "Pid.h"

DPID::DPID(double* in, double* out, double* kp, double* ki, double* kd){
	//Ignoring ki for now
	k[0] = kp;
	k[1] = ki;
	k[2] = kd;
	
	kiError = 0;
	input = in;
	output = out;
}

void DPID::setSetpoint(double newSet){
	setpoint = newSet;
}

void DPID::calculate(){
	double error = *input - setpoint;
	
	// Normalize error for yaw (-180 to +180)
	if(error > 180) error -= 360;
	else if(error < -180) error += 360;
	
	currentTimeStamp = micros();
	timeElapsed = currentTimeStamp - lastTimeStamp;

	// Lock to 400Hz (2.5ms period)
	if(timeElapsed <= 2500) return; // Skip if less than 2.5ms has passed
	//timeElapsed = 2500; // Force constant time step at 400Hz
	
	if(*k[1] == 0){
		kiError = 0;
	}else{
		kiError += (*k[1] * error * timeElapsed);
		if(kiError > max) kiError = max;
		else if(kiError < -max) kiError = -max;
	}

	
	// Calculate D term with no filtering
	double dTerm = 0;
	if(timeElapsed > 0) {
		dTerm = (error - lastError) / timeElapsed;
	}
	
	double tempOutput = (*k[0] * error) + kiError - (*k[2] * dTerm);
	
	// Output limiting
	if(tempOutput > max) tempOutput = max;
	else if(tempOutput < -max) tempOutput = -max;
	
	*output = tempOutput;
	lastError = error;
	lastTimeStamp = currentTimeStamp;
}



