/*
*	Odometer.cpp - Handles the odometer (speed encoder) sensor of the Smartcar.
*	Version: 0.2
*	Author: Dimitris Platis (based on the Smartcar project by Team Pegasus)
* 	License: GNU GPL v3 http://www.gnu.org/licenses/gpl-3.0.html
*/
#include "CaroloCup.h"

/* ---- ODOMETER ---- */
void updateCounter1(); //ISR for the odometer
void updateCounter2();
// Macro to convert from odometer pulses to centimeters. (ca. 33 pulses per meter, that's why we multiply by 3 to find value in cm)
#define PulsesToCentimeters(pulses) (pulses * 3.3)

const float MILLIMETERS_PER_PULSE = 30.3; //we divide 1000 by how many pulses per meter we get
volatile float _measuredSpeed[2] = {0};
volatile unsigned long _previousPulseTime[2] = {0};
volatile unsigned long _pulseCounter[2] = {0};
static unsigned short odometers = 0;

Odometer::Odometer(){
_odometerID = odometers++;
}

int Odometer::attach(unsigned short odometerPin){
	_odometerInterruptPin = digitalPinToInterrupt(odometerPin);
	if (_odometerInterruptPin != NOT_AN_INTERRUPT){
		if (!_odometerID){
			attachInterrupt(_odometerInterruptPin, updateCounter1, RISING);
		}else if (_odometerID == 1){
			attachInterrupt(_odometerInterruptPin, updateCounter2, RISING);
		}else{
			return -1; //too many encoders attached
		}
		return 1; //everything went as it should
	}
	return 0; //invalid interrupt pin
}

void Odometer::begin(){
	_pulseCounter[_odometerID] = 0; //initialize the counter
}

unsigned long Odometer::getDistance(){
	return PulsesToCentimeters(_pulseCounter[_odometerID]);
}

void Odometer::detach(){
	_pulseCounter[_odometerID] = 0; //reinitialize the counter so if distance is calculated again, result will be 0 and not what was left from before
	detachInterrupt(_odometerInterruptPin);
}

float Odometer::getSpeed(){
	return _measuredSpeed[_odometerID];
}

void updateOdometerSpeed(unsigned short odometerID){ //calculates the speed for the given odometer
	unsigned long currentTime = millis();
	unsigned long dt = currentTime - _previousPulseTime[odometerID]; //in milliseconds
	if (dt){ //avoid division by 0 in case we had a very fast (less than 1 ms) pulse
		_measuredSpeed[odometerID] = MILLIMETERS_PER_PULSE / dt; //x & t are in milli scale, so the result can be in m/s
	}
	_previousPulseTime[odometerID] = currentTime;
}

void updateCounter1(){
	updateOdometerSpeed(0);
	_pulseCounter[0]++;
}
void updateCounter2(){
	updateOdometerSpeed(1);
	_pulseCounter[1]++;
}
