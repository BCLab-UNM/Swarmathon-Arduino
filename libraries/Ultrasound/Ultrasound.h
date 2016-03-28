#ifndef Ultrasound_h
#define Ultrasound_h

#include "Arduino.h"

class Ultrasound {
	public:
		//Constructors
		Ultrasound(byte signalPin);
		
		//Legacy Functions
		float distance();
		float microsecondsToCentimeters(long microseconds);
	
	private:
		//Variables
        byte _signalPin;
};

#endif