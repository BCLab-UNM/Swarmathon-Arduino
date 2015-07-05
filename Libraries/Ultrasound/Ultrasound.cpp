#include <Ultrasound.h>

/**
 *	Constructor arg is single signal pin for transmitting and receiving US pulse
 **/
Ultrasound::Ultrasound(byte signalPin) {
    _signalPin = signalPin;
}

/**
 *	Returns 'float' in cm of distance from Ping))) ultrasonic rangefinder to object
 **/
float Ultrasound::distance() {
    long duration;
    
    //Transmit pulse
    pinMode(_signalPin, OUTPUT);
    digitalWrite(_signalPin, LOW);
    delayMicroseconds(2);
    digitalWrite(_signalPin, HIGH);
    delayMicroseconds(5);
    digitalWrite(_signalPin, LOW);
    
    //Receive pulse
    pinMode(_signalPin, INPUT);
    duration = pulseIn(_signalPin, HIGH);
    
    //Convert time to distance and return
    return microsecondsToCentimeters(duration);
}

/**
 * Convert microseconds to cm assuming speed of sound at an elevation of 5000 ft is 31.2 microseconds per cm
 **/
float Ultrasound::microsecondsToCentimeters(long microseconds) {
    // Divide by 2 because pulse travels out and back
    return microseconds / 31.2 / 2.0;
}