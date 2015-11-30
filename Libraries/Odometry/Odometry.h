#ifndef Odometry_h
#define Odometry_h

#include "Arduino.h"

class Odometry {
public:
    //Constructors
    Odometry(byte rightEncoderAPin, byte rightEncoderBPin, byte leftEncoderAPin, byte leftEncoderBPin, float wheelBase, float wheelDiameter, int cpr);
    
    //Functions
    void update();
    
    //Variables
    float x,y;

private:
    //Variables
    float _wheelBase, _wheelDiameter;
    int _cpr;
};

#endif