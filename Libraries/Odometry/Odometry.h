#ifndef Odometry_h
#define Odometry_h

#include "Arduino.h"

class Odometry {
public:
    //Constructors
    Odometry(byte rightEncoderAPin, byte rightEncoderBPin, byte leftEncoderAPin, byte leftEncoderBPin, float wheelBase, float wheelDiameter, int cpr);
    
    //Functions
    void update();
    
private:
    //Variables
    float _wheelBase, _wheelDiameter, _cpr;
    float x,y;
};

#endif