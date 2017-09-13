#ifndef Odometry_h
#define Odometry_h

#include "Arduino.h"

class Odometry {
public:
    //Constructors
    Odometry(byte rightEncoderAPin, byte rightEncoderBPin, byte leftEncoderAPin, byte leftEncoderBPin);
    
    //Functions
    void update();
    
    //Variables
    int16_t right, left;
    unsigned long clock;
};

#endif
