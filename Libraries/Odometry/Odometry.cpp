#include <Odometry.h>

//Global Functions
void rightEncoderARise();
void rightEncoderAFall();
void rightEncoderBRise();
void rightEncoderBFall();
void leftEncoderARise();
void leftEncoderAFall();
void leftEncoderBRise();
void leftEncoderBFall();

//Global Variables
bool rightEncoderAStatus;
bool rightEncoderBStatus;
bool leftEncoderAStatus;
bool leftEncoderBStatus;
int rightEncoderCounter;
int leftEncoderCounter;

/**
 *	Constructor arg are pins for channels A and B on right and left encoders
 **/
Odometry::Odometry(byte rightEncoderAPin, byte rightEncoderBPin, byte leftEncoderAPin, byte leftEncoderBPin, float wheelBase, float wheelDiameter, int cpr) {
    pinMode(rightEncoderAPin, OUTPUT);
    pinMode(rightEncoderBPin, OUTPUT);
    pinMode(leftEncoderAPin, OUTPUT);
    pinMode(leftEncoderBPin, OUTPUT);
    rightEncoderAStatus = false;
    rightEncoderBStatus = false;
    leftEncoderAStatus = false;
    leftEncoderBStatus = false;
    rightEncoderCounter = 0.;
    leftEncoderCounter = 0.;
    attachInterrupt(digitalPinToInterrupt(rightEncoderAPin), rightEncoderARise, RISING);
    attachInterrupt(digitalPinToInterrupt(rightEncoderAPin), rightEncoderAFall, FALLING);
    attachInterrupt(digitalPinToInterrupt(rightEncoderBPin), rightEncoderBRise, RISING);
    attachInterrupt(digitalPinToInterrupt(rightEncoderBPin), rightEncoderBFall, FALLING);
    attachInterrupt(digitalPinToInterrupt(leftEncoderAPin), leftEncoderARise, RISING);
    attachInterrupt(digitalPinToInterrupt(leftEncoderAPin), leftEncoderAFall, FALLING);
    attachInterrupt(digitalPinToInterrupt(leftEncoderBPin), leftEncoderBRise, RISING);
    attachInterrupt(digitalPinToInterrupt(leftEncoderBPin), leftEncoderBFall, FALLING);
    _wheelBase = wheelBase;
    _wheelDiameter = wheelDiameter;
    _cpr = cpr;
}

void Odometry::update() {
    //Calculate linear distance that each wheel has traveled
    float rightWheelDistance = rightEncoderCounter / _cpr * _wheelDiameter * PI;
    float leftWheelDistance = leftWheelDistance / _cpr * _wheelDiameter * PI;
    
    //Calculate angle that robot has turned
    float theta = (rightWheelDistance - leftWheelDistance) / _wheelBase;
    
    //Decompose linear distance into its component values
    float meanWheelDistance = (rightWheelDistance + leftWheelDistance) / 2;
    x = meanWheelDistance * sin(theta);
    y = meanWheelDistance * cos(theta);
    
    //Reset counters
    rightEncoderCounter = 0;
    leftEncoderCounter = 0;
}

void rightEncoderARise() {
    if (rightEncoderBStatus == true) {
        rightEncoderCounter++;
    }
    else {
        rightEncoderCounter--;
    }
}

void rightEncoderAFall() {
    if (rightEncoderBStatus == false) {
        rightEncoderCounter++;
    }
    else {
        rightEncoderCounter--;
    }
}

void rightEncoderBRise() {
    if (rightEncoderAStatus == false) {
        rightEncoderCounter++;
    }
    else {
        rightEncoderCounter--;
    }
}

void rightEncoderBFall() {
    if (rightEncoderAStatus == true) {
        rightEncoderCounter++;
    }
    else {
        rightEncoderCounter--;
    }
}

void leftEncoderARise() {
    if (leftEncoderBStatus == false) {
        leftEncoderCounter++;
    }
    else {
        leftEncoderCounter--;
    }
}

void leftEncoderAFall() {
    if (leftEncoderBStatus == true) {
        leftEncoderCounter++;
    }
    else {
        leftEncoderCounter--;
    }
}

void leftEncoderBRise() {
    if (leftEncoderAStatus == true) {
        leftEncoderCounter++;
    }
    else {
        leftEncoderCounter--;
    }
}

void leftEncoderBFall() {
    if (leftEncoderAStatus = false) {
        leftEncoderCounter++;
    }
    else {
        leftEncoderCounter--;
    }
}