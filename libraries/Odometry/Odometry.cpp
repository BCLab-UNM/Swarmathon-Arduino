#include <Odometry.h>

//Global Functions
void rightEncoderAChange();
void setupPinChangeInterrupt(byte pin);
void leftEncoderAChange();
void leftEncoderBChange();

//Global Variables
int rightEncoderCounter;
int leftEncoderCounter;
byte _rightEncoderAPin;
byte _rightEncoderBPin;
byte _leftEncoderAPin;
byte _leftEncoderBPin;

/**
 *	Constructor arg are pins for channels A and B on right and left encoders
 **/
Odometry::Odometry(byte rightEncoderAPin, byte rightEncoderBPin, byte leftEncoderAPin, byte leftEncoderBPin) {
    pinMode(rightEncoderAPin, INPUT);
    pinMode(rightEncoderBPin, INPUT);
    pinMode(leftEncoderAPin, INPUT);
    pinMode(leftEncoderBPin, INPUT);
    digitalWrite(leftEncoderBPin, HIGH);
    rightEncoderCounter = 0.;
    leftEncoderCounter = 0.;
    attachInterrupt(digitalPinToInterrupt(rightEncoderAPin), rightEncoderAChange, CHANGE);
    setupPinChangeInterrupt(rightEncoderBPin);
    attachInterrupt(digitalPinToInterrupt(leftEncoderAPin), leftEncoderAChange, CHANGE);
    attachInterrupt(digitalPinToInterrupt(leftEncoderBPin), leftEncoderBChange, CHANGE);
    _rightEncoderAPin = rightEncoderAPin;
    _rightEncoderBPin = rightEncoderBPin;
    _leftEncoderAPin = leftEncoderAPin;
    _leftEncoderBPin = leftEncoderBPin;
    right = 0;
    left = 0;
    clock = millis();
}

void Odometry::update() {
    // Record ticks.
    left = leftEncoderCounter;
    right = rightEncoderCounter;

    leftEncoderCounter = 0;
    rightEncoderCounter = 0;
    
    // Store timestamp
    clock = millis();
}

void rightEncoderAChange() {
    bool rightEncoderAStatus = digitalRead(_rightEncoderAPin);
    bool rightEncoderBStatus = digitalRead(_rightEncoderBPin);
    if (((rightEncoderAStatus == HIGH) && (rightEncoderBStatus == LOW)) || ((rightEncoderAStatus == LOW) && (rightEncoderBStatus == HIGH))) {
        rightEncoderCounter++;
    }
    else {
        rightEncoderCounter--;
    }
}

ISR (PCINT0_vect) {
    bool rightEncoderAStatus = digitalRead(_rightEncoderAPin);
    bool rightEncoderBStatus = digitalRead(_rightEncoderBPin);
    if (((rightEncoderAStatus == HIGH) && (rightEncoderBStatus == HIGH)) || ((rightEncoderAStatus == LOW) && (rightEncoderBStatus == LOW))) {
        rightEncoderCounter++;
    }
    else {
        rightEncoderCounter--;
    }
}

void leftEncoderAChange() {
    bool leftEncoderAStatus = digitalRead(_leftEncoderAPin);
    bool leftEncoderBStatus = digitalRead(_leftEncoderBPin);
    if (((leftEncoderAStatus == HIGH) && (leftEncoderBStatus == HIGH)) || ((leftEncoderAStatus == LOW) && (leftEncoderBStatus == LOW))) {
        leftEncoderCounter++;
    }
    else {
        leftEncoderCounter--;
    }
}

void leftEncoderBChange() {
    bool leftEncoderAStatus = digitalRead(_leftEncoderAPin);
    bool leftEncoderBStatus = digitalRead(_leftEncoderBPin);
    if (((leftEncoderAStatus == HIGH) && (leftEncoderBStatus == LOW)) || ((leftEncoderAStatus == LOW) && (leftEncoderBStatus == HIGH))) {
        leftEncoderCounter++;
    }
    else {
        leftEncoderCounter--;
    }
}

void setupPinChangeInterrupt(byte pin) {
    *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
    PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
    PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}
