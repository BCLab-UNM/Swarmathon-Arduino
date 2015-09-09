//////////////////////////
////Required Libraries////
//////////////////////////

//Built-in Arduino libraries
#include <Wire.h>

//Custom libraries located in Swarmie-Arduino repo
#include <L3G.h>
#include <LPS.h>
#include <LSM303.h>
#include <Movement.h>
#include <Ultrasound.h>


////////////////
////Settings////
////////////////

//Movement (VNH5019 Motor Driver Carrier)
byte rightDirectionA = 8; //"clockwise" input
byte rightDirectionB = 9; //"counterclockwise" input
byte rightSpeedPin = 10; //PWM input
byte leftDirectionA = 11; //"clockwise" input
byte leftDirectionB = 12; //"counterclockwise" input
byte leftSpeedPin = 13; //PWM input

//Serial (USB <--> Intel NUC)
String rxBuffer;
String txBuffer;

//Ultrasound (Ping))))
byte leftSignal = 4;
byte centerSignal = 5;
byte rightSignal = 6;


////////////////////////////
////Class Instantiations////
////////////////////////////

L3G gyroscope;
LSM303 magnetometer_accelerometer;
LPS pressure;
Movement move = Movement(rightSpeedPin, rightDirectionA, rightDirectionB, leftSpeedPin, leftDirectionA, leftDirectionB);
Ultrasound leftUS = Ultrasound(leftSignal);
Ultrasound centerUS = Ultrasound(centerSignal);
Ultrasound rightUS = Ultrasound(rightSignal);


/////////////
////Setup////
/////////////

void setup()
{
  Serial.begin(115200);
  while (!Serial) {} //wait for Serial to complete initialization before moving on

  Wire.begin();

  gyroscope.init();
  gyroscope.enableDefault();
  magnetometer_accelerometer.init();
  magnetometer_accelerometer.enableDefault();
  pressure.init();
  pressure.enableDefault();

  rxBuffer = "";
}


/////////////////
////Main Loop////
/////////////////

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == ',' || c == '\n') {
      parse();
      rxBuffer = "";
    }
    else if (c > 0) {
      rxBuffer += c;
    }
  }
}


////////////////////////
//Parse receive buffer//
////////////////////////

void parse() {
  if (rxBuffer == "m") {
    int speed = Serial.parseInt();
    if (speed >= 0) {
      move.forward(speed, speed);
    }
    else {
      move.backward(abs(speed), abs(speed));
    }
  }
  else if (rxBuffer == "t") {
    int speed = Serial.parseInt();
    if (speed >= 0) {
      move.rotateLeft(speed);
    }
    else {
      move.rotateRight(abs(speed));
    }
  }
  else if (rxBuffer == "s") {
    move.stop();
  }
  else if (rxBuffer == "d") {
    update();
    Serial.println(txBuffer);
  }
}


//////////////////////////
//Update transmit buffer//
//////////////////////////

void update() {
  //Update current sensor values
  gyroscope.read();
  magnetometer_accelerometer.read();

  txBuffer = String(magnetometer_accelerometer.heading()) + "," +
             String(magnetometer_accelerometer.a.x) + "," +
             String(magnetometer_accelerometer.a.y) + "," +
             String(magnetometer_accelerometer.a.z) + "," +
             String(gyroscope.g.x) + "," +
             String(gyroscope.g.y) + "," +
             String(gyroscope.g.z) + "," +
             String(leftUS.distance()) + "," +
             String(centerUS.distance()) + "," +
             String(rightUS.distance());
}
