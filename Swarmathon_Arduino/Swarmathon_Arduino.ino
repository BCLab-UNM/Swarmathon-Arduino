//////////////////////////
////Required Libraries////
//////////////////////////

//Built-in Arduino libraries
#include <Servo.h>
#include <Wire.h>

//Custom libraries located in Swarmathon-Arduino repo
#include <L3G.h>
#include <LPS.h>
#include <LSM303.h>
#include <Movement.h>
#include <Odometry.h>
#include <Ultrasound.h>


////////////////
////Settings////
////////////////

//Gripper (HS-485HB Servo)
byte fingersPin = 9;
byte wristPin = 12;

//Movement (VNH5019 Motor Driver Carrier)
byte rightDirectionA = A3; //"clockwise" input
byte rightDirectionB = A2; //"counterclockwise" input
byte rightSpeedPin = 11; //PWM input
byte leftDirectionA = A5; //"clockwise" input
byte leftDirectionB = A4; //"counterclockwise" input
byte leftSpeedPin = 10; //PWM input
bool turning = false; //disables encoders when rotating

//Odometry (8400 CPR Encoder)
byte rightEncoderA = 7;
byte rightEncoderB = 8;
byte leftEncoderA = 0;
byte leftEncoderB = 1;
float wheelBase = 27.8; //distance between left and right wheels (in cm)
float wheelDiameter = 12.2; //diameter of wheel (in cm)
int cpr = 8400; //"cycles per revolution" -- number of encoder increments per one wheel revolution

//Serial (USB <--> Intel NUC)
String rxBuffer;
String txBuffer;
unsigned long watchdogTimer = 1000; //fail-safe in case of communication link failure (in ms)
unsigned long lastCommTime = 0; //time of last communication from NUC (in ms)

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
Odometry odom = Odometry(rightEncoderA, rightEncoderB, leftEncoderA, leftEncoderB, wheelBase, wheelDiameter, cpr);
Servo fingers;
Servo wrist;
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
  magnetometer_accelerometer.m_min = (LSM303::vector<int16_t>){ -2523, -2802, -2688};
  magnetometer_accelerometer.m_max = (LSM303::vector<int16_t>){ +2710, +1151, +1549};
  pressure.init();
  pressure.enableDefault();

  fingers.attach(fingersPin,647,1472);
  fingers.write(0);
  wrist.attach(wristPin,750,2400);
  wrist.write(0);

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
      lastCommTime = millis();
    }
    else if (c > 0) {
      rxBuffer += c;
    }
  }
  if (millis() - lastCommTime > watchdogTimer) {
    move.stop();
  }
}


////////////////////////
//Parse receive buffer//
////////////////////////

void parse() {
  if (rxBuffer == "m") {
    turning = false;
    int speed = Serial.parseInt();
    if (speed >= 0) {
      move.forward(speed, speed);
    }
    else {
      move.backward(abs(speed), abs(speed));
    }
  }
  else if (rxBuffer == "t") {
    turning = true;
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
  else if (rxBuffer == "f") {
    int angle = Serial.parseInt();
    if (angle == -1) {
      fingers.write(fingers.read()+1);
    }
    else if (angle == -2) {
      fingers.write(fingers.read()-1);
    }
    else {
      fingers.write(angle);
    }
  }
  else if (rxBuffer == "w") {
    int angle = Serial.parseInt();
    if (angle == -1) {
      wrist.write(wrist.read()+1);
    }
    else if (angle == -2) {
      wrist.write(wrist.read()-1);
    }
    else {
      wrist.write(angle);
    }
  }
}


//////////////////////////
//Update transmit buffer//
//////////////////////////

void update() {
  //Update current sensor values
  gyroscope.read();
  magnetometer_accelerometer.read();

  //Collect updated values
  LSM303::vector<int16_t> acc = magnetometer_accelerometer.a;
  L3G::vector<int16_t> gyro = gyroscope.g;
  LSM303::vector<int16_t> mag = magnetometer_accelerometer.m;

  //Convert accelerometer digits to milligravities, then to gravities, and finally to meters per second squared
  LSM303::vector<float> linear_acceleration = {acc.y*0.061/1000*9.81, -acc.x*0.061/1000*9.81, acc.z*0.061/1000*9.81};

  //Convert gyroscope digits to millidegrees per second, then to degrees per second, and finally to radians per second
  L3G::vector<float> angular_velocity = {gyro.y*8.75/1000*(PI/180), -gyro.x*8.75/1000*(PI/180), gyro.z*8.75/1000*(PI/180)};

  //Combine normalized magnetometer and accelerometer digits to produce Euler angles, i.e. pitch, roll, and yaw
  LSM303::vector<float> orientation = {(float)mag.x, (float)mag.y, (float)mag.z};
  orientation.x -= (magnetometer_accelerometer.m_min.x + magnetometer_accelerometer.m_max.x) / 2;
  orientation.y -= (magnetometer_accelerometer.m_min.y + magnetometer_accelerometer.m_max.y) / 2;
  orientation.z -= (magnetometer_accelerometer.m_min.z + magnetometer_accelerometer.m_max.z) / 2;
  LSM303::vector_normalize(&orientation);
  float roll = atan2(linear_acceleration.y, sqrt(pow(linear_acceleration.x,2) + pow(linear_acceleration.z,2)));
  float pitch = -atan2(linear_acceleration.x, sqrt(pow(linear_acceleration.y,2) + pow(linear_acceleration.z,2)));
  float yaw = atan2(-orientation.y*cos(roll) + orientation.z*sin(roll), orientation.x*cos(pitch) + orientation.y*sin(pitch)*sin(roll) + orientation.z*sin(pitch)*cos(roll)) + PI;
  orientation = {roll, pitch, yaw};

  odom.update(orientation.z);
  if (turning) {
    odom.x = 0;
    odom.y = 0;
  }

  //Append data to buffer
  txBuffer = String(linear_acceleration.x) + "," +
             String(linear_acceleration.y) + "," +
             String(linear_acceleration.z) + "," +
             String(angular_velocity.x) + "," +
             String(angular_velocity.y) + "," +
             String(angular_velocity.z) + "," +
             String(orientation.x) + "," +
             String(orientation.y) + "," +
             String(orientation.z) + "," +
             String(odom.x) + "," +
             String(odom.y) + "," +
             String(odom.theta) + "," +
             String(odom.vx) + "," +
             String(odom.vy) + "," +
             String(odom.vtheta) + "," +
             String(leftUS.distance()) + "," +
             String(centerUS.distance()) + "," +
             String(rightUS.distance());
}
