//GPS Car

#include <PWMServo.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <LSM303.h>

//Initialising variables
const int RXPin = 4, TXPin = 3;  //RX and TX for NEO-6M
const uint32_t GPSBaud = 9600;   //Default baud of NEO-6M is 9600

int motorPwmVal = 70;     //Motor speed
const int motorPin = 11;  //Pin controns motor mosfet

int targetCourse;   //Course from car to target
int currentCourse;  //Course measured by compass

bool A7_state;                 //State of A7, required since A7 cannot be a digital input
bool wayPointReached = false;  // Confirms that the car has arrived at target co-ordinates
bool finished = false;


TinyGPSPlus gps;                         // the TinyGPS++ object
SoftwareSerial gpsSerial(RXPin, TXPin);  // the serial interface to the GPS device
PWMServo steerServo;                     //Servo object
LSM303 compass;                          //Compass object

double final_target_lat;
double final_target_lon;

double initial_target_lat = -31.979616;
double initial_target_lon = 115.817880;

double target_lat;
double target_lon;

void setup() {
  Serial.begin(9600);
  gpsSerial.begin(GPSBaud);

  pinMode(13, OUTPUT);

  digitalWrite(13, HIGH);

  steerServo.attach(SERVO_PIN_A);  //pin 9 for arduino nano (handled by library)

  Wire.begin();
  compass.init();
  compass.enableDefault();
  compass.m_min = (LSM303::vector<int16_t>){ -32767, -32767, -32767 };
  compass.m_max = (LSM303::vector<int16_t>){ +32767, +32767, +32767 };

  while (!((gpsSerial.available() > 0) && (gps.location.isValid()))) {
    delay(500);
  }

  final_target_lat = gps.location.lat();
  final_target_lon = gps.location.lng();

  digitalWrite(13, LOW);
  Serial.println(final_target_lat, 8);
  Serial.println(final_target_lon, 8);

  target_lat = initial_target_lat;
  target_lon = initial_target_lon;
}

void goStraight() {
  steerServo.write(90);
}

void turnSlightLeft() {
  steerServo.write(82);
}

void turnSlightRight() {
  steerServo.write(103);
}

void turnLeft() {
  steerServo.write(67);
}

void turnRight() {
  steerServo.write(118);
}

void startMoving() {
  analogWrite(motorPin, motorPwmVal);
}

void stopMoving() {
  analogWrite(motorPin, 0);
}


void loop() {

  if (analogRead(A7) > 500) {
    A7_state = HIGH;
    startMoving();
  }

  else {
    A7_state = LOW;
    stopMoving();
  }

  if ((gpsSerial.available() > 0) && (gps.location.isValid())) {
    double latitude = gps.location.lat();
    double longitude = gps.location.lng();
    unsigned long distanceToTarget = TinyGPSPlus::distanceBetween(latitude, longitude, target_lat, target_lon);
    targetCourse = TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), target_lat, target_lon);

    compass.read();
    currentCourse = compass.heading();

    float errorCourse = (360 + targetCourse - currentCourse) % 360;

    if (errorCourse >= 345 || errorCourse < 15) {
      goStraight();
    }

    else if (errorCourse >= 315 && errorCourse < 345) {
      turnSlightLeft();
    }

    else if (errorCourse >= 15 && errorCourse < 45) {
      turnSlightRight();
    }

    else if (errorCourse >= 180 && errorCourse < 315) {
      turnLeft();
    } else if (errorCourse >= 45 && errorCourse < 180) {
      turnRight();
    }

    delay(1000);

    if (distanceToTarget <= 4) {
      motorPwmVal = 30;
    }

    if (distanceToTarget <= 1 && wayPointReached) {
      while (1) {
        stopMoving();
      }
    }

    if (distanceToTarget <= 1) {
      wayPointReached = true;
      target_lat = final_target_lat;
      target_lon = final_target_lon;
      stopMoving();
      motorPwmVal = 70;
      delay(3000);
      startMoving();
    }
  }
}