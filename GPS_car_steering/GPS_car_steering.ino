/*
   Created by ArduinoGetStarted.com

   This example code is in the public domain

   Tutorial page: https://arduinogetstarted.com/tutorials/arduino-gps
*/

#include <PWMServo.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <LSM303.h>

const int RXPin = 4, TXPin = 3;
const uint32_t GPSBaud = 9600; //Default baud of NEO-6M is 9600

int motorPwmVal = 50;
const int motorPin = 11;

float targetCourse;
float currentCourse;


TinyGPSPlus gps; // the TinyGPS++ object
SoftwareSerial gpsSerial(RXPin, TXPin); // the serial interface to the GPS device

PWMServo steerServo;

LSM303 compass;

const double target_lat = -31.98015;
const double target_lon = 115.81791;

void setup() {
  Serial.begin(9600);
  gpsSerial.begin(GPSBaud);

  steerServo.attach(SERVO_PIN_A); //pin 9 for arduino nano (handled by library)

  Wire.begin();
  compass.init();
  compass.enableDefault();
  compass.m_min = (LSM303::vector<int16_t>) {
    -32767, -32767, -32767
  };
  compass.m_max = (LSM303::vector<int16_t>) {
    +32767, +32767, +32767
  };
}

void loop() {



  if (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      if (gps.location.isValid()) {
        double latitude = gps.location.lat();
        double longitude = gps.location.lng();
        unsigned long distanceToTarget = TinyGPSPlus::distanceBetween(latitude, longitude, target_lat, target_lon);

        //        Serial.print(F("- latitude: "));
        //        Serial.println(latitude, 8);
        //
        //        Serial.print(F("- longitude: "));
        //        Serial.println(longitude, 8);

        //        Serial.print(F("- distance to target: "));
        //        Serial.println(distanceKm, 8);

        //        Serial.print(F("speed in km/h: "));
        //        Serial.println(gps.speed.kmph());

        Serial.print("course to target: ");
        targetCourse =TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), target_lat, target_lon);
        Serial.print(targetCourse);

        compass.read();
        currentCourse = compass.heading();
        Serial.print(" Current Course: ");
        Serial.println(currentCourse);


        //        Serial.print(F("current course: "));
        //        Serial.println(gps.course.deg());

        if (currentCourse < targetCourse)
        {
          turnLeft();
//          delay(500);
//          goStraight();
        }

        else if (currentCourse > targetCourse)
        {
          turnRight();
//          delay(500);
//          goStraight();
        }

        else if (abs((currentCourse - targetCourse)) < 10)
        {
          goStraight();
        }

        if (distanceToTarget < 10)
        {
          motorPwmVal = 100;
        }

        else if (distanceToTarget < 3)
        {
          motorPwmVal = 50;
        }
        else if (distanceToTarget == 0)
        {
          stopMoving();
        }

      } else {
        Serial.println(F("- location: INVALID"));
      }

      Serial.println();
    }
  }

  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println(F("No GPS data received: check wiring"));
}

void goStraight()
{
  steerServo.write(90);
  //Serial.println("Go straight");
}

void turnSlightLeft()
{
  steerServo.write(80);
  //Serial.println("Turn Left SLightly");
}

void turnSlightRight()
{
  steerServo.write(100);
  //Serial.println("Turn Right Slightly");
}

void turnLeft()
{
  steerServo.write(70);
  //Serial.println("Turn Left");
}

void turnRight()
{
  steerServo.write(110);
  //Serial.println("Turn Right");
}

void startMoving()
{
  analogWrite(motorPin, motorPwmVal);
  //Serial.println("Start Moving");
}

void stopMoving()
{
  analogWrite(motorPin, 0);
  //Serial.println("Stop Moving");
}
