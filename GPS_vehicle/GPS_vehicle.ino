//
//
//
//Enter your taregt latitude and longitude values here

double initial_target_lat = <Enter Latitude Here>;
double initial_target_lon = <Enter Longitude Here>;

//Do not edit any of the below code
//
//
//



//GPS Car

#include <PWMServo.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <LSM303.h>

//Initialising variables
const int RXPin = 4, TXPin = 3;  //RX and TX for NEO-6M
const uint32_t GPSBaud = 9600;   //Default baud of NEO-6M is 9600

int motorPwmVal = 80;     //Motor speed
const int motorPin = 11;  //Pin controns motor mosfet

int targetCourse;   //Course from car to target
int currentCourse;  //Course measured by compass

bool A7_state;                 //State of A7, required since A7 cannot be a digital input
bool wayPointReached = false;  // Confirms that the car has arrived at target co-ordinates
bool finished = false;

bool final_assigned = false;

TinyGPSPlus gps;                         // the TinyGPS++ object
SoftwareSerial gpsSerial(RXPin, TXPin);  // the serial interface to the GPS device
PWMServo steerServo;                     //Servo object
LSM303 compass;                          //Compass object

double final_target_lat;
double final_target_lon;

double target_lat;
double target_lon;

uint32_t then = 0;

void setup() {
  Serial.begin(9600);
  gpsSerial.begin(GPSBaud);

  pinMode(13, OUTPUT);

  //digitalWrite(13, HIGH);

  steerServo.attach(SERVO_PIN_A);  //pin 9 for arduino nano (handled by library)

  Wire.begin();
  compass.init();
  compass.enableDefault();
  compass.m_min = (LSM303::vector<int16_t>) {
    -32767, -32767, -32767
  };
  compass.m_max = (LSM303::vector<int16_t>) {
    +32767, +32767, +32767
  };

  //  while (gps.charsProcessed() < 10)
  //  {
  //    //do nothing
  //    Serial.println("doing nothing");
  //  }

  //steering test

  //    goStraight();
  //    delay(1000);
  //    turnSlightLeft();
  //    delay(1000);
  //    turnLeft();
  //    delay(1000);
  //    goStraight();
  //    delay(1000);
  //    turnRight();
  //    delay(1000);
  //    turnSlightRight();
  //    delay(1000);
  //    goStraight();

  target_lat = initial_target_lat;
  target_lon = initial_target_lon;
}




void loop() {



  while (final_assigned == false)
  {
    digitalWrite(13, HIGH);
    if (gpsSerial.available() > 0) {
      if (gps.encode(gpsSerial.read())) {
        if (gps.location.isValid())
        {
          final_target_lat = gps.location.lat();
          Serial.println(final_target_lat, 8);
          final_target_lon = gps.location.lng();
          Serial.println(final_target_lon, 8);
        }
      }
    }

    if (millis() > 10000)
    {
      final_assigned = true;
    }
  }

  digitalWrite(13, LOW);

  if (analogRead(A7) > 700) {
    A7_state = HIGH;
  }

  else {
    A7_state = LOW;
    analogWrite(motorPin, 0);
  }

  if (A7_state == HIGH) {
    startMoving();
  }

  else {
    stopMoving();
  }

  if (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      if (gps.location.isValid()) {
        double latitude = gps.location.lat();
        double longitude = gps.location.lng();
        unsigned long distanceToTarget = TinyGPSPlus::distanceBetween(latitude, longitude, target_lat, target_lon);

        //        Serial.print(F("- latitude: "));
        //        Serial.println(latitude, 8);

        //        Serial.print(F("- longitude: "));
        //        Serial.println(longitude, 8);

        //        Serial.print(F("- distance to target: "));
        //        Serial.println(distanceKm, 8);

        //        Serial.print(F("speed in km/h: "));
        //        Serial.println(gps.speed.kmph());

        //        Serial.print("course to target: ");
        targetCourse = TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), target_lat, target_lon);
        //        Serial.print(targetCourse);

        compass.read();
        currentCourse = compass.heading();
        //        Serial.print(" Current Course: ");
        //        Serial.println(currentCourse);

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

        //delay(500);

        if (distanceToTarget <= 10)
        {
          analogWrite(motorPin, 50);
        }

        if (distanceToTarget <= 2)
        {
          wayPointReached = true;
          target_lat = final_target_lat;
          target_lon = final_target_lon;
          stopMoving();
          delay(3000);
          startMoving();
        }

        if (wayPointReached == false)
        {
          then = millis();
        }

        if ((millis() - then > 20000) && (distanceToTarget <= 2))
        {
          finished == true;
        }

        while (finished == true)
        {
          stopMoving();
        }

      }
      else
      {
        Serial.println(F("- location: INVALID"));
      }
    }
  }
  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS data received: check wiring"));
  }
}

void goStraight() {
  steerServo.write(90);
  //Serial.println("Go straight");
}

void turnSlightLeft() {
  steerServo.write(84);
  //Serial.println("Turn Left SLightly");
}

void turnSlightRight() {
  steerServo.write(103);
  //Serial.println("Turn Right Slightly");
}

void turnLeft() {
  steerServo.write(67);
  //Serial.println("Turn Left");
}

void turnRight() {
  steerServo.write(116);
  //Serial.println("Turn Right");
}

void startMoving() {
  analogWrite(motorPin, motorPwmVal);
  //Serial.println("Start Moving");
}

void stopMoving() {
  analogWrite(motorPin, 0);
  //Serial.println("Stop Moving");
}
