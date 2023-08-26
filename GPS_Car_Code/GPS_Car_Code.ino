#include <TinyGPS++.h> //Libary handles gps location
#include <SoftwareSerial.h> //Communication between Nano and GPS module

const int RXPin = 4, TXPin = 3; //Setting transmit and recieve pins for GPS module
const uint32_t GPSBaud = 9600; //Commmunication speed between GPS module and Nano

const int servo_pin = 11;
const int gate_pin = 6;

int pwm_val = 150;
const unsigned int MAX_MESSAGE_LENGTH = 12;

const bool display_coordinates = true;

double last_latitude = 0;
double last_longitude = 0;
double last_distance_to_target;
double last_bearing = 0;
double last_non_zero_current_bearing = 0;

bool correct_lat;
bool correct_lon;

bool initial_coord_assigned = false;

double initial_latitude;
double initial_longitude;

TinyGPSPlus gps; //Initialise GPS
SoftwareSerial gpsSerial(RXPin, TXPin); //Assinging transmit and receive pins for GPS module

const double target_lat = -31.999661; //Global variables that represent target coordinates for GPS car
const double target_lon = 115.904465; //Global variables that represent target coordinates for GPS car

void setup()
{
  Serial.begin(9600); //Begin serial communication through usb
  gpsSerial.begin(GPSBaud); //Begin Serial communication with GPS module

  pinMode(servo_pin, OUTPUT);
  analogWrite(servo_pin, pwm_val);
  pinMode(gate_pin, OUTPUT);
}

void loop() {

  if (gpsSerial.available() > 0) //Checks if GPS location is avaliable
  {
    if (gps.encode(gpsSerial.read())) //Encodes GPS data
    {
      if (gps.location.isValid()) //
      {
        double latitude = gps.location.lat(); //Assigns Latitude value
        double longitude = gps.location.lng(); //Assigns Longitude Value
        //unsigned long distanceM = TinyGPSPlus::distanceBetween(latitude, longitude, target_lat, target_lon); //Calculates distance to target with library function

        if (initial_coord_assigned == false)
        {
          initial_latitude = latitude;
          initial_longitude = longitude;

          initial_coord_assigned = true;
        }

        //                    Serial.print(latitude,8);
        //                    Serial.print("  ");
        //                    Serial.print(target_lat,8);
        //                    Serial.print("  ");
        //                    Serial.print(longitude,8);
        //                    Serial.print("  ");
        //                    Serial.print(target_lon,8);

        double distance_to_target = ((sqrt((sq(target_lat - latitude) + sq(target_lon - longitude)))) * 111111);
        //Serial.print("Distance to target: ");
        //Serial.println(distance_to_target, 8);

        if (distance_to_target != last_distance_to_target && abs(distance_to_target - last_distance_to_target) > 0.3)
        {
          //Serial.println("updated location");

          if (distance_to_target > last_distance_to_target)
          {
            //Serial.println("you have moved further from the target");
          }
          if (distance_to_target < last_distance_to_target)
          {
            //Serial.println("you have moved closer to the target");
          }



          last_distance_to_target = distance_to_target;
        }

        double required_bearing = ((atan2((target_lon - longitude), (target_lat - latitude)) * 180) / PI);
        Serial.print("Required bearing: ");
        Serial.print(required_bearing, 8);
        Serial.print("     ");


        double current_bearing = ((atan2((longitude - last_longitude), (latitude - last_latitude)) * 180) / PI);
        if (current_bearing != last_bearing)
        {
          //Serial.print("Current bearing: ");
          //Serial.print(current_bearing);

          if (current_bearing != 0 && (abs(last_non_zero_current_bearing - current_bearing) < 65))
          {
            last_non_zero_current_bearing = current_bearing;
          }

        }

        last_longitude = longitude;
        last_latitude = latitude;
        last_bearing = current_bearing;

        Serial.print("Non zero current bearing: ");
        Serial.println(last_non_zero_current_bearing);

        if (abs(required_bearing - last_non_zero_current_bearing) <= 15)
        {
          Serial.println("straight");
          analogWrite(servo_pin, 150);
        }

        if (required_bearing - last_non_zero_current_bearing < 1)
        {
          Serial.println("right");
          analogWrite(servo_pin, 200);
        }
        if (required_bearing - last_non_zero_current_bearing > 1)
        {
          Serial.println("left");
          analogWrite(servo_pin, 100);
        }

        if (distance_to_target > 1)
        {
          digitalWrite(gate_pin, HIGH);
        }

        else
        {
          digitalWrite(gate_pin, LOW);
        }

      }

      else
      {
        Serial.println(F("- location: INVALID"));
      }
      Serial.println();
    }
  }

  if (millis() > 5000 && gps.charsProcessed() < 10) //Checks if it has been more than 5 seconds since GPS should have transmitted
  {
    //Serial.println(F("No GPS data received: check wiring")); //Connection to GPS module timed out
  }
}
