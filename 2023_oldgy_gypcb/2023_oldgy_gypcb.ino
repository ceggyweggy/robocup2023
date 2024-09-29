//USE ARDUINO PRO/PRO MINI --> ATMEGA328P (3.3V 8MHZ)
/*
 * CALIBRATION STEPS:
 * 1. FIND X OFFSET
 * 2. FIND Y OFFSET
 * 3. FIND MAPPING VALUES
 */

#include <Wire.h>
#include <HMC5883L.h>
#include <Servo.h>

HMC5883L gy;
Servo fakeservo;

bool debug = false; //to print everything

const float decl_ang = 0.10 * PI/180.0; //declination angle
float zero_ang;
float raw_heading;
int heading;
float final_ang = 999.0;
float write_time;

float map_ang[360];
float v0 = 0.0;
float v90 = 110.0;
float v180 = 202.0;
float v270 = 282.0;
float v360 = 360.0;

int button_pin = 0;
int led_pin_right = 11; //mosi
int led_pin_left = 12; //miso soup

void setup() {
  if (debug) Serial.begin(19200);
  if (!debug) fakeservo.attach(1);

  while (!gy.begin()) delay(500);
  
  gy.setRange(HMC5883L_RANGE_0_88GA);
  gy.setMeasurementMode(HMC5883L_CONTINOUS);
  gy.setDataRate(HMC5883L_DATARATE_75HZ);
  gy.setSamples(HMC5883L_SAMPLES_8);
  gy.setOffset(1025, 1006); //use calibration code

  pinMode(led_pin_right, OUTPUT);
  pinMode(led_pin_left, OUTPUT);
  pinMode(button_pin, INPUT);

  for (int i = 0; i <= 360; i++) {
    if (i >= v0 && i <= v90) map_ang[i] = map(i, v0, v90, 0.0, 90.0);
    else if (i > v90 && i <= v180) map_ang[i] = map(i, v90, v180, 90.0, 180.0);
    else if (i > v180 && i <= v270) map_ang[i] = map(i, v180, v270, 180.0, 270.0);
    else if (i > v270 && i <= v360) map_ang[i] = map(i, v270, v360, 270.0, 360.0);
  }
}

void loop() {
  while (digitalRead(button_pin)) zero_ang = map_ang[heading]; //set zero value
  
  Vector norm = gy.readNormalize();

  raw_heading = atan2(norm.YAxis, norm.XAxis); //raw angle in radians
  heading = (raw_heading + decl_ang) * 180.0/PI; //angle in degrees incl declination angle
  heading = fmod(heading + 360.0, 360.0); //wraparound

  //fakeservo.writeMicroseconds(heading * 3.0 + 600.0); //to get the mapping values

  final_ang = map_ang[heading];
  final_ang = fmod(final_ang - zero_ang + 360.0, 360.0);

  if (final_ang <= 5.0 || final_ang >= 355.0) {
    digitalWrite(led_pin_right, LOW);
    digitalWrite(led_pin_left, HIGH);
  }
  else {
    digitalWrite(led_pin_right, HIGH);
    digitalWrite(led_pin_left, LOW);
  }

  write_time = final_ang * 3.0 + 600.0; //value from 600-1680
  if (!debug) fakeservo.writeMicroseconds(write_time);
//  fakeservo.writeMicroseconds(1000);

  if (debug) {
    Serial.print("raw = ");
    Serial.print(heading);
    Serial.print("   mapped = ");
    Serial.print(map_ang[heading]);
    Serial.print("   zero_ang = ");
    Serial.print(zero_ang);
    Serial.print("   final_ang = ");
    Serial.print(final_ang);
    Serial.println();
  }
}
