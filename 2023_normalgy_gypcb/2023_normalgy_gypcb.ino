//USE ARDUINO PRO/PRO MINI --> ATMEGA328P (3.3V 8MHZ)

#include <Wire.h>
#include <Servo.h>

#define MAGN_ADDRESS ((int16_t)0x0E)
#define WIRE_SEND(b) Wire.write((byte)b)
#define WIRE_RECEIVE() Wire.read()

Servo fakeservo;

bool debug = false;

const float decl_ang = 0.10 * PI/180.0;
float zero_ang;
float raw_heading;
int heading; 
float final_ang = 999.0;
float write_time;
float mag_x;
float mag_y;

float map_ang[360];
float v0 = 0.0;
float v90 = 72.0;
float v180 = 132.0;
float v270 = 213.0;
float v360 = 360.0;

int button_pin = 0; //rx
int led_pin_right = 11; //mosi
int led_pin_left = 12; //miso soup

void setup() {
  Wire.begin();
  if (debug) Serial.begin(19200);
  if (!debug) fakeservo.attach(1);

  pinMode(button_pin, INPUT);
  pinMode(led_pin_right, OUTPUT);
  pinMode(led_pin_left, OUTPUT);

  for (int i = 0; i <= 360; i++) {
    if (i >= v0 && i <= v90) map_ang[i] = map(i, v0, v90, 0.0, 90.0);
    else if (i > v90 && i <= v180) map_ang[i] = map(i, v90, v180, 90.0, 180.0);
    else if (i > v180 && i <= v270) map_ang[i] = map(i, v180, v270, 180.0, 270.0);
    else if (i > v270 && i <= v360) map_ang[i] = map(i, v270, v360, 270.0, 360.0);
  }
}

void loop() {
  while (digitalRead(button_pin)) zero_ang = map_ang[heading]; //set zero value
  
  int i = 0;
  uint8_t buff[6];
  
  Wire.beginTransmission(MAGN_ADDRESS); //set sensor to single measurement mode
  WIRE_SEND(0x0A);
  WIRE_SEND(0x01);
  Wire.endTransmission();
  Wire.beginTransmission(MAGN_ADDRESS);
  WIRE_SEND(0x08);
  Wire.endTransmission();
  Wire.beginTransmission(MAGN_ADDRESS);
  WIRE_SEND(0x07);
  Wire.endTransmission();
  Wire.beginTransmission(MAGN_ADDRESS);
  WIRE_SEND(0x06);
  Wire.endTransmission();
  Wire.beginTransmission(MAGN_ADDRESS);
  WIRE_SEND(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(MAGN_ADDRESS);
  WIRE_SEND(0x04);
  Wire.endTransmission();
  Wire.beginTransmission(MAGN_ADDRESS);
  WIRE_SEND(0x03);
  Wire.endTransmission();
  Wire.beginTransmission(MAGN_ADDRESS);
  Wire.requestFrom(MAGN_ADDRESS, 6);

  while (Wire.available()) {
    buff[i] = WIRE_RECEIVE();
    i++;
  }

  mag_x = -1 * (int16_t)((((uint16_t)buff[1]) << 8) | buff[0]);
  mag_y = (int16_t)(((((uint16_t)buff[3]) << 8) | buff[2]));
  //mag_z = -1 * (int16_t)(((((uint16_t)buff[5]) << 8) | buff[4]));

  //mag_x = mag_x * cos_pitch + mag_y * sin_roll * sin_pitch + mag_z * cos_roll * sin_pitch;
  //mag_y = mag_y * cos_roll - mag_z * sin_roll;

  raw_heading = atan2(mag_y, mag_x);
  heading = (raw_heading + decl_ang) * 180.0/PI;
  heading = fmod(heading + 360.0, 360.0);

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

  write_time = final_ang * 3.0 + 600.0;
  if (!debug) fakeservo.writeMicroseconds(write_time);

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
