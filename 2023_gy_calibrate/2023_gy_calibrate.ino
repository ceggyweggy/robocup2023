//upload code and rotate the gy 360deg, use offX and offY in main code at setOffset(x, y)

#include <Wire.h>
#include <HMC5883L.h>
#include <Servo.h>

Servo servo;

HMC5883L gy;

int minX = 0;
int maxX = 0;
int minY = 0;
int maxY = 0;
int offX = 0;
int offY = 0;

void setup() {
  //Serial.begin(19200);
  servo.attach(1);
  
  while (!gy.begin()) delay(500);

  gy.setRange(HMC5883L_RANGE_0_88GA);
  gy.setMeasurementMode(HMC5883L_CONTINOUS);
  gy.setDataRate(HMC5883L_DATARATE_75HZ);
  gy.setSamples(HMC5883L_SAMPLES_8);
}//193 111

void loop() {
  Vector mag = gy.readRaw();

  servo.writeMicroseconds(mag.XAxis+1000.0);
//  servo.writeMicroseconds(mag.YAxis+1000.0);

//  if (mag.XAxis < minX) minX = mag.XAxis;
//  if (mag.XAxis > maxX) maxX = mag.XAxis;
//  if (mag.YAxis < minY) minY = mag.YAxis;
//  if (mag.YAxis > maxY) maxY = mag.YAxis; //-18 x131
//
//  offX = (maxX + minX)/2;
//  offY = (maxY + minY)/2;

//  Serial.print(mag.XAxis);
//  Serial.print(":");
//  Serial.print(mag.YAxis);
//  Serial.print(":");
//  Serial.print(minX);
//  Serial.print(":");
//  Serial.print(maxX);
//  Serial.print(":");
//  Serial.print(minY);
//  Serial.print(":");
//  Serial.print(maxY);
//  Serial.print(":");`
//  Serial.print(offX); //need these values
//  Serial.print(":");
//  Serial.print(offY);
//  Serial.print("\n");
}
