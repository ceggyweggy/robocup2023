/* This file is part of the Razor AHRS Firmware */

// Output angles: yaw, pitch, roll
void output_angles()
{
  yaw = fmod(yaw + 2*PI, 2*PI);

  while (digitalRead(button_pin)) zero_ang = yaw; //set zero value
  
  final_ang = fmod(yaw - zero_ang + 2*PI, 2*PI);
  final_ang = TO_DEG(final_ang);
  
  write_time = final_ang * 3.0 + 600.0;
  fakeservo.writeMicroseconds(write_time);
}
