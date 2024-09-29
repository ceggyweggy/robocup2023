void setup() {
  // put your setup code here, to run once:
  pinMode(22, INPUT);

}

int minn = 0;
int maxx = 0;
int off = 0;

void loop() {
  // put your main code here, to run repeatedly:
  int val = pulseIn(22, HIGH);
//  val -= 1000;
  val = (val-600)/3;

  Serial.println(val);

//  if (val < minn) minn = val;
//  if (val > maxx) maxx = val;
//
//  off = (maxx + minn) / 2;
//  Serial.print(val);
//  Serial.print("  ");
//  Serial.print(maxx);
//  Serial.print("  ");
//  Serial.print(minn);
//  Serial.print("  "); //-219 -222 359 350
//  Serial.print(off); //-27 -24 433 430 425 438 429
//  /*1702
//   * x 394 -454 = -30
//   * y 290 -454 = -82
//   */
//  Serial.println();
}
