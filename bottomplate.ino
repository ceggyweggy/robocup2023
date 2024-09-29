bool debugTemt = false;
bool debugKick = false;
int receivePin = 0; //from top processor
int sendPin = 1;

int kickPin1 = 9;
int kickPin2 = 10;
int kick = 0; //kick command
int kickState = 0; //0=waiting, 1=kicking, 2=cooldown, 3=reset
int kickTime = 100;
int cooldownTime = 3000; //to prevent relay from dying
elapsedMillis elapsedTime;

int temts[12] = {A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11};
int maxTemts[12] = {282, 330, 278, 137, 172, 216, 398, 456, 291,  325, 252, 290}; //practice field
int minTemts[12] = {8, 18, 11, 31, 23, 29, 74, 66, 34, 46, 37, 30};
//double maxTemts[12] = {0}; //for calibration
//double minTemts[12] = {1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000};
int scaledTemts[12] = {0};
int countLine = 0;
elapsedMillis lineTime;
unsigned long sendTime = 50;

void setup() {
  for (int i = 0; i < 12; i++) pinMode(temts[i], INPUT);
  pinMode(receivePin, INPUT);
  pinMode(kickPin1, OUTPUT);
  pinMode(kickPin2, OUTPUT);
  pinMode(sendPin, OUTPUT);
}

void getMinMax() { //get max and min for all temts wo pain
  for (int i = 0; i < 12; i++) {
    double val = analogRead(temts[i]);
    if (val > maxTemts[i]) maxTemts[i] = val;
    if (val < minTemts[i]) minTemts[i] = val;
    Serial.print(i);
    Serial.print("  ");
    Serial.print(maxTemts[i]);
    Serial.print("  ");
    Serial.println(minTemts[i]);
  }
}

void scaleTemts() { //scale temt values btn 0-1 for green-white
  countLine = 0;
  for (int i = 0; i < 12; i++) {
    double val = analogRead(temts[i]);
    scaledTemts[i] = round((val - minTemts[i])/(maxTemts[i] - minTemts[i])); //round so anyth above thresh goes to 1
    countLine += scaledTemts[i];
    //Serial.print(i);
    //Serial.print("  ");
    //Serial.println(scaledTemts[i]);
  }
}

void loop() {
  kick = digitalRead(receivePin);
  //kick = 1;
  //Serial.println(kick);
  
  switch (kickState) {
    case 0: //wait for command
      analogWrite(kickPin1, 0);
      analogWrite(kickPin2, 0);
      if (kick) {
        if (debugKick) Serial.println("start kick");
        elapsedTime = 0; //reset time
        kickState++;
      }
      break;
    case 1: //kick
      analogWrite(kickPin1, 255);
      analogWrite(kickPin2, 0);
      if (elapsedTime > kickTime) {
        if (debugKick) Serial.println("finished kick");
        kickState++;
      }
      break;
    case 2: //cooldown
      analogWrite(kickPin1, 0);
      analogWrite(kickPin2, 0);
      if (elapsedTime > (kickTime + cooldownTime)) {
        if (debugKick) Serial.println("finished cooldown");
        kickState++;
      }
      break;
    case 3: //reset
      kickState = 0;
      break;
  }

//  getMinMax();

  scaleTemts();
  if (debugTemt) {
    Serial.print(countLine);
    Serial.print(" ");
  }
  
  if (countLine) {
    digitalWrite(sendPin, HIGH);
    if (debugTemt) Serial.println("send");
    lineTime = 0;
  }
  else {
    if (lineTime < sendTime) {
      if (debugTemt) Serial.println("send");
      digitalWrite(sendPin, HIGH);
    }
    else digitalWrite(sendPin, LOW);
  }
}
