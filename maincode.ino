// tuning ball track: take out line, keep y bound (cos goals r immovable) but can artificially change x bound

#include <Robot.h>
#include <RobotFunctions.h>
Robot bot;
Output runn;

double speedd = 0.2;

int swswitchPin = 23;
int swswitch = 0;

int statee, defaultState;  //0 is striker, 1 is goalie
int otherHeartbeat;

//aiming
elapsedMillis aimTime;
elapsedMillis accelTime;
elapsedMillis ballcapTime;
elapsedMillis slowTime;

bool outFlag = 0;
double newTarget;
double moveToPointCap = 1.0;

//bt stuff
int myBall = -1;
int otherBall = 0;
elapsedMillis btLastSendTime;
int minSend = 15; //send every 15ms

//home positions
double goalie_home_x = 910.0 - ROBOT_DIAMETER / 2.0;
double goalie_home_y = listFGoalie[(int)goalie_home_x];
double striker_home_x = 910.0 + ROBOT_DIAMETER / 2.0;
double striker_home_y = 600.0 + 180.0 + 50.0;

//random stuff for prints that isnt used in main code
elapsedMillis looper;
int longestBall;
int countcount;
int firstswitch = 0;

int mtp = 0;  //if goalie doing mtp dont do fwd slowdown

void setup() {
  Serial.begin(9600);
  bot.setup();
  botSetup();
  pinMode(swswitchPin, INPUT);
  pinMode(13, OUTPUT);

  if (pika) defaultState = STRIKER;
  else defaultState = GOALIE;

  statee = defaultState;
}

void loop() {
//  Serial.print("x: ");
//  Serial.print(bot.ball_x);
//  Serial.print(" ");
//  Serial.print(bot.prevball_x);
//  Serial.print("   y: ");
//  Serial.print(bot.ball_y);
//  Serial.print(" ");
//  Serial.print(bot.prevball_y);
//  Serial.print(" ");
//  Serial.print(mtp);
//  Serial.print(" ");
//  Serial.print(bot.ballStayed);
//  Serial.print(" ");
//  Serial.println(otherHeartbeat);
//  Serial.println(statee);
//  delay(1);
  Serial.print(bot.prevSpeed);
  swswitch = digitalRead(swswitchPin);
  
  bot.updateReading(); 
  bot.updatePos(statee); //0 for striker

  // min kick speed is prop from dist to goal
  double minKickSpeed = pow(constrain(1.0 - (bot.y_mid - listB[(int)bot.x_mid]) / (STRIKER_FRONT_BOUND_KICK_SPEED - listB[(int)bot.x_mid]), 0.0, 1.0), 1.25); //2090 is 12cm out + 25cm penalty

//  bot.debugDist(1);
//  bot.debugDist(2);
//  bot.debugDist(3);
//  bot.debugCamera();
//  Serial.println(bot._gy_angle);

  //CHECK IF OTHER BOT ALIVE
  if (bot.anyBluetooth <= noBluetooth) otherHeartbeat = 1;
  else otherHeartbeat = 0;

//  otherHeartbeat = 1; //ADD IF BT TRANSITION MESSES STUFF UP

  //BT SEND
  myBall = ballSendPos(bot.ball_x, bot.ball_y, bot.anyBall, bot.conf_x, bot.conf_y);
  if (btLastSendTime > minSend && !bot.ballcap) { //send every 15ms, dont send if ball is in bcz (cos other bot will chase)
    bot.btSend(myBall);
    btLastSendTime = 0;
  }

  //BT RECEIVE
  if (otherHeartbeat) {
    ballReceive(bot.btReceive, bot.x_mid, bot.y_mid);
    goalie_home_x = 910.0 - ROBOT_DIAMETER / 2.0;
    statee = defaultState;
  }
  else {
    otherBallAng = -1;
    otherBallDist = -1;
    goalie_home_x = 910.0;
    statee = GOALIE;
  }
  goalie_home_y = listFGoalie[(int)goalie_home_x];

  if (otherBallAng != -1 && otherBallDist != -1) otherBall = 1; //does other bot have valid ball
  else otherBall = 0;

  if ((statee == STRIKER && bot.anyBall > noBall && otherBall) || (statee == GOALIE && bot.anyBall > noBallGoalie && otherBall)) {
    bot.ball_ang = otherBallAng;
    bot.ball_dist = otherBallDist;
  }

  //TRANSITION FROM GOALIE TO STRIKER
  if (bot.ballStayed > 5000 && bot.anyBall < noBall && (bot.ball_ang > 270 || bot.ball_ang < 90)) {
    statee = STRIKER;
  }

  if (bot.y_mid > 1215.0 - 270.0) statee = STRIKER; // to facilitate the striker to goalie transition yay

//  Serial.println(statee);
  
  if (!bot.ballcap) ballcapTime = 0;

  //MAIN CODE
  if (statee == STRIKER) {
    if (bot.ballcap) aimTime = 0;

    if (bot.anyBall < noBall) digitalWrite(13, HIGH);
    else digitalWrite(13, LOW);

    if (aimTime < 300 && slowTime > 100) { // aim point depends on x coord of bot
      //double aim_side_dist = LOCAIM_DIST_GOAL + (GOAL_SIZE - 2.0 * LOCAIM_DIST_GOAL) / 4.0; // not using this anymore, but this works by taking closest aim point 
      if (bot.x_mid < GOAL_LEFT_X) { //  + LOCAIM_DIST_GOAL //aim right
        runn = moveToPoint(bot.x_mid, bot.y_mid, bot._gy_angle, GOAL_LEFT_X+LOCAIM_DIST_GOAL, FIELD_Y - OUT_FIELD_SIZE);
      }
      else if (bot.x_mid < GOAL_RIGHT_X) { //  - LOCAIM_DIST_GOAL //aim middle
        runn = moveToPoint(bot.x_mid, bot.y_mid, bot._gy_angle, FIELD_X / 2.0, FIELD_Y);
      }
      else { //aim left
        runn = moveToPoint(bot.x_mid, bot.y_mid, bot._gy_angle, GOAL_RIGHT_X-LOCAIM_DIST_GOAL, FIELD_Y - OUT_FIELD_SIZE);
      }
      
      runn.speed = min(0.85, 0.1 + accelTime * 0.0005); //stable for both striker and goalie
      //runn.speed = min(0.85, 0.1 + accelTime * 0.00075);

      if (bot.ballcap && bot.goal_kick && runn.speed >= minKickSpeed) bot.kick(1);
      else bot.kick(0);
    }
    else {
      accelTime = 0; //stop accelerating once no more aiming
      if (bot.anyBall < noBall || (bot.anyBall > noBall && otherBall)) { //((defaultState == STRIKER && (bot.anyBall < noBall || (bot.anyBall > noBall && otherBall))) || (bot.anyBall < noBall && bot.ball_dist <= 30.0 && defaultState == GOALIE)) {
        runn = ballTrack(speedd, bot.ball_ang, bot.ball_dist, bot.x_mid, bot._gy_angle);
      }
      else {
        if (defaultState == GOALIE) runn = moveToPoint(bot.x_mid, bot.y_mid, bot._gy_angle, 910.0 - ROBOT_DIAMETER / 2.0, striker_home_y, 0.0, 0.0, 0.0, bot.prevSpeed); // opp of striker?>??? 
        else runn = moveToPoint(bot.x_mid, bot.y_mid, bot._gy_angle, striker_home_x, striker_home_y); //cap?
        //else runn = moveToPoint(bot.x_mid, bot.y_mid, bot._gy_angle, goalie_home_x, goalie_home_y);
        runn.speed = constrain(runn.speed, 0.0, moveToPointCap);
        if (defaultState == GOALIE && strikerToGoalie) statee = GOALIE;
      }
    }
    
    runn = slowDown(runn, bot.x_mid, bot.y_mid, bot._gy_angle, bot.lb_x, bot.ub_x, bot.lb_y, bot.ub_y);

//    Serial.print(" aft sd: ");
//    Serial.print(runn.speed);
//    Serial.print(" ");
//    Serial.print(runn.angle);
//    Serial.print(" ");
//    Serial.print(runn.rotation);
  
    if (bot.line) outFlag = 1; // take this out when tuning ball track
    
    if (outFlag) {
      runn = moveToPoint(bot.x_mid, bot.y_mid, bot._gy_angle, striker_home_x, striker_home_y, 0.1);
      runn = conf(runn, bot.conf_x, bot.conf_y, STRIKER_LINE_CONF_X, STRIKER_LINE_CONF_Y);
  
      if (bot.lb_x > 210.0 && bot.ub_x < 1610.0 && bot.ub_y < FIELD_Y - listF[(int)bot.x_mid] && bot.lb_y > listB[(int)bot.x_mid]) outFlag = 0;
    }
    else runn = conf(runn, bot.conf_x, bot.conf_y, STRIKER_CONF_X, STRIKER_CONF_Y);
//    runn = conf(runn, bot.conf_x, bot.conf_y, STRIKER_CONF_X, STRIKER_CONF_Y);

//    Serial.print(" conf x: ");
//    Serial.print(bot.conf_x);
//    Serial.print(" conf y: ");
//    Serial.print(bot.conf_y);
//    Serial.println();

//    Serial.print(" aft conf: ");
//    Serial.print(runn.speed);
//    Serial.print(" ");
//    Serial.print(runn.angle);
//    Serial.print(" ");
//    Serial.print(runn.rotation);
  }
  else {
    if (bot.ballcap && bot.lid_front_raw > 100.0) bot.kick(1);
    else bot.kick(0);
    
    if (bot.anyBall < noBallGoalie || (bot.anyBall > noBallGoalie && otherBall)) {
      mtp = 0;
      runn = ballTrackGoalie(speedd, bot.ball_ang, bot.ball_dist, bot.x_mid, bot.y_mid, bot._gy_angle);
      runn.speed = constrain(runn.speed, 0.0, 0.5); //cos it slip n die
    }
    else {
      //should move to intermediate point first
//      if (bot.x_mid > GOAL_RIGHT_X) {
////        Serial.println("onright");
//        runn = moveToPoint(bot.x_mid, bot.y_mid, bot._gy_angle, GOAL_RIGHT_X-90.0, listFGoalie[(int)(GOAL_RIGHT_X-90.0)], 0.0, 0.04);
//      }
//      else if (bot.x_mid < GOAL_LEFT_X) {
////        Serial.println("onleft");
//        runn = moveToPoint(bot.x_mid, bot.y_mid, bot._gy_angle, GOAL_LEFT_X+90.0, listFGoalie[(int)(GOAL_RIGHT_X+90.0)], 0.0, 0.04);
//      }
//      else {
//        Serial.println("centre");
        runn = moveToPoint(bot.x_mid, bot.y_mid, bot._gy_angle, goalie_home_x, goalie_home_y, 0.0, 0.04, 0.0, bot.prevSpeed);
        mtp = 1;
//      }
      runn.speed = constrain(runn.speed, 0.0, moveToPointCap);
    }

//    Serial.print(" bef sd: ");
//    Serial.print(runn.speed);
//    Serial.print(" ");
//    Serial.print(runn.angle);
//    Serial.print(" ");
//    Serial.print(runn.rotation);
    
    runn = slowDownGoalie(runn, bot.x_mid, bot.y_mid, bot._gy_angle, bot.lb_x, bot.ub_x, otherHeartbeat, mtp);

//    Serial.print(" bef conf: ");
//    Serial.print(runn.speed);
//    Serial.print(" ");
//    Serial.print(runn.angle);
//    Serial.print(" ");
//    Serial.print(runn.rotation);
//    Serial.print(" conf x: ");
//    Serial.print(bot.conf_x);
//    Serial.print(" conf y: ");
//    Serial.print(bot.conf_y);

    //OUT OF BOUNDS
    if (bot.line) { // only if bot knows it is on the side (cos dw to detect penalty area)
      if (bot.x_mid > FIELD_X / 2.0 && bot.lb_x > GOAL_RIGHT_X + 180.0) outFlag = 1; // 180.0 is random lol cos its detecting goal bound still
      else if (bot.x_mid < FIELD_X / 2.0 && bot.ub_x < GOAL_LEFT_X - 180.0) outFlag = 1;
      else outFlag = 0;
    }
    else outFlag = 0;
    
//    if (min(bot.lidBR, bot.lidBL) < 200.0) outFlag = 1;

    if (outFlag) {
//      Serial.print("out");
      runn = moveToPoint(bot.x_mid, bot.y_mid, bot._gy_angle, striker_home_x, striker_home_y, 0.1);
      runn = conf(runn, bot.conf_x, bot.conf_y, STRIKER_LINE_CONF_X, STRIKER_LINE_CONF_Y);
  
      if (bot.lb_x > 210.0 && bot.ub_x < 1610.0 && bot.ub_y < FIELD_Y - listF[(int)bot.x_mid] && bot.lb_y > listB[(int)bot.x_mid]) outFlag = 0;
    }
    else runn = conf(runn, bot.conf_x, bot.conf_y, GOALIE_CONF_X, GOALIE_CONF_Y);

    if (bot.ball_ang > 90.0 && bot.ball_ang < 270.0) {
      double prop_dist = (constrain(bot.ball_dist - MIN_BALL_DIST - 2.0, 0, MAX_BALL_DIST) / (MAX_BALL_DIST - MIN_BALL_DIST - 2.0));
      runn.speed = constrain(runn.speed, 0.0, prop_dist);
    }
  }

  //calculate dir to face in
  // TUNE THIS FOR FIELD: on auto field it is permanently tilted left fsr
  if (statee == STRIKER) {
    if (bot.x_mid > MID_X) newTarget = ((bot.x_mid - MID_X) / MID_X) * (max(bot.y_mid - 600.0, 0.0) / (FIELD_Y - 600.0)) * (-LEFT_STRIKER_TILT_ANGLE); //y bound by striker frontbound // is fine without tilt
    else newTarget = ((bot.x_mid - MID_X) / MID_X) * (max(bot.y_mid - 600.0, 0.0) / (FIELD_Y - 600.0)) * (-RIGHT_STRIKER_TILT_ANGLE); //y bound by striker frontbound
  }
  else newTarget =  (bot.x_mid - MID_X) / MID_X * GOALIE_TILT_ANGLE; //face other way for goalie
  //runn.rotation = proportionalCompass(runn.speed, bot._gy_angle, newTarget);

  if (runn.speed <= 0.04 || (bot._gy_angle >= 45.0  && bot._gy_angle <= 315.0)) runn = pureRotation(bot._gy_angle, newTarget); //run pure rotation if speed too slow to translate
  else runn.rotation = proportionalCompass(runn.speed, bot._gy_angle, newTarget);

  runn.speed = constrain(runn.speed, 0.0, 0.97);
  bot.prevSpeed = runn.speed;

//  runn.speed = 0.3;
//  runn.rotation = proportionalCompass(0.3, bot._gy_angle, newTarget);

  if (swswitch) bot.setDrive(runn.speed, runn.angle, runn.rotation);
  else bot.setDrive(0.0, 0.0, 0.0);

  /*runn.speed = 0.5;
  runn.rotation = proportionalCompass(runn.speed, bot._gy_angle, 0.0);
  runn.angle = 0.0;

  if (swswitch) {
    if (!firstswitch) {
      firstswitch = 1;
      delay(500);
    }
    bot.setDrive(runn.speed, runn.angle, runn.rotation);
  }
  else {
    firstswitch = 0;
    bot.setDrive(0.0, 0.0, 0.0);
  }*/
  
//  Serial.print(statee);
//  Serial.print("  ");
//  Serial.print(bot.ballcap);
//  Serial.print("  ");
//  Serial.println(immAim);
//  Serial.print(bot.lb_y);
//  Serial.print(" ");
//  Serial.print(bot.ub_y);
//  Serial.print(" ");
//  Serial.print(listF[(int)bot.x_mid]);
//  Serial.print(" ");
//  Serial.print(listB[(int)bot.x_mid]);
//  Serial.print(" ");
//  Serial.print(listAngGoalieRight[(int)bot.x_mid]);
//  Serial.print(" ");

//  Serial.print(newTarget);
//  Serial.print(" ");
//  Serial.print(swswitch);
//  Serial.print(" ");
//  Serial.print(bot.ballcap);
//  Serial.print(" ");
//  Serial.print(bot.line);
//  Serial.print(" ");
//  Serial.print(bot.ball_ang);
//  Serial.print(" ");
//  Serial.print(bot.ball_dist);
//  Serial.print(" ");
//  Serial.print(bot.lightgate);
//  Serial.print(bot._gy_angle);
//  Serial.print(" ");
//  Serial.print(bot.lb_x);
//  Serial.print(" ");
//  Serial.print(bot.ub_x);
//  Serial.print(" ");
//  Serial.print(bot.lidBR);
//  Serial.print(" ");
//  Serial.print(bot.lidBL);
//  Serial.print(" ");
//  Serial.print(bot.x_mid);
//  Serial.print(" ");
//  Serial.print(bot.y_mid);
//  Serial.print(" ");
//  Serial.print(" conf x: ");
//  Serial.print(bot.conf_x);
//  Serial.print(" conf y: ");
//  Serial.print(bot.conf_y);
//  Serial.print(outFlag);
//  Serial.print(accelTime);
//  Serial.print(" speed ");
//  Serial.print(runn.speed);
//  Serial.print(" ang ");
//  Serial.print(runn.angle);
//  Serial.print(" rot ");
//  Serial.print(runn.rotation);
//  Serial.print(" ");
//  Serial.println();
//  delay(1);
//  Serial.println(ballcapTime);

  //print loop time
//  if (bot.anyBall > longestBall){
//    longestBall = bot.anyBall;
//    Serial.println(longestBall);
//  }
//  countcount++;
//  if (!(countcount % 1000)) {
//    int help = looper;
//    looper = 0;
//    Serial.println(help);
//  }
}
