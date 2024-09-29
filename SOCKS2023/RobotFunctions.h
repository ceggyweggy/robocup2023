#ifndef RobotFunctions_h
#define RobotFunctions_h

#include <Math.h>
#include <Meth.h>
#include <Arduino.h>
#include <elapsedMillis.h>
#include <Config2022.h>
#include <Tuning.h>

double listF[1821];
double listB[1821];
double listFGoalie[1821];
double listAngGoalieRight[1821];
double listAngGoalieLeft[1821];
double listLeftStriker[2431];
double listRightStriker[2431];
double listBNoStriker[1821];
double listFNoStriker[1821];
//double listBallDist[maxballdist
bool aiming = 0;
double aimAngle = 0.0;
double ballcapY = 12.0;
bool goalieTransition = 0;

bool camGoal = 0;
elapsedMillis camGoalTime;

double otherBallAng = -1;
double otherBallDist = -1;

//reserve 60 for ball dist
// int distreserve = 60+1;
// int btremaining = 255-1-distreserve;
int btremaining = 254;
int noBluetooth = 45;

elapsedMillis aimCount;

elapsedMillis heartbeat;
elapsedMillis otherbeat;
elapsedMillis ballBeat = 1000;
elapsedMillis dribTime;

elapsedMillis currMillis;
unsigned long long lastCallMillis;

int beatTime = 250; //time to receive heartbeat from other bot
int beatBuffer = 10;
bool otheralive = 1;

bool dribbling = 0;
bool kicking = 0;

bool curBall = 0; //not used
int noBall = 250; //half of goalie?? //not received for 3 loops of camera sending
int noBallGoalie = 500; // ??? 

bool balling = 0; //there is a ball????

int strikerToGoalie = 0;

void botSetup() { //everything here in mm
    //calculate front/back arcs
    // (250, 250), (1570, 250), (910, 520) : y=-1/3(-sqrt(-9x^2+16380x+527725)+1265)
    // (250, 340), (1570, 340), (910, 520) : y=sqrt(-x^2+1820x+861900)-780
    // (610, 430), (910, 520), (1210, 430) : y=sqrt(-x^{2}+1820x-531075)-25
    // (610, 450), (910, 520), (1210, 450) : y=1/7(sqrt(-49x^{2}+89180x-18061875)-1105)
    // (610, 460), (910, 520), (1210, 460) : y=(-x^{2}+1820x-219700)^{0.5}-260

    double prevY = sqrt(-1.0 - 1820.0 - 219700.0) - 260.0;

    for (int i = 0; i <= 1820; i++) {
        double y;
        y = sqrt(-pow(i, 2.0) + 1820.0*i - 219700.0) - 260.0 + 90.0;
        if (y < 430.0) y = 430.0; //out + half robot + 90.0 cos arc is shifted fwd
        listB[i] = y;
    }
    for (int i = 0; i <= 1820; i++) {
        double y;
        y = sqrt(-pow(i, 2.0) + 1820.0*i - 219700.0) - 260.0;
        // double y = -1.0 / 3.0 * (-sqrt(-9.0 * pow(i, 2.0) + 16380.0 * i + 527725.0) +1265.0);
        // listF[i] = FIELD_Y - STRIKER_FRONT_BOUND;
        if (y < 340.0) y = 340.0;
        listF[i] = y;
    }
    for (int i = 0; i <= 1820; i++) {
        double y;
        y = sqrt(-pow(i, 2.0) + 1820.0*i - 219700.0) - 260.0;
        // double y = -1.0 / 3.0 * (-sqrt(-9.0 * pow(i, 2.0) + 16380.0 * i + 527725.0) +1265.0);

        // GOALIE ARCS cos goalie arcs r the same for front n back so only need to compute 1
        listFGoalie[i] = y;
        listFNoStriker[i] = y + 90.0; // when no striker, goalie move forward 9cm (will normally block striker)

        double angRight = rad2deg(atan2(1.0, (y-prevY))); //angle to next coord for travelling right/left
        double angLeft = fmod(angRight + 180.0 + 360.0, 360.0);
        listAngGoalieRight[i] = angRight;
        listAngGoalieLeft[i] = angLeft;
        prevY = y;
    }

    for (int i=0; i<=2430; i++) {
        double x;
    
        if (i <= 250.0 || i > (2430.0 - 250.0)) x = 610.0;
        else if (i <= (1215.0 - 300.0)) x = -2.0/133.0 * (36.0 * i - 49565.0);
        else if (i > (1215.0 - 300.0) && i <= (1215.0 + 300.0)) x = 250.0;
        else x = 2.0/133.0 * (36.0 * i - 37915.0);

        x = 210.0; //3005 230.0;
        listLeftStriker[i] = x;
      }

    for (int i=0; i<=2430; i++) {
        double x;

        if (i <= 250.0 || i > (2430.0 - 250.0)) x = 1210.0;
        else if (i <= (1215.0 - 300.0)) x = 2.0/133.0 * (36.0 * i + 71465.0);
        else if (i > (1215.0 - 300.0) && i <= (1215.0 + 300.0)) x = 1820.0 - 250.0;
        else x = -2.0/133.0 * (36.0 * i - 158945.0);

        x = 1820.0 - 210.0; // 3005
        listRightStriker[i] = x;
    }
}

struct Output {
    double speed = 0.0;
    double angle = 0.0;
    double rotation = 0.0;
};

void printOutput(Output temp) {
    Serial.print(temp.speed);
    Serial.print("   ");
    Serial.print(temp.angle);
    Serial.print("   ");
    Serial.println(temp.rotation);
}

//calculate correction ie rate of rotatio n to face 0
double proportionalCompass(double speed, double compass, double target=360.0) {
    double gain;
    if (pika) gain = 2.78 * pow(10, -3) * pow(speed, -1.35);
    else gain = 2.21 * pow(10, -3) * pow(speed, -1.54); //0.00147 * pow(speed, -1.66);
    if (target <= 0.0) target += 360.0;
    // mod  = a - floor(a/n) * n
    double a = target - compass + 180.0;
    double modded = a - floor(a / 360.0) * 360.0;
    double error = modded - 180.0;
    // Serial.print(" ERROR: ");
    // Serial.print(error);
    return error * gain;
}

Output pureRotation(double compass, double target=0.0) {
    Output temp;
    double a = target - compass + 180.0;
    double modded = a - floor(a / 360.0) * 360.0;
    double error = modded - 180.0;

    temp.speed = error * 0.0025;
    if (temp.speed > 0) temp.rotation = 1.0;
    else temp.rotation = -1.0;
    temp.angle = 0.0;
    temp.speed = constrain(abs(temp.speed), 0.05, 1.0); // make it higher than actual base speed so it can correct

    // Serial.print(compass);
    // Serial.print("  ");
    // Serial.print(target);
    // Serial.print("  ");
    // Serial.print(error);
    // Serial.print("  ");
    // Serial.print(temp.speed);
    // Serial.println();
    return temp;
}

/*
double proportionalCompass(double compass, double target=0, double gain=1.0/52.5) {//52.5
    double error = compass>180.0?360.0-compass:-compass; //-180 to 180
    return error*gain;
}
*/

//calculate angle from the robot to own goal (back of the wall)
double botToGoal(double currentX, double currentY) {
    double yGoal = currentY - 250.0; //in mm // to white line 
    double xGoal = currentX - MID_X; //in mm
    double botToGoal = rad2deg(atan2(xGoal, yGoal)); //-180 to 180

    return botToGoal;
}

// Output accel(Output temp, double prevSpeed){
//  // if(tempOut.speed > prevSpeed) 
//      // how solve the cfuck
//      //flipping
// }

/*
case 1: if the robot is at middle +-3cm just run straight so it doesnt oscillate?
case 2: robot on the left then calc x distance to (middle-3cm) and y distance to y aiming coord, find aiming angle
case 3: robot on right calc x dist to (middle+3cm) and y dist to y aiming, find aiming angle
100323 currently not changing speed
*/
Output locAim(double currentX, double currentY, double currentCompass, double targetCompass = 0){
    Output temp;
    //if dead zone, run forward
    if (currentX >= LOCAIM_DEAD_L && currentX <= LOCAIM_DEAD_R) {
        // Serial.println("dead :(");
        temp.angle = 0.0;
        double yDiff = LOCAIM_Y - currentY; //why do we need this
        // temp.speed = (yDiff/1500.0);
    }
    else if (currentX < LOCAIM_DEAD_L) {
        // Serial.println ("field left");
        double xDiff = LOCAIM_L_X - currentX; //mm
        double yDiff = LOCAIM_Y - currentY; //mm
        temp.angle = rad2deg(atan2(xDiff, yDiff));
        // temp.speed = (sqrt(pow(xDiff, 2) + pow(yDiff, 2))/1500.0);
    }
    else { //if (currentX > LOCAIM_DEAD_R){
        // Serial.println("field right");
        double xDiff = LOCAIM_R_X - currentX; //mm
        double yDiff = LOCAIM_Y - currentY; //mm
        temp.angle = rad2deg(atan2(xDiff, yDiff));
        // temp.speed = (sqrt(pow(xDiff, 2) + pow(yDiff, 2))/1500.0);
    }

    // Serial.println(temp.angle);
    // temp.speed = 0.05 + (sqrt(pow(xDiff, 2) + pow(yDiff, 2))/1500.0);

        //temp.speed = (1500.0-sqrt(pow(xDiff, 2) + pow(yDiff, 2)))/1500.0;
        //temp.speed = 0.3; 
        // temp.speed += 0.001

    temp.angle = fmod(temp.angle + 360.0, 360.0);
    // Serial.println(temp.angle);
    temp.rotation = proportionalCompass(temp.speed, currentCompass, targetCompass);
    temp.speed = constrain(temp.speed, 0.0, 1.0);
    
    return temp;
}

/*
1. calculate x/y dist to target coord and angle to move at
2. scale speed based on distance to target coords (further travel faster)
*/
/*
Output moveToPoint(int currentX, int currentY, double currentCompass, int targetX, int targetY, double setSpeed=0.0, double baseSpeed=0.0, double targetCompass=0.0) { //0 is striker, 1 is goalie
    Output temp;
    double xDiff = targetX - currentX; //mm
    double yDiff = targetY - currentY; //mm

    temp.angle = rad2deg(atan2(xDiff, yDiff));
    // Serial.println(temp.angle);
    // temp.speed = 0.05 + (sqrt(pow(xDiff, 2) + pow(yDiff, 2))/1500.0);

        //temp.speed = (1500.0-sqrt(pow(xDiff, 2) + pow(yDiff, 2)))/1500.0;
        //temp.speed = 0.3; 
        // temp.speed += 0.001;
    if (setSpeed == 0.0) {
        temp.speed = baseSpeed + ((1.0 - baseSpeed) * (sqrt(pow(xDiff, 2) + pow(yDiff, 2)) / MOVETOPOINT_DIST_SCALING));//0.05 + (sqrt(pow(xDiff, 2) + pow(yDiff, 2))/1500.0);
    }
    else temp.speed = setSpeed;
    temp.rotation = proportionalCompass(temp.speed, currentCompass, targetCompass);
    temp.speed = constrain(temp.speed, 0.0, 1.0);
    
    return temp;
}
*/

Output moveToPoint(int currentX, int currentY, double currentCompass, int targetX, int targetY, double setSpeed=0.0, double baseSpeed=0.0, double targetCompass=0.0, double prevSpeed=0.0) { //0 is striker, 1 is goalie
    unsigned long long currentMillis = currMillis; //these two r the same but teensy doesnt let us compare elapedmillis and ull

    if (currentMillis - lastCallMillis > 10) currMillis = 0; //first loop of mtp reset the acceltime counter
    Output temp;
    double xDiff = targetX - currentX; //mm
    double yDiff = targetY - currentY; //mm

    temp.angle = rad2deg(atan2(xDiff, yDiff));

    currentCompass = fmod((currentCompass + 180.0), 360.0) - 180.0; //-180 to 180
    temp.angle -= currentCompass; //if robot is tilted to the right (+ve currentCompass) you wanna subtract from the angle, if left then add (i.e. subtract -ve currentCompass)
    temp.angle = fmod(temp.angle + 360.0, 360.0); //change to 0-360 so the adding/subtracting is the correct direction
    temp.angle = fmod((temp.angle + 180.0), 360.0) - 180.0; //change back to -180-180

    if (setSpeed == 0.0) {
        temp.speed = baseSpeed + ((1.0 - baseSpeed) * (sqrt(pow(xDiff, 2) + pow(yDiff, 2)) / MOVETOPOINT_DIST_SCALING));//0.05 + (sqrt(pow(xDiff, 2) + pow(yDiff, 2))/1500.0);
    }
    else temp.speed = setSpeed;

    // Serial.print(" cur mil: ");
    // Serial.print(currentMillis * 0.001);
    // Serial.print(" lastCallMillis: ");
    // Serial.print(lastCallMillis * 0.001);
    // Serial.print(" timediff: ");
    // Serial.print((int)(currentMillis - lastCallMillis));
    // Serial.print((currentMillis-lastCallMillis)*0.01);
    // Serial.print(" prev sped: ");
    // Serial.print(prevSpeed + (currentMillis-lastCallMillis)*0.0005);
    // Serial.println();
    // delay(1);

    //prevSpeed + (currentMillis-lastCallMillis)*0.0005

    temp.speed = min(currentMillis * 0.001, min(temp.speed, prevSpeed + (currentMillis-lastCallMillis)*0.0005)); // accel shd be until bot looks like slippin
    // temp.speed = 0.1;

    temp.rotation = proportionalCompass(temp.speed, currentCompass, targetCompass);
    temp.speed = constrain(temp.speed, 0.0, 1.0);

    if (sqrt(pow(xDiff, 2) + pow(yDiff, 2)) < 90.0) strikerToGoalie = 1;
    // else if (currentY < 600.0 + 180.0 + 50.0) strikerToGoalie = 1; // if the bot is behind the striker home pt
    else strikerToGoalie = 0;

    lastCallMillis = currentMillis;
    
    return temp;
}

/*
1. calc x/y speed vector
2. conf_x/conf_y calculated it Robot.cpp (updatePos), power aggressiveness of confidence (confXPow/confYPow) i.e. find max/min speed to run at
3. constrain input speed based on max/min speed
4. recalculate angle/speed
*/
Output conf(Output tempIn, double conf_x, double conf_y, double confXPow=6.0, double confYPow=2.0) { //default 2.0 unless goalie then 6.0
    Output tempOut;
    double x = tempIn.speed * sin(deg2rad(tempIn.angle));
    double y = tempIn.speed * cos(deg2rad(tempIn.angle));
    
    conf_x = pow(conf_x, confXPow);
    conf_y = pow(conf_y, confYPow);

    double newX = constrain(x, -conf_x, conf_x);
    double newY = constrain(y, -conf_y, conf_y); //adj eventually asymmetric
    
    tempOut.angle = rad2deg(atan2(newX, newY));
    if (tempOut.angle < 0) tempOut.angle += 360.0;
    tempOut.speed = sqrt(pow(newX, 2) + pow(newY, 2));
    tempOut.rotation = tempIn.rotation;

    // Serial.print(" x: ");
    // Serial.print(x);
    // Serial.print(" y: ");
    // Serial.print(y);
    // Serial.print(" newx: ");
    // Serial.print(newX);
    // Serial.print(" newy: ");
    // Serial.print(newY);
    // Serial.print(" newang: ");
    // Serial.print(tempOut.angle);
    // Serial.println();

    return tempOut;
}

/*
x-axis:
1. slow down when approaching line, speed back in if out of bounds
    case 1: robot moving to the left
    1. bot x is still in bounds (>LR_LB_SD)
    -> calculate max speed it can run at twds the line (distance to the line/max range) (max speed occurs at opposite end of the field)
    2. bot x out of bounds (<)
    -> calculate speed to reenter the field (distance from line/outer size of field)
    case 2: robot moving to the right
    same thing but check if its </> upper bound
2. failsafe?? if the robot is out of bounds then change speed direction (weird abs(x) thing)

y-axis:
same cases as x-axis but front/back, bounds are following the goal arc

then just recalc angle/speed to run at
*/
Output slowDown(Output tempIn, int x_mid, int y_mid, double currentCompass, double lb_x, double ub_x, double lb_y, double ub_y, double targetCompass=0) {//int ub_x, int lb_x, int ub_y, int lb_y, double currentCompass, double targetCompass=0) {
    Output tempOut;

    double x = tempIn.speed*sin(deg2rad(tempIn.angle));
    double y = tempIn.speed*cos(deg2rad(tempIn.angle));

    double constr_x;

    double left_bound = listLeftStriker[y_mid];
    double right_bound = listRightStriker[y_mid];

    if (tempIn.angle >= 180 && tempIn.angle < 360) { //moving to left so check right bound (ub)
        if (x_mid > left_bound) { //ub_x < 1570 &&
            constr_x = constrain((((double)x_mid-left_bound)/(right_bound-left_bound)), -1.0, 1.0);
            x = constrain(x, -constr_x, 1.0);
            // Serial.print(" in bounds ");
            // Serial.print(constr_x);
        }
        else { //if (ub_x <= 340) {
            constr_x = constrain(((left_bound-(double)x_mid)/left_bound), -1.0, 1.0); //change to 1 eventually
            x = constrain(x, constr_x, 1.0);
            // Serial.print(" out bounds ");
            // Serial.print(constr_x);
        }
    }
    else {
        if (x_mid < right_bound) { //lb_x > 250 &&
            constr_x = constrain(((right_bound-(double)x_mid)/(right_bound-left_bound)), -1.0, 1.0);
            x = constrain(x, -1.0, constr_x);
            // Serial.print(" in bounds ");
            // Serial.print(constr_x);
        }
        else {//if (lb_x >= 1480) {
            constr_x = constrain((((double)x_mid-right_bound)/left_bound), -1.0, 1.0); //change to -1 eventually
            x = constrain(x, -1.0, -constr_x);
            // Serial.print(" out bounds ");
            // Serial.print(constr_x);
        }
    }

    if (ub_x < left_bound) x = abs(x);
    else if (lb_x > right_bound) x = -abs(x);

    // if (currentX > 1260.0) {
    //     x = x*(lid_right_r/460.0);
    //     y = y*(lid_right_r/460.0);
    // }

    // Serial.print(" y: ");
    // Serial.print(y);
    
    double yMaxF = listF[x_mid];
    double yMaxB = listB[x_mid];
    double constr_y;
    if (tempIn.angle > 270 || tempIn.angle < 90) { // y = sqrt(947371.29 - pow(i-910.0, 2.0)) - 463.33;
        if (y_mid < FB_SD-yMaxF) {
            // constr_y = constrain(((2430.0-yMaxF-y_mid)/(2430.0-yMaxF)), -1.0, 1.0); //constrain it to the center
            //constr_y = constrain(((2430.0-yMaxF-y_mid)/(0.5*(2430.0-yMaxF))), -1.0, 1.0); //constrain it to the center
            //0.5 used to be here
            constr_y = constrain(((FB_SD-yMaxF-y_mid)/(MAX_SPEED_POINT*(FB_SD-yMaxF))), -1.0, 1.0); //constrain it to the center //increase number to make bot slow, decrease to make fast when closer to goal
            y = constrain(y, -1.0, constr_y);
            // Serial.print(" yMaxF: ");
            // Serial.print(yMaxF);
            // Serial.print(" in bounds  ");
            // Serial.println(constr_y);
        }
        else {
            constr_y = constrain(((y_mid-(FB_SD-yMaxF))/yMaxF), -1.0, 1.0); //yeet back
            y = constrain(y, -1.0, -constr_y);
            // Serial.print(" out bounds  ");
            // Serial.print(-constr_y);
        }
    }
    else {
        if (y_mid > yMaxB) {
            constr_y = constrain(((y_mid-yMaxB)/(FB_SD-yMaxB)), -1.0, 1.0);
            y = constrain(y, -constr_y, 1.0);
            // Serial.print("in bounds  ");

        }
        else {
            constr_y = constrain(((yMaxB-y_mid)/yMaxB), -1.0, 1.0);
            y = constrain(y, constr_y, 1.0);
            // Serial.print("out bounds  ");
            // Serial.println(constr_y);
        }
    }
    if (lb_y > FIELD_Y - OUT_FIELD_SIZE && ub_y > FIELD_Y - OUT_FIELD_SIZE) y = -abs(y);
    else if (lb_y < OUT_FIELD_SIZE && ub_y < OUT_FIELD_SIZE) y = abs(y);
    
    // double yMaxF = listFGoalie[x_mid];
    // double yMaxB = listB[x_mid];
    // double constr_y;
    // if (y_mid > yMaxF) {
    //     constr_y = constrain(((y_mid-yMaxF)/1200.0), -1.0, 1.0);
    //     y = constrain(y, -1.0, -constr_y);
    //     // Serial.print("FRONT ");
    //     // Serial.println(-constr_y);
    // }
    // else {
    //     constr_y = constrain(((yMaxB-y_mid)/yMaxB), -1.0, 1.0);
    //     y = constrain(y, constr_y, 1.0);
        // Serial.print("BACK  ");
        // Serial.println(constr_y);
//    }
    tempOut.angle = fmod(rad2deg(atan2(x, y)) + 360.0, 360.0); //0-360
    tempOut.speed = sqrt(pow(x, 2) + pow(y, 2));
    tempOut.rotation = proportionalCompass(tempOut.speed, currentCompass, targetCompass);
    
    return tempOut;
}

/*
Output slowDown(Output tempIn, int x_mid, int y_mid, double currentCompass, double targetCompass=0) {//int ub_x, int lb_x, int ub_y, int lb_y, double currentCompass, double targetCompass=0) {
    Output tempOut;

    double x = tempIn.speed*sin(deg2rad(tempIn.angle));
    double y = tempIn.speed*cos(deg2rad(tempIn.angle));
    // Serial.print(x);
    // Serial.print("   ");
    // Serial.print(y);
    // Serial.print("   ");
    //check why speed maxing out when seeing above wall
    double constr_x;
    if (tempIn.angle >= 180 && tempIn.angle < 360) { //moving to left so check right bound (ub)
        if (x_mid > 340) { //ub_x < 1570 &&
            constr_x = constrain((((double)x_mid-340.0)/1480.0), -1.0, 1.0);
            x = constrain(x, -constr_x, 1.0);
            // Serial.print("in bounds  ");
            // Serial.println(-constr_x);
        }
        else {//if (ub_x <= 340) {
            constr_x = constrain(((340.0-(double)x_mid)/340.0), -1.0,1.0); //change to 1 eventually
            x = constrain(x, constr_x, 1.0);
            // Serial.print("out bounds  ");
            // Serial.println(constr_x);
        }
    }

    else {
        if (x_mid < 1480) { //lb_x > 250 &&
            constr_x = constrain(((1480.0-(double)x_mid)/1480.0), -1.0, 1.0);
            x = constrain(x, -1.0, constr_x);
        }
        else {//if (lb_x >= 1480) {
            constr_x = constrain((((double)x_mid-1480.0)/340.0), -1.0, 1.0); //change to -1 eventually
            x = constrain(x, -1.0, -constr_x);
        }
    }
    // Serial.println(constr_x);

    // if (currentX > 1260.0) {
    //     x = x*(lid_right_r/460.0);
    //     y = y*(lid_right_r/460.0);
    // }
    
    double yMaxF = listF[x_mid];
    double yMaxB = listB[x_mid];
    double constr_y;
    if (tempIn.angle > 270 || tempIn.angle < 90) { // y = sqrt(947371.29 - pow(i-910.0, 2.0)) - 463.33;
        if (y_mid < 2430.0-yMaxF) {
//            constr_y = constrain(((2430.0-yMaxF-y_mid)/(2430.0-yMaxF)), -1.0, 1.0); //constrain it to the center
            //constr_y = constrain(((2430.0-yMaxF-y_mid)/(0.5*(2430.0-yMaxF))), -1.0, 1.0); //constrain it to the center
            constr_y = constrain(((2430.0-yMaxF-y_mid)/0.5*(2430.0-yMaxF)), -1.0, 1.0); //constrain it to the center
            y = constrain(y, -1.0, constr_y);
//             Serial.print("in bounds  ");
//             Serial.println(constr_y);
        }
        else {
            constr_y = constrain(((y_mid-(2430.0-yMaxF))/yMaxF), -1.0, 1.0); //yeet back
            y = constrain(y, -1.0, -constr_y);
            // Serial.print("out bounds  ");
            // Serial.println(-constr_y);
        }
    }
    else {
        if (y_mid > yMaxB) {
            constr_y = constrain(((y_mid-yMaxB)/(2430.0-yMaxB)), -1.0, 1.0);
            y = constrain(y, -constr_y, 1.0);
            // Serial.print("in bounds  ");

        }            // Serial.println(-constr_y);
        
        else {
            constr_y = constrain(((yMaxB-y_mid)/yMaxB), -1.0, 1.0);
            y = constrain(y, constr_y, 1.0);
            // Serial.print("out bounds  ");
            // Serial.println(constr_y);
        }
    }
    
    // double yMaxF = listFGoalie[x_mid];
    // double yMaxB = listB[x_mid];
    // double constr_y;
    // if (y_mid > yMaxF) {
    //     constr_y = constrain(((y_mid-yMaxF)/1200.0), -1.0, 1.0);
    //     y = constrain(y, -1.0, -constr_y);
    //     // Serial.print("FRONT ");
    //     // Serial.println(-constr_y);
    // }
    // else {
    //     constr_y = constrain(((yMaxB-y_mid)/yMaxB), -1.0, 1.0);
    //     y = constrain(y, constr_y, 1.0);
        // Serial.print("BACK  ");
        // Serial.println(constr_y);
//    }

    // Serial.print(currentX);
    // Serial.print("   ");
    // Serial.println(y);
    tempOut.angle = rad2deg(atan2(x, y));
    if (tempOut.angle < 0) tempOut.angle+=360.0;
    tempOut.speed = sqrt(pow(x, 2) + pow(y, 2));
    tempOut.rotation = proportionalCompass(currentCompass);
    // Serial.print("lbub");
    // Serial.print(lb_x);
    // Serial.print(" ");
    // Serial.println(ub_x);
    // Serial.print("asr");
    // Serial.print(tempOut.angle);
    // Serial.print("   ");
    // Serial.print(tempOut.speed);
    // Serial.print("   ");
    // Serial.println(tempOut.rotation);
    return tempOut;
}
*/

/*
x-axis: same as striker
y-axis: if in front of goal arc move back, if behind the goal arc move front (so it just stays on arc all the time)
*/
Output slowDownGoalie(Output tempIn, double x_mid, double y_mid, double currentCompass, double lb_x, double ub_x, double otherHeartbeat, int mtp=0, double targetCompass=0) {//int ub_x, int lb_x, int ub_y, int lb_y, double currentCompass, double targetCompass=0) {
    Output tempOut;

    if (tempIn.angle < 0) tempIn.angle += 360.0;

    double x = tempIn.speed * sin(deg2rad(tempIn.angle));
    double y = tempIn.speed * cos(deg2rad(tempIn.angle));

    // can try decreasing the max speed possible here 
    
    //check why speed maxing out when seeing above wall
    double constr_x;
    if (tempIn.angle >= 180.0 && tempIn.angle < 360.0) { //moving to left so check right bound (ub)
        if (x_mid > LR_LB_SD) { //ub_x < 1570 &&
            constr_x = constrain(((x_mid-LR_LB_SD)/(LR_UB_SD-LR_LB_SD)), 0.0, GOALIE_MAX_SPEED);
            x = constrain(x, -constr_x, 1.0);
            // Serial.print(" in bounds  ");
            // Serial.print(-constr_x);

        }
        else {//if (ub_x <= 340) {
            constr_x = constrain(((LR_LB_SD-x_mid)/LR_LB_SD), 0.0, GOALIE_MAX_SPEED); //change to 1 eventually
            x = constrain(x, constr_x, 1.0);
            // Serial.print(" out bounds  ");
            // Serial.print(constr_x);
        }
    }
    else {
        if (x_mid < LR_UB_SD) { //lb_x > 250 &&
            constr_x = constrain(((LR_UB_SD-x_mid)/(LR_UB_SD-LR_LB_SD)), 0.0, GOALIE_MAX_SPEED);
            x = constrain(x, -1.0, constr_x);
            // Serial.print(" in bounds  ");
            // Serial.print(-constr_x);
        }
        else {//if (lb_x >= 1480) {
            constr_x = constrain(((x_mid-LR_UB_SD)/LR_LB_SD), 0.0, GOALIE_MAX_SPEED); //change to -1 eventually
            x = constrain(x, -1.0, -constr_x);
            // Serial.print(" out bounds  ");
            // Serial.print(-constr_x);
        }
    }
    
    // Serial.println(constr_x);
    if (lb_x < LR_LB_SD && ub_x < LR_LB_SD) x = abs(x);
    else if (ub_x > LR_UB_SD && lb_x > LR_UB_SD) x = -abs(x);

    // if (currentX > 1260.0) {
    //     x = x*(lid_right_r/460.0);
    //     y = y*(lid_right_r/460.0);
    // }
    double yMaxF, yMaxB; 
    if (otherHeartbeat) {
        yMaxF = listFGoalie[(int)x_mid]; //these are the same arcs so the goalie stays in front of goal
        yMaxB = listFGoalie[(int)x_mid];
    }
    else {
        yMaxF = listFNoStriker[(int)x_mid];
        yMaxB = listFNoStriker[(int)x_mid];
    }
    double constr_y;
    // Serial.print(" x: ");
    // Serial.print(x);
    // Serial.print(" y: ");
    // Serial.print(y);
    // Serial.print(" ");
    // Serial.print(lb_x);
    // Serial.print("  ");
    // Serial.print(ub_x);
    // Serial.print(" x mid: ");
    // Serial.print(x_mid);
    // Serial.print(" y mid: ");
    // Serial.print(y_mid);
    // Serial.print(" ");
    // Serial.print(" yMaxB: ");
    // Serial.print(yMaxB);
    // Serial.print(" yMaxF: ");
    // Serial.print(yMaxF);
    // Serial.println();
    // Serial.print("  ");
    // Serial.print(tempOut.angle);

    // Serial.print(" MTP: ");
    // Serial.print(mtp);
    if (y_mid > yMaxF && !mtp) { // dont do if mtp
        // constr_y = constrain(((y_mid-yMaxF)/340.0), -1.0, 1.0);
        constr_y = (y_mid-yMaxF)/(FIELD_Y - OUT_PENALTY_SIZE); // new field: 2050.0
        y = constrain(y, -1.0, -constr_y);
        // Serial.print(" FRONT ");
        // Serial.print(-constr_y);
    }
    else if (y_mid < yMaxB) {
        constr_y = constrain(((yMaxB-y_mid)/yMaxB), 0.0, 1.0);
        // constr_y = pow(constr_y, 0.5);
        y = constrain(y, constr_y, 1.0);
        // Serial.print(" BACK  ");
        // Serial.print(constr_y);
    }
    // Serial.println();

    tempOut.angle = rad2deg(atan2(x, y));
    if (tempOut.angle < 0.0) tempOut.angle += 360.0;
    tempOut.speed = sqrt(pow(x, 2) + pow(y, 2));
    tempOut.rotation = proportionalCompass(tempOut.speed, currentCompass);
    
    return tempOut;
}

/*
need to rewrite to include ballcap sensor
ball_dist = ballDist bc they used to convert px to cm on teensy but now weer doing it on the openmv side

1. check if there is a ball
2. determine whether to aim
-> if there is a ball and ball is in ballcap (ballDist == 0) aim, reset aim time
-> otherwise if ball not in ball cap and time since ball is in bc <=15ms or less then continue aiming (ball flicker)
-> otherwise if no ballcap and time since ball is in bc <=15ms then stop aiming (either score alr or lost ball)
3. check if camera is seeing goal
??????
*/
/*
void ballCapping(int ballAng, int ballDist, unsigned long long anyBall) {
    // double ball_dist = BALL_DIST_A*pow((double)ballDist, 2) + BALL_DIST_B*(double)ballDist + BALL_DIST_C;
    double ball_dist = ballDist;
    double ballX = ball_dist * sin(deg2rad(ballAng));
    double ballY = ball_dist * cos(deg2rad(ballAng));
    
    //ballDist = 0 means that the ball is in the ballcap, ballAng not blankchar means can see yellow goal
    if (anyBall < noBall) balling = 1; //there is ball
    else balling = 0;

    if (ballDist == 0 && balling) { //&& ballAng != blankChar) {
        aiming = 1;
        goalieTransition = 0;
        aimCount = 0; //time since in ballcap
    }
    else if (aimCount < 16) { //within 10ms of balldist = 0, continue aiming (jic flicker)
        goalieTransition = 0; //goalie to striker
        aiming = 1;
    }
    else {
        goalieTransition = 0;
        aiming = 0;
    }

    //does camera see goal
    if (((ballDist == 0 && ballAng < 360.0)) && balling) {
        camGoal = 1;
        camGoalTime = 0;
    }
    else if (camGoalTime < 16) camGoal = 1;
    else camGoal = 0;
    //100323 removed dribbler
}
*/

/*
1. calculate multipliers
-if using bt dist must /10 bc sending in mm
-back_mul -> run straight twds ball at 100cm away, run pure orbit when (balldist-15) to prevent bot from colliding with the bol
-dist_mul -> run faster further away from ball
-angle_mul -> run faster when bol more behind the bot

2. ball orbit
case 1: in ballcap
-> further away, move more left/right, as bot approaches bol move straight twds it
-> (not added in) change orbit direction based on which side of field bot is on

case 2: not in ball cap
case 2.1: ball on the left
-> if not near the ballcap cone (0-30deg) then more aggressive orbit (run tangent to ball when right next to it)
-> if near ballcap cone then run less aggressive to move more twds bol (nearer to the cone, less aggressive)
case 2.2: ball on the right
-> same as 2.1 but the other direction

3. then calculate speed using distance from bol and bol angle
*/

//new balltrack
Output ballTrack(double speed, double ballAng, double ballDist, double x_mid, double currentCompass, bool bt = 0, double targetCompass=0.0){
    Output tempOut;
    //back mul (then u multiply the 90 by the back mul so it ensures it actly goes in that direction
    double ball_front_pow = 1.0; //not used
    double ball_back_pow = 5.5; //not used
    //double ball_dist = listBallDist[ballDist];
    // double ball_dist = listBallDist[ballDist];

    double ball_dist = 0;
    //if(!bt) ball_dist = BALL_DIST_A*pow((double)ballDist, 2) + BALL_DIST_B*(double)ballDist + BALL_DIST_C;
    if(!bt) ball_dist = ballDist;
    else ball_dist = ballDist / 10.0; //bc other bot sending in mm
    
    //if((ballAng > 90) && (ballAng < 180)) ballDist -= 13.0;
    double back_mul = (MAX_BALL_DIST - constrain(ball_dist - MIN_BALL_DIST, 0, MAX_BALL_DIST)) / (MAX_BALL_DIST); //100- //70.0-
    double dist_mul = constrain((ball_dist / FIELD_INNER_DIAGONAL), 0.0, 1.0);

    // Serial.print(" ");
    // Serial.print(dist_mul);
    // Serial.print(" ");
    // Serial.print(back_mul);
    // Serial.print(" ");
    // Serial.println();

    double a = fmod((ballAng + 180.0), 360.0) - 180.0; //-180 to 180
    double angle_mul = constrain( (abs(a)-12.0) / 168.0, 0.0, 1.0); // change this so that 12 deg is 0 speed
    tempOut.angle = ballAng;

    // double ballX = ball_dist * sin(deg2rad(ballAng));
    // double ballY = ball_dist * cos(deg2rad(ballAng));
    // ballcapY = 12; //100323 previously used to check ballcap here but moved to ballcapping

    if (ballAng >= BCZ_CAM_ANG_LEFT || ballAng <= BCZ_CAM_ANG_RIGHT) { //if in ballcap cone (find min max range of ballcap)
        double correction = pow((ball_dist / 20.0), 1.0); //max is 47.65????
        correction = constrain(correction, 0.0, 1.0);
        
        if (ballAng > 180.0) tempOut.angle = (360.0-((360.0-(double)ballAng)*correction));
        else tempOut.angle *= correction;
        
        tempOut.angle = constrain(tempOut.angle, 0.0, 360.0);
        // tempOut.speed = 0.3 - dist_mul * 0.3;
        tempOut.speed = 0.14 + 0.43*pow(dist_mul, 1.0) + 0.43*pow(angle_mul, 1.0);//constrain(((ballDist-minDist)*maxSpeed)/(maxDist-minDist), minSpeed, maxSpeed);
    }
    // if (ballX > -1.375 && ballX < 1.375 /*&& ballAng > 180.0 && ballAng < 90.0*/) {
    //     tempOut.angle *= (ball_dist - 12.0)/300.0;
    //     tempOut.angle = constrain(tempOut.angle, 0, 359);
    //     Serial.println("in bc");
    // }
    else {
        double leftAng, rightAng;
        // back_mul = pow(back_mul, 1.5);
        // want to max out angle at the boundary
        // range of angles expands as we get closer to boundary
        rightAng = 180.0 - abs(x_mid - MID_X) / MID_X * 80.0;
        leftAng = 180.0 + abs(x_mid - MID_X) / MID_X * 80.0;
        // Serial.print(abs(max(x_mid - 210.0, 0.0) - 910.0));
        // Serial.print(" left: ");
        // Serial.print(leftAng);
        // Serial.print(" right: ");
        // Serial.print(rightAng);
        // Serial.print(back_mul);
        // Serial.print(" ");
        // Serial.print(pow(back_mul, 2.0));
        // Serial.println();
        // (ballAng >= rightAng && ballAng < 180 && x_mid < 910.0) || (ballAng > 180 && ballAng <= leftAng && x_mid > 910.0)
        if (ballAng >= rightAng && ballAng <= leftAng) {//ballAng >= 135 && ballAng <= 225) { // should also check that bot is on "wrong" side
            back_mul = pow(back_mul, 1.5);
            // Serial.print("  wrong orbit ");
            // Serial.print(ballAng);
            if (x_mid < MID_X) tempOut.angle -= 90.0 * back_mul; // 90.0
            else tempOut.angle += 90.0 * back_mul; // 270.0
            tempOut.angle = fmod(tempOut.angle + 360.0, 360.0);
            // Serial.print(x_mid);
            // Serial.print("  ");
            // Serial.println(tempOut.angle);
            tempOut.speed = 0.14 + 0.86*pow(dist_mul, 0.5);//constrain(((ballDist-minDist)*maxSpeed)/(maxDist-minDist), minSpeed, maxSpeed);
        }
        else {
            back_mul = pow(back_mul, 0.75);
            // Serial.print(" yes ");
            //if(ballAng < 90)tempOut.angle *= front_mul; //0-89
            //else if (ballAng >=270) tempOut.angle = 360-((360-tempOut.angle)*front_mul); //270-359
            if (ballAng >= 180.0) {
                // tempOut.angle -= 90*back_mul;
                if (ballAng < 330.0) tempOut.angle -= 90.0 * back_mul; //orbit right
                else tempOut.angle -= 3.0 * (360.0 - tempOut.angle) * back_mul;
            }
            else if (ballAng < 180.0) {
                if (ballAng > 30.0) tempOut.angle += 90 * back_mul; //orbit left
                else tempOut.angle += 3.0 * fmod(tempOut.angle, 360.0) * back_mul;
            }
            tempOut.speed = 0.14 + 0.43*pow(dist_mul, 1.0) + 0.43*pow(angle_mul, 1.0);//constrain(((ballDist-minDist)*maxSpeed)/(maxDist-minDist), minSpeed, maxSpeed);
        }
        // tempOut.speed = 0.1 + 0.45 * pow(dist_mul, 1.0) + 0.45 * pow(angle_mul, 1.0);
    }

    if (tempOut.angle < 0.0) tempOut.angle += 360.0; 

    // tempOut.speed = 0.05 + 0.475 * pow(dist_mul, 1.0) + 0.475 * pow(angle_mul, 1.0);
    // tempOut.speed = 0.2 + 0.4 * pow(dist_mul, 1.0) + 0.4 * pow(angle_mul, 1.0);
    // Serial.print(tempOut.speed);
    // tempOut.speed = 0.2;
    // tempOut.speed = 0.1;

    // use the move angle instead of angle mul
    //leftright 0, back max, front 0
    // double final_ang_error = max((90.0 - abs(tempOut.angle - 180.0)) / 90.0, 0.0);
    // constrain(abs(fmod(tempOut.angle - 180.0, 360.0) - 90.0) / 90.0, 0.0, 1.0);
    // Serial.print(tempOut.angle);
    // Serial.print(" ");
    // Serial.println(final_ang_error);

    // tempOut.speed = 0.2; // + 0.3 * pow(final_ang_error, 2.0);
    tempOut.speed = constrain(tempOut.speed, 0.0, 1.0);
    tempOut.rotation = proportionalCompass(tempOut.speed, currentCompass, targetCompass);

    // Serial.println(tempOut.angle);

    return tempOut;
}

/*
OLD BALL TRACK
Output ballTrack(double speed, double ballAng, double ballDist, double x_mid, double currentCompass, bool bt = 0, double targetCompass=0.0){
    Output tempOut;
    //back mul (then u multiply the 90 by the back mul so it ensures it actly goes in that direction
    double ball_front_pow = 1.0; //not used
    double ball_back_pow = 5.5; //not used
    //double ball_dist = listBallDist[ballDist];
    // double ball_dist = listBallDist[ballDist];

    double ball_dist = 0;
    //if(!bt) ball_dist = BALL_DIST_A*pow((double)ballDist, 2) + BALL_DIST_B*(double)ballDist + BALL_DIST_C;
    if(!bt) ball_dist = ballDist;
    else ball_dist = ballDist / 10.0; //bc other bot sending in mm
    
    //if((ballAng > 90) && (ballAng < 180)) ballDist -= 13.0;
    double back_mul = (MAX_BALL_DIST - constrain(ball_dist - MIN_BALL_DIST, 0, MAX_BALL_DIST)) / (MAX_BALL_DIST); //100- //70.0-
    back_mul = pow(back_mul, 0.5);
    double dist_mul = constrain((ball_dist / FIELD_INNER_DIAGONAL), 0.0, 1.0);

    // Serial.print(" ");
    // Serial.print(dist_mul);
    // Serial.print(" ");
    // Serial.print(back_mul);
    // Serial.print(" ");
    // Serial.println();

    double a = fmod((ballAng + 180.0), 360.0) - 180.0; //-180 to 180
    double angle_mul = constrain( (abs(a)-12.0) / 168.0, 0.0, 1.0); // change this so that 12 deg is 0 speed
    tempOut.angle = ballAng;

    // double ballX = ball_dist * sin(deg2rad(ballAng));
    // double ballY = ball_dist * cos(deg2rad(ballAng));
    // ballcapY = 12; //100323 previously used to check ballcap here but moved to ballcapping

    if (ballAng >= 348.0 || ballAng <= 12.0) { //if in ballcap cone (find min max range of ballcap)
        double correction = pow((ball_dist / 20.0), 1.0); //max is 47.65????
        correction = constrain(correction, 0.0, 1.0);
        
        if (ballAng > 180.0) tempOut.angle = (360.0-((360.0-(double)ballAng)*correction));
        else tempOut.angle *= correction;
        
        tempOut.angle = constrain(tempOut.angle, 0.0, 360.0);
        // tempOut.speed = 0.3 - dist_mul * 0.3;
    }
    // if (ballX > -1.375 && ballX < 1.375 && ballAng > 180.0 && ballAng < 90.0) {
    //     tempOut.angle *= (ball_dist - 12.0)/300.0;
    //     tempOut.angle = constrain(tempOut.angle, 0, 359);
    //     Serial.println("in bc");
    // }
    else {
        double leftAng, rightAng;
        // want to max out angle at the boundary
        // should use constant
        rightAng = 180.0 - abs(x_mid - 910.0) / 910.0 * 80.0;
        leftAng = 180.0 + abs(x_mid - 910.0) / 910.0 * 80.0; // range of angles expands as we get closer to boundary
        // Serial.print(abs(max(x_mid - 210.0, 0.0) - 910.0));
        // Serial.print(" left: ");
        // Serial.print(leftAng);
        // Serial.print(" right: ");
        // Serial.print(rightAng);
        if (ballAng >= rightAng && ballAng <= leftAng) {//ballAng >= 135 && ballAng <= 225) {
            
            // Serial.print(ballAng);
            if (x_mid < 910.0) tempOut.angle -= 90.0 * back_mul; // 90.0
            else tempOut.angle += 90.0 * back_mul; // 270.0
            tempOut.angle = fmod(tempOut.angle + 360.0, 360.0);
            // Serial.print(x_mid);
            // Serial.print("  ");
            // Serial.println(tempOut.angle);
        }
        else {
            // Serial.print(" yes ");
            //if(ballAng < 90)tempOut.angle *= front_mul; //0-89
            //else if (ballAng >=270) tempOut.angle = 360-((360-tempOut.angle)*front_mul); //270-359
            if (ballAng >= 180.0) {
                // tempOut.angle -= 90*back_mul;
                if (ballAng < 330) tempOut.angle -= 90 * back_mul; //orbit right
                else tempOut.angle -= 3 * (360 - tempOut.angle) * back_mul;
            }
            else if (ballAng < 180) {
                if (ballAng > 30) tempOut.angle += 90 * back_mul; //orbit left
                else tempOut.angle += 3 * fmod(tempOut.angle, 360) * back_mul;
            }
        }
        // tempOut.speed = 0.1 + 0.45 * pow(dist_mul, 1.0) + 0.45 * pow(angle_mul, 1.0);
    }

    if (tempOut.angle < 0.0) tempOut.angle += 360.0; 

   tempOut.speed = 0.14 + 0.43*pow(dist_mul, 1.0) + 0.43*pow(angle_mul, 1.0);//constrain(((ballDist-minDist)*maxSpeed)/(maxDist-minDist), minSpeed, maxSpeed);
    // tempOut.speed = 0.05 + 0.475 * pow(dist_mul, 1.0) + 0.475 * pow(angle_mul, 1.0);
    // tempOut.speed = 0.2 + 0.4 * pow(dist_mul, 1.0) + 0.4 * pow(angle_mul, 1.0);
    // Serial.print(tempOut.speed);
    // tempOut.speed = 0.2;
    // tempOut.speed = 0.1;

    // use the move angle instead of angle mul
    //leftright 0, back max, front 0
    // double final_ang_error = max((90.0 - abs(tempOut.angle - 180.0)) / 90.0, 0.0);
    // constrain(abs(fmod(tempOut.angle - 180.0, 360.0) - 90.0) / 90.0, 0.0, 1.0);
    // Serial.print(tempOut.angle);
    // Serial.print(" ");
    // Serial.println(final_ang_error);

    // tempOut.speed = 0.2; // + 0.3 * pow(final_ang_error, 2.0);
    tempOut.speed = constrain(tempOut.speed, 0.0, 1.0);
    tempOut.rotation = proportionalCompass(tempOut.speed, currentCompass, targetCompass);

    // Serial.println(tempOut.angle);

    return tempOut;
}
*/

/*
Output ballTrack(double speed, int ballAng, int ballDist, double x_mid, double currentCompass, double targetCompass=0){
    Output tempOut;
    //back mul (then u multiply the 90 by the back mul so it ensures it actly goes in that direction
    double maxDist = 172.0-17.0, minDist = 80.0; // to see
    double maxSpeed = 0.6, minSpeed = 0.13;
    double ball_front_pow = 1.0;
    double ball_back_pow = 5.5;
    //search lookupt
    //double ball_dist = listBallDist[ballDist];
    double ball_dist = 0.0351*pow((double)ballDist, 2) - 1.553*(double)ballDist + 26.889; //in cm //make into lookup table
    
    // double front_mul =1.0+constrain((abs(maxDist-ball_dist)/(maxDist-minDist),ball_front_pow), 0.0, 1.0); //needs
    //if((ballAng > 90) && (ballAng < 180)) ballDist -= 13.0;
    double back_mul = constrain(abs((70.0-ball_dist)/70.0), 0.0, 1.0); //100-
    double dist_mul = constrain((ball_dist/100.0), 0.0, 1.0);
    double a = (ballAng + 180) % 360 - 180;
    double angle_mul = constrain(abs(a/180), 0.0, 1.0);
    tempOut.angle = ballAng;
    // Serial.print("x pos: ");
    // Serial.print(x_mid);
    // Serial.print(" angle: ");
    // Serial.print(ballAng);
    // Serial.print(" distance: ");
    // Serial.println(ball_dist);
    // Serial.print("   ");
    double ballX = ball_dist * sin(deg2rad(ballAng));
    double ballY = ball_dist * cos(deg2rad(ballAng));
    //if (ballX > -3.0  && ballX < 3.0  && (ballAng < 90.0 || ballAng > 270.0)) {
    ballcapY = 12;
    if (ballAng > 351.0 || ballAng < 9.0) {//ballX > -3.0  && ballX < 3.0  && (ballAng < 90.0 || ballAng > 270.0)) {//ballAng > 351.0 || ballAng < 9.0) {// //in ballcap rectangle
        double correction = pow((ball_dist/100.0), 1.0); //max is 47.65????
        correction = constrain(correction, 0.0, 1.0);
        // Serial.println("bruh");
        // Serial.println(ballX);
        // Serial.println(ballAng);
        // Serial.println(ballY);
        if (ballY < ballcapY && ballY > 0){ //front in ball cap
            // Serial.println("in bc");
            aiming = 1;
        }
        if (ballAng > 180.0) {
            tempOut.angle = (360.0-((360.0-(double)ballAng)*correction));
            // Serial.println(360.0-(360.0-(double)ballAng)*correction);
        }
        else {
            tempOut.angle *= correction;
        }
        // tempOut.angle *= (ball_dist - 12.0)/300.0;
        tempOut.angle = constrain(tempOut.angle, 0.0, 360.0);
        // Serial.print("in bc rect   angle: ");
        // Serial.print(ballAng);
        // Serial.print(" bot angle: ");
        // Serial.println(tempOut.angle);
    }
    // if (ballX > -1.375 && ballX < 1.375 /*&& ballAng > 180.0 && ballAng < 90.0) {
    //     tempOut.angle *= (ball_dist - 12.0)/300.0;
    //     tempOut.angle = constrain(tempOut.angle, 0, 359);
    //     Serial.println("in bc");
    // }
    // else if (ballAng >= 177.0 && ballAng <= 183.0) {//ballAng >= 135 && ballAng <= 225) {
    //     if (x_mid > 900.0) { //do scaling
    //         tempOut.angle += 270.0;//90*back_mul; //270.0 (27- BALLANG
    //         // Serial.print("orbit left");
    //     }
    //     else if (x_mid < 900.0) {
    //         tempOut.angle -= 90*back_mul;
    //         // Serial.print("orbit right");
    //     }
    // }
    else {
        // Serial.println(ballAng);
        //if(ballAng < 90)tempOut.angle *= front_mul; //0-89
        //else if (ballAng >=270) tempOut.angle = 360-((360-tempOut.angle)*front_mul); //270-359
        if (ballAng >= 180) {
            // tempOut.angle -= 90*back_mul;
            if (ballAng < 330) {
                tempOut.angle -= 90*back_mul; //180-269
                // Serial.println("here");
            }
            else {
                tempOut.angle -= 3*(360 - tempOut.angle)*back_mul;
                // Serial.print(a);
                // Serial.print("   ");
                
            }
        }
        else if (ballAng < 180) {
            if (ballAng > 30) {
                tempOut.angle += 90*back_mul; //90-179
                // Serial.println("no here");
            }
            else {
                tempOut.angle += 3*fmod(tempOut.angle, 360)*back_mul;
                // (180-tempOut.angle)/(150)
                // Serial.print(a);
                // Serial.print("   ");
                // Serial.println(tempOut.angle);
            }
        }

        // Serial.println("yes ball");
    }
    // Serial.print(ballAng);
    // Serial.print("   ");
    // Serial.print(dist_mul);
    // Serial.print("   ");
    // Serial.print(ball_dist);
    // Serial.print("   ");

    // Serial.println(tempOut.angle);
    
    tempOut.speed = 0.5*pow(dist_mul, 1.0) + 0.5*pow(angle_mul, 1.0);//constrain(((ballDist-minDist)*maxSpeed)/(maxDist-minDist), minSpeed, maxSpeed);
    tempOut.speed = constrain(tempOut.speed, 0.0, 1.0);
    // Serial.println(tempOut.speed);
    // tempOut.speed = constrain(tempOut.speed, 0.0, 0.4);
    tempOut.rotation = proportionalCompass(tempOut.speed, currentCompass, targetCompass);

    return tempOut;
}
*/

/*
1. calculate multipliers
-if using bt dist must /10 bc sending in mm
-dist_mul -> (NOT USED) run faster when ball is further away (?)
-angle_mul -> run faster when bol more behind the bot

2. move to block ball
case 1: ball in front
-> calculate angleerror btn (robot and goal) and (ball and goal) so if bot is directly blocking the bol angleerr = 0
case 1.1: if ball not directly in front
-> calc distRatio (same as dist mul) -> run faster when ball further away
-> if robot is to the left of the line joining the ball and the goal, move right owise left
case 1.2: if ball directly in front no move

case 2: ball at back -> move left/right depending on which side it is at at fixed speed

(theres some random orbit based on where bot is that shouldnt be there)
*/
Output ballTrackGoalie(double speed, int ballAng, int ballDist, double x_mid, double y_mid, double currentCompass, bool bt=0, double targetCompass=0 ){
    Output tempOut;
    //back mul (then u multiply the 90 by the back mul so it ensures it actly goes in that direction

    //BALL_DIST_A*pow((double)ballDist, 2) + BALL_DIST_B*(double)ballDist + BALL_DIST_C; //cm
    double ball_dist = 0;
    if(!bt) ball_dist = ballDist;
    else ball_dist = ballDist / 10.0;

    double botToGoall = botToGoal(x_mid, y_mid); //angle to own goal, -180 to 180
    //if((ballAng > 90) && (ballAng < 180)) ballDist -= 13.0;
    tempOut.angle = ballAng;
    if (currentCompass <= 180.0) ballAng = fmod(ballAng + currentCompass, 360.0);
    else ballAng = fmod(ballAng - (360.0 - currentCompass) + 360.0, 360.0);

    double percentageDist = pow(constrain((max(ball_dist - MIN_BALL_DIST, 0.0) / (MAX_BALL_DIST - MIN_BALL_DIST)), 0.0, 1.0), GOALIE_PERCENT_DIST_POW);
    // double a = (ballAng + 180) % 360 - 180;
    // double angle_mul = constrain(abs(a / 180), 0.0, 1.0);

    if (ballAng < 90.0 || ballAng > 270.0) { // in front
        double goalError = ballAng - botToGoall;
        goalError = fmod((goalError + 180.0), 360.0) - 180.0; //-180 to 180
        double ballError = fmod((ballAng + 180.0), 360.0) - 180.0; //-180 to 180
        double angleError = percentageDist * goalError + (1.0 - percentageDist) * ballError;
        // Serial.print(" x mid: ");
        // Serial.print(x_mid);
        // Serial.print(" y mid: ");
        // Serial.print(y_mid);
        // Serial.print(" goal error: ");
        // Serial.print(goalError);
        // Serial.print(" ball error: ");
        // Serial.print(ballError);
        // Serial.print(" dist: ");
        // Serial.print(percentageDist);
        // Serial.print(" ang error: ");
        // Serial.println(angleError);

        if (abs(angleError) >= 3.0) { //previously !=, ball not directly in front
            double distRatio = constrain((MAX_BALL_DIST - constrain(ball_dist-MIN_BALL_DIST, 0.0, MAX_BALL_DIST)) / MAX_BALL_DIST, 0.0, 1.0);
            // 0.1 base speed cos it is stable at 0.1 speed
            tempOut.speed = 0.05 + constrain(pow(abs(angleError) / 90.0, 1.0), 0.0, distRatio);
            // move super fast to hit ball when near bot
            // when far from bot, put in an ideal position
        
            if (angleError > 0) { //right
                // tempOut.angle = 90.0; //eventually want to follow arc
                tempOut.angle = listAngGoalieRight[(int)x_mid];
            }
            else { //to the left
                // tempOut.angle = 270.0;
                tempOut.angle = listAngGoalieLeft[(int)x_mid];
            }
            // Serial.print(" balltrack ang: ");
            // Serial.print(tempOut.angle);
        }
        else tempOut.speed = 0; //change to pure compass correction
    }
    else { // in back
        // move to be in front of ball
        //cap so that gy doesnt die, since ball is behind no one will score (probably)
        
        //supposed to orbit (inwards) if the ball is behind so it doesnt own goal
        /*double back_mul = (MAX_BALL_DIST - constrain(ball_dist - MIN_BALL_DIST, 0, MAX_BALL_DIST)) / (MAX_BALL_DIST); //100- //70.0-
        double dist_mul = constrain((ball_dist / FIELD_INNER_DIAGONAL), 0.0, 1.0);

        double a = fmod((ballAng + 180.0), 360.0) - 180.0; //-180 to 180
        double angle_mul = constrain( (abs(a)-12.0) / 168.0, 0.0, 1.0); // change this so that 12 deg is 0 speed
        tempOut.angle = ballAng;

        double leftAng, rightAng;

        back_mul = pow(back_mul, 1.5);
        if (x_mid > MID_X) { //if on the right side of field orbit inwards (left)
            //if (ballAng > 30.0)
            tempOut.angle += 90 * back_mul; //orbit left
            //else tempOut.angle += 3.0 * fmod(tempOut.angle, 360.0) * back_mul;
        }
        else {
            //if (ballAng < 330.0)
            tempOut.angle -= 90.0 * back_mul; //orbit right
            //else tempOut.angle -= 3.0 * (360.0 - tempOut.angle) * back_mul;
        }
        tempOut.speed = 0.14 + 0.43 * pow(dist_mul, 1.0) + 0.43 * pow(angle_mul, 1.0);

        rightAng = 180.0 - abs(x_mid - MID_X) / MID_X * 80.0;
        leftAng = 180.0 + abs(x_mid - MID_X) / MID_X * 80.0;

        if (ballAng >= rightAng && ballAng <= leftAng) {
            back_mul = pow(back_mul, 3.0);
            if (x_mid < MID_X) tempOut.angle -= 90.0 * back_mul; // 90.0
            else tempOut.angle += 90.0 * back_mul; // 270.0
            tempOut.angle = fmod(tempOut.angle + 360.0, 360.0);
            tempOut.speed = 0.14 + 0.86 * pow(dist_mul, 0.5);//constrain(((ballDist-minDist)*maxSpeed)/(maxDist-minDist), minSpeed, maxSpeed);
        }
        else {
            back_mul = pow(back_mul, 1.5);
            if (ballAng >= 180.0) {
                if (ballAng < 330.0) tempOut.angle -= 90.0 * back_mul; //orbit right
                else tempOut.angle -= 3.0 * (360.0 - tempOut.angle) * back_mul;
            }
            else if (ballAng < 180.0) {
                if (ballAng > 30.0) tempOut.angle += 90 * back_mul; //orbit left
                else tempOut.angle += 3.0 * fmod(tempOut.angle, 360.0) * back_mul;
            }
            tempOut.speed = 0.14 + 0.43 * pow(dist_mul, 1.0) + 0.43 * pow(angle_mul, 1.0);
        }

        if (tempOut.angle < 0.0) tempOut.angle += 360.0; 

        tempOut.speed = constrain(tempOut.speed, 0.0, 1.0);
        tempOut.rotation = proportionalCompass(tempOut.speed, currentCompass, targetCompass);*/

        //normal left/right  movement
        //should decrease the angle at which we fall into this case
        tempOut.speed = abs(ballAng - 180.0) / 90.0;
        tempOut.speed = constrain(tempOut.speed, 0.0, 0.6); //slower?????
        if (ballAng < 180) tempOut.angle = listAngGoalieRight[int(x_mid)]; //90.0
        else tempOut.angle = listAngGoalieLeft[(int)x_mid]; // 270.0
    }
    // 2206: incorporate compass correction

    currentCompass = fmod((currentCompass + 180.0), 360.0) - 180.0; //-180 to 180
    tempOut.angle -= currentCompass; //if robot is tilted to the right (+ve currentCompass) you wanna subtract from the angle, if left then add (i.e. subtract -ve currentCompass)
    tempOut.angle = fmod(tempOut.angle + 360.0, 360.0); //change to 0-360 so the adding/subtracting is the correct direction
    tempOut.angle = fmod((tempOut.angle + 180.0), 360.0) - 180.0; //change back to -180-180

    tempOut.rotation = proportionalCompass(tempOut.speed, currentCompass, targetCompass);
    tempOut.speed = constrain(tempOut.speed, 0.0, 1.0);

    return tempOut;
}

//TODO: only send if conf treshold is above
/*
-number of rectangles depends on number available characters to send (d)
-calculate the h/w of each rectangle
-find the x-rectangle and y-rectangle ball is in
-calculate max x-rectangles
??cant figure
*/
int ballSendPos(int ball_x, int ball_y, unsigned long long anyBall, double conf_x, double conf_y) {
    double d = floor(sqrt(btremaining));
    double height = 2430 / d; //height of each rectangle in the grid
    double width = 1820 / d; //botwidth

    int botwidth = constrain(round((ball_x/width)-0.5), 0, d-1); 
    int botheight = constrain(round((ball_y/height)-0.5), 0, d-1); 
    
    int ballint = (botheight * d) + botwidth + 1;
    
    // Serial.print(d);
    // Serial.print(" ");
    // Serial.print(ball_x);
    // Serial.print(" ");
    // Serial.print(ball_y);
    // Serial.print(" ");
    // Serial.print(botwidth);
    // Serial.print(" ");
    // Serial.print(botheight);
    // Serial.print(" ball int: ");
    // Serial.print(ballint);
    // Serial.print(" ");
    // Serial.print(conf_x);
    // Serial.print(" ");
    // Serial.print(conf_y);
    // Serial.println();
    if (anyBall > noBall || conf_x < 0.8 || conf_y < 0.8) return 255;
    else return ballint;
}

void ballReceive(int ball_char, double x_mid, double y_mid) { //ball char is wtv other bot is sending
    if (ball_char != 255) { //if bot alive and sees ball
        ball_char -= 1;
        double d = floor(sqrt(btremaining));
        // Serial.print(d);
        // Serial.print("  ");
        double height = 2430 / d; //height of each rectangle in the grid
        double width = 1820 / d; //botwidth

        double ball_y = floor((ball_char/d)); // y
        double ball_x = ball_char - ball_y*d; // x

        ball_x = (width*ball_x) + width/2.0;
        ball_y = (height*ball_y) + height/2.0; 
        double to_bot_x = (double)ball_x - x_mid;
        double to_bot_y = (double)ball_y - y_mid;

        otherBallAng = rad2deg(atan2(to_bot_x, to_bot_y));
        otherBallAng = fmod(otherBallAng + 360.0, 360.0);
        otherBallDist = sqrt(pow(to_bot_x, 2.0) + pow(to_bot_y, 2.0));
        otherBallDist /= 10.0;
    }
    else {
        otherBallAng = -1;
        otherBallDist = -1;
    }
}

#endif