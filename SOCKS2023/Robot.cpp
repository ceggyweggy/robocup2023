#include <Robot.h>

Kicker kk(26);
Dribbler dribb;

Lidar2022 LidarF(FSERIAL, FRAME_1000);
Lidar2022 LidarR(RSERIAL, FRAME_1000);
Lidar2022 LidarL(LSERIAL, FRAME_1000);
Lidar2022 LidarBR(BRSERIAL, FRAME_1000);
Lidar2022 LidarBL(BLSERIAL, FRAME_1000);

//IF CHANGE LIDAR CHANGE THE SERIAL EVENT

GY2022 gy;

Motor2022 FL = Motor2022(FLP, FLN);
Motor2022 BL = Motor2022(BLP, BLN);
Motor2022 FR = Motor2022(FRP, FRN);
Motor2022 BR = Motor2022(BRP, BRN);
DriveBase db(&FL, &FR, &BL, &BR);


Camera2022 cam = Camera2022(CAMSERIAL);
BT2022 bbt(BTSERIAL);

// double prevAng = 0; //not being used
// int prevDist = 0;
double tempAng = -1; //prev ^1
double tempDist = -1;

// elapsedMillis _anyBall;

Robot::Robot() {
    pinMode(receivePin, INPUT); //receive from pro micro for line
    pinMode(sendPin, OUTPUT); //send to pro micro for kick
    pinMode(ballcapPin, INPUT);
}

void Robot::setup(int reset/*=0*/) {
    // if (reset == 2) bbt.clear()
    // analogWriteResolution(15);
}

void Robot::btSend(byte data) {
    bbt.send(data);
}

void Robot::debugGy() {
    Serial.println(_gy_angle);
}

void Robot::debugCamera(){
    // Serial.print("anyBall: ");
    // Serial.print(anyBall);
    Serial.print("  BALL A/D: ");
    Serial.print(ball_ang);
    Serial.print(" ");
    Serial.print(ball_dist);
    Serial.print("  GOAL L/R: ");
    Serial.print(goal_left_ang);
    Serial.print(" ");
    Serial.print(goal_right_ang);
    Serial.print("  anyGoal: ");
    Serial.print((int)anyGoal);
    // Serial.print("  YGOAL A/D: ");
    // Serial.print(ygoal_ang);
    // Serial.print(" ");
    // Serial.print(ygoal_dist);
    Serial.println();
}


void Robot::debugDist(int mode /*=3*/){
    if (mode == 1) { //tuning for lidar offsets
        Serial.print("RAW FRONT: ");
        Serial.print(lid_front_raw);
        Serial.print("  RAW RIGHT: ");
        Serial.print(lid_right_raw);
        Serial.print("  RAW LEFT: ");
        Serial.print(lid_left_raw);
        Serial.print("  RAW BACKRIGHT: ");
        Serial.print(lid_backright_raw);
        Serial.print("  RAW BACKLEFT: ");
        Serial.print(lid_backleft_raw);
        Serial.print(" xmid: ");
        Serial.print(x_mid);
        Serial.print(" ymid: ");
        Serial.print(y_mid);
        Serial.println();

        // Serial.print("  FRONT: ");
        // Serial.print(lid_front_r);
        // Serial.print("  RIGHT: ");
        // Serial.print(lid_right_r);
        // Serial.print("  LEFT: ");
        // Serial.print(lid_left_r);
        // Serial.print("BACKRIGHT: ");
        // Serial.print(lid_backright_r);
        // Serial.print("  BACKLEFT: ");
        // Serial.print(lid_backleft_r);
        // Serial.println();

        // Serial.print("CORRECTED  ");
        // Serial.print("FRONT: ");
        // Serial.print(lidF);
        // Serial.print("  RIGHT: ");
        // Serial.print(lidR);
        // Serial.print("  LEFT: ");
        // Serial.print(lidL);
        // Serial.print("  BACKRIGHT: ");
        // Serial.print(lidBR);
        // Serial.print("  BACKLEFT: ");
        // Serial.print(lidBR);
        // Serial.println();
    } else if (mode == 2) { //tuning for goal bounds
        Serial.print("CORRECTED  ");
        Serial.print("FRONT: ");
        Serial.print(lidF);
        Serial.print("  RIGHT: ");
        Serial.print(lidR);
        Serial.print("  LEFT: ");
        Serial.print(lidL);
        Serial.print("  BACKRIGHT: ");
        Serial.print(lidBR);
        Serial.print("  BACKLEFT: ");
        Serial.print(lidBR);
        Serial.print(" LB X: ");
        Serial.print(lb_x);
        Serial.print(" UB X: ");
        Serial.print(ub_x);
        Serial.print(" x mid: ");
        Serial.print(x_mid);
        Serial.print("  y mid: ");
        Serial.print(y_mid);
        Serial.println();

        // Serial.print("RAW FRONT: ");
        // Serial.print(lid_front_raw);

        // Serial.print("  BACKLEFT: ");
        // Serial.print(lidBL);
        // Serial.print("  BACKRIGHT: ");
        // Serial.print(lidBR);

        // Serial.print("  rBACKLEFT: ");
        // Serial.print(lid_backleft_raw);
        // Serial.print("  rBACKRIGHT: ");
        // Serial.print(lid_backright_raw);

        // Serial.print("  rLEFT: ");
        // Serial.print(lid_left_raw);
        // Serial.print("  rRIGHT: ");
        // Serial.print(lid_right_raw);


        // Serial.print(" LB X: ");
        // Serial.print(lb_x);
        // Serial.print(" UB X: ");
        // Serial.print(ub_x);
        // Serial.println();
    } else { //other random prints
        // Serial.print("angle: ");
        // Serial.print(_gy_angle);

        Serial.print("x mid: ");
        Serial.print(x_mid);
        Serial.print("  y mid: ");
        Serial.print(y_mid);
        // Serial.print("  ");
        Serial.print(" conf x: ");
        Serial.print(conf_x);
        Serial.print(" conf y: ");
        Serial.print(conf_y);
        // Serial.println();
        // Serial.print(lidL);
        // Serial.print("   ");
        // Serial.print(lidR);

        Serial.print(" LB X: ");
        Serial.print(lb_x);
        Serial.print(" UB X: ");
        Serial.print(ub_x);
        Serial.print("  LB Y: ");
        Serial.print(lb_y);
        Serial.print(" UB Y: ");
        Serial.print(ub_y);
        Serial.println();
    }
}

void Robot::updateReading() { //sense for line
    lid_left_raw = LidarL.get_reading(); //in cm
    // lid_left_raw = 0.0;
    lid_backright_raw = LidarBR.get_reading();
    lid_backleft_raw = LidarBL.get_reading();
    lid_front_raw = LidarF.get_reading();
    lid_right_raw = LidarR.get_reading();
    // lid_right_raw = 0.0;
    lid_front_r = constrain(lid_front_raw, 0.0, 243.0);
    lid_right_r = constrain(lid_right_raw, 0.0, 182.0);
    lid_left_r = constrain(lid_left_raw, 0.0, 182.0);
    lid_backright_r = constrain(lid_backright_raw, 0.0, 243.0);
    lid_backleft_r = constrain(lid_backleft_raw, 0.0, 243.0);

    _gy_angle = gy.get_angle(); //0-360

    line = digitalRead(receivePin);

    lightgate = analogRead(ballcapPin);

    angDist ballCur = cam.get_ball(); 
    tempAng = ball_ang; //0-360
    tempDist = ball_dist; //cm
    ball_ang = ballCur.ang; // should be compass corrected
    ball_dist = ballCur.dist; //pix
    
    //double temp_ball_ang; //NOT USED
    if (pika) {
        ball_dist_0 = 2.91 * exp(0.0332 * ball_dist);
        ball_dist_90 = 2.92 * exp(0.032 * ball_dist);
        ball_dist_180 = 2.05 * exp(0.0331 * ball_dist);
        ball_dist_270 = 2.69 * exp(0.0296 * ball_dist);

        double ang_error; //0-1 btn each quadrant
        if (ball_ang <= 90.0) {
            ang_error = ball_ang / 90.0;
            ball_dist = (1 - ang_error) * ball_dist_0 + ang_error * ball_dist_90;
        }
        else if (ball_ang <= 180.0) {
            ang_error = (ball_ang - 90.0) / 90.0;
            ball_dist = (1 - ang_error) * ball_dist_90 + ang_error * ball_dist_180;
        }
        else if (ball_ang <= 270.0) {
            ang_error = (ball_ang - 180.0) / 90.0;
            ball_dist = (1 - ang_error) * ball_dist_180 + ang_error * ball_dist_270;
        }
        else {
            ang_error = (ball_ang - 270.0) / 90.0;
            ball_dist = (1 - ang_error) * ball_dist_270 + ang_error * ball_dist_0;
        }
    }
    else ball_dist = 3.1883 * exp(ball_dist * 0.0303);

    // Serial.print(" lg: ");
    // Serial.print(lightgate);
    // Serial.print(" ball ang: ");
    // Serial.print(ball_ang);
    // Serial.print(" ball dist: ");
    // Serial.print(ball_dist);
    // Serial.println();

    // if (lightgate > 1000 || ((ball_ang > 350.0 || ball_ang <= 10.0) && ball_dist <= 8.0)) ballcap = 1;
    // if (lightgate > 1000 || ((ball_ang >= 354.0 || ball_ang <= 9.0) && ball_dist <= 9.0)) ballcap = 1; //bot 2
    // if (lightgate > 1000 || ((ball_ang >= 352.0 || ball_ang <= 15.0) && ball_dist <= 7.0)) ballcap = 1; //bot 1
    // else ballcap = 0;

    if (ballcap) { // light gate alr triggered, can refresh using camera or light gate
        if (lightgate > LG_THRESH || ((ball_ang >= BCZ_CAM_ANG_LEFT || ball_ang <= BCZ_CAM_ANG_RIGHT) && ball_dist <= BCZ_CAM_DIST)) {
            // Serial.println(" blop ");
            ballcap = 1;
        }
        else ballcap = 0;
    }
    else { // if triggering for the first time, must be through ballcap (so no false detection)
        if (lightgate > LG_THRESH) ballcap = 1; //bot 1
        else ballcap = 0;
    }

    leftRight goalCur = cam.get_goal(); 
    goal_left_ang = goalCur.left;
    goal_right_ang = goalCur.right;

    // Serial.println(goal_kick);
    
    anyBall = cam.anyBall; //time since last received
    anyGoal = cam.anyGoal;
    anyMessage = cam.anyMessage;
    
    btReceive = bbt.getReceiveVal();
    anyBluetooth = bbt.anyBluetooth;
}

void Robot::updatePos(int goalie /*=0*/) {
    lid_front_r += LID_FC;
    // lid_right_r += LID_RC; // since we r using scaling now
    // lid_left_r += LID_LC;
    lid_backright_r += LID_BRC;
    lid_backleft_r += LID_BLC;

    double correction = cos(deg2rad(_gy_angle)); 
    lidF = 10.0 * (double)lid_front_r * correction; //mm
    lidR = 10.0 * (double)lid_right_r * correction;
    lidL = 10.0 * (double)lid_left_r * correction;
    lidBR = 10.0 * (double)lid_backright_r * correction;
    lidBL = 10.0 * (double)lid_backleft_r * correction;

    if (_gy_angle <= 180.0) { //correct angles based on gy
        goal_right_ang = fmod(goal_right_ang + _gy_angle, 360.0);
        goal_left_ang = fmod(goal_left_ang + _gy_angle, 360.0);
    }
    else {
        goal_right_ang = fmod(goal_right_ang - (360.0 - _gy_angle) + 360.0, 360.0);
        goal_left_ang = fmod(goal_left_ang - (360.0 - _gy_angle) + 360.0, 360.0);
    }

    if ((goal_left_ang < 90.0 || goal_left_ang > 270.0) && (goal_right_ang < 90.0 || goal_right_ang > 270.0)) front_goal = 1; //which goal in front
    else front_goal = 0;

    if (front_goal) {
        // should not happen with new goal
        if (goal_left_ang >= 180.0 && goal_left_ang < 270.0) goal_left_ang = 270.0;
        else if (goal_left_ang > 90.0 && goal_left_ang < 180.0) goal_left_ang = 90.0;
        if (goal_right_ang >= 180.0 && goal_right_ang < 270.0) goal_right_ang = 270.0;
        else if (goal_right_ang > 90 && goal_right_ang < 180.0) goal_right_ang = 90.0;

        goal_ang_width = fmod((goal_right_ang - goal_left_ang + 360.0), 360.0);
        goal_mid_ang = fmod((goal_left_ang + 0.5 * goal_ang_width), 360.0);
        if (goal_left_ang > 180.0 && goal_left_ang < 359.0 && goal_right_ang > 1.0 && goal_right_ang < 180.0) goal_kick = 1; // if angle contains 0
        else goal_kick = 0;
        // if (goal_mid_ang <= 10.0 || goal_mid_ang >= 350.0) goal_kick = 1;
        // else goal_kick = 0;

        // check if need swap -- should not happen with new goal
        double remap_left_ang = fmod(goal_left_ang + 360.0 - 90.0, 180.0) - 90.0;
        double remap_right_ang = fmod(goal_right_ang + 360.0 - 90.0, 180.0) - 90.0;
        if (remap_right_ang < remap_left_ang) {
            // Serial.print(" SWAPPP ");
            double temp = goal_left_ang;
            goal_left_ang = goal_right_ang;
            goal_right_ang = temp;
        }
    }
    else {
        // should not happen with new goal
        goal_left_ang = min(270.0, max(90.0, goal_left_ang));
        goal_right_ang = min(270.0, max(90.0, goal_right_ang));

        goal_ang_width = fmod((goal_left_ang - goal_right_ang + 360.0), 360.0);
        goal_mid_ang = fmod((goal_right_ang + 0.5 * goal_ang_width), 360.0);
        if (goal_right_ang > 180.0 && goal_left_ang < 180.0) goal_kick = 1;
        else goal_kick = 0;
        // if (goal_mid_ang >= 170.0 && goal_mid_ang <= 190.0) goal_kick = 1;
        // else goal_kick = 0;

        // check if need swap -- should not happen with new goal
        if (goal_left_ang < goal_right_ang) {
            // Serial.print(" SWAPPP ");
            double temp = goal_left_ang;
            goal_left_ang = goal_right_ang;
            goal_right_ang = temp;
        }
    }
    
    if (front_goal) {
        corr_goal_left_ang = fmod(360.0 + 90.0 - goal_left_ang, 360.0);
        corr_goal_right_ang = fmod(360.0 + 90.0 - goal_right_ang, 360.0);

        goal_left_tan = tan(deg2rad(corr_goal_left_ang));
        goal_right_tan = tan(deg2rad(corr_goal_right_ang));

        goal_left_constant = GOAL_FRONT_Y - goal_left_tan * GOAL_LEFT_X;
        goal_right_constant = GOAL_FRONT_Y - goal_right_tan * GOAL_RIGHT_X;
    }
    else {
        corr_goal_left_ang = 270.0 - goal_left_ang;
        corr_goal_right_ang = 270.0 - goal_right_ang;

        goal_left_tan = tan(deg2rad(corr_goal_left_ang));
        goal_right_tan = tan(deg2rad(corr_goal_right_ang));

        goal_left_constant = GOAL_BACK_Y - goal_left_tan * GOAL_LEFT_X;
        goal_right_constant = GOAL_BACK_Y - goal_right_tan * GOAL_RIGHT_X;
    }

    bot_cam_x = (goal_left_constant - goal_right_constant) / (goal_right_tan - goal_left_tan); //(c2-c1)/(m1-m2)
    bot_cam_y = goal_right_tan * bot_cam_x + goal_right_constant;

    lb_x = lidL; //min((lidL), (FIELD_X - lidR)); //mm
    ub_x = FIELD_X - lidR; //max((lidL), (FIELD_X - lidR)); 

    lb_x = (lidL - LID_LC) / LID_LPROP; //min((lidL), (FIELD_X - lidR)); //mm
    ub_x = (lidR - LID_RC) / LID_RPROP; //max((lidL), (FIELD_X - lidR));

    // Serial.print("  ");
    // Serial.print(lb_x);
    // Serial.print("  ");
    // Serial.print(ub_x);
    // Serial.print("  ");

    if (ub_x < lb_x) ub_x = lb_x;

    // 3005: if bounds overlap, use sensor that returns shorter reading
    if (ub_x < lb_x) {
        if (lidL < lidR) ub_x = lb_x; // left lidar closer
        else if (lidR < lidL) lb_x = ub_x; // right lidar closer
    }

    lb_x = constrain(lb_x, 0.0, FIELD_X);
    ub_x = constrain(ub_x, 0.0, FIELD_X);
    //back adjusting
    bool bl_fix = 0, br_fix = 0;

    if (anyGoal < 100) {
        if (goal_left_ang == goal_right_ang) {
            // Serial.print(" IN CORNER ");
            // cannot tell which goal we are detecting, so cannot change y bound
            // but can change x bound based on angle
            if (goal_left_ang >= 0.0 && goal_left_ang <= 180.0 && goal_right_ang >= 0.0 && goal_right_ang <= 180.0) {
                // on left side of field
                ub_x = min(ub_x, GOAL_LEFT_X);
            }
            else {
                // on right side of field
                lb_x = max(lb_x, GOAL_RIGHT_X);
            }
        }
        // check validity of goal coord
        else {
            // x clipping limit to goal bounds
            if (bot_cam_x <= GOAL_LEFT_X) {
                ub_x = min(ub_x, GOAL_RIGHT_X);
            }
            else if (bot_cam_x >= GOAL_RIGHT_X) lb_x = max(lb_x, GOAL_LEFT_X);
            else {
                // lb_x and ub_x btwn 610 and 1210
                lb_x = max(lb_x, GOAL_LEFT_X);
                ub_x = min(ub_x, GOAL_RIGHT_X);
            }
        }

        if (lb_x > ub_x) { // swap, happens when cam val is > 1210, but lidars say within goal bounds
            double temp = lb_x;
            lb_x = ub_x;
            ub_x = temp;
        }
    }

	// Serial.print(" lb_x: ");
	// Serial.print(lb_x);
	// Serial.print(" ub_x: ");
	// Serial.print(ub_x);
	// Serial.print(" og lidBL: ");
	// Serial.print(lidBL);
	// Serial.print(" og lidBR: ");
	// Serial.print(lidBR);
	// Serial.print(" x mid: ");
	// Serial.print(x_mid);
	// Serial.print(" y mid: ");
	// Serial.print(y_mid);
	// Serial.print(" gy angle: ");
	// Serial.print(_gy_angle);
    // Serial.println();

    if ((((lb_x >= POST_LB_X) && (lb_x <= POST_UB_X) && (ub_x >= POST_LB_X) && (ub_x <= POST_UB_X)))) { //completely in, if goalie, both in
        if ((lidBL - lidBR) > 20) lidBR += POST_BACK_BR;
		// else if ((lidBL - lidBR) > 10) {
		// 	lidBR += GOAL_BACK_BR;
		// 	lidBL += POST_BACK_BL;
		// }
		else if ((lidBR - lidBL) > 20) lidBL += POST_BACK_BL;
		// else if ((lidBR - lidBL) > 10) {
		// 	lidBR += POST_BACK_BR;
		// 	lidBL += GOAL_BACK_BL;
		// }
		else {
			lidBL += POST_BACK_BL; //mm
            lidBR += POST_BACK_BR;
		}
		lidF += POST_BACK_F;

        /*if ((!bl_fix) && (!br_fix)) {
            lidBL += POST_BACK_BL; //mm
            lidBR += POST_BACK_BR;
            bl_fix = 1;
            br_fix = 1;
        }
        else if ((lidBL - lidBR) > 20) { //left out, right in
            lidBR += POST_BACK_BR;
        }
        else { //left in, right out 
            lidBL += POST_BACK_BL;
        }
        lidF += POST_BACK_F; //conf out oso*/
    }

    //y computing
    int lidB = max(lidBL, lidBR); 
    lb_y = lidB; //mm
    ub_y = FIELD_Y - lidF;
    if (ub_y < lb_y) ub_y = lb_y;
    lb_y = constrain(lb_y, 0, FIELD_Y);
    ub_y = constrain(ub_y, 0, FIELD_Y);

    if (anyGoal < 100) {
        if (goal_left_ang != goal_right_ang) {
            if (front_goal) { // update the lower bound
                if (bot_cam_y > ub_y) lb_y = ub_y;
                else if (bot_cam_y > lb_y) lb_y = bot_cam_y;
            }
            else {
                if (bot_cam_y < lb_y) ub_y = lb_y;
                else if (bot_cam_y < ub_y) ub_y = bot_cam_y;
            }
        }
    }

    // Serial.print(" lb x: ");
    // Serial.print(lb_x);
    // Serial.print(" ub x: ");
    // Serial.print(ub_x);
    // Serial.print(" lb y: ");
    // Serial.print(lb_y);
    // Serial.print(" ub y: ");
    // Serial.print(ub_y);

    // Serial.print(" gl: ");
    // Serial.print(goal_left_ang);
    // Serial.print(" gr: ");
    // Serial.print(goal_right_ang);
    // Serial.println();

    // Serial.print(" fg: ");
    // Serial.print(front_goal);
    // Serial.print(" gl: ");
    // Serial.print(goal_left_ang);
    // Serial.print(" gr: ");
    // Serial.print(goal_right_ang);
    // Serial.print(" corr gl: ");
    // Serial.print(corr_goal_left_ang);
    // Serial.print(" corr gr: ");
    // Serial.print(corr_goal_right_ang);
    // Serial.print(" gl grad: ");
    // Serial.print(goal_left_tan);
    // Serial.print(" gr grad: ");
    // Serial.print(goal_right_tan);
    // Serial.print(" gl const: ");
    // Serial.print(goal_left_constant);
    // Serial.print(" gr const: ");
    // Serial.print(goal_right_constant);
    // Serial.print(" cx: ");
    // Serial.print(bot_cam_x);
    // Serial.print(" cy: ");
    // Serial.print(bot_cam_y);
    // Serial.print(" lb x: ");
    // Serial.print(lb_x);
    // Serial.print(" ub x: ");
    // Serial.print(ub_x);
    // Serial.print(" lb y: ");
    // Serial.print(lb_y);
    // Serial.print(" ub y: ");
    // Serial.print(ub_y);
    // Serial.println();

    //overall*****
    x_mid = (lb_x + ub_x) / 2.0; //mm
    if (!goalie) y_mid = (lb_y + ub_y) / 2.0;
    else y_mid = lb_y;

    size_x = constrain((ub_x-lb_x), 0.0, FIELD_X);
    size_y = constrain((ub_y-lb_y), 0.0, FIELD_Y);

    // confidence calculation
    conf_x = max(0.0, X_FULLY_BLOCKED - (double)size_x) / X_FULLY_BLOCKED;
    conf_y = max(0.0, Y_FULLY_BLOCKED - (double)size_y) / Y_FULLY_BLOCKED; //where arc :( 243 - 52 1910

    // Serial.print(" conf x: ");
    // Serial.print(conf_x);
    // Serial.print(" conf y: ");
    // Serial.print(conf_y);
    // Serial.println();

    // ballcm_dist = BALL_DIST_A*pow((double)ball_dist, 2) + BALL_DIST_B*(double)ball_dist + BALL_DIST_C;
    // ballcm_dist = ball_dist;
    //QN: NEED GY ANG CORRECTION?

    // Serial.print(" ball x relative ");
    // Serial.print(ballcm_dist*sin((float)ball_ang * ((float)PI) / ((float)180.0)));

    /*YL added*/
    prevball_x = ball_x;
    prevball_y = ball_y;
    ball_x = x_mid + 10.0 * ball_dist * sin(deg2rad((double)ball_ang)); //coordinates of ball
    ball_y = y_mid + 10.0 * ball_dist * cos(deg2rad((double)ball_ang));

    int a = tempAng - ball_ang; //-360 to 360
    a = fmod(a + 180.0, 360.0) - 180.0; //-180 to 180
    a = abs(a);

    //or x y dont reset or raw dont reset
    //ball moving or robot moving relative to ball
    /*if ((tempDist != 0 && ball_dist != 0) && anyBall < 16) { //is there ball
        if (a >= 3 || abs(tempDist - ball_dist) > 1) { //a resolution is 3, is robot moving relative to ball
            if (abs(ball_x - prevball_x) > 40 || abs(ball_y - prevball_y) > 40) { //is ball moving
                ballStayed = 0;
            }  
        }
    }*/

    if ((tempDist != 0 && ball_dist != 0) && anyBall < 30) { //is there ball
        if ((double)a >= 1.5 || abs(tempDist - ball_dist) > 1) { //a resolution is 3 (2023: 1.5), is robot moving relative to ball
            if (abs(ball_x - prevball_x) > 40 || abs(ball_y - prevball_y) > 40) { //40//is ball moving
                ballStayed = 0;
            }  
        }
    }

    // if (ballStayed == 0) {
    //     Serial.print((int)anyBall);
    //     Serial.print(" ");
    //     Serial.print(a);
    //     Serial.print(" ");
    //     Serial.print(tempDist);
    //     Serial.print(" ");
    //     Serial.print(ball_dist);
    //     Serial.print(" ");
    //     Serial.print(ball_x);
    //     Serial.print(" ");
    //     Serial.print(prevball_x);
    //     Serial.print(" ");
    //     Serial.print(ball_y);
    //     Serial.print(" ");
    //     Serial.print(prevball_y);
    //     Serial.print(" ");
    //     Serial.println();
    // }
}

void Robot::setDrive(double speed, int angle, double rotation) {
    // prevSpeed = speed;
    db.correction(speed, angle, rotation);
}

void Robot::kick(bool kicking){
    kk.kick(kicking);
}

/*void Robot::drib(bool dribbling, int ball_Dist){
    dribb.run(dribbling, ball_Dist);
}*/

double Robot::getConfX() {
    return conf_x;
}

double Robot::getConfY() {
    return conf_y;
}

double Robot::getCompass() {
    return _gy_angle;
}

//RMB TO CHANGE, SERIALS NEED TO MATCH
void serialEvent1() { //todo: see if can friend var or sth the lids LOL it probs doesn't waste a lot of stuff but like,,, well.
//    Lidar2022 lids;
    while (Serial1.available()) {
        char c = Serial1.read();
        LidarBR.updateBuffer(c);
        LidarBR.compute();
    }
    // Serial.println("1");
}

void serialEvent5() { //todo: see if can friend var or sth the lids LOL it probs doesn't waste a lot of stuff but like,,, well.
//    Lidar2022 lids;
    while (Serial5.available()) {
        char c = Serial5.read();
        LidarBL.updateBuffer(c);
        LidarBL.compute();
    }
    // Serial.println("2");
}

void serialEvent6() {
//    Lidar2022 lids;
    while (Serial6.available()) {
        char c = Serial6.read();
        LidarR.updateBuffer(c);
        LidarR.compute();
    }
    // Serial.println("3");
}

void serialEvent7() {
//    Lidar2022 lids;
    while (Serial7.available()) {
        char c = Serial7.read();
        LidarF.updateBuffer(c);
        LidarF.compute();
    }
    // Serial.println("4");
}

void serialEvent8() { 
//    Lidar2022 lids;
    while (Serial8.available()) {
        char c = Serial8.read();
        LidarL.updateBuffer(c);
        LidarL.compute();
    }
    // Serial.println("5");
}

void serialEvent3() {
    // Serial.println("no");
    cam.read();
}

void serialEvent2() {
    // Serial.println("no");
    bbt.receive();
}