#ifndef Robot_h
#define Robot_h

#include <Arduino.h>
#include <Meth.h>
#include <Math.h>
#include <Config2022.h>
#include <GY2022.h>
#include <Motor2022.h>
#include <Lidar2022.h>
#include <DriveBase.h>
// #include <AltSoftSerial.h>
#include <Camera2022.h>
#include <BT2022.h>
#include <Tuning.h>
#include <Dribbler.h>
#include <Kicker.h>

#define X_FULLY_BLOCKED 1500 //3005 //1390 //1500
#define Y_FULLY_BLOCKED 1730 //1840

// #define X_FULLY_BLOCKED FIELD_X - OUT_FIELD_SIZE - ROBOT_DIAMETER //conf is 0 when robot is at inner edge of line w opponent bot directly beside it
// #define Y_FULLY_BLOCKED FIELD_Y - OUT_PENALTY_SIZE - ROBOT_DIAMETER //conf is 0 when robot is at inner edge of line w opponent bot directly beside it

class Robot {
    public:
    	Robot();
    	void setup(int reset=0);
    	void btSend(byte data);
    	void updateReading();
    	void updatePos(int goalie=0);
    	double getCompass();
    	double getConfX();
    	double getConfY();
    	void debugDist(int mode=3);
		void debugCamera();
		void debugGy();
    	void setDrive(double speed, int angle, double rotation);
    	// void correction(float speed, int angle, double rotation);

    	void kick(bool kicking);
    	void drib(bool dribbling, int ball_Dist);

    	double lb_x;
		double ub_x;
		double lb_y;
		double ub_y;

		double lid_brr;
		double lid_blr;
		double lid_fr;
		double lid_lr;
		double lid_rr;

		double ball_dist_0, ball_dist_90, ball_dist_180, ball_dist_270;

		double lid_left_raw;
		double lid_backleft_raw;
		double lid_backright_raw;
		double lid_front_raw;
		double lid_right_raw;
		double lid_front_r;
		double lid_right_r;
		double lid_left_r;
		double lid_backright_r;
		double lid_backleft_r;
		double lid_back_r;

		double lidF;
		double lidR;
		double lidL;
		double lidBL;
		double lidBR;

		double x_mid;
		double y_mid;

		double size_x;
		double size_y;
		double conf_x;
		double conf_y;

		double _gy_angle;
		double ball_ang, ball_dist;
		double ball_x, ball_y;
		double ballcm_dist;

		double goal_left_ang, goal_right_ang, goal_mid_ang, goal_ang_width;
		double corr_goal_left_ang, corr_goal_right_ang;
        double goal_left_tan, goal_right_tan, goal_left_constant, goal_right_constant;
        double bot_cam_x, bot_cam_y;
		int front_goal;
		int goal_kick;

        double prevSpeed;
        double gyTime;

        unsigned long long anyBall;
		unsigned long long anyGoal;
		// unsigned long long anyBlueGoal;
		// unsigned long long anyYellowGoal;
		unsigned long long anyMessage;
        elapsedMillis ballStayed;

        int btReceive = -1;
		unsigned long long anyBluetooth;
        // int ballCap = 0;

        /*YL added 1*/
        double prevball_x;
        double prevball_y;

        int line;
		int ballcap;
		int lightgate;

		int receivePin = 32;
		int sendPin = 26;
		int ballcapPin = 27;

    private:
    	
		
};


#endif