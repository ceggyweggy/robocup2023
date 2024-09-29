#ifndef Config2022_h
#define Config2022_h

#include <Arduino.h>
#include <Lidar2022.h>
#include <Motor2022.h>

// #define PIKACHU 1

#define GOALIE 1
#define STRIKER 0

#define ROBOT_DIAMETER 180.0 //mm

#define FIELD_X 1820.0 //mm
#define MID_X 910.0
#define FIELD_Y 2430.0
#define FIELD_INNER_DIAGONAL 300 //cm
#define GOAL_LEFT_X 610.0 //left corner of goal for cam localisation
#define GOAL_RIGHT_X 1210.0
#define GOAL_SIZE 600.0 // goal width
#define GOAL_FRONT_Y 2160.0 //goal in front
#define GOAL_BACK_Y 270.0 //goal behind back of the goal (including the thickness of the goal)

#define OUT_FIELD_SIZE 250 // 3005 //mm, out of bounds area
#define OUT_PENALTY_SIZE 520 //mm, out of bounds area + penalty size 25+25+2

// #define ORBIT_LB 550
// #define ORBIT_UB FIELD_X - ORBIT_LB

#ifdef PIKACHU

#define pika 1
//bot one config
// #define LID_LC 3.0 //3  
// #define LID_RC 6.0
#define LID_FC 5.0 //7.0
#define LID_BRC 4.0 //3.0
#define LID_BLC 4.0 //3.0
#define FSERIAL &Serial7
#define RSERIAL &Serial6
#define LSERIAL &Serial8
#define BRSERIAL &Serial1
#define BLSERIAL &Serial5
#define CAMSERIAL &Serial3
#define BTSERIAL &Serial2
#define FLP 3
#define FLN 2
#define FRP 9
#define FRN 10
#define BLP 5
#define BLN 4
#define BRP 11
#define BRN 12

#define POST_LB_X 550 //540//520
#define POST_UB_X 1290 //1240
#define POST_BACK_BR 210//66 //250
#define POST_BACK_BL 210//66
#define POST_BACK_F 210//66

#define LEFT_STRIKER_TILT_ANGLE 20.0 // when it is on the right side it tilts to the LEFT
#define RIGHT_STRIKER_TILT_ANGLE 30.0
#define GOALIE_TILT_ANGLE 2.5

#define AIM_ACCEL_SPEED 0.0005
#define AIM_ANGLE_LEFT_MUL 0.67
#define AIM_ANGLE_RIGHT_MUL 0.67

#define LID_LPROP 1.0
#define LID_LC -50.0 // 5cm
#define LID_RPROP -1.0
#define LID_RC 1750.0 // 7cm

// #define LID_LPROP 0.977
// #define LID_LC -24.3
// #define LID_RPROP -1.15
// #define LID_RC 2018.0

#define LG_THRESH 1000
#define BCZ_CAM_ANG_LEFT 352.0
#define BCZ_CAM_ANG_RIGHT 15.0
#define BCZ_CAM_DIST 9.0

#else
#define pika 0

// bot two config
// #define LID_LC 3.0//9.2 //cm
// #define LID_RC 3.0//9.2
#define LID_FC 5.0//3.0
#define LID_BRC 4.0//3.5
#define LID_BLC 4.0
#define FSERIAL &Serial7
#define RSERIAL &Serial6
#define LSERIAL &Serial8
#define BRSERIAL &Serial1
#define BLSERIAL &Serial5
#define CAMSERIAL &Serial3
#define BTSERIAL &Serial2
#define FLP 3
#define FLN 2
#define FRP 9
#define FRN 10
#define BLP 5
#define BLN 4
#define BRP 11
#define BRN 12

#define POST_LB_X 580 //540//520
#define POST_UB_X 1270 //1240
#define POST_BACK_BR 210//66//210 //should be 66
#define POST_BACK_BL 210//66//210
#define POST_BACK_F 210//66//210

#define LEFT_STRIKER_TILT_ANGLE 20.0 // when it is on the right side it tilts to the LEFT
#define RIGHT_STRIKER_TILT_ANGLE 20.0
#define GOALIE_TILT_ANGLE 3.0 //angle to face outwards

#define AIM_ACCEL_SPEED 0.00025 //aiming acceleration speed (additional speed added per loop)
#define AIM_ANGLE_LEFT_MUL 1.0 //aiming twd the right
#define AIM_ANGLE_RIGHT_MUL 0.67 //aiming twd the left

#define LID_LPROP 1.0
#define LID_LC -60.0
#define LID_RPROP -0.955
#define LID_RC 1689.0

#define LG_THRESH 1022 // max
#define BCZ_CAM_ANG_LEFT 343.0
#define BCZ_CAM_ANG_RIGHT 4.0
#define BCZ_CAM_DIST 9.0

#endif

#endif
