//maincode
#define MOVETOPOINT_CAP 0.3

//ballTrack
#define DIST_MUL_POW 0.55 //balltrack dist mul
#define GOALIE_DIST_MUL_POW 2.5 //goalie dist ratio multiplier
#define GOALIE_ANG_MUL_POW 1.25 //goalie angle error multiplier
#define GOALIE_PERCENT_DIST_POW 0.2
#define MAX_BALL_DIST 101.0
#define MIN_BALL_DIST 12.0

//slowDown //FB: front back (x axis), LB: lower bound, UB: upper bound
//CHANGE FOR COMP
// #define LR_LB_SD 210.0 //lower bound for x axis of slowdown, out of bounds + half robot
// #define LR_UB_SD 1610.0
#define LR_LB_SD 230.0 //500.0 //300.0 //lower bound for x axis of slowdown, out of bounds + half robot
#define LR_UB_SD 1590.0 //1320.0 //1520.0
#define FB_SD 2430.0 //no bounds because using arc 

//slowDown
#define MAX_SPEED_POINT 0.88//0.95 //0.8?? //0.88 //how far away from end of field then bot is max speed (e.g. 0.5 = half of field at max speed, 0.75  = 3 quarters away) so if u increase number then bot is slower (hits max speed if at 0.75 y (starting at 0,0))
#define GOALIE_MAX_SPEED 1.0 // max speed at LB compare to UB

//main code
#define ACCEL_CONST_LOC 0.001 //not being used currently
#define ACCEL_CONST_CAM 0.00025
#define AIMING_CAP 0.3 //does this have a purpose

//main code
// #define LINE_CONF_X 200.0
// #define LINE_CONF_Y 2.0
// #define GOALIE_CONF_X 6.0
// #define GOALIE_CONF_Y 6.0
// #define STRIKER_CONF_X 1.0
// #define STRIKER_CONF_Y 1.0

// need to retune this for cam loc
#define STRIKER_OUT_CONF_X 8.0//25.0
#define STRIKER_OUT_CONF_Y 11.0//18.0
#define STRIKER_LINE_CONF_X 6.0
#define STRIKER_LINE_CONF_Y 6.0
#define GOALIE_CONF_X 6.0 // tune this tmr make sure robots not blocking each other
#define GOALIE_CONF_Y 6.0
#define STRIKER_CONF_X 4.0
#define STRIKER_CONF_Y 1.0

//main code //speed constrains
#define GOALIE_CONSTRAIN 1.0//1.0 //0.3
#define STRIKER_CONSTRAIN 0.9

//localisation aiming
#define LOCAIM_DEAD_R 940.0
#define LOCAIM_DEAD_L 880.0
#define LOCAIM_Y 2150.0
#define LOCAIM_R_X 940.0//for ref, center is 910.0
#define LOCAIM_L_X 880.0
#define LOCAIM_DIST_GOAL 50.0 // mm, distance of aim point from left/right of goalpost

//move to point
#define MOVETOPOINT_DIST_SCALING 1500.0

//front striker bound
//CHANGE FOR COMP
// #define STRIKER_FRONT_BOUND 2090.0
#define STRIKER_FRONT_BOUND 1930.0 //stops when robot mid is on the line, leave like this cos otherwise robot is slow
#define STRIKER_FRONT_BOUND_KICK_SPEED 1930.0 - ROBOT_DIAMETER * 0.5 - 2.0 //min speed = 0 is just before robot touches line

//cam based localisation
// #define LEFT_GOAL_X 560
// #define RIGHT_GOAL_X 1230
// #define GOAL_FRONT_Y 200
// #define GOAL_BACK_Y 2230
// #define BLUE_GOAL 1
// #define YELLOW_GOAL 0
// #define FRONT_GOAL BLUE_GOAL // ??????