#include <DriveBase.h>

DriveBase::DriveBase(Motor2022 *FL, Motor2022 *FR, Motor2022 *BL, Motor2022 *BR) {
    _FL = FL, _FR = FR, _BL = BL, _BR = BR;
}


void DriveBase::correction(double speed, double angle, double rotation) {
    speed = constrain(speed, -1.0, 1.0);
    rotation = constrain(rotation, -1.0, 1.0);
    if (angle < 0.0) angle+=360.0;
    double angle_d = fmod((angle + 45.0), 360.0);
    double angle_r = deg2rad(angle_d);

    // Serial.println(powY);
    double x = speed * sin(angle_r);
    double y = speed * cos(angle_r);

    double x_opp = x - (x * abs(rotation) * 2);
    double y_opp = y - (y * abs(rotation) * 2);

    if (0.0 < angle_d && angle_d < 180.0) {
        if (rotation > 0.0) {
            (*_FL).moveOne(x);
            (*_BR).moveOne(x_opp);
        }
        else {
            (*_FL).moveOne(x_opp);
            (*_BR).moveOne(x);
        }
    }
    else {
        if (rotation > 0.0) {
            (*_FL).moveOne(x_opp);
            (*_BR).moveOne(x);
        }
        else {
            (*_FL).moveOne(x);
            (*_BR).moveOne(x_opp);
        }
    }

    if (90.0 <= angle_d && angle_d <= 270.0) {
        if (rotation > 0.0) {
            (*_FR).moveOne(y);
            (*_BL).moveOne(y_opp);
        }
        else {
            (*_FR).moveOne(y_opp);
            (*_BL).moveOne(y);
        }
    }
    else {
        if (rotation > 0.0) {
            (*_FR).moveOne(y_opp);
            (*_BL).moveOne(y);
        }
        else {
            (*_FR).moveOne(y);
            (*_BL).moveOne(y_opp);
        }
    }
}


