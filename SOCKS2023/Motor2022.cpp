#include <Motor2022.h>
/*
Motor2022::Motor2022(int flp, int fln, int frp, int frn, int blp, int bln, int brp, int brn){
    _flp = flp, _fln = fln, _frp = frp, _frn = frn, _brp = brp, _brn = brn, _blp = blp, _bln = bln;
    pinMode(flp, OUTPUT);
    pinMode(fln, OUTPUT);
    pinMode(frp, OUTPUT);
    pinMode(frn, OUTPUT);
    pinMode(brp, OUTPUT);
    pinMode(brn, OUTPUT);
    pinMode(blp, OUTPUT);
    pinMode(bln, OUTPUT);
}
*/
Motor2022::Motor2022(int pos_pin, int neg_pin) {
    _pos_pin = pos_pin, _neg_pin = neg_pin;
    pinMode(pos_pin, OUTPUT);
    pinMode(neg_pin, OUTPUT);
    // analogWriteFrequency(pos_pin, 1831.055); 
    //analogWriteFrequency(neg_pin, 1831.055);//58593.75);
}

void Motor2022::moveOne(float speed){
    if(speed >= 0) {
        // analogWrite(_pos_pin, speed*32767);
        analogWrite(_pos_pin, speed*255);
        analogWrite(_neg_pin, 0);
    }else{
        analogWrite(_pos_pin, 0);
        // analogWrite(_neg_pin, abs(speed)*32767);
        analogWrite(_neg_pin, abs(speed)*255);
    }
}
/*
void Motor2022::move(int ang, float speed){
    if (ang<0) ang += 360; // you shldn't need this but jic
    ang = ang+45; //top left wheel y axis
    
    float leftax = speed*(sin(deg2rad(ang)));
    float rightax = speed*(cos(deg2rad(ang)));
    
    moveOne(_flp, _fln, leftax);
    moveOne(_blp, _bln, leftax);
    
    moveOne(_frp, _frn, rightax);
    moveOne(_brp, _brn, rightax);
}

void Motor2022::correction(float speed, int angle, double rotation) {
    speed = constrain(speed, -1.0, 1.0);
    rotation = constrain(rotation, -1.0, 1.0);
    double angle_d = (int)(angle + 45.0) % 360;
    double angle_r = deg2rad(angle_d);
    double x = speed * sin(angle_r);
    double y = speed * cos(angle_r);

    double x_opp = x - (x * abs(rotation) * 2);
    double y_opp = y - (y * abs(rotation) * 2);

    if (0.0 < angle_d && angle_d < 180.0) {
        if (rotation > 0.0) {
            moveOne(_flp, _fln, x);
            moveOne(_brp, _brn, x_opp);
        }
        else {
            moveOne(_flp, _fln, x_opp);
            moveOne(_brp, _brn, x);
        }
    }
    else {
        if (rotation > 0.0) {
            moveOne(_flp, _fln, x_opp);
            moveOne(_brp, _brn, x);
        }
        else {
            moveOne(_flp, _fln, x);
            moveOne(_brp, _brn, x_opp);
        }
    }

    if (90.0 <= angle_d && angle_d >= 270.0) {
        if (rotation > 0.0) {
            moveOne(_frp, _frn, y_opp);
            moveOne(_blp, _bln, y);
        }
        else {
            moveOne(_frp, _frn, y);
            moveOne(_blp, _bln, y_opp);
        }
    }
    else {
        if (rotation > 0.0) {
            moveOne(_frp, _frn, y);
            moveOne(_blp, _bln, y_opp);
        }
        else {
            moveOne(_frp, _frn, y_opp);
            moveOne(_blp, _bln, y);
        }
    }

}
*/
