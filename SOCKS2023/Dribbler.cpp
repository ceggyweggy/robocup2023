#include <Dribbler.h>

Dribbler::Dribbler(){

}

/*void Dribbler::setup(){
    Serial.begin(9600);
    servo.attach(A16, 1000, 2000);
    servo.writeMicroseconds(1000);
    delay(3000);
    // Serial.println("HIGH");
    servo.writeMicroseconds(2000);
    delay(1500);
    // Serial.println("LOW");
    servo.writeMicroseconds(1000);
    delay(1500);
}

void Dribbler::run(bool run, double ballDist){
    // int ball_dist = BALL_DIST_A*pow((double)ballDist, 2) + BALL_DIST_B*(double)ballDist + BALL_DIST_C; //cm
    // double dist_mul = constrain(((DRIB_MAX_DIST -  ball_dist)/DRIB_MAX_DIST), 0.0, 1.0);
    // Serial.print(ballDist);
    // Serial.print(' ');
    double dist_mul = constrain(((DRIB_MAX_DIST -  (ballDist-30.0))/DRIB_MAX_DIST), 0.0, 1.0);
    // Serial.print(dist_mul);
    // Serial.print(' ');
    int speed = constrain((1700 * dist_mul), 1400, 1700); //wont exceed 1700 anywas but
    // Serial.println(speed);

    if(run == 1) servo.writeMicroseconds(1700);//speed);//speed);//1600 1800 max
    else servo.writeMicroseconds(1000);
}*/