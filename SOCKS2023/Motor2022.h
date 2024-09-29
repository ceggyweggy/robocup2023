#ifndef Motor2022_h
#define Motor2022_h

#include <Arduino.h>
#include <Config2022.h>
#include <Meth.h>
#include <Math.h>
#include <GY2022.h>

class Motor2022{
    public:
    	Motor2022(int pos_pin, int neg_pin);
    	void moveOne(float speed);
    //Motor2022(int flp, int fln, int frp, int frn, int blp, int bln, int brp, int brn);
    //void moveOne(int pos, int neg, float speed);
    //void move(int ang, float speed);
    //void correction(float speed, int angle, double rotation);
    // double error;

    // GY2022 gy;

    private:
    int _pos_pin, _neg_pin;
    int angle, error;
};

#endif
