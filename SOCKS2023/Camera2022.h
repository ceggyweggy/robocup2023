#ifndef Camera2022_h
#define Camera2022_h

#include <Arduino.h>
#include <Config2022.h>
#include <Meth.h>

struct angDist{
    double ang, dist;
};

struct leftRight{
    double left, right;
};

class Camera2022{
    friend void serialEvent3();

    public:
        Camera2022(HardwareSerial *serial);
        void debug_camera();
        void read();
        struct angDist get_ball();
        struct leftRight get_goal();
        bool _stringComplete = 0;

        int counter = 0;
        elapsedMillis anyBall;
        elapsedMillis anyGoal;
        elapsedMillis anyMessage;
        elapsedMillis timecheck;
        int thousand = 0;
        
    

    private:
        HardwareSerial *_serial;
    
        String _camString = "";
        char _instr[15];
        char _inChar;
        char * _tok;
        
        // int _ballbyte = 253;
        // int _ygoalbyte = 251;
        // int _bgoalbyte = 252;
        int endbyte = 254;
        char endchar = (wchar_t)254;
        int remaining = 120;
        
        struct angDist _ball;
        struct leftRight _goal;
};

#endif
