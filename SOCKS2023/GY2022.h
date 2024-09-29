#ifndef GY2022_h
#define GY2022_h

#include <Arduino.h>
#include <Config2022.h>
// #include <AltSoftSerial.h>
#include <Math.h>

class GY2022 {
    public:
    	GY2022();
    	void angle();
    	// void checkInterrupt();
    	// void angle_old();
        double get_angle();
    	// void setZero();
    	// AltSoftSerial gy;

    private:
    	// int _pin = 14;
		double _gy_angle = 999.0;
		double reading = 999.0;
		// double _zero = 0.0;
		
		// volatile unsigned long _tBegin = micros();
		// volatile unsigned long _tEnd = micros();
		// volatile bool _newPulse = false;
		unsigned long _duration = 0;
};

#endif
