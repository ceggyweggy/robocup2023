#ifndef Kicker_h
#define Kicker_h

#include <Arduino.h>
#include <Config2022.h>
// #include <AltSoftSerial.h>
#include <Math.h>


class Kicker {
    public:
        Kicker(int pin);
        void kick(bool kick);
    private:
        int _sendPin = 26;
		
};

#endif