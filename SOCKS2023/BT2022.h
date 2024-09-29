#ifndef BT2022_h
#define BT2022_h

#include <Arduino.h>
#include <Config2022.h>
#include <Math.h>

class BT2022 {
    public:
    	BT2022(HardwareSerial *serial);
    	void send(byte val);
    	void receive();
		int getReceiveVal();
		int receiveVal = 255;

		elapsedMillis anyBluetooth;
    private:
		HardwareSerial *_serial;
};

#endif