#include <BT2022.h>

BT2022::BT2022(HardwareSerial *serial) {
    _serial = serial;
    _serial->begin(38400);
}

void BT2022::send(byte val) {
    _serial->write(val);
}

void BT2022::receive() {
	while (_serial->available()) {
        receiveVal = _serial->read();
        anyBluetooth = 0;
    }
}

int BT2022::getReceiveVal() {
    return receiveVal;
}