#include <Kicker.h>

Kicker::Kicker(int pin){
    _sendPin = pin;
    pinMode(_sendPin, OUTPUT);
}

void Kicker::kick(bool kicking){
    if(kicking) digitalWrite(_sendPin, HIGH);
    else digitalWrite(_sendPin, LOW);
}

