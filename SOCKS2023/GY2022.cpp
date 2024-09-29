#include <GY2022.h>
// #include <Config2022.h>

int pin = 22;
volatile unsigned long tBegin = micros();
volatile unsigned long tEnd = micros();
volatile bool newPulse = false;

void checkInterrupt() {
	if (digitalRead(pin) == HIGH) tBegin = micros();
	else {
		tEnd = micros();
		newPulse = true;
	}
}

GY2022::GY2022() {
	pinMode(pin, INPUT);
	attachInterrupt(digitalPinToInterrupt(pin), checkInterrupt, CHANGE);
}

void GY2022::angle() {
	if (newPulse) {
		newPulse = false;
		_duration = constrain(tEnd - tBegin, 600, 1680); //value from 1000-1720
		_gy_angle = (_duration - 600.0) / 3.0;
		// _gy_angle = fmod(_gy_angle, 360.0);
	}
}

double GY2022::get_angle() {
	angle();
	return _gy_angle;
}

/*
void GY2022::setZero(int reset=0) {
	unsigned long long startt = millis();
	if (reset) {
		while (!newPulse);
		_duration = constrain(tEnd - tBegin, 600, 1680); //value from 1000-1720
		_zero = (_duration - 600.0) / 3.0;
	}
	else {
		while ((millis() - startt) < 2000) {
			while (!newPulse);
			_duration = constrain(tEnd - tBegin, 600, 1680); //value from 1000-1720
			_zero = (_duration - 600.0) / 3.0;
			//Serial.println(_zero);
		}
	}
}*/

/*void GY2022::setZero_old() {
	double a = 0.0;
	for (int i=0; i<300; i++) {
		while (!gy.available());
		a = (gy.read()/255.0)*360.0;
	}
	// for (int i=0; i<10; i++) {}
	// while (a <= 0.0) a = (gy.read()/255.0)*360.0;
	Serial.println(a);
	_zero = a;
}*/