#include <Camera2022.h>

Camera2022::Camera2022(HardwareSerial *serial){
    _serial = serial;
    _serial->begin(500000);
}

void Camera2022::debug_camera(){
    Serial.print("anyBall: ");
    Serial.print(anyBall);
    Serial.print("  BALL A/D: ");
    Serial.print(_ball.ang);
    Serial.print(" ");
    Serial.print(_ball.dist);
    
    Serial.print("  GOAL L/R: ");
    Serial.print(_goal.left);
    Serial.print(" ");
    Serial.print(_goal.right);

    // Serial.print("  YGOAL L/R: ");
    // Serial.print(_ygoal.left);
    // Serial.print(" ");
    // Serial.print(_ygoal.right);

    Serial.println();
}

int counter = 0;

void Camera2022::read(){
    while (_serial->available() && !_stringComplete) {
        _inChar = (char)_serial->read();
        if (_inChar == endchar) _stringComplete = 1;
        _camString += _inChar;
        // counter++;
    }
    if (_stringComplete) {
        /*for (int i = 0; i <= 9; i++) {
            Serial.print(i);
            Serial.print(": ");
            Serial.println(byte(_camString[i]));
        }*/
        if ((byte)_camString[4] == endbyte) { //endchar
            anyMessage = 0;
            // counter++;
            // Serial.println(counter);
            // if(counter >= 1000){
            //     thousand = timecheck/counter;
            //     counter = 0;
            //     Serial.println(thousand);
            //     timecheck = 0;
            // }
            // Serial.println((byte)_camString[3]);
            if ((byte)_camString[0] != 255) {
                anyBall = 0; //see a ball
                _ball.ang = ((byte)_camString[0]) / 240.0 * 360.0;
                _ball.dist = ((byte)_camString[1]);// / 120.0;
            }

            if ((byte)_camString[2] != 255) {
                anyGoal = 0;
                _goal.left = ((byte)_camString[2]) / 240.0 * 360.0;//1004 changed from /180
                _goal.right = ((byte)_camString[3]) / 240.0 * 360.0; // / 120.0;
            }

            /*if ((byte)_camString[4] != 255) {
                anyYellowGoal = 0;
                _ygoal.left = ((byte)_camString[4]) / 240.0 * 360.0;
                _ygoal.right = ((byte)_camString[5]) / 240.0 * 360.0; // / 120.0;
            }*/
        } 
        // Serial.print(_ball.ang);
        // Serial.print("  ");
        // Serial.print(_ball.dist);
        _stringComplete = 0;
        _camString = "";
        // counter = 0;
        // Serial.println();
    }
    //Serial.println(_serial->read());
    // Serial.println(_camString);
}

/*void Camera2022::read_2(){
    while(_serial->available()){
        _inChar = (char)_serial->read();
        if (_inChar == endchar) _stringComplete = 1;
        _camString += _inChar;
    }
    // Serial.println(_camString);

    if (_stringComplete) { //string received: a, b, y (camString is correct)
        // for (intr i = 0; i <= 7; i++) {
        //     _camString
        // }
        Serial.println(_camString);
        _camString.toCharArray(_instr, 20);
        _tok = strtok(_instr, endchar);
        while (_tok) {
            if ((int)_tok[0] == _ballbyte) { //ball
                anyBall = 0;
                _ball.ang = _tok[1]*360/remaining;
                _ball.dist =  _tok[2]*360/remaining;

            }
            else if ((int)_tok[0] == _ygoalbyte) { //ygoal
                _ygoal.ang = _tok[1]*360/remaining;
                _ygoal.dist = _tok[2]*360/remaining;
            }
            else if ((int)_tok[0] == _bgoalbyte) { //bgoal
                _bgoal.ang = _tok[1]*360/remaining;
                _bgoal.dist = _tok[2]*360/remaining;
            }
            _tok = strtok(0, endchar);
       }
       _stringComplete = 0;
       _camString = "";
   }
}*/

struct angDist Camera2022::get_ball(){
    return _ball;
}

struct leftRight Camera2022::get_goal(){
    return _goal;
}

// struct leftRight Camera2022::get_bgoal(){
//     return _bgoal;
// }

