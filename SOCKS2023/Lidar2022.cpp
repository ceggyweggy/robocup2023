#include <Lidar2022.h>

Lidar2022::Lidar2022(HardwareSerial *serial, uint32_t param) {
    _HardSerial = serial;
    _HardSerial->begin(115200);
    this->framerate(param);
}

void Lidar2022::updateBuffer(char c) {
    _frame[TFMP_FRAME_SIZE] = c;
    memcpy( _frame, _frame + 1, TFMP_FRAME_SIZE);
}

void Lidar2022::compute() { //
    if (( _frame[0] == 0x59) && (_frame[1] == 0x59)){
        chkSum = 0;
        // Add together all bytes but the last.
        for( uint8_t i = 0; i < ( TFMP_FRAME_SIZE - 1); i++) chkSum += _frame[i];
        //  If the low order byte does not equal the last byte...
        if( ( uint8_t)chkSum != _frame[ TFMP_FRAME_SIZE - 1])
        {
            status = TFMP_CHECKSUM;
            memset( _frame, 0, sizeof(_frame));// then set error... //you have failed :(
            return;
        }

        dist = _frame[2] + ( _frame[ 3] << 8);
        
        if( dist == -1) status = TFMP_WEAK;
        else if( flux == -1) status = TFMP_STRONG;
        else if( dist == -4) status = TFMP_FLOOD;
        else status = TFMP_READY;
        
        if( status != TFMP_READY){
            memset( _frame, 0, sizeof(_frame)); //you failed >:(
        }
        else {
            memset(_frame, 0, sizeof(_frame));
            _reading = dist;
        }
    }
}

void Lidar2022::framerate(uint32_t param) {
    this->sendCommand(SET_FRAME_RATE, param, _HardSerial);
    // this->sendCommand(SET_FRAME_RATE, param, FSERIAL);
    // this->sendCommand(SET_FRAME_RATE, param, RSERIAL);
    // this->sendCommand(SET_FRAME_RATE, param, LSERIAL);
    // this->sendCommand(SET_FRAME_RATE, param, BRSERIAL);
    // this->sendCommand(SET_FRAME_RATE, param, BLSERIAL);
}

// *** look at framerate *** (UNTESTED)
void Lidar2022::sendCommand(uint32_t cmnd, uint32_t param, Stream *ptr){ //only run if needed, don't run at start of everything
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Step 1 - Build the command data to send to the device
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    static uint8_t cmndLen;             // Length of command
    static uint8_t replyLen;            // Length of command reply data
    static uint8_t cmndData[8];        // 8 byte send command array
    pSerial = ptr;

    memset(cmndData, 0, 8);            // Clear the send command array.
    memcpy(&cmndData[0], &cmnd, 4);   // Copy 4 bytes of data: reply length, command len, command num, as 1 in
    replyLen = cmndData[ 0];            // Save the first byte as reply length.
    cmndLen = cmndData[ 1];             // Save the second byte as command length.
    cmndData[ 0] = 0x5A;                // Set the first byte to HEADER code.

    if( cmnd == SET_FRAME_RATE) memcpy( &cmndData[ 3], &param, 2); // If the command is Set FrameRate... add 2 byte FR param
    else if( cmnd == SET_BAUD_RATE) memcpy( &cmndData[ 3], &param, 4); //3 byte baud rate

    // Create a checksum byte for the command data array.
    chkSum = 0; // Add together all bytes but the last...
    for( uint8_t i = 0; i < ( cmndLen - 1); i++) chkSum += cmndData[ i];
    // and save it as the last byte of command data.
    cmndData[ cmndLen - 1] = (uint8_t)chkSum;

    //send data to device
    while( (*pSerial).available()) (*pSerial).read();  // flush input buffer //lol SerialEvent is quaking
    (*pSerial).flush();                         // flush output buffer waits for transmission to complete
    for( uint8_t i = 0; i < cmndLen; i++) (*pSerial).write( cmndData[ i]);
    
    if( replyLen == 0) return; //good job you're done // wow (doesn't expect a reply anyway)
    
    //get command reply
    uint32_t serialTimeout = millis() + 1000; //timeout if nv appears
      // Clear out the entire command reply data buffer
    memset( reply, 0, sizeof( reply)); //byte from buffer
    while( ( reply[ 0] != 0x5A) || ( reply[ 1] != replyLen))
    {
        if( (*pSerial).available()) { //read one byte into reply buffer
            // Read one byte into the reply buffer's
            // last-plus-one position.
            reply[replyLen] = (*pSerial).read(); //
            memcpy(reply, reply+1, 8);    // Shift the last nine bytes one byte left.
        }
        if( millis() >  serialTimeout) //header pattern or serial data unavailable
        {
            status = TFMP_TIMEOUT;  // then set error...
            return;
            //return false;           // and return "false".
        }
    }
    //chksum test
    chkSum = 0; // clear var dd together all bytes but the last...
    for( uint8_t i = 0; i < ( replyLen - 1); i++) chkSum += reply[ i]; // If the low order byte of the Sum does not equal the last byte...
    if( reply[ replyLen - 1] != (uint8_t)chkSum){ // if low order != last byte
        status = TFMP_CHECKSUM;  // then set error...
        return;//return false;            // and return "false."
    }
    if( cmnd == OBTAIN_FIRMWARE_VERSION) //interpret responses
    {
        version[ 0] = reply[5];  // set firmware version.
        version[ 1] = reply[4];
        version[ 2] = reply[3];
    }
    else
    {
        if( cmnd == SYSTEM_RESET ||
            cmnd == RESTORE_FACTORY_SETTINGS ||
            cmnd == SAVE_SETTINGS )
        {
            if( reply[ 3] == 1)      // If PASS/FAIL byte not zero ...
            {
                status = TFMP_FAIL;  // set status 'FAIL'...
                Serial.println("failed");
                return; //return false;        // and return 'false'.
            }
        }
    }
    status = TFMP_READY;
    return;//return true;
    
}

int Lidar2022::get_reading() {
    // Serial.println(distList[which]);
    return _reading;//distList[which];
}

void Lidar2022::set_reading(int reading, byte which) {
    distList[which] = reading;
    Serial.print(distList[which]);
    Serial.print("    ");
    
}

//0 front, 1 right, 2 left, 3 backright, 4 backleft
// void Lidar2022::check(int SerialPort, int dist){
//     switch(SerialPort){
//       case 0: //front
// //            lid_front_r = abs(((float)(10 * dist)) * cos(deg2rad(360 - gyro_angle)));
//             lid_fr = abs(((float)(10 * dist)) * cos(deg2rad(360 - gyro_angle)));
//         break;
//     case 1: //right
// //            lid_right_r =  abs(((float)(10 * dist)) * cos(deg2rad(360 - gyro_angle)));
//             lid_rr = abs(((float)(10 * dist)) * cos(deg2rad(360 - gyro_angle)));
//         break;
//     case 2: //left
// //            lid_left_r =  abs(((float)(10 * dist)) * cos(deg2rad(360 - gyro_angle)));
//             lid_lr =  abs(((float)(10 * dist)) * cos(deg2rad(360 - gyro_angle)));
//         break;
//     case 3: //backrigth
// //            lid_backright_r =  dist*10;//abs(((float)(10 * dist)) * cos(deg2rad(360 - gyro_angle)));
//             lid_brr = abs(((float)(10 * dist)) * cos(deg2rad(360 - gyro_angle)));
//         break;
//     case 4: //backleft
//             lid_blr = abs(((float)(10 * dist)) * cos(deg2rad(360 - gyro_angle)));
//         break;
            
//     }
// }


// void Lidar2022::calc(){
//     //x computing
    
// //    Serial.print("lolaaaa");
// //    Serial.println(lid_backright_r);
//     lid_backright_r = lid_brr;
//     lid_backleft_r = lid_blr;
//     lid_front_r = lid_fr;
//     lid_left_r = lid_lr;
//     lid_right_r = lid_rr;
// //    Serial.println(lid_brr);
// //    Serial.println(lid_backright_r);
    
//     lb_x = min((lid_left_r+LID_LC), (FIELD_X - lid_right_r-LID_RC));
//     ub_x = max((lid_left_r+LID_LC), (FIELD_X - lid_right_r-LID_RC));

//     //back adjusting
//     bool bl_fix = 0, br_fix = 0;
//     if(abs(lid_backright_r-lid_backleft_r) > POST_BACK){ //one in one out
//         if(lid_backright_r > lid_backleft_r){ //right out
// //            Serial.println("diff, rightout");
// //            Serial.print("lol");
// //            Serial.println(lid_backright_r);
//             lid_backright_r = lid_backright_r + 180;
// //            br_fix = 1;
//         }else{ // left out //to test
// //            Serial.println("diff, leftout");
//             lid_backleft_r += POST_BACK;
//             bl_fix = 1;
//         }
//     }

    
// //TO TEST
// //    for goalie, if didn't catch somehow

//     if (((lb_x >= (POST_LB_X-LID_LC-LID_RC)) && (lb_x <= POST_LB_X)) && ((ub_x <= (POST_LB_X+LID_LC+LID_RC)) && (ub_x >= POST_LB_X))){ //left out
// //        Serial.println("leftout");
//         if (!br_fix){
//             lid_backright_r += POST_BACK;
//             br_fix = 1;
//         }
//         if (ub_x <= (POST_UB_X+LID_LC)) lid_front_r += POST_BACK;

//     }else if (((lb_x >= (POST_UB_X-LID_LC-LID_RC)) && (lb_x <= POST_UB_X)) && ((ub_x > POST_UB_X) && (ub_x <= (POST_UB_X + LID_LC + LID_RC)))){//right out
// //        Serial.println("rightout");
//         if (!bl_fix){
//             lid_backleft_r += POST_BACK;
//             bl_fix = 1;
//         }

//         if (lb_x >= (POST_LB_X-LID_LC))lid_front_r += POST_BACK; //conf alr 1 out 1 in, so if that one is liddat


//     }else if ((((lb_x >= POST_LB_X) && (ub_x <= POST_UB_X)) || role == 0)){//completely in, if goalie, both in
// //        Serial.println("bothin");

//         if ((!bl_fix) && (!br_fix)){
//             lid_backleft_r += POST_BACK;
//             lid_backright_r += POST_BACK;
//             bl_fix = 1;
//             br_fix = 1;
//         }
//         lid_front_r += POST_BACK; //conf out oso
//     }

//     //y computing
//     lid_back_r = max(lid_backright_r, lid_backleft_r);
//     lb_y = min((lid_back_r + LID_BC), (FIELD_Y - lid_front_r - LID_FC));
//     ub_y = max((lid_back_r + LID_BC), (FIELD_Y - lid_front_r - LID_FC));

//     //overall*****
//     x_mid = (lb_x + ub_x)/2;
//     y_mid = (lb_y + ub_y)/2;
// //
// //    // confidence (localisation! waow) [might split into another func but for now this works]
// //    //bounding box size
// //    size_x = constrain((ub_x-lb_x), 0, FIELD_X);
// //    size_y = constrain((ub_y-lb_y), 0, FIELD_Y);
// ////    size = size_x * size_y;
// //    //confidence calculation
//    conf_x = constrain(((FIELD_X - size_x)/FIELD_X), 0, 1.0);
//    conf_y = constrain(((FIELD_Y - size_y)/FIELD_Y), 0, 1.0);
// //    //adjust
//    // conf_x = pow(conf_x, CONF_POW_X);
//    // conf_y = pow(conf_y, CONF_POW_Y);
// //    // TODO IN MOVEMENT: MULTIPLY BY CONF_X AND CONF_Y for x and y movement components (or can disable)
// }

// void Lidar2022::debug_dist(){
//     Serial.print("FRONT: ");
//     Serial.println(lid_front_r);
//     Serial.print("RIGHT: ");
//     Serial.println(lid_right_r);
//     Serial.print("LEFT: ");
//     Serial.println(lid_left_r);
//     Serial.print("BACKRIGHT: ");
//     Serial.println(lid_backright_r);
//     Serial.print("BACKLEFT: ");
//     Serial.print(lid_backleft_r);
//     Serial.print("    BACK:");
//     Serial.println(lid_back_r); // doing this so it stays in 1 place on monitor lol
    
// }

// void Lidar2022::debug_pos(){
//    Serial.print("X: ");
//    Serial.println(x_mid);
//    Serial.print("Y: ");
//    Serial.println(y_mid);
// }

//literally just for setting framerates
//.sendCommand(SET_FRAME_RATE, FRAME_250)
//*****only run when u want to change or set the framerate separately, not at the start of every run

// Lidar2022 LidarF(&Serial1); //TO TEST

// /*//0 front, 1 right, 2 left, 3 backright, 4 backleft
//  #define SERIAL1 0
//  #define SERIAL2 1
//  #define SERIAL3 3
//  #define SERIAL4 4
//  #define SERIAL5 2*/

// void serialEvent1(){ //todo: see if can friend var or sth the lids LOL it probs doesn't waste a lot of stuff but like,,, well.
// //    Lidar2022 lids;
//     char c = Serial1.read();
//     lida
//     int dist = lidar.event(&Serial1, SERIAL1); //TO TEST!! if this works life is great lol
//     if (dist > 0) {
//         lidar.set_reading(dist, 0);
//     }
//     // if(dist >= 0)lidar.check(SERIAL1, dist);

// }

// void serialEvent2(){
//     dist = lidar.event(&Serial2, SERIAL2);
//     // if(dist >= 0)lidar.check(SERIAL2, dist);
//     lidar.set_reading(dist, 1);
// }

// void serialEvent3(){
// //    Serial.println("evented");
//     dist = lidar.event(&Serial3, SERIAL3);
//     lidar.set_reading(dist, 2);
// //    Serial.println(dist);
//     // if(dist >= 0)lidar.check(SERIAL3, dist);
// }

// void serialEvent4(){
//     dist = lidar.event(&Serial4, SERIAL4);
//     lidar.set_reading(dist, 3);
//     // if(dist >= 0)lidar.check(SERIAL4, dist);
// }

// void serialEvent5(){
//     dist = lidar.event(&Serial5, SERIAL5);
//     lidar.set_reading(dist, 4);
//     // if(dist >= 0)lidar.check(SERIAL5, dist);
// }

