#ifndef Lidar2022_h
#define Lidar2022_h

#include <Arduino.h>
#include <Config2022.h>
#include <Meth.h>
// #include <AltSoftSerial.h>

#define    SET_FRAME_RATE 0x00030606   // These commands each return
#define    FRAME_0            0x0000    // internal measurement rate
#define    FRAME_1            0x0001    // expressed in hexidecimal
#define    FRAME_2            0x0002
#define    FRAME_5            0x0003
#define    FRAME_10           0x000A
#define    FRAME_20           0x0014
#define    FRAME_25           0x0019
#define    FRAME_50           0x0032
#define    FRAME_100          0x0064
#define    FRAME_125          0x007D
#define    FRAME_200          0x00C8
#define    FRAME_250          0x00FA
#define    FRAME_500          0x01F4
#define    FRAME_1000         0x03E8

#define    SET_BAUD_RATE              0x00060808
#define    BAUD_9600          0x002580   // UART serial baud rate
#define    BAUD_14400         0x003840   // expressed in hexidecimal
#define    BAUD_19200         0x004B00
#define    BAUD_56000         0x00DAC0
#define    BAUD_115200        0x01C200
#define    BAUD_460800        0x070800
#define    BAUD_921600        0x0E1000

#define    OBTAIN_FIRMWARE_VERSION    0x00010407

#define TFMP_CHECKSUM        3  // checksum doesn't match
#define TFMP_TIMEOUT         4  // I2C timeout
#define TFMP_PASS            5  // reply from some system commands
#define TFMP_FAIL            6  //
#define TFMP_HEADER          2 

#define    SYSTEM_RESET               0x00020405   // returns a 1 byte pass/fail (0/1)
#define    RESTORE_FACTORY_SETTINGS   0x00100405   //           "
#define    SAVE_SETTINGS              0x00110405   // This must follow


class Lidar2022 {
    friend void serialEvent1(); // declare for all the rest as well
    friend void serialEvent2(); // declare for all the rest as well
    friend void serialEvent3();
    friend void serialEvent4();
    friend void serialEvent5();
    
    public:
        Lidar2022(HardwareSerial *serial, uint32_t param);
        
        int event(Stream *ptr, int SerialPort);
        // void check(int SerialPort, int dist);
        // void calc();
        
        // void debug_dist();
        // void debug_pos();
        
        void sendCommand(uint32_t cmnd, uint32_t param, Stream *ptr);
        void framerate(uint32_t param);
        int get_reading();
        void updateBuffer(char c);
        void compute();
        void set_reading(int reading, byte which);
        int distList[5];
    //kinda exclusive to this lib, probably can private
    private:
        Stream *pSerial;
        Stream *pStream;

        uint8_t _frame[10];
        int _reading = 0;

        int TFMP_FRAME_SIZE = 9;
        int TFMP_WEAK = 10;  // Signal Strength ≤ 100
        int TFMP_STRONG = 11;  // Signal Strength saturation
        int TFMP_FLOOD = 12; // Ambient Light saturation
        int TFMP_MEASURE = 13;
        int TFMP_READY = 0;

    
        //1 2 3 by side facing (so can share serialport var LOL)
        uint8_t frame0[10]; //frame size (9) + 1
        uint8_t frame1[10];
        uint8_t frame2[10];
        uint8_t frame3[10];
        uint8_t frame4[10];
    
        uint8_t reply[9]; //replysize (8) + 1
        uint8_t version[ 3];   // to save firmware version
        uint16_t chkSum;
        uint8_t status;
        int dist, flux, temp;

        HardwareSerial *_HardSerial;
        
    
    // LIDAR_FRONT_SER
};

#endif
