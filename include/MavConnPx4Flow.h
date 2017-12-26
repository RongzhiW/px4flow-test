/**************************************************************
* Author : Danping Zou
* Email  : dpzou@sjtu.edu.cn
* 
* Laboratory of Navigation and Location-based Service
* Shanghai Jiao Tong Unversity
* Notice!! - Distribution is strictly not allowed without the 
* author's permission
**************************************************************/

#ifndef MAVCONNPx4Flow_H
#define MAVCONNPx4Flow_H

#include "include/apm_mavlink/common/mavlink.h"
#include "include/apm_mavlink/mavlink_types.h"
#include "include/apm_mavlink/mavlink_helpers.h"
#include "include/apm_mavlink/mavlink_conversions.h"
#include "SerialPort.h"


class MAVCONNPx4Flow
{
protected:
    pthread_t read_tid;
public:
    enum{
        STABILIZE = 0,                     // hold level position
        ACRO = 1,                          // rate control
        ALT_HOLD = 2,                      // AUTO control
        AUTO = 3,                          // AUTO control
        GUIDED = 4,                        // AUTO control
        LOITER = 5,                        // Hold a single location
        RTL = 6,                           // AUTO control
        CIRCLE = 7,                        // AUTO control
        LAND = 9,                          // AUTO control
        OF_LOITER =10  ,                   // Hold a single location using optical flow sensor
        DRIFT =11       ,                 // DRIFT mode (Note: 12 is no longer used)
        SPORT =13        ,                // earth frame rate control
        FLIP    =    14   ,               // flip the vehicle on the roll axis
        AUTOTUNE =   15    ,              // autotune the vehicle's roll and pitch gains
        POSHOLD   =  16     ,             // position hold with manual override
        NUM_MODES  = 17
        
    };
    
    uint8_t system_id;
    uint8_t companion_id;
    bool quit;
    SerialPort sp;
    
    uint8_t _base_mode;
    uint32_t _custom_mode;
    MAVCONNPx4Flow();
    MAVCONNPx4Flow(const char* name, int baudrate = 115200);
    ~MAVCONNPx4Flow();
public:
    void _readingLoop();
public:
    void open();
    //void setMode(uint8_t mode);
    //void sendHeartbeat();
    //void arm(bool on_off);
    //void overRC(uint16_t ch[4]);
    void startReadingLoop();
};

#endif // MAVCONNPx4Flow_H
