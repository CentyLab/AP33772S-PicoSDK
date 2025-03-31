/* Structs and basic function ported from "I²C Master Sample Code Introduction for AP33772S" by Diodes
 * Added class and class functions by CentyLab for PicoPD Pro evaluation board
 * Created 30 Jan 2025
 */

#ifndef __AP33772S__H
#define __AP33772S__H

#include "pico/stdlib.h"
#include <stdio.h>
#include "hardware/i2c.h"
#include "hardware/timer.h"
#include <stdarg.h>

#define AP33772S_ADDRESS 0x52

#define MAX_PDO_ENTRIES 13 // Define the maximum number of PDO entries you expect
#define READ_BUFF_LENGTH 128
#define WRITE_BUFF_LENGTH 6
#define SRCPDO_LENGTH 28

#define CMD_STATUS 0x01 // Reset to 0 after very Read
#define CMD_MASK 0x02
#define CMD_OPMODE 0x03
#define CMD_CONFIG 0x04
#define CMD_PDCONFIG 0x05
#define CMD_SYSTEM 0x06
// Temperature setting register
#define CMD_TR25 0x0C
#define CMD_TR50 0x0D
#define CMD_TR75 0x0E
#define CMD_TR100 0x0F

// Power reading related
#define CMD_VOLTAGE 0x11
#define CMD_CURRENT 0x12
#define CMD_TEMP 0x13
#define CMD_VREQ 0x14
#define CMD_IREQ 0x15

#define CMD_VSELMIN 0x16 // Minimum Selection Voltage
#define CMD_UVPTHR 0x17
#define CMD_OVPTHR 0x18
#define CMD_OCPTHR 0x19
#define CMD_OTPTHR 0x1A
#define CMD_DRTHR 0x1B

#define CMD_SRCPDO 0x20

#define CMD_PD_REQMSG 0x31
#define CMD_PD_CMDMSG 0x32
#define CMD_PD_MSGRLT 0x33

// Timer for AVS reminder signal
#define ALARM_NUM1 1 // Timer 1
#define ALARM_IRQ1 TIMER_IRQ_1
#define DELAY1 500000 // In usecond , 0.5s

// DONE
typedef enum
{
    STARTED_MSK = 1 << 0, // 0000 0001
    READY_MSK = 1 << 1,   // 0000 0010
    NEWPDO_MSK = 1 << 2,  // 0000 0100
    UVP_MSK = 1 << 3,     // 0001 0000
    OVP_MSK = 1 << 4,     // 0010 0000
    OCP_MSK = 1 << 5,     // 0100 0000
    OTP_MSK = 1 << 6      // 1000 0000
} AP33772S_MASK;

// typedef enum
// {
//   STARTED_MSK   = 1 << 0,     // 0000 0001
//   READY_MSK     = 1 << 1,     // 0000 0010
//   NEWPDO_MSK    = 1 << 2,     // 0000 0100
//   UVP_MSK       = 1 << 3,     // 0001 0000
//   OVP_MSK       = 1 << 4,     // 0010 0000
//   OCP_MSK       = 1 << 5,     // 0100 0000
//   OTP_MSK       = 1 << 6      // 1000 0000
// } AP33772_STATUS;

// Not sure if used
typedef struct
{
    union
    {
        struct
        {
            uint8_t newNegoSuccess : 1;
            uint8_t newNegoFail : 1;
            uint8_t negoSuccess : 1;
            uint8_t negoFail : 1;
            uint8_t reserved_1 : 4;
        };
        uint8_t negoEvent;
    };
    union
    {
        struct
        {
            uint8_t ovp : 1;
            uint8_t ocp : 1;
            uint8_t otp : 1;
            uint8_t dr : 1;
            uint8_t reserved_2 : 4;
        };
        uint8_t protectEvent;
    };
} EVENT_FLAG_T;

// DONE
typedef struct
{
    union
    {
        struct
        {
            unsigned int voltage_max : 8;  // Bits 7:0, VOLTAGE_MAX field
            unsigned int peak_current : 2; // Bits 9:8, PEAK_CURRENT field
            unsigned int current_max : 4;  // Bits 13:10, CURRENT_MAX field
            unsigned int type : 1;         // Bit 14, TYPE field
            unsigned int detect : 1;       // Bit 15, DETECT field
        } fixed;
        struct
        {
            unsigned int voltage_max : 8; // Bits 7:0, VOLTAGE_MAX field
            unsigned int voltage_min : 2; // Bits 9:8, VOLTAGE_MIN field
            unsigned int current_max : 4; // Bits 13:10, CURRENT_MAX field
            unsigned int type : 1;        // Bit 14, TYPE field
            unsigned int detect : 1;      // Bit 15, DETECT field
        } pps;
        struct
        {
            unsigned int voltage_max : 8; // Bits 7:0, VOLTAGE_MAX field
            unsigned int voltage_min : 2; // Bits 9:8, VOLTAGE_MIN field
            unsigned int current_max : 4; // Bits 13:10, CURRENT_MAX field
            unsigned int type : 1;        // Bit 14, TYPE field
            unsigned int detect : 1;      // Bit 15, DETECT field
        } avs;
        struct
        {
            uint8_t byte0;
            uint8_t byte1;
        };
    };
    unsigned long data;
} SRC_SPRandEPR_PDO_Fields;

// DONE
typedef struct
{
    union
    {
        struct
        {
            unsigned int VOLTAGE_SEL : 8; // Bits 7:0, Output Voltage Select
            unsigned int CURRENT_SEL : 4; // Bits 11:8, Operating Current Select
            unsigned int PDO_INDEX : 4;   // Bits 15:12, Source PDO index select

        } REQMSG_Fields;
        struct
        {
            uint8_t byte0;
            uint8_t byte1;
        };
        unsigned long data;
    };
} RDO_DATA_T;



class DebugPrinter
{
private:
    bool debugFlag;

public:
    DebugPrinter() : debugFlag(false) {}    

    void enableDebug() { debugFlag = true; }
    void disableDebug() { debugFlag = false; }

    void debugPrint(const char *format, ...) {
        if (debugFlag) {
            va_list args;
            va_start(args, format);
            vprintf(format, args);  // Use vprintf for formatted output
            va_end(args);
        }
    }
};

class AP33772S
{
public:
    AP33772S(i2c_inst_t *i2c);
    void begin();
    void displayPDOInfo(int pdoIndex);
    void displayProfiles();
    void mapPPSAVSInfo();
    void setFixPDO(int pdoIndex, int max_current);
    void setPPSPDO(int pdoIndex, int target_voltage, int max_current);
    void setAVSPDO(int pdoIndex, int target_voltage, int max_current);
    // void setVoltage(int targetVoltage); // Unit in mV
    void setNTC(int TR25, int TR50, int TR75, int TR100);
    bool setOutput(uint8_t flag);
    // void setMask(AP33772_MASK flag);
    // void clearMask(AP33772_MASK flag);

    // Monitor functions
    int readTemp();
    int readVoltage();
    int readCurrent();

    // Adjustment functions
    int readVREQ();
    int readIREQ();
    int readVSELMIN();
    void setVSELMIN(int voltage);
    int readUVPTHR();
    void setUVPTHR(int value);
    int readOVPTHR();
    void setOVPTHR(int value);
    int readOCPTHR();
    void setOCPTHR(int value);
    int readOTPTHR();
    void setOTPTHR(int value);
    int readDRTHR();
    void setDRTHR(int value);

    // Advance function to handle in other lib

    int getPPSIndex();
    int getAVSIndex();

    uint8_t existPPS = 0; // PPS flag for setVoltage()
    uint8_t existAVS = 0; // AVS flag for setVoltage()
    
    // Debug On/Off function
    void enableDebug() { debugger.enableDebug(); }
    void disableDebug() { debugger.disableDebug(); }

private:
    void i2c_read(uint8_t cmdAddr, uint8_t len);
    void i2c_write(uint8_t cmdAddr, uint8_t len);
    i2c_inst_t *i2c_;
    static AP33772S *instance; // Static instance pointer

    DebugPrinter debugger;

    static uint8_t readBuf[READ_BUFF_LENGTH];
    static uint8_t writeBuf[WRITE_BUFF_LENGTH];

    int _indexPPSUser = -1; // for getPPSIndex();
    int _indexAVSUser = -1; // for getAVSIndex();

    EVENT_FLAG_T event_flag = {0};
    RDO_DATA_T rdoData = {0};

    // Use for timer;
    static int _voltageAVSbyte;
    static int _currentAVSbyte;
    static int _indexAVS;

    static void timerISR1();
    void setupAVSTimer();
    void cancelAVSTimer();

    SRC_SPRandEPR_PDO_Fields SRC_SPRandEPRpdoArray[MAX_PDO_ENTRIES] = {0};

    // Helper functions
    void displaySPRVoltageMin(unsigned int current_max);
    void displayEPRVoltageMin(unsigned int current_max);
    void displayCurrentRange(unsigned int current_max);
    int currentMap(int current);

};



#endif