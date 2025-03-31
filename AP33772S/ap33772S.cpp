/* Structs and Resgister list ported from "AP33772 I2C Command Tester" by Joseph Liang
 * Created 11 April 2022
 * Added class and class functions by VicentN for PicoPD evaluation board
 * Created 8 August 2023
 * Updated 6 Oct 2024 - Expose more internal class variable and include better PPS functions
 * Updated 5 Jan 2025 - Porting over for PicoSDK in C++
 */

#include "ap33772S.h"

AP33772S* AP33772S::instance = nullptr;
uint8_t AP33772S::readBuf[READ_BUFF_LENGTH] = {0};
uint8_t AP33772S::writeBuf[WRITE_BUFF_LENGTH] = {0};
int AP33772S::_voltageAVSbyte = 0;
int AP33772S::_currentAVSbyte = 0;
int AP33772S::_indexAVS = 0;

/**
 * @brief Class constuctor
 * @param i2c pass in i2c0 or i2c1
 */

AP33772S::AP33772S(i2c_inst_t* i2c)noexcept
        :i2c_(i2c)
{
    instance = this;
}

/**
 * @brief Fetch the PDO profile at start up
 */
void AP33772S::begin()
{
    i2c_read(CMD_SRCPDO, 26);

    for (int i = 0; i < 26; i += 2) {
        // Store the bytes in the array of structs
        int pdoIndex = (i / 2);  // Calculate the PDO index
        SRC_SPRandEPRpdoArray[pdoIndex].byte0 = readBuf[i];
        SRC_SPRandEPRpdoArray[pdoIndex].byte1 = readBuf[i + 1];
        // displayPDOInfo(pdoIndex);
    }

    //Populate internal variable
    mapPPSAVSInfo();
}


/**
 * @brief Print all available profile out to Serial
 */
void AP33772S::displayProfiles()
{
  printf("Profile list detected: \n");
  for (int i = 0; i < 26; i += 2) {
    int pdoIndex = (i / 2);  // Calculate the PDO index
    displayPDOInfo(pdoIndex);
  }
}


/**
 * @brief Decode PDO information from SRC_SPRandEPR_PDO_Fields
 * @param pdoIndex feed from loop
 */
void AP33772S::displayPDOInfo(int pdoIndex) {
  // Determine if it's SPR or EPR based on pdoIndex
  bool isEPR = (pdoIndex >= 7 && pdoIndex <= 12);  // 1-6 for SPR, 7-12 for EPR
  // Check if both bytes are zero
  if (SRC_SPRandEPRpdoArray[pdoIndex].byte0 == 0 && SRC_SPRandEPRpdoArray[pdoIndex].byte1 == 0) {
    return;  // If both bytes are zero, exit the function
  }
  
  // Print the PDO type and index
  debugger.debugPrint(pdoIndex <= 6 ? " SRC_SPR_PDO" : " SRC_EPR_PDO");
  debugger.debugPrint("%d",pdoIndex+1);
  debugger.debugPrint(": \n");
  
  // Now, the individual fields can be accessed through the union in the struct
  if (SRC_SPRandEPRpdoArray[pdoIndex].fixed.type == 0) {  // Fixed PDO
    // Print parsed values
    debugger.debugPrint("Fixed PDO: ");
    debugger.debugPrint("%d",SRC_SPRandEPRpdoArray[pdoIndex].fixed.voltage_max * (isEPR ? 200 : 100)); // Voltage in 200mV units for EPR, 100mV for SPR
    debugger.debugPrint("mV ");
    displayCurrentRange(SRC_SPRandEPRpdoArray[pdoIndex].fixed.current_max);  // Assuming displayCurrentRange function is available
  } else {  // PPS or AVS PDO
    // Print parsed values
    debugger.debugPrint(isEPR ? "AVS PDO: " : "PPS PDO: ");
    if (isEPR) {
      displayEPRVoltageMin(SRC_SPRandEPRpdoArray[pdoIndex].avs.voltage_min);  // Assuming displayVoltageMin function is available
    } else {
      displaySPRVoltageMin(SRC_SPRandEPRpdoArray[pdoIndex].pps.voltage_min);  // Assuming displayVoltageMin function is available
    }
    debugger.debugPrint("%d", SRC_SPRandEPRpdoArray[pdoIndex].fixed.voltage_max * (isEPR ? 200 : 100)); // Maximum Voltage in 200mV units for EPR, 100mV for SPR
    debugger.debugPrint("mV ");
    displayCurrentRange(SRC_SPRandEPRpdoArray[pdoIndex].fixed.current_max);  // Assuming displayCurrentRange function is available
  }
  debugger.debugPrint("\n");
}



//We have multilple profile stay as array, SPR (<= 20V), PPS, SPR in EPR. AVS as of right now only known to have one profile
/**
 * @brief Search through the list of profile and look for PPS, AVS
 * @bug If system has 2 PPS profiles. The index only show the last one. Use displayPDOInfo() to check.
 */
void AP33772S::mapPPSAVSInfo()
{
  for(int i = 1; i<=13; i++)
  {
    if(i < 8 && SRC_SPRandEPRpdoArray[i-1].pps.type == 1)
    {
      debugger.debugPrint("Found PPS profile\n");
      _indexPPSUser = i;
    }
    else if(i >= 8 && SRC_SPRandEPRpdoArray[i-1].avs.type == 1)
    {
      debugger.debugPrint("Found AVS profile\n");
      _indexAVSUser = i;
    }
  }
}

/**
 * @brief Request fixed PDO voltage, work for both standard and EPR mode
 * @param pdoIndex index 1
 * @param max_current unit in mA
 */
void AP33772S::setFixPDO(int pdoIndex, int max_current) 
{
  cancelAVSTimer();

  RDO_DATA_T rdoData;

  // Max current sanity check
  if(max_current <= 0) return;

  // For Fix voltage, only need to set PDO_INDEX and CURRENT_SEL
  // No need to change the selected voltage
  // handle the same in standard as well as EPR

  // PDO index need to be fixed type
  if(SRC_SPRandEPRpdoArray[pdoIndex-1].fixed.type == 0){
    // Now that we are in fix PDO mode
    debugger.debugPrint("Type is fixed.\n"); // DEBUG

    rdoData.REQMSG_Fields.PDO_INDEX = pdoIndex;  // Index 1

    // Serial.debugger.debugPrint("You entered current: %dmA", max_current); //DEBUG
    // Serial.println(SRC_SPRandEPRpdoArray[pdoIndex].fixed.current_max); //DEBUG
    // Serial.println(currentMap(max_current)); //DEBUG
    if(currentMap(max_current) > SRC_SPRandEPRpdoArray[pdoIndex-1].fixed.current_max) 
    {
      debugger.debugPrint("Current not in range.\n"); // DEBUG
      return; // Check if current setting is in range
    }

    rdoData.REQMSG_Fields.CURRENT_SEL = currentMap(max_current);
    // Note: For profile less than or equal to 3A power, CURRENT_SEL = 9 will not work.
    // rdoData.REQMSG_Fields.CURRENT_SEL = 9; 
    writeBuf[0] = rdoData.byte0;  // Store the upper 8 bits
    writeBuf[1] = rdoData.byte1;  // Store the lower 8 bits
    i2c_write(CMD_PD_REQMSG, 2);
  }
  return;
}

/**
 * @brief Request PPS voltage
 * @param pdoIndex index 1
 * @param target_voltage unit in mV
 * @param max_current unit in mA
 * @bug only work if min PPS voltage is 3.3V
 */
void AP33772S::setPPSPDO(int pdoIndex, int target_voltage, int max_current) 
{
  cancelAVSTimer();
  RDO_DATA_T rdoData;
  
  int voltage_min_decoded;
  // Sanity check include, check if the value is in EPR range (index < 8) and also AVS mode
  if(pdoIndex < 8 && SRC_SPRandEPRpdoArray[pdoIndex-1].pps.type == 1)
  {
    debugger.debugPrint("Type is PPS.\n"); // DEBUG
    // Now that we are in PPS mode

    rdoData.REQMSG_Fields.PDO_INDEX = pdoIndex;  // Index 1

    if(currentMap(max_current) > SRC_SPRandEPRpdoArray[pdoIndex-1].pps.current_max) 
    {
      debugger.debugPrint("PPS Current not in range.\n"); // DEBUG
      return; // Check if current setting is in range
    }

    //Decode voltage_min
    if(SRC_SPRandEPRpdoArray[pdoIndex-1].pps.voltage_min > 0) voltage_min_decoded = 3300;

    if(target_voltage < voltage_min_decoded || 
          target_voltage > SRC_SPRandEPRpdoArray[pdoIndex-1].pps.voltage_max*100 ) 
    {
      debugger.debugPrint("PPS Voltage not in range.\n"); // DEBUG
      return; // Check if current setting is in range
    }

    rdoData.REQMSG_Fields.VOLTAGE_SEL = target_voltage/100;  // Output Voltage in 200mV units
    rdoData.REQMSG_Fields.CURRENT_SEL = currentMap(max_current);

    writeBuf[0] = rdoData.byte0;  // Store the upper 8 bits
    writeBuf[1] = rdoData.byte1;  // Store the lower 8 bits
    i2c_write(CMD_PD_REQMSG, 2);
  }
  return;
}

/**
 * @brief Request AVS voltage
 * @param pdoIndex index 1
 * @param target_voltage unit in mV
 * @param max_current unit in mA
 * @bug only work if min AVS voltage is 15V, AVS max voltage is not capped at 30V
 */
void AP33772S::setAVSPDO(int pdoIndex, int target_voltage, int max_current) 
{
  cancelAVSTimer();

  RDO_DATA_T rdoData;

  int voltage_min_decoded;
  // Sanity check include, check if the value is in EPR range (index >= 8) and also AVS mode
  if(pdoIndex >= 8 && SRC_SPRandEPRpdoArray[pdoIndex-1].avs.type == 1)
  {
    debugger.debugPrint("Type is AVS.\n"); // DEBUG
    // Now that we are in AVS mode

    //Decode voltage_min
    if(SRC_SPRandEPRpdoArray[pdoIndex-1].avs.voltage_min > 0) voltage_min_decoded = 15000;

    rdoData.REQMSG_Fields.PDO_INDEX = pdoIndex;  // Index 1

    if(currentMap(max_current) > SRC_SPRandEPRpdoArray[pdoIndex-1].avs.current_max) 
    {
      debugger.debugPrint("AVS Current not in range.\n"); // DEBUG
      return; // Check if current setting is in range
    }

    //Decode voltage_min
    if(SRC_SPRandEPRpdoArray[pdoIndex-1].avs.voltage_min > 0) voltage_min_decoded = 15000;

    if(target_voltage < voltage_min_decoded || 
          target_voltage > SRC_SPRandEPRpdoArray[pdoIndex-1].avs.voltage_max*200 ) 
    {
      debugger.debugPrint("AVS Voltage not in range.\n"); // DEBUG
      return; // Check if current setting is in range
    }

    rdoData.REQMSG_Fields.VOLTAGE_SEL = target_voltage/200;  // Output Voltage in 200mV units
    rdoData.REQMSG_Fields.CURRENT_SEL = currentMap(max_current);

    writeBuf[0] = rdoData.byte0;  // Store the upper 8 bits
    writeBuf[1] = rdoData.byte1;  // Store the lower 8 bits
    i2c_write(CMD_PD_REQMSG, 2);

    _indexAVS = rdoData.REQMSG_Fields.PDO_INDEX;
    _voltageAVSbyte = rdoData.REQMSG_Fields.VOLTAGE_SEL; 
    _currentAVSbyte = rdoData.REQMSG_Fields.CURRENT_SEL;

    // Required to maintain AVS voltage negotiation.
    setupAVSTimer();
  }
  return;
}

void AP33772S::timerISR1()
{
  // Clear the alarm irq
  hw_clear_bits(&timer_hw->intr, 1u << ALARM_NUM1);

  // Reset the alarm register
  timer_hw->alarm[ALARM_NUM1] = timer_hw->timerawl + DELAY1;

  RDO_DATA_T rdoData;
  rdoData.REQMSG_Fields.PDO_INDEX = _indexAVS;
  rdoData.REQMSG_Fields.VOLTAGE_SEL = _voltageAVSbyte;
  rdoData.REQMSG_Fields.CURRENT_SEL = _currentAVSbyte;

  // Trying to call non static function/variable
  // TODO Check scope if keep spawning too many object
  writeBuf[0] = rdoData.byte0;  // Store the upper 8 bits
  writeBuf[1] = rdoData.byte1;  // Store the lower 8 bits
  instance->i2c_write(CMD_PD_REQMSG, 2);
}

void AP33772S::setupAVSTimer()
{
  // Set up 0.5s timer using timer1
  hw_set_bits(&timer_hw->inte, 1u << ALARM_NUM1);
  // Associate an interrupt handler with the ALARM_IRQ
  irq_set_exclusive_handler(ALARM_IRQ1, timerISR1);
  // Enable the alarm interrupt
  irq_set_enabled(ALARM_IRQ1, true);
  // Write the lower 32 bits of the target time to the alarm register, arming it.
  timer_hw->alarm[ALARM_NUM1] = timer_hw->timerawl + DELAY1;
}

void AP33772S::cancelAVSTimer()
{
  hw_clear_bits(&timer_hw->inte, 1u << ALARM_NUM1); // Disable alarm interrupt
  hw_set_bits(&timer_hw->intr, 1u << ALARM_NUM1);   // Clear pending interrupt
}


/**
 * @brief Set resistance value of 10K NTC at 25C, 50C, 75C and 100C.
 *          Default is 10000, 4161, 1928, 974Ohm
 * @param TR25, TR50, TR75, TR100 unit in Ohm
 * @attention Blocking function due to long I2C write, min blocking time 15ms
 */
void AP33772S::setNTC(int TR25, int TR50, int TR75, int TR100)
{
  writeBuf[0] = TR25 & 0xff;
  writeBuf[1] = (TR25 >> 8) & 0xff;
  i2c_write(CMD_TR25, 2);
  sleep_ms(5);
  writeBuf[0] = TR50 & 0xff;
  writeBuf[1] = (TR50 >> 8) & 0xff;
  i2c_write(CMD_TR50, 2);
  sleep_ms(5);
  writeBuf[0] = TR75 & 0xff;
  writeBuf[1] = (TR75 >> 8) & 0xff;
  i2c_write(CMD_TR75, 2);
  sleep_ms(5);
  writeBuf[0] = TR100 & 0xff;
  writeBuf[1] = (TR100 >> 8) & 0xff;
  i2c_write(CMD_TR100, 2);
}

/**
 * @brief Read NTC temperature
 * @return tempearture in C
 */
int AP33772S::readTemp()
{
    i2c_read(CMD_TEMP, 1);
    return readBuf[0]; // I2C read return 1C/LSB
}

/**
 * @brief Read VBUS voltage
 * @return voltage in mV
 */
int AP33772S::readVoltage()
{
    i2c_read(CMD_VOLTAGE, 2);
    return ((readBuf[1] << 8) | readBuf[0]) * 80; // I2C read return 80mV/LSB
}

/**
 * @brief Read VBUS current
 * @return current in mA
 */
int AP33772S::readCurrent()
{
    i2c_read(CMD_CURRENT, 1);
    return readBuf[0] * 24; // I2C read return 24mA/LSB
}

/**
 * @brief Read VREQ The latest requested voltage negotiated with the source
 * @return voltage in mV
 */
int AP33772S::readVREQ()
{
    i2c_read(CMD_VREQ, 1);
    return readBuf[0] * 50; // I2C read return 50mV/LSB
}

/**
 * @brief Read IREQ The latest requested voltage negotiated with the source
 * @return current in mA
 */
int AP33772S::readIREQ()
{
    i2c_read(CMD_IREQ, 1);
    return readBuf[0] * 10; // I2C read return 10mA/LSB
}

/**
 * @brief Read VSELMIN register. The Minimum Selection Voltage
 * @return voltage in mV
 */
int AP33772S::readVSELMIN()
{
  i2c_read(CMD_VSELMIN, 1);
  return readBuf[0] * 200; // I2C read return 200mV/LSB
}

/**
 * @brief Set VSELMIN register. The Minimum Selection Voltage
 * @param voltage in mV
 */
void AP33772S::setVSELMIN(int voltage)
{
  writeBuf[0] = voltage/200; // 200mV/LSB
  i2c_write(CMD_VSELMIN, 1);
}

/**
 * @brief Read UVP Threshold, percentage(%) of VREQ
 * @return percentage, should only return 80%, 75%, or 70%. -1 for error
 */
int AP33772S::readUVPTHR()
{
  i2c_read(CMD_UVPTHR, 1);
  switch(readBuf[0])
  {
    case 1:
      return 80;
    case 2:
      return 75;
    case 3:
      return 70;
  }
  return -1;
}

/**
 * @brief Set UVP Threshold, percentage(%) of VREQ
 * @param value percentage. If 80% then value = 80
 */
void AP33772S::setUVPTHR(int value)
{
  if(value >= 70 && value <= 80)
  {
    switch(value)
    {
      case 80:
        writeBuf[0] = 1;
        break;
      case 75:
        writeBuf[0] = 2;
        break;
      case 70:
        writeBuf[0] = 3;
        break;
      default:
        return; // Error
    }
  }
  else return;
}

/**
 * @brief Read OVP Threshold Voltage is the VREQ voltage plus OVPTHR offset voltage (mV)
 * @return voltage in mV
 */
int AP33772S::readOVPTHR()
{
  i2c_read(CMD_OVPTHR, 1);
  return readBuf[0] * 80; // I2C read return 80mV/LSB
}

/**
 * @brief Set OVP Threshold Voltage is the VREQ voltage plus OVPTHR offset voltage (mV)
 * @param voltage in mV
 */
void AP33772S::setOVPTHR(int value)
{
  writeBuf[0] = value/80; //80mV/LSB
  i2c_write(CMD_OVPTHR, 1);
}

int AP33772S::readOCPTHR()
{
  i2c_read(CMD_OCPTHR, 1);
  return readBuf[0] * 50; // I2C read return 50mA/LSB
}
void AP33772S::setOCPTHR(int value)
{
  writeBuf[0] = value/50; // 50mA/LSB
  i2c_write(CMD_OCPTHR, 1);
}
int AP33772S::readOTPTHR()
{
  i2c_read(CMD_OTPTHR, 1);
  return readBuf[0]; // I2C read return 1C/LSB
}
void AP33772S::setOTPTHR(int value)
{
  writeBuf[0] = value; // 1C/LSB
  i2c_write(CMD_OTPTHR, 1);
}
int AP33772S::readDRTHR()
{
  i2c_read(CMD_DRTHR, 1);
  return readBuf[0]; // I2C read return 1C/LSB
}
void AP33772S::setDRTHR(int value)
{
  writeBuf[0] = value; // 1C/LSB
  i2c_write(CMD_DRTHR, 1);
}


/**
 * @brief Get internal PPS profile index (index start at 1)
 * @return indexPPS
 */
int AP33772S::getPPSIndex()
{
  return _indexPPSUser;
}

/**
 * @brief Get internal AVS profile index (index start at 1)
 * @return indexAVS
 */
int AP33772S::getAVSIndex()
{
  return _indexAVSUser;
}


void AP33772S::displaySPRVoltageMin(unsigned int current_max) {
  switch (current_max) {
    case 0:
      debugger.debugPrint("Reserved");
      break;
    case 1:
      debugger.debugPrint("3300mV~");
      break;
    case 2:
      debugger.debugPrint("3300mV < VOLTAGE_MIN ≤ 5000mV ");
      break;
    case 3:
      debugger.debugPrint("others");
      break;
    default:
      debugger.debugPrint("Invalid value");
      break;
  }
}


void AP33772S::displayEPRVoltageMin(unsigned int current_max) {
  switch (current_max) {
    case 0:
      debugger.debugPrint("Reserved");
      break;
    case 1:
      debugger.debugPrint("15000mV~");
      break;
    case 2:
      debugger.debugPrint("15000mV < VOLTAGE_MIN ≤ 20000mV ");
      break;
    case 3:
      debugger.debugPrint("others");
      break;
    default:
      debugger.debugPrint("Invalid value");
      break;
  }
}

/**
 * @brief take in current in mA unit
 * @return value from 0 to 15
 * @return -1 if there is an error
 */
int AP33772S::currentMap(int current)
{
  // Check if the value is out of bounds
  if (current < 0 || current > 5000) {
      return -1; // Return -1 for invalid inputs
  }

  // If value is below 1250, return 0
  if (current < 1250) {
      return 0;
  }

  // Calculate the result for ranges above 1250
  return ((current - 1250) / 250) + 1;
}

void AP33772S::displayCurrentRange(unsigned int current_max) {
  switch (current_max) {
    case 0:
      debugger.debugPrint("0.00A ~ 1.24A (Less than)");
      break;
    case 1:
      debugger.debugPrint("1.25A ~ 1.49A");
      break;
    case 2:
      debugger.debugPrint("1.50A ~ 1.74A");
      break;
    case 3:
      debugger.debugPrint("1.75A ~ 1.99A");
      break;
    case 4:
      debugger.debugPrint("2.00A ~ 2.24A");
      break;
    case 5:
      debugger.debugPrint("2.25A ~ 2.49A");
      break;
    case 6:
      debugger.debugPrint("2.50A ~ 2.74A");
      break;
    case 7:
      debugger.debugPrint("2.75A ~ 2.99A");
      break;
    case 8:
      debugger.debugPrint("3.00A ~ 3.24A");
      break;
    case 9:
      debugger.debugPrint("3.25A ~ 3.49A");
      break;
    case 10:
      debugger.debugPrint("3.50A ~ 3.74A");
      break;
    case 11:
      debugger.debugPrint("3.75A ~ 3.99A");
      break;
    case 12:
      debugger.debugPrint("4.00A ~ 4.24A");
      break;
    case 13:
      debugger.debugPrint("4.25A ~ 4.49A");
      break;
    case 14:
      debugger.debugPrint("4.50A ~ 4.99A");
      break;
    case 15:
      debugger.debugPrint("5.00A ~ (More than)");
      break;
    default:
      debugger.debugPrint("Invalid value");
      break;
  }
}

void BinaryStrZeroPad(int Number,char ZeroPadding){
//ZeroPadding = nth bit, e.g for a 16 bit number nth bit = 15
signed char i=ZeroPadding;

	while(i>=0){
	    if((Number & (1<<i)) > 0) printf("1");
	    else printf("0");
	    --i;
	}
        printf("\n");
}

/**
 * @brief Turn on/off the NMOS switch
 * @param flag 0 or 1 for OFF/ON
 * @return 1 if flag make sense
 * @bug can add code to check Vout voltage to ensure on or off, worry about settle time required for VOUT
 */
bool AP33772S::setOutput(uint8_t flag){
    switch(flag){
        case 0:
            writeBuf[0] = 0b00010001; //turn off
            i2c_write(CMD_SYSTEM, 1);
            return 1;
            break; //Sanity
        case 1:
            writeBuf[0] = 0b00010010; //turn on
            i2c_write(CMD_SYSTEM, 1);
            return 1;
            break; //Sanity
        default:
            return 0; //Error, dont know the input
    }
}


/**
 * @brief read using I2C hardware pin
 * @param cmdAddr target register
 * @param len number of data to read in Bytes
 */
void AP33772S::i2c_read(uint8_t cmdAddr, uint8_t len)
{
    for (uint8_t i = 0; i < READ_BUFF_LENGTH; i++)
    {
        readBuf[i] = 0;
    }

    i2c_write_blocking(i2c_, AP33772S_ADDRESS, &cmdAddr, 1, true);
    i2c_read_blocking(i2c_, AP33772S_ADDRESS, readBuf, len, false);

}

/**
 * @brief write using I2C hardware pin
 * @param cmdAddr target register
 * @param len number of data to write in Bytes
 */
void AP33772S::i2c_write(uint8_t cmdAddr, uint8_t len)
{
    uint8_t msg[len + 1];

    // Append register address to front of data packet
    msg[0] = cmdAddr;
    for (int i = 0; i < len; i++) {
        msg[i + 1] = writeBuf[i];
    }

    i2c_write_blocking(i2c_, AP33772S_ADDRESS, msg, len+1, false);
}