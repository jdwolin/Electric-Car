 // code for Jason's EV beetle. 10/4/2019
// 10/6/2019 menu option to send fake random data between 3.6 and 4.2 volts
// added this comment line 11/2/2019 11:00pm. NO CODE changes!

#include <LTC681x.h>

#include <Arduino.h>
#include <stdint.h>
#include "Linduino.h"
#include "LT_SPI.h"
#include "UserInterface.h"
#include "LTC681x.h"
#include "LTC6813.h"
#include <SPI.h>
#include <Wire.h>
#include <string.h>

#define ENABLED 1
#define DISABLED 0

#define DATALOG_ENABLED 1
#define DATALOG_DISABLED 0

void print_menu();
void print_cells(uint8_t datalog_en);
void print_aux(uint8_t datalog_en);
void print_stat();
void print_open();
void print_config();
void print_rxconfig();
void print_comm();
void print_rxcomm();
void print_pwm();
void print_rxpwm();
void print_sctrl();
void print_rxsctrl();
void print_pwm_sctrlb();
void print_rxpsb(); 
void print_statsoc();
void print_aux1();
void check_error(int error);
void print_pec();
char get_char();


/**********************************************************
  Setup Variables
  The following variables can be modified to
  configure the software.

***********************************************************/
const uint8_t TOTAL_IC = 2;//!<number of ICs in the daisy chain

//ADC Command Configurations---------------------See LTC681x.h for options
const uint8_t ADC_OPT = ADC_OPT_DISABLED;
const uint8_t ADC_CONVERSION_MODE =MD_7KHZ_3KHZ; //MD_422HZ_1KHZ  //MD_7KHZ_3KHZ
const uint8_t ADC_DCP = DCP_DISABLED;//dw: DCP_DISABLED
const uint8_t CELL_CH_TO_CONVERT =CELL_CH_ALL;
const uint8_t AUX_CH_TO_CONVERT = AUX_CH_ALL;
const uint8_t STAT_CH_TO_CONVERT = STAT_CH_ALL;
const uint8_t NO_OF_REG = REG_ALL;

const uint16_t MEASUREMENT_LOOP_TIME = 1000;//milliseconds(mS) dw:500

//Under Voltage and Over Voltage Thresholds
const uint16_t OV_THRESHOLD = 37000; // Over voltage threshold ADC Code. LSB = 0.0001 dw: was 41000
const uint16_t UV_THRESHOLD = 30000; // Under voltage threshold ADC Code. LSB = 0.0001 dw:was 33000

//Loop Measurement Setup These Variables are ENABLED or DISABLED Remember ALL CAPS
const uint8_t WRITE_CONFIG = DISABLED; // This is ENABLED or DISABLED dw: was disabled
const uint8_t READ_CONFIG = DISABLED; // This is ENABLED or DISABLED dw: was disabled
const uint8_t MEASURE_CELL = ENABLED; // This is ENABLED or DISABLED
const uint8_t MEASURE_AUX = DISABLED; // This is ENABLED or DISABLED dw:was disabled
const uint8_t MEASURE_STAT = DISABLED; //This is ENABLED or DISABLED  dw: was disabled
const uint8_t PRINT_PEC = DISABLED; //This is ENABLED or DISABLED

// I2C stuff
#define SLAVE_ADDRESS 0x60
byte xx = 0x45;
float CellVoltage; // for sending cell voltages to mothership
int CellNumber;
int RandomFlag=0; // for debug

/************************************
  END SETUP
*************************************/

/******************************************************
 *** Global Battery Variables received from 681x commands
 These variables store the results from the LTC6813
 register reads and the array lengths must be based
 on the number of ICs on the stack
 ******************************************************/

// variables for IC0
uint8_t    DwStbr2_IC0=0;
uint8_t    DwStbr3_IC0=0;
uint8_t    DwStbr4_IC0=0;
uint8_t    DwAvdr4_IC0=0;
uint8_t    DwAvdr5_IC0=0;
bool    DwDccbits_IC0[12]= {false,false,false,false,false,false,false,false,false,false,false,false}; //Dcc 1,2,3... thru 12
bool    DwDccbits_IC0B[7]= {false,false,false,false,false,false,false}; // Dcc 0,13,14,15,16,17,18
bool    DcbitsOff[18]= {false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false};   //Dcc 1,2,3... thru 18

// variables for IC1
uint8_t    DwStbr2_IC1=0;
uint8_t    DwStbr3_IC1=0;
uint8_t    DwStbr4_IC1=0;
uint8_t    DwAvdr4_IC1=0;
uint8_t    DwAvdr5_IC1=0;
bool    DwDccbits_IC1[12]= {false,false,false,false,false,false,false,false,false,false,false,false}; //Dcc 1,2,3,4,5,6,7,8,9,10,11,12
bool    DwDccbits_IC1B[7]= {false,false,false,false,false,false,false}; // Dcc 0,13,14,15,16,17,18

cell_asic bms_ic[TOTAL_IC];

const uint8_t PWMREG=0;
const uint8_t STREG=0; 

//Set the configuration bits

bool REFON = true;
bool ADCOPT = false;
bool gpioBits_a[5] = {false,false,false,true,true}; // Gpio 1,2,3,4,5
bool gpioBits_dw[5] = {false,false,false,true,true}; // Gpio 1,2,3,4,5
uint16_t UV=UV_THRESHOLD;
uint16_t OV=OV_THRESHOLD;
// bool dccBits_a[12] = {false,false,false,false,false,false,false,false,false,false,false,false}; //Dcc 1,2,3,4,5,6,7,8,9,10,11,12
bool dctoBits[4] = {false,false,false,false}; //Dcto 0,1,2,3 
bool FDRF = false;
bool DTMEN = false; // 
bool psBits[2]= {false,false}; //ps-0,1
bool gpioBits_b[4] = {false,false,false,false}; // Gpio 6,7,8,9
bool dccBits_b[7]= {false,false,false,false,false,false,false}; //Dcc 0,13,14,15,16,17,18


/*!**********************************************************************
 \brief  Inititializes hardware and variables
 ***********************************************************************/
void setup()
{
  Serial.begin(115200);
  quikeval_SPI_connect();
  spi_enable(SPI_CLOCK_DIV32); // This will set the Linduino to have a 1MHz Clock
  LTC6813_init_cfg(TOTAL_IC, bms_ic);
  LTC6813_init_cfgb(TOTAL_IC,bms_ic);
  for (uint8_t current_ic = 0; current_ic<TOTAL_IC;current_ic++) 
  {
    LTC6813_set_cfgr(current_ic,bms_ic,REFON,ADCOPT,gpioBits_a,DwDccbits_IC0, dctoBits, UV, OV);
    LTC6813_set_cfgrb(current_ic,bms_ic,FDRF,DTMEN,psBits,gpioBits_b,DwDccbits_IC0B);   
  }   
  LTC6813_reset_crc_count(TOTAL_IC,bms_ic);
  LTC6813_init_reg_limits(TOTAL_IC,bms_ic);

  // I2C stuff
  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  Serial.println("I2C up and running... waiting for I2C communication");
  print_menu();

float a=3.1200;
Serial.print("a=");
Serial.println(a,4);
char str[10];
dtostrf(a, 6, 4, str);     // convert to string,6 total characters, 4 digits precision
Serial.print("string equivalent=");
Serial.println(str);

  
}

/*!*********************************************************************
  \brief main loop
***********************************************************************/
void loop()
{
  if (Serial.available())           // Check for user input
  {
    uint32_t user_command;
    user_command = read_int();      // Read the user command
    Serial.println(user_command);
    run_command(user_command);
  }
  else 
  {
//    ToggleLed();
//    delay(500);
  }
}


float RequestVoltage (int CellNum){
char MyString[10];
char c;
float f;
int i;
    Wire.beginTransmission(0x60);
    Wire.write("CV ");
    itoa(MyString,CellNum,10); // convert integer number to ascii characters
    Wire.write(MyString);
    Wire.write("\r");       // terminate with carriage return delimiter. This is what I will look for when parsing.
    Wire.endTransmission();
    delay(100);
    Serial.println("Requesting Data");
    
    Wire.requestFrom(SLAVE_ADDRESS, 250); // now request some data
    i=0;
    while (Wire.available()) { // slave may send less than requested
      c = Wire.read();         // get character
      MyString[i++] = c;       // build character string
     //  if(c==255) break;     // i dont think this is needed anymore
      Serial.print(c);         // print the character
    }       
    
    MyString[i]=0;      // add the NULL character, may not be needed (already there) but doesn't hurt
    f=atof(MyString);   // convert string to float
    return f;           // return float value
}



void requestEvent() // master is requesting the voltage from a cell number passed during the I2C receive event
{
char str_temp[8];
  if((CellNumber > 0) && (CellNumber <= 36)) {
    if(CellNumber == 1) {
        wakeup_sleep(TOTAL_IC);
        LTC6813_wrcfg(TOTAL_IC,bms_ic);
        LTC6813_wrcfgb(TOTAL_IC,bms_ic);
        measurement_loop(DATALOG_ENABLED); // read all cell voltages
    }
    if(RandomFlag == 0){
      
     CellVoltage = GetCellVoltage(CellNumber); // get the requested cell voltage
    } else {
  float   LowLim = 3.0;
  float   HighLim = 4.2;
      CellVoltage = LowLim + random(0,(HighLim-LowLim)*1000)/1000.0; // debug line ... generate fake voltage
    }
 //   Serial.print("GCV CN CV");Serial.print(CellNumber);Serial.print(" ");Serial.println(CellVoltage);
    Serial.print("Master Requested Cell");
    Serial.print(CellNumber);
    Serial.print("...");
    dtostrf(CellVoltage, 5, 3, str_temp);     // convert to string, 3 digits precision
    Wire.write(str_temp);                     // send string to I2C
    Serial.print("Sending...");
    Serial.println(str_temp);                 // send string to rs232
  }else {
    Serial.print("Bad cell number ");
    Serial.print(CellNumber,DEC); Serial.print("\n");
  }
}

void receiveEvent(int bytes) // number of characters to read is passed into this routine.
{
char Cmd[15]; // enough space for "CV 36" plus null character or "SV 27500 38000" plus null
unsigned int BmsLowVoltage, BmsHighVoltage;
char *CmdPtr  = NULL;  // command pointer
char *Arg1Ptr = NULL;  // argument 1 pointer
char *Arg2Ptr = NULL;  // argument 2 pointer

CellNumber = 0;
//Serial.print("rcvEvnt bytes to read=");Serial.println(bytes);
  for(int i = 0; i< bytes; i++) // read in characters from I2C buffer
  {
    xx = Wire.read();
    Cmd[i] = xx; // save incoming characters into a string
//    Serial.print("Received: ");
//    Serial.print(xx, HEX);
//    Serial.print("\n");
  }
// Processing I2C commands
  //Parse the command string.
  CmdPtr  = strtok(Cmd, " \r"); // Pointer to first char of Cmd, marks when a SPACE or \r found
  Arg1Ptr = strtok(0," \r");     // 0 continues parsing from previous stopping point.
  Arg2Ptr = strtok(0," \r");     // 0 continues parsing from previous stopping point.

// test for valid commands
  if (strcmp(CmdPtr, "CV") == 0) {
    CellNumber = atoi(Arg1Ptr);     // assign cell number from arg 1
 //   Serial.print("CN=");Serial.println(CellNumber);
  }
  else if (strcmp(CmdPtr, "SV") == 0) {
    BmsLowVoltage  = atoi(Arg1Ptr);
    BmsHighVoltage = atoi(Arg2Ptr);
    UV = BmsLowVoltage;  // update global thresholds
    OV = BmsHighVoltage; // update global thresholds
    LTC6813_set_cfgr(0,bms_ic,REFON,ADCOPT,gpioBits_a,DwDccbits_IC1, dctoBits, BmsLowVoltage, BmsHighVoltage); // cells 1 thru 18
    LTC6813_set_cfgr(1,bms_ic,REFON,ADCOPT,gpioBits_a,DwDccbits_IC0, dctoBits, BmsLowVoltage, BmsHighVoltage); // cells 19 thru 36
    wakeup_sleep(TOTAL_IC);
    LTC6813_wrcfg(TOTAL_IC,bms_ic);
    LTC6813_wrcfgb(TOTAL_IC,bms_ic);
    
    UpdateFets();
    Serial.print("updating BMS thresholds...");
    Serial.print("Low Voltage =");
    Serial.print(BmsLowVoltage);
    Serial.print("  High Voltage =");
    Serial.print(BmsHighVoltage);
    Serial.println();
  }
  else {
    Serial.print(CmdPtr);
    Serial.println(" is a bad I2C command");
  }
}


//***************************************************
void ToggleLed(void){
  gpioBits_dw[0] = gpioBits_dw[0] ^ 1; // gpio 1 led toggle test
  wakeup_sleep(TOTAL_IC);
  LTC6813_wrcfg(TOTAL_IC,bms_ic);
  LTC6813_wrcfgb(TOTAL_IC,bms_ic);
  LTC6813_set_cfgr(1,bms_ic,REFON,ADCOPT,gpioBits_dw,DcbitsOff, dctoBits, UV, OV); // update fet gate control IC0
  LTC6813_set_cfgrb(1,bms_ic,FDRF,DTMEN,psBits,gpioBits_b,dccBits_b);  

  LTC6813_set_cfgr(0,bms_ic,REFON,ADCOPT,gpioBits_dw,DcbitsOff, dctoBits, UV, OV); // update fet gate control IC1
  LTC6813_set_cfgrb(0,bms_ic,FDRF,DTMEN,psBits,gpioBits_b,dccBits_b);   
}


//*************************************************************
void UpdateFets(void){
  LTC6813_rdstat(0,TOTAL_IC,bms_ic); //  read back all stat registers (lower cells over voltage status here)
  LTC6813_rdaux(0,TOTAL_IC,bms_ic);  //  read back all aux registers  (upper cells over voltage status here)
  DwStbr2_IC0 = bms_ic[0].stat.flags[0]; // status reg group B, cells 1 thru 4 
  DwStbr3_IC0 = bms_ic[0].stat.flags[1]; // status reg group B, cells 5 thru 8
  DwStbr4_IC0 = bms_ic[0].stat.flags[2]; // status reg group B, cells 9 thru 12
  DwAvdr4_IC0 = bms_ic[0].aux.a_codes[11]; // Aux  reg group D, cells 13 thru 16 (advr4)
  DwAvdr5_IC0 = bms_ic[0].aux.a_codes[11]>>8; // Aux  reg group D, cells 17 thru 18 (advr5)
  
  DwStbr2_IC1 = bms_ic[1].stat.flags[0]; // status reg group B, cells 19 thru 22 
  DwStbr3_IC1 = bms_ic[1].stat.flags[1]; // status reg group B, cells 23 thru 26
  DwStbr4_IC1 = bms_ic[1].stat.flags[2]; // status reg group B, cells 27 thru 30
  DwAvdr4_IC1 = bms_ic[1].aux.a_codes[11]; // Aux  reg group D, cells 31 thru 34  (advr4)
  DwAvdr5_IC1 = bms_ic[1].aux.a_codes[11]>>8; // Aux  reg group D, cells 35 thru 36 (advr5)

  if(DwStbr2_IC0 & 0x02) DwDccbits_IC0[0] = true; else DwDccbits_IC0[0] = false;// cell 1
  if(DwStbr2_IC0 & 0x08) DwDccbits_IC0[1] = true; else DwDccbits_IC0[1] = false;// cell 2
  if(DwStbr2_IC0 & 0x20) DwDccbits_IC0[2] = true; else DwDccbits_IC0[2] = false;// cell 3
  if(DwStbr2_IC0 & 0x80) DwDccbits_IC0[3] = true; else DwDccbits_IC0[3] = false;// cell 4
  if(DwStbr3_IC0 & 0x02) DwDccbits_IC0[4] = true; else DwDccbits_IC0[4] = false;// cell 5
  if(DwStbr3_IC0 & 0x08) DwDccbits_IC0[5] = true; else DwDccbits_IC0[5] = false;// cell 6
  if(DwStbr3_IC0 & 0x20) DwDccbits_IC0[6] = true; else DwDccbits_IC0[6] = false;// cell 7
  if(DwStbr3_IC0 & 0x80) DwDccbits_IC0[7] = true; else DwDccbits_IC0[7] = false;// cell 8
  if(DwStbr4_IC0 & 0x02) DwDccbits_IC0[8] = true; else DwDccbits_IC0[8] = false;// cell 9
  if(DwStbr4_IC0 & 0x08) DwDccbits_IC0[9] = true; else DwDccbits_IC0[9] = false;// cell 10
  if(DwStbr4_IC0 & 0x20) DwDccbits_IC0[10] = true; else DwDccbits_IC0[10] = false;// cell 11
  if(DwStbr4_IC0 & 0x80) DwDccbits_IC0[11] = true; else DwDccbits_IC0[11] = false;// cell 12 
  DwDccbits_IC0B[0] = false; // cell 0
  if(DwAvdr4_IC0 & 0x02) DwDccbits_IC0B[1] = true; else DwDccbits_IC0B[1] = false;// cell 13
  if(DwAvdr4_IC0 & 0x08) DwDccbits_IC0B[2] = true; else DwDccbits_IC0B[2] = false;// cell 14
  if(DwAvdr4_IC0 & 0x20) DwDccbits_IC0B[3] = true; else DwDccbits_IC0B[3] = false;// cell 15
  if(DwAvdr4_IC0 & 0x80) DwDccbits_IC0B[4] = true; else DwDccbits_IC0B[4] = false;// cell 16
  if(DwAvdr5_IC0 & 0x02) DwDccbits_IC0B[5] = true; else DwDccbits_IC0B[5] = false;// cell 17
  if(DwAvdr5_IC0 & 0x08) DwDccbits_IC0B[6] = true; else DwDccbits_IC0B[6] = false;// cell 18

  if(DwStbr2_IC1 & 0x02) DwDccbits_IC1[0] = true; else DwDccbits_IC1[0] = false;// cell 19
  if(DwStbr2_IC1 & 0x08) DwDccbits_IC1[1] = true; else DwDccbits_IC1[1] = false;// cell 20
  if(DwStbr2_IC1 & 0x20) DwDccbits_IC1[2] = true; else DwDccbits_IC1[2] = false;// cell 21
  if(DwStbr2_IC1 & 0x80) DwDccbits_IC1[3] = true; else DwDccbits_IC1[3] = false;// cell 22
  if(DwStbr3_IC1 & 0x02) DwDccbits_IC1[4] = true; else DwDccbits_IC1[4] = false;// cell 23
  if(DwStbr3_IC1 & 0x08) DwDccbits_IC1[5] = true; else DwDccbits_IC1[5] = false;// cell 24
  if(DwStbr3_IC1 & 0x20) DwDccbits_IC1[6] = true; else DwDccbits_IC1[6] = false;// cell 25
  if(DwStbr3_IC1 & 0x80) DwDccbits_IC1[7] = true; else DwDccbits_IC1[7] = false;// cell 26
  if(DwStbr4_IC1 & 0x02) DwDccbits_IC1[8] = true; else DwDccbits_IC1[8] = false;// cell 27
  if(DwStbr4_IC1 & 0x08) DwDccbits_IC1[9] = true; else DwDccbits_IC1[9] = false;// cell 28
  if(DwStbr4_IC1 & 0x20) DwDccbits_IC1[10] = true; else DwDccbits_IC1[10] = false;// cell 29
  if(DwStbr4_IC1 & 0x80) DwDccbits_IC1[11] = true; else DwDccbits_IC1[11] = false;// cell 30 
DwDccbits_IC1B[0] = false; // cell 0
  if(DwAvdr4_IC1 & 0x02) DwDccbits_IC1B[1] = true; else DwDccbits_IC1B[1] = false;// cell 13
  if(DwAvdr4_IC1 & 0x08) DwDccbits_IC1B[2] = true; else DwDccbits_IC1B[2] = false;// cell 14
  if(DwAvdr4_IC1 & 0x20) DwDccbits_IC1B[3] = true; else DwDccbits_IC1B[3] = false;// cell 15
  if(DwAvdr4_IC1 & 0x80) DwDccbits_IC1B[4] = true; else DwDccbits_IC1B[4] = false;// cell 16
  if(DwAvdr5_IC1 & 0x02) DwDccbits_IC1B[5] = true; else DwDccbits_IC1B[5] = false;// cell 17
  if(DwAvdr5_IC1 & 0x08) DwDccbits_IC1B[6] = true; else DwDccbits_IC1B[6] = false;// cell 18
  

  gpioBits_dw[0] = gpioBits_dw[0] ^ 1; // gpio 1 led toggle test
  wakeup_sleep(TOTAL_IC);
  LTC6813_set_cfgr(1,bms_ic,REFON,ADCOPT,gpioBits_dw,DwDccbits_IC0, dctoBits, UV, OV); // update fet gate control IC0
  LTC6813_set_cfgrb(1,bms_ic,FDRF,DTMEN,psBits,gpioBits_b,DwDccbits_IC0B);  

  LTC6813_set_cfgr(0,bms_ic,REFON,ADCOPT,gpioBits_dw,DwDccbits_IC1, dctoBits, UV, OV); // update fet gate control IC1
  LTC6813_set_cfgrb(0,bms_ic,FDRF,DTMEN,psBits,gpioBits_b,DwDccbits_IC1B);   

  LTC6813_wrcfg(TOTAL_IC,bms_ic);
  LTC6813_wrcfgb(TOTAL_IC,bms_ic);
}

/*!*****************************************
  \brief executes the user command
*******************************************/
void run_command(uint32_t cmd)
{
  int8_t error = 0;
  uint32_t conv_time = 0;
  uint32_t user_command;
  int8_t readIC=0;
  uint16_t DwReadIC=0;
  char input = 0;
  uint32_t adcstate =0;
  switch (cmd)
  {

    case 1:
      wakeup_sleep(TOTAL_IC);
      LTC6813_wrcfg(TOTAL_IC,bms_ic);
      LTC6813_wrcfgb(TOTAL_IC,bms_ic);
      print_config();
      error = LTC6813_rdcfg(TOTAL_IC,bms_ic);
      check_error(error);
      error = LTC6813_rdcfgb(TOTAL_IC,bms_ic);
      check_error(error);
      print_rxconfig();
    break;

    case 2: // Start Cell ADC Measurement
      wakeup_sleep(TOTAL_IC);
      LTC6813_adcv(ADC_CONVERSION_MODE,ADC_DCP,CELL_CH_TO_CONVERT);
      conv_time = LTC6813_pollAdc();
      Serial.print(F("Cell conversion completed in:"));
      Serial.print(((float)conv_time/1000), 1);
      Serial.println(F("mS"));
      Serial.println();
    break;
    
    case 3: // Read Cell Voltage Registers
      wakeup_sleep(TOTAL_IC);
      error = LTC6813_rdcv(NO_OF_REG,TOTAL_IC,bms_ic); 
      check_error(error);
      print_cells(DATALOG_DISABLED);
      break;
    
    case 31: // input over voltage threshold. Dale's new command

      Serial.print(F("Please enter Over Voltage threshold in mV"));
      Serial.println();
      DwReadIC = (uint16_t)read_int();
      if(DwReadIC != 0){
        OV = DwReadIC;              
        Serial.print(OV,DEC);
        Serial.println();  
        wakeup_sleep(TOTAL_IC);
        LTC6813_wrcfg(TOTAL_IC,bms_ic);
        LTC6813_wrcfgb(TOTAL_IC,bms_ic);
  
        for(uint8_t x=1;x < 5;x++){ // weird pipeline bug... need to do this 4 times
           measurement_loop(DATALOG_ENABLED); // read the cell voltages
           UpdateFets();
        }
      }

      while (input != 'm'){
        if (Serial.available() > 0)
        {
           input = read_char();
        }
         measurement_loop(DATALOG_ENABLED); // read the cell voltages
         UpdateFets();
         delay(MEASUREMENT_LOOP_TIME); // milliseconds
      }
      break;

    case 32:
      Serial.print("bms low threshold=");Serial.println(UV);
      Serial.print("bms high threshold=");Serial.println(OV);
    break;
    
    case 33:
      Serial.print(F("Random data flag (0 or 1)"));
      Serial.println();
      RandomFlag = (uint16_t)read_int();
    break;

   
    case 'm': //prints menu
      print_menu();
      break;

    default:
      Serial.println(F("Incorrect Option"));
      break;
  }
}

void measurement_loop(uint8_t datalog_en) // ***************************************************************
{
  int8_t error = 0;
  
  if (WRITE_CONFIG == ENABLED)
  {
    wakeup_idle(TOTAL_IC);
    LTC6813_wrcfg(TOTAL_IC,bms_ic);
    LTC6813_wrcfgb(TOTAL_IC,bms_ic);
    print_config();
  }

  if (READ_CONFIG == ENABLED)
  {
    wakeup_idle(TOTAL_IC);
    error = LTC6813_rdcfg(TOTAL_IC,bms_ic);
    check_error(error);
    error = LTC6813_rdcfgb(TOTAL_IC,bms_ic); 
    check_error(error);
    print_rxconfig();
  }

  if (MEASURE_CELL == ENABLED)
  {
    wakeup_idle(TOTAL_IC);
    LTC6813_adcv(ADC_CONVERSION_MODE,ADC_DCP,CELL_CH_TO_CONVERT);
    LTC6813_pollAdc();
    wakeup_idle(TOTAL_IC);
    error = LTC6813_rdcv(0, TOTAL_IC,bms_ic);
    check_error(error);
    print_cells(datalog_en);  //uncommented causes data to jason to mess up fro some reason

  }

  if (MEASURE_AUX == ENABLED)
  {
    wakeup_idle(TOTAL_IC);
    LTC6813_adax(ADC_CONVERSION_MODE , AUX_CH_ALL);
    LTC6813_pollAdc();
    wakeup_idle(TOTAL_IC);
    error = LTC6813_rdaux(0,TOTAL_IC,bms_ic); // Set to read back all aux registers
    check_error(error);
    print_aux(datalog_en);
  }
 
  if (MEASURE_STAT == ENABLED)
  {
    wakeup_idle(TOTAL_IC);
    LTC6813_adstat(ADC_CONVERSION_MODE, STAT_CH_ALL);
    LTC6813_pollAdc();
    wakeup_idle(TOTAL_IC);
    error = LTC6813_rdstat(0,TOTAL_IC,bms_ic); // Set to read back all stat registers
    check_error(error);
    print_stat();
  }

  if(PRINT_PEC == ENABLED)
  {
    print_pec();
  }  
}
/*!****************************************************************************
  Function to check error flag and print PEC error message
 *****************************************************************************/
void check_error(int error)
{
  if (error == -1)
  {
    Serial.println(F("A PEC error was detected in the received data"));
  }
}

/*!*********************************
  \brief Prints the main menu
***********************************/
void print_menu()
{
  Serial.println(F("Write and Read Configuration: 1                                | Loop measurements with datalog output : 11            | Write and Read of PWM : 21 "));
  Serial.println(F("Start Cell Voltage Conversion: 2                               | Run Mux Self Test : 12                                | Write and  Read of Scontrol : 22 "));
  Serial.println(F("Read Cell Voltages: 3                                          | Run ADC Self Test: 13                                 | Write and Read of PWM/S control Register B : 23 "));
  Serial.println(F("Start Aux Voltage Conversion: 4                                | ADC overlap Test : 14                                 | Clear S control register : 24 "));
  Serial.println(F("Read Aux Voltages: 5                                           | Run Digital Redundancy Test : 15                      | SPI Communication  : 25 "));
  Serial.println(F("Start Stat Voltage Conversion: 6                               | Open Wire Test : 16                                   | I2C Communication Write to Slave :26 "));
  Serial.println(F("Read Stat Voltages: 7                                          | Print PEC Counter: 17                                 | I2C Communication Read from Slave :27"));
  Serial.println(F("Start Combined Cell Voltage and GPIO1, GPIO2 Conversion: 8     | Reset PEC Counter: 18                                 | Enable MUTE : 28"));
  Serial.println(F("Start  Cell Voltage and Sum of cells : 9                       | Set Discharge: 19                                     | Disable MUTE : 29"));
  Serial.println(F("loop Measurements: 10                                          | Clear Discharge: 20                                   | Clear Registers: 30 "));
  Serial.println(F("Over Voltage Threshold: 31                                     | disp bms thresholds: 32                                    | Random I2C cell data: 33"));
  Serial.println();
  Serial.println(F("Print 'm' for menu"));
  Serial.println(F("Please enter command: "));
  Serial.println();
}

/*!************************************************************
  \brief Prints cell voltage codes to the serial port
 *************************************************************/
void print_cells(uint8_t datalog_en)
{
  for (int current_ic = 0 ; current_ic < TOTAL_IC; current_ic++)
  {
    if (datalog_en == 0)
    {
      Serial.print("IC ");
      Serial.print(current_ic+1,DEC);
      Serial.print(",");
      for (int i=0; i<bms_ic[0].ic_reg.cell_channels; i++)
      {

        Serial.print(" C");
        Serial.print(i+1,DEC);
        Serial.print(":");
        Serial.print(bms_ic[current_ic].cells.c_codes[i]*0.0001,3);
        Serial.print(",");
      }
      Serial.println();
    }
    else
    {
      Serial.print("Cells, ");
      for (int i=0; i<bms_ic[0].ic_reg.cell_channels; i++)
      {
        Serial.print(bms_ic[current_ic].cells.c_codes[i]*0.0001,3);
        Serial.print(",");
      }
    }
  }
  Serial.println();
}
//************************************************************************************
float GetCellVoltage(int CellNum)
{
int current_ic; // bms bank. 0 or 1
char str_temp[8]; // string for float conversion
float f;

      if(CellNum > 18){ // determine which bms bank to read from.
        CellNum = CellNum - 18;
        current_ic = 1;
      }
      else current_ic = 0;
      //Serial.print("GCN=");Serial.println(CellNum);
      f=bms_ic[current_ic].cells.c_codes[CellNum-1]*0.0001; // get the cell voltage
      return(f);
}



/*!****************************************************************************
  \brief Prints Open wire test results to the serial port
 *****************************************************************************/
void print_open()
{
  for (int current_ic =0 ; current_ic < TOTAL_IC; current_ic++)
  {
    if (bms_ic[current_ic].system_open_wire == 0)
    {
      Serial.print("No Opens Detected on IC: ");
      Serial.print(current_ic+1, DEC);
      Serial.println();
    }
    else
    {
      for (int cell=0; cell<bms_ic[0].ic_reg.cell_channels+1; cell++)
      {
        if ((bms_ic[current_ic].system_open_wire &(1<<cell))>0)
        {
          Serial.print(F("There is an open wire on IC: "));
          Serial.print(current_ic + 1,DEC);
          Serial.print(F(" Channel: "));
          Serial.println(cell,DEC);
        }
      }
    }
  }
}

/*!****************************************************************************
  \brief Prints GPIO voltage codes and Vref2 voltage code onto the serial port
 *****************************************************************************/
void print_aux(uint8_t datalog_en)
{
  for (int current_ic =0 ; current_ic < TOTAL_IC; current_ic++)
  {
    if (datalog_en == 0)
    {
      Serial.print(" IC ");
      Serial.print(current_ic+1,DEC);
      for (int i=0; i < 5; i++)
      {
        Serial.print(F(" GPIO-"));
        Serial.print(i+1,DEC);
        Serial.print(":");
        Serial.print(bms_ic[current_ic].aux.a_codes[i]*0.0001,4);
        Serial.print(",");
      }
      Serial.print(F(" Vref2"));
      Serial.print(":");
      Serial.print(bms_ic[current_ic].aux.a_codes[5]*0.0001,4);
      for (int i=6; i < 10; i++)
      {
        Serial.print(F(", GPIO-"));
        Serial.print(i,DEC);
        Serial.print(":");
        Serial.print(bms_ic[current_ic].aux.a_codes[i]*0.0001,4);
        
      }
        Serial.println();
        Serial.print(" Flags : 0x");
        Serial.print(bms_ic[current_ic].aux.a_codes[11],HEX);
      Serial.println();
    }
    else
    {
      Serial.print("AUX, ");

      for (int i=0; i < 12; i++)
      {
        Serial.print(bms_ic[current_ic].aux.a_codes[i]*0.0001,4);
        Serial.print(",");
      }
    }
  }
  Serial.println();
}

/*!****************************************************************************
  \brief Prints Status voltage codes and Vref2 voltage code onto the serial port
 *****************************************************************************/
void print_stat()
{

  for (int current_ic =0 ; current_ic < TOTAL_IC; current_ic++)
  {
    Serial.print(F(" IC "));
    Serial.print(current_ic+1,DEC);
    Serial.print(F(" SOC:"));
    Serial.print(bms_ic[current_ic].stat.stat_codes[0]*0.0001*30,4);
    Serial.print(F(","));
    Serial.print(F(" Itemp:"));
    Serial.print(bms_ic[current_ic].stat.stat_codes[1]*0.0001,4);
    Serial.print(F(","));
    Serial.print(F(" VregA:"));
    Serial.print(bms_ic[current_ic].stat.stat_codes[2]*0.0001,4);
    Serial.print(F(","));
    Serial.print(F(" VregD:"));
    Serial.print(bms_ic[current_ic].stat.stat_codes[3]*0.0001,4);
    Serial.println();
    Serial.print(F("Flags:"));
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].stat.flags[0]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].stat.flags[1]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].stat.flags[2]);
     Serial.print(F("\tMux fail flag:"));
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].stat.mux_fail[0]);
     Serial.print(F("\tTHSD:"));
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].stat.thsd[0]);
    Serial.println();  
  }
  Serial.println();
}

/*!******************************************************************************
 \brief Prints the configuration data that is going to be written to the LTC6813
 to the serial port.
 ********************************************************************************/
void print_config()
{
  int cfg_pec;
  Serial.println(F("Written Configuration: "));
  for (int current_ic = 0; current_ic<TOTAL_IC; current_ic++)
  {
    Serial.print(F("CFGA IC "));
    Serial.print(current_ic+1,DEC);
    for(int i = 0;i<6;i++)
    {
      Serial.print(F(", 0x"));
      serial_print_hex(bms_ic[current_ic].config.tx_data[i]);
    }
    Serial.print(F(", Calculated PEC: 0x"));
    cfg_pec = pec15_calc(6,&bms_ic[current_ic].config.tx_data[0]);
    serial_print_hex((uint8_t)(cfg_pec>>8));
    Serial.print(F(", 0x"));
    serial_print_hex((uint8_t)(cfg_pec));
    Serial.println();
    
    Serial.print(F("CFGB IC "));
    Serial.print(current_ic+1,DEC);
    for(int i = 0;i<6;i++)
    {
      Serial.print(F(", 0x"));
      serial_print_hex(bms_ic[current_ic].configb.tx_data[i]);
    }
    Serial.print(F(", Calculated PEC: 0x"));
    cfg_pec = pec15_calc(6,&bms_ic[current_ic].configb.tx_data[0]);
    serial_print_hex((uint8_t)(cfg_pec>>8));
    Serial.print(F(", 0x"));
    serial_print_hex((uint8_t)(cfg_pec));
    Serial.println();
  }
  Serial.println();
}

/*!*****************************************************************
 \brief Prints the configuration data that was read back from the
 LTC6813 to the serial port.
 *******************************************************************/
void print_rxconfig()
{
  Serial.println(F("Received Configuration "));
  for (int current_ic=0; current_ic<TOTAL_IC; current_ic++)
  {
    Serial.print(F("CFGA IC "));
    Serial.print(current_ic+1,DEC);
    for(int i = 0;i<6;i++)
    {
      Serial.print(F(", 0x"));
      serial_print_hex(bms_ic[current_ic].config.rx_data[i]);
    };
    Serial.print(F(", Received PEC: 0x"));
    serial_print_hex(bms_ic[current_ic].config.rx_data[6]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].config.rx_data[7]);
    Serial.println();

    Serial.print(F("CFGB IC "));
    Serial.print(current_ic+1,DEC);
    for(int i = 0;i<6;i++)
    {
      Serial.print(F(", 0x"));
      serial_print_hex(bms_ic[current_ic].configb.rx_data[i]);
    };
    Serial.print(F(", Received PEC: 0x"));
    serial_print_hex(bms_ic[current_ic].configb.rx_data[6]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].configb.rx_data[7]);
    Serial.println();
  }
  Serial.println();
}

void print_pec()
{
  for (int current_ic=0; current_ic<TOTAL_IC; current_ic++)
  {
      Serial.println("");
      Serial.print(bms_ic[current_ic].crc_count.pec_count,DEC);
      Serial.print(F(" : PEC Errors Detected on IC"));
      Serial.println(current_ic+1,DEC);
  }
}
/*!****************************************************************************
  \brief Prints GPIO voltage codes (GPIO1 & 2)
 *****************************************************************************/
void print_aux1(uint8_t datalog_en)
{

  for (int current_ic =0 ; current_ic < TOTAL_IC; current_ic++)
  {
    if (datalog_en == 0)
    {
      Serial.print(" IC ");
      Serial.print(current_ic+1,DEC);
      for (int i=0; i < 2; i++)
      {
        Serial.print(F(" GPIO-"));
        Serial.print(i+1,DEC);
        Serial.print(":");
        Serial.print(bms_ic[current_ic].aux.a_codes[i]*0.0001,4);
        Serial.print(",");
      }
    }
    else
    {
      Serial.print("AUX, ");

      for (int i=0; i < 12; i++)
      {
        Serial.print(bms_ic[current_ic].aux.a_codes[i]*0.0001,4);
        Serial.print(",");
      }
    }
  }
  Serial.println();
}
/*!****************************************************************************
  \brief Prints Status voltage codes for SOC onto the serial port
 *****************************************************************************/
void print_statsoc()
{

  for (int current_ic =0 ; current_ic < TOTAL_IC; current_ic++)
  {
    Serial.print(F(" IC "));
    Serial.print(current_ic+1,DEC);
    Serial.print(F(" SOC:"));
    Serial.print(bms_ic[current_ic].stat.stat_codes[0]*0.0001*30,4);
    Serial.print(F(","));
  }

  Serial.println();
}
/*!****************************************************************************
  \brief Prints received data from COMM register onto the serial port
 *****************************************************************************/

void print_rxcomm()
{
  Serial.println(F("Received Data"));
  for (int current_ic=0; current_ic<TOTAL_IC; current_ic++)
  {
    Serial.print(F(" IC "));
    Serial.print(current_ic+1,DEC);
    Serial.print(F(": 0x"));
    serial_print_hex(bms_ic[current_ic].com.rx_data[0]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].com.rx_data[1]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].com.rx_data[2]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].com.rx_data[3]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].com.rx_data[4]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].com.rx_data[5]);
    Serial.print(F(", Received PEC: 0x"));
    serial_print_hex(bms_ic[current_ic].com.rx_data[6]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].com.rx_data[7]);
    Serial.println();
  }
  Serial.println();
}
/*!****************************************************************************
  /prints data which is written on COMM register onto the serial port
 *****************************************************************************/
void print_comm()
{
  int comm_pec;

  Serial.println(F("Written Data in COMM Resgiter: "));
  for (int current_ic = 0; current_ic<TOTAL_IC; current_ic++)
  {
    Serial.print(F(" IC- "));
    Serial.print(current_ic+1,DEC);
    Serial.print(F(": "));
    Serial.print(F("0x"));
    serial_print_hex(bms_ic[current_ic].com.tx_data[0]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].com.tx_data[1]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].com.tx_data[2]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].com.tx_data[3]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].com.tx_data[4]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].com.tx_data[5]);
    Serial.print(F(", Calculated PEC: 0x"));
    comm_pec = pec15_calc(6,&bms_ic[current_ic].com.tx_data[0]);
    serial_print_hex((uint8_t)(comm_pec>>8));
    Serial.print(F(", 0x"));
    serial_print_hex((uint8_t)(comm_pec));
    Serial.println();
  }
  Serial.println();
}
/*!****************************************************************************
  /prints data which is written on PWM register onto the serial port
 *****************************************************************************/
void print_pwm()
{
  int pwm_pec;

  Serial.println(F("Written Data in PWM: "));
  for (int current_ic = 0; current_ic<TOTAL_IC; current_ic++)
  {
    Serial.print(F(" IC "));
    Serial.print(current_ic+1,DEC);
    Serial.print(F(": "));
    Serial.print(F("0x"));
    serial_print_hex(bms_ic[current_ic].pwm.tx_data[0]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].pwm.tx_data[1]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].pwm.tx_data[2]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].pwm.tx_data[3]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].pwm.tx_data[4]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].pwm.tx_data[5]);
    Serial.print(F(", Calculated PEC: 0x"));
    pwm_pec = pec15_calc(6,&bms_ic[current_ic].pwm.tx_data[0]);
    serial_print_hex((uint8_t)(pwm_pec>>8));
    Serial.print(F(", 0x"));
    serial_print_hex((uint8_t)(pwm_pec));
    Serial.println();
  }
  Serial.println();
}
/*!****************************************************************************
  \brief Prints received data from PWM register onto the serial port
 *****************************************************************************/
void print_rxpwm()
{
  Serial.println(F("Received Data in PWM"));
  for (int current_ic=0; current_ic<TOTAL_IC; current_ic++)
  {
    Serial.print(F(" IC "));
    Serial.print(current_ic+1,DEC);
    Serial.print(F(": 0x"));
    serial_print_hex(bms_ic[current_ic].pwm.rx_data[0]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].pwm.rx_data[1]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].pwm.rx_data[2]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].pwm.rx_data[3]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].pwm.rx_data[4]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].pwm.rx_data[5]);
    Serial.print(F(", Received PEC: 0x"));
    serial_print_hex(bms_ic[current_ic].pwm.rx_data[6]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].pwm.rx_data[7]);
    Serial.println();
  }
  Serial.println();
}
/*!****************************************************************************
  /prints data which is written on S Control register 
 *****************************************************************************/
void print_sctrl()
{
  int sctrl_pec;

  Serial.println(F("Written Data in s ctrl register: "));
  for (int current_ic = 0; current_ic<TOTAL_IC; current_ic++)
  {
    Serial.print(F(" IC "));
    Serial.print(current_ic+1,DEC);
    Serial.print(F(": "));
    Serial.print(F("0x"));
    serial_print_hex(bms_ic[current_ic].sctrl.tx_data[0]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].sctrl.tx_data[1]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].sctrl.tx_data[2]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].sctrl.tx_data[3]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].sctrl.tx_data[4]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].sctrl.tx_data[5]);
    Serial.print(F(", Calculated PEC: 0x"));
    sctrl_pec = pec15_calc(6,&bms_ic[current_ic].sctrl.tx_data[0]);
    serial_print_hex((uint8_t)(sctrl_pec>>8));
    Serial.print(F(", 0x"));
    serial_print_hex((uint8_t)(sctrl_pec));
    Serial.println();
  }
  Serial.println();
}
/*!****************************************************************************
  /prints data which is read back from S Control register 
 *****************************************************************************/
void print_rxsctrl()
{
  Serial.println(F("Received Data in S control reg"));
  for (int current_ic=0; current_ic<TOTAL_IC; current_ic++)
  {
    Serial.print(F(" IC "));
    Serial.print(current_ic+1,DEC);
    Serial.print(F(": 0x"));
    serial_print_hex(bms_ic[current_ic].sctrl.rx_data[0]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].sctrl.rx_data[1]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].sctrl.rx_data[2]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].sctrl.rx_data[3]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].sctrl.rx_data[4]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].sctrl.rx_data[5]);
    Serial.print(F(", Received PEC: 0x"));
    serial_print_hex(bms_ic[current_ic].sctrl.rx_data[6]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].sctrl.rx_data[7]);
    Serial.println();
  }
  Serial.println();
}
/*!****************************************************************************
  /prints data which is written on PWM / S Control register 
 *****************************************************************************/
void print_pwm_sctrlb()
{
  int sctrlb_pec;

  Serial.println(F("Written Data in s ctrl register: "));
  for (int current_ic = 0; current_ic<TOTAL_IC; current_ic++)
  {
    Serial.print(F(" IC "));
    Serial.print(current_ic+1,DEC);
    Serial.print(F(": "));
    Serial.print(F("0x"));
    serial_print_hex(bms_ic[current_ic].pwmb.tx_data[0]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].pwmb.tx_data[1]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].pwmb.tx_data[2]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].sctrlb.tx_data[3]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].sctrlb.tx_data[4]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].sctrlb.tx_data[5]);
    Serial.print(F(", Calculated PEC: 0x"));
    sctrlb_pec = pec15_calc(6,&bms_ic[current_ic].sctrlb.tx_data[0]);
    serial_print_hex((uint8_t)(sctrlb_pec>>8));
    Serial.print(F(", 0x"));
    serial_print_hex((uint8_t)(sctrlb_pec));
    Serial.println();
  }
  Serial.println();
}
/*!****************************************************************************
  /prints data which is read back from PWM/ S Control register B
 *****************************************************************************/
 void print_rxpsb()
 {
  Serial.println(F("Received Data in PWM/S control reg B :"));
  for (int current_ic=0; current_ic<TOTAL_IC; current_ic++)
  {
    Serial.print(F(" IC "));
    Serial.print(current_ic+1,DEC);
    Serial.print(F(": 0x"));
    serial_print_hex(bms_ic[current_ic].sctrlb.rx_data[0]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].sctrlb.rx_data[1]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].sctrlb.rx_data[2]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].sctrlb.rx_data[3]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].sctrlb.rx_data[4]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].sctrlb.rx_data[5]);
    Serial.print(F(", Received PEC: 0x"));
    serial_print_hex(bms_ic[current_ic].sctrlb.rx_data[6]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].sctrlb.rx_data[7]);
    Serial.println();
  }
  Serial.println();
 }
