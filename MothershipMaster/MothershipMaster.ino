#include <OneWire.h>
#include <DallasTemperature.h>
#include <M5Stack.h>
#include <EEPROM.h>
#include <Wire.h>
#include "BluetoothSerial.h"
#include "mcp_can.h"
#include <SimpleTimer.h>

#define EEPROM_SIZE 512
#define ONE_WIRE_BUS 26
#define SLAVE_ADDRESS 0x60  // i2C Address
#define CAN0_INT 15                              // Set INT to pin 2
#define BGS1_ID 0x1806E5F4 // BGS1 message code BGS1
#define VCU_ID  0x1800F5E5 // VCU message code
#define READWRITE_PINS

MCP_CAN CAN0(12);     // Set CS to pin 10

DeviceAddress AddrList[12] = {
      {0x28, 0xFF, 0xDF, 0x20, 0xB2, 0x17, 0x02, 0xC4}, // probe 1
      {0x28, 0xFF, 0x81, 0x2B, 0xB2, 0x17, 0x02, 0x67}, // probe 2
      {0x28, 0xFF, 0x9E, 0x68, 0xB2, 0x17, 0x01, 0xFE}, // probe 3
      {0x28, 0xFF, 0xAF, 0x9D, 0xB2, 0x17, 0x02, 0xA5}, // probe 4
      {0x28, 0xFF, 0x7C, 0x66, 0xB2, 0x17, 0x01, 0xB2}, // probe 5
      {0x28, 0xFF, 0xD2, 0x6C, 0xC4, 0x17, 0x04, 0x39}, // probe 6
      {0x28, 0xFF, 0x75, 0x59, 0xC4, 0x17, 0x05, 0xCC}, // probe 7
      {0x28, 0xFF, 0xD1, 0x58, 0xC4, 0x17, 0x04, 0x31}, // probe 8
      {0x28, 0xFF, 0x19, 0x7B, 0x90, 0x17, 0x05, 0x6E}, // probe 9
      {0x28, 0xFF, 0x2D, 0x54, 0xB2, 0x17, 0x04, 0x9A}, // probe 10
      {0x28, 0x24, 0x7E, 0x79, 0x97, 0x15, 0x03, 0x23}, // probe 11
      {0x28, 0xD4, 0xBD, 0x79, 0x97, 0x15, 0x03, 0x3D}  // probe 12
};

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
BluetoothSerial SerialBT;
SimpleTimer timer;
int temploop = 0, shutoffchargecounter = 0;
unsigned long TemperatureTimer=0, celltimer = 0, temptimer = 0, chargetimer = 0, resettimer = 0, keepalivetimer = 0, keepalivecounter = 0, bmstimer = 0; 
int cellpacket = 1, buttonpacket = 1, chargepacket = 1, temppacket = 1,  bmssetpacket = 1; // timer variables
int lastvalue; // variable for button loop
float bmssetting,setchargervoltshv, setchargervoltslv, setchargerampshv, setchargerampslv, chargervoltshv, chargervoltslv, chargerampshv, chargervoltstotal, chargerampslv, chargertemperature, bmstemperature, zillatemperature, ambienttemperature;
float CellVoltage, TotalPackVoltage = 0, batteryamps = 0, minvoltage = 0, maxvoltage = 0, deltavoltage = 0; // for sending cell voltages to mothership
float TempPack[15] = {0};
float CellVoltages[40] = {3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2};
int CellState[40] = {0};
char bmslights[40] = {0};
char input;   //maybe uneeded?
unsigned int priority = 0;
unsigned int rpm = 0;
unsigned int kpa = 992; // 99.2
unsigned int tps = 965; // 96.5
unsigned int clt = 80;  // 80 - 10
unsigned int digitalPins = 0;
unsigned int ButtonOutput[12] = {0};
unsigned int ButtonOutputUpdated[12] = {0};
unsigned int incomingFramePos = 0;
unsigned char stmp[8];
unsigned long canbustime;  //timer
const unsigned long serialBlockTag = 0x11223344;  // canbus definition for realdash
byte incomingFrame[17] = { 0 };  // incoming data
byte data[8] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
INT32U rxId;  //canbus variables
INT8U len = 0;
INT8U rxBuf[8];

void init_can();  // compiler setup 

void setup()
{
delay(5000);
M5.begin();
Serial.begin(115200); 
pinMode(26, INPUT);  //
pinMode(25, OUTPUT);  // set speaker pin so it doesn't squawk

timer.setInterval(10000, setbmslevel);
timer.setInterval(9000, keepme);
timer.setInterval(15000, gettemperatures);


  EEPROM.begin(512);  // reset eeprom stored values.
    delay(10);
//    EEPROM.put(0, 3500); //bms 
    EEPROM.put(10, 1); //charger on/off
    EEPROM.put(20, 16); // pack current
//    EEPROM.put(30, 5);  // 12v current
//    EEPROM.put(40, 1);  // fan on/off
//    EEPROM.put(50, 80);  // fan set temp
    EEPROM.put(60, 1);  // reset
//    EEPROM.put(70, 144);  // extra
    EEPROM.commit();  
    yield();
    EEPROM.end(); 

sensors.setWaitForConversion(0); // don't wait for temperature conversion to complete
sensors.begin();
sensors.setResolution(11); 
EEPROM.begin(EEPROM_SIZE);
EEPROM.get(10, ButtonOutput[2]);  // charger on/off
EEPROM.get(20, ButtonOutput[3]);  // pack current
EEPROM.get(30, ButtonOutput[4]);  //12 v current
EEPROM.get(40, ButtonOutput[5]);  // fan on/off
EEPROM.get(50, ButtonOutput[6]);  // fan set temp
EEPROM.get(60, ButtonOutput[7]);  // BMS on/off
EEPROM.get(70, ButtonOutput[8]);  // MaxHV voltage
EEPROM.end();

if(ButtonOutput[8]>150){   // check to make sure maxHV is not overset
ButtonOutput[8]= 150;
} 

buttonpacket = 1;  // this will fill all values to start with

Serial.println("CAN BUS Shield init");
//canbus start setup
  canbustime = millis();
  init_can();
Serial.println("Test CAN...");
Serial.println("CAN BUS Shield init ok!");      
//canbus end setup

 Wire.begin();
 Serial.println("I2C up and running... waiting for I2C communication");
 SerialBT.register_callback(callback);
 
  if(!SerialBT.begin("ESP32")){
    Serial.println("An error occurred initializing Bluetooth");
  }else{
    Serial.println("Bluetooth initialized");
  }


}

void loop()
{
SendCANFramesToSerial();
ReadIncomingSerialData();
//UpdateAmpReading();
timer.run();

if (ButtonOutput[7]==1){  // if BMS Turned on.
if (millis() - celltimer >= 2000) {  // time to update cell voltages
          celltimer = millis();
cellpacket = 1;
}
}

if (ButtonOutput[2]==1){  // if charger turned on from tablet
if (millis() - chargetimer >= 800) {   // time to update charger
  chargetimer = millis();
  chargepacket = 1;
if (maxvoltage > 4190){  // safety shutoff if any cell too high!!!
  chargepacket = 0;
  chargervoltshv = 0;
  chargerampshv = 0;
  ButtonOutput[2]=0;  // toggle the button
  Serial.println("Voltage too high!  Charger Turned off");
}
readcharger(); 
}
}
  // keepalive swish bar

if (tps++ > 1000)
{
tps = 0;
}
}


void UpdateAmpReading()
{
  batteryamps=analogRead(26);
  Serial.print("Car Amps from Battery: ");
  Serial.println(batteryamps,3);
  batteryamps=(batteryamps*300)/4095;
  //batteryamps= batteryamps/1000;
  Serial.print("Car Amps from Battery: ");
  Serial.println(batteryamps,3);
}


void SendCANFramesToSerial()
{
  byte buf[8];


  memcpy(buf, &rpm, 2);
  memcpy(buf + 2, &kpa, 2);
  memcpy(buf + 4, &clt, 2);
  memcpy(buf + 6, &tps, 2);
  SendCANFrameToSerial(3200, buf);

  int amps1 = (int) batteryamps;
  memcpy(buf, &amps1, 2);
  memcpy(buf + 2, &amps1, 2);
  memcpy(buf + 4, &amps1, 2);
  memcpy(buf + 6, &amps1, 2);
  SendCANFrameToSerial(3201, buf);

if(buttonpacket == 1){

  memcpy(buf, &ButtonOutput[1], 2);
  memcpy(buf + 2, &ButtonOutput[2], 2);
  memcpy(buf + 4, &ButtonOutput[3], 2);
  memcpy(buf + 6, &ButtonOutput[4], 2);
  SendCANFrameToSerial(3100, buf);

  memcpy(buf, &ButtonOutput[5], 2);
  memcpy(buf + 2, &ButtonOutput[6], 2);
  memcpy(buf + 4, &ButtonOutput[7], 2);
  memcpy(buf + 6, &ButtonOutput[8], 2);
  SendCANFrameToSerial(3106, buf);

buttonpacket = 0;
}

if(chargepacket == 1)
{
  float v1 = setchargervoltshv * 10;   // the setting itself
  float v2 = setchargervoltslv * 1000;
  float v3 = setchargerampshv * 10;
  float v4 = setchargerampslv * 1000;
  int voltage1 = (int) v1;
  int voltage2 = (int) v2;
  int voltage3 = (int) v3;
  int voltage4 = (int) v4;
  memcpy(buf, &voltage1, 2);
  memcpy(buf + 2, &voltage2, 2);
  memcpy(buf + 4, &voltage3, 2);
  memcpy(buf + 6, &voltage4, 2);
  SendCANFrameToSerial(3101, buf);

  v1 = chargervoltshv * 10;  // real time values
  v2 = chargervoltslv * 1000;
  v3 = chargerampshv * 10;
  v4 = chargerampslv * 1000;
  voltage1 = (int) v1;
  voltage2 = (int) v2;
  voltage3 = (int) v3;
  voltage4 = (int) v4;
  memcpy(buf, &voltage1, 2);  
  memcpy(buf + 2, &voltage2, 2);
  memcpy(buf + 4, &voltage3, 2);
  memcpy(buf + 6, &voltage4, 2);
  SendCANFrameToSerial(3102, buf);

  v1 = chargertemperature;   // charger temperature
  v2 = minvoltage;  // the actual bms setting
  v3 = v3 * 100;
  v4 = setchargervoltshv;
  voltage1 = (int) v1;
  voltage2 = (int) v2;
  voltage3 = (int) v3;
  voltage4 = (int) v4;
  memcpy(buf, &voltage1, 2);
  memcpy(buf + 2, &voltage2, 2);
  memcpy(buf + 4, &voltage3, 2);
  memcpy(buf + 6, &voltage4, 2);
  SendCANFrameToSerial(3103, buf);


}

if(cellpacket == 1){
// voltage packets
TotalPackVoltage = 0;
int framepacket = 3202;
    for (int i = 1; i < 37; i = i + 4) {
      
     float v1 = RequestVoltage(i) * 1000;
     float v2 = RequestVoltage(i+1) * 1000;
     float v3 = RequestVoltage(i+2) * 1000;
     float v4 = RequestVoltage(i+3) * 1000;
     if (v1 > 1 && v1 < 5000){   // filter out stray values
     CellVoltages[i] = v1;      
     }
     if (v2 > 1 && v2 < 5000){
     CellVoltages[i+1] = v2;
     }
     if (v3 > 1 && v3 < 5000){
     CellVoltages[i+2] = v3;      
     }
     if (v4 > 1 && v4 < 5000){
     CellVoltages[i+3] = v4;
     }

     int voltage1 = (int) CellVoltages[i];
     int voltage2 = (int) CellVoltages[i+1];
     int voltage3 = (int) CellVoltages[i+2];
     int voltage4 = (int) CellVoltages[i+3];
     TotalPackVoltage = TotalPackVoltage + CellVoltages[i] + CellVoltages[i+1] + CellVoltages[i+2] + CellVoltages[i+3];
      memcpy(buf, & voltage1, 2);
      memcpy(buf + 2, & voltage2, 2);
      memcpy(buf + 4, & voltage3, 2);
      memcpy(buf + 6, & voltage4, 2);
      Serial.print(v1,1); Serial.print(", "); Serial.print(v2,1); Serial.print(", "); Serial.print(v3,1); Serial.print(", "); Serial.print(v4,1); Serial.print(", ");
      SendCANFrameToSerial(framepacket, buf);
      framepacket++;
  }

Serial.println("");
  int voltage = (int) TotalPackVoltage / 100;  // prepare pack voltage to send

minvoltage = 50000 ; // Find lowest voltage level.               // 
for (int i = 1; i<37; i++) // Changed this.
{
    if (CellVoltages[i] < minvoltage) {
         minvoltage = CellVoltages[i];
      //   Serial.print("min voltage: "); Serial.println(minvoltage);
    }
}

maxvoltage = CellVoltages[1] ; // Find highest voltage level.               // 
for (int i = 1; i<37; i++) // Changed this.
{
    if (CellVoltages[i] > maxvoltage) {
         maxvoltage = CellVoltages[i];
    }
}
Serial.print("max voltage: "); Serial.println(maxvoltage);

deltavoltage = maxvoltage - minvoltage;
int dvoltage = (int) deltavoltage;
int xvoltage = (int) minvoltage;
int nvoltage = (int) maxvoltage;
  memcpy(buf, &voltage, 2);
  memcpy(buf + 2, &nvoltage, 2);
  memcpy(buf + 4, &xvoltage, 2);
  memcpy(buf + 6, &dvoltage, 2);
  SendCANFrameToSerial(3107, buf);

  Serial.print("Total Pack Voltage: "); Serial.println(TotalPackVoltage);
  Serial.print("voltage:"); Serial.println(voltage);
  //M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setCursor(5, 10);
  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.setTextSize(3);

  M5.Lcd.setCursor(50, 10);
  M5.Lcd.print("V: ");
  M5.Lcd.fillRect(100, 10, 200, 30, TFT_BLACK);
  M5.Lcd.print(voltage/10);

framepacket = 3108; 
RequestBMSStateBS1();
for (int i = 1; i < 37; i = i + 4) {


      int light1 = (char) bmslights[i];
      int light2 = (char) bmslights[i+1];
      int light3 = (char) bmslights[i+2];
      int light4 = (char) bmslights[i+3];      
      memcpy(buf, & light1, 2);
  //    Serial.print("bmslight: ");
  //    Serial.println(light1);
      memcpy(buf + 2, & light2, 2);
      memcpy(buf + 4, & light3, 2);
      memcpy(buf + 6, & light4, 2);
      SendCANFrameToSerial(framepacket, buf);
      framepacket++;
}

  cellpacket = 0;
}

if (temppacket == 1){
// temp packets
float t1 = TempPack[0]*100;
float t2 = TempPack[1]*100;
float t3 = TempPack[2]*100;
float t4 = TempPack[3]*100;
int temp1 = (int) t1;
int temp2 = (int) t2;
int temp3 = (int) t3;
int temp4 = (int) t4;
memcpy(buf, &temp1, 2);
memcpy(buf + 2, &temp2, 2);
memcpy(buf + 4, &temp3, 2);
memcpy(buf + 6, &temp4, 2);
SendCANFrameToSerial(3211, buf);

t1 = TempPack[4]*100;
t2 = TempPack[5]*100;
t3 = TempPack[6]*100;
t4 = TempPack[7]*100;
temp1 = (int) t1;
temp2 = (int) t2;
temp3 = (int) t3;
temp4 = (int) t4;

memcpy(buf, &temp1, 2);
memcpy(buf + 2, &temp2, 2);
memcpy(buf + 4, &temp3, 2);
memcpy(buf + 6, &temp4, 2);
SendCANFrameToSerial(3212, buf);

t1 = TempPack[8]*100;
t2 = TempPack[9]*100;
t3 = TempPack[10]*100;
t4 = TempPack[11]*100;
temp1 = (int) t1;
temp2 = (int) t2;
temp3 = (int) t3;
temp4 = (int) t4;

memcpy(buf, &temp1, 2);
memcpy(buf + 2, &temp2, 2);
memcpy(buf + 4, &temp3, 2);
memcpy(buf + 6, &temp4, 2);
SendCANFrameToSerial(3213, buf);

memcpy(buf, &chargertemperature, 2);
memcpy(buf + 2, &bmstemperature, 2);
memcpy(buf + 4, &zillatemperature, 2);
memcpy(buf + 6, &ambienttemperature, 2);
SendCANFrameToSerial(3102, buf);


temppacket = 0;
}

}

void SendCANFrameToSerial(unsigned long canFrameId, const byte* frameData)
{

//Serial.println(digitalPins);
  // the 4 byte identifier at the beginning of each CAN frame
  // this is required for RealDash to 'catch-up' on ongoing stream of CAN frames
  SerialBT.write((const byte * ) & serialBlockTag, 4);

  // the CAN frame id number
  SerialBT.write((const byte * ) & canFrameId, 4);

  // CAN frame payload
  SerialBT.write(frameData, 8);

//Serial.println("Writeframe");

}

void ReadIncomingSerialData()
{

//Serial.println("Readframe");

 while (SerialBT.available() > 0)
  {
    // little bit of extra effort here, since especially Bluetooth connections
    // may leave unsent/received data in internal buffer for a long time
    // therefore, we cannot be sure that incoming byte stream really starts at
    // where we expect it to start.
    // read one byte from serial stream

    incomingFrame[incomingFramePos++] = SerialBT.read();

    // check the first incoming bytes tag (0x44, 0x33, 0x22, 0x11)
    if (incomingFrame[0] != 0x44)
    {
      // first incoming byte is not 0x44, 
      // the tag at the beginning of the frame does not match, this is an invalid frame
      // just zero the incomingFrame buffer and start expecting first byte again
      memset(incomingFrame, 0, 17);
      incomingFramePos = 0;
    }

    if (incomingFramePos >= 17)
    {
      // frame complete, process it
      ProcessIncomingFrame(incomingFrame);
      
      // zero the incomingFrame buffer and start expecting first byte again
      memset(incomingFrame, 0, 17);
      incomingFramePos = 0;
    }
  }
}

void ProcessIncomingFrame(const byte* frame)
{

  // first four bytes contain set value frame separator bytes, always 0x44,0x33,0x22,x11
  // check that first 4 bytes match the tag
  if (frame[0] != 0x44 ||
      frame[1] != 0x33 ||
      frame[2] != 0x22 ||
      frame[3] != 0x11)
  {
    // frame tag does not match, wait for another frame
    return;
  }

  // next four bytes contain set value CAN frame id in little endian form
  unsigned long canFrameId = 0;
  memcpy(&canFrameId, frame + 4, 4);

  // next 8 bytes are the frame data
  // ...
  
  // last byte is check byte calculated as sum of previous 13 bytes (ignore overflow)
  byte checkByte = 0;
  for (int i=0; i<16; i++)
  {
    checkByte += frame[i];
  }

  if (frame[16] == checkByte)
  {
    // checksum match, this is a valid set value-frame:
    // the frame payload data is in frame + 8 bytes
    HandleIncomingSetValueFrame(canFrameId, frame + 8);
  }
}

void HandleIncomingSetValueFrame(unsigned long canFrameId, const byte* frameData)
{
  if (canFrameId == 3100)
  {
    memcpy(&ButtonOutput[1], frameData + 0, 2);
    memcpy(&ButtonOutput[2], frameData + 2, 2);
    memcpy(&ButtonOutput[3], frameData + 4, 2);
    memcpy(&ButtonOutput[4], frameData + 6, 2);
 
  for(int bx=0; bx<4; bx++){   //cycle through the incoming button variables to see if anything needs to be updated
    if(ButtonOutput[bx+1]!=ButtonOutputUpdated[bx+1]){
    EEPROM.begin(512);
    EEPROM.put(0+bx*10, ButtonOutput[bx+1]);
    ButtonOutputUpdated[bx+1]=ButtonOutput[bx+1];
    Serial.print("EEPROM Updated: ");
    Serial.println(ButtonOutput[bx+1]);
    EEPROM.commit();  
    yield();
    EEPROM.end(); 
    } 
  }

 buttonpacket = 1;

  }


   if (canFrameId == 3106)
  {
    memcpy(&ButtonOutput[5], frameData + 0, 2);
    memcpy(&ButtonOutput[6], frameData + 2, 2);
    memcpy(&ButtonOutput[7], frameData + 4, 2);
    memcpy(&ButtonOutput[8], frameData + 6, 2);
  

  for(int bx=4; bx<8; bx++){   //cycle through the incoming button variables to see if anything needs to be updated
    if(ButtonOutput[bx+1]!=ButtonOutputUpdated[bx+1]){
    EEPROM.begin(512);
    EEPROM.put(0+bx*10, ButtonOutput[bx+1]);
    ButtonOutputUpdated[bx+1]=ButtonOutput[bx+1];
    Serial.print("EEPROM Updated: ");
    Serial.println(ButtonOutput[bx+1]);
    EEPROM.commit();  
    yield();
    EEPROM.end(); 
    } 
  }

    buttonpacket = 1;
  }

}



void SendCanBGS1_Cmd(unsigned int volts, unsigned int amps)
{
//     Serial.print("Sending Can BGS1 cmd\r\n");
  // send data:  id, standard frame, data len = 8, stmp: data buf

  stmp[0]=(volts*10)>>8;     //high byte. 0.1 volts/bit
  stmp[1]=(volts*10)&0x00ff; //low byte.
  stmp[2]=(amps*10)>>8;     //high byte. 0.1 amps/bit
  stmp[3]=(amps*10)&0x00ff; //low byte.
  stmp[4]=0x00; // 00=charger 0n, 01=charger off
  stmp[5]=0x00; // battery mode, 01=heating mode
  stmp[6]=0; // reserved
  stmp[7]=0; // reserved

 //     Serial.print("stmp0=0x");
 //     Serial.println(stmp[0],HEX);
//      Serial.print("stmp1=0x");
//      Serial.println(stmp[1],HEX);
//      Serial.print("stmp2=0x");
//      Serial.println(stmp[2],HEX);
//      Serial.print("stmp3=0x");
//      Serial.println(stmp[3],HEX);
  
  CAN0.sendMsgBuf(BGS1_ID,1, 8, stmp);
}


void SendCanVCU_Cmd(unsigned int volts, unsigned int amps)
{
   //   Serial.print("Sending Can VCU cmd\r\n");
      // send data:  id , standard frame, data len = 8, stmp: data buf
      stmp[0]=0; // reserved
      stmp[1]=1; // power on = 1, 0=off
      stmp[2]=(volts*10)&0x00ff; //low byte.
      stmp[3]=(volts*10)>>8;     //high byte. 0.1 volts/bit
      stmp[4]=(amps*10)&0x00ff; //low byte.
      stmp[5]=(amps*10)>>8;     //high byte. 0.1 amps/bit     
      stmp[6]=0; // reserved
      stmp[7]=0; // reserved

//      Serial.print("stmp2=0x");
//      Serial.println(stmp[2],HEX);
//      Serial.print("stmp3=0x");
//      Serial.println(stmp[3],HEX);
//      Serial.print("stmp4=0x");
//      Serial.println(stmp[4],HEX);
//      Serial.print("stmp5=0x");
//      Serial.println(stmp[5],HEX);
      
      CAN0.sendMsgBuf(VCU_ID,1, 8, stmp);
}



void SetBMSLimit (int HighLimit){
char MyString[10];
    Wire.beginTransmission(SLAVE_ADDRESS);
    Wire.write("SV 30000 ");  
    itoa(HighLimit,MyString,10); // convert integer number to ascii characters
    Wire.write(MyString);
    Wire.write("\r");       // terminate with carriage return delimiter. This is what I will look for when parsing.
    Wire.endTransmission();
    Serial.print("Sent BMS Data...");
  M5.Lcd.fillRect(50, 150, 350, 175, TFT_BLACK);
  M5.Lcd.setCursor(10, 150);
  M5.Lcd.print("BMS:");
  M5.Lcd.setCursor(100, 150);
  M5.Lcd.print(minvoltage);
  Serial.print("Minimum Voltage: ");
  Serial.println(minvoltage);
}


void init_can(){
   // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
  if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK)
    Serial.println("MCP2515 Initialized Successfully!");
  else
    Serial.println("Error Initializing MCP2515...");
  CAN0.setMode(MCP_NORMAL);                     // Set operation mode to normal so the MCP2515 sends acks to received data.
  pinMode(CAN0_INT, INPUT);                            // Configuring pin for /INT input

}



void readcharger(){

//canbus loop code start
unsigned char len = 0;
//unsigned char CanStat;
if(!digitalRead(CAN0_INT))                         // If CAN0_INT pin is low, read receive buffer
{         // check if data coming 
    CAN0.readMsgBuf(&rxId, &len, rxBuf);    // read data,  len: data length, buf: data buf 

 //   Serial.print("rxId=");
 //   Serial.println(rxId,HEX);
    rxId = rxId & 0x1fffffff;
 //     Serial.println(rxId, HEX);
    if(rxId == 0x18ff50e5)
    {
 ///       Serial.println("***********************************");
  //       Serial.print("Get data from CCS1 ID: 0x");
  //       Serial.println(rxId,HEX);
        for(int i = 0; i<len; i++)    // print the data
        {
  //          Serial.print(rxBuf[i], HEX);
  //          Serial.print("\t");
        }
        shutoffchargecounter = 0;
  //      Serial.println();
        chargervoltshv = (rxBuf[0]*256+rxBuf[1])/10.0;
        chargerampshv = (rxBuf[2]*256+rxBuf[3])/10.0;
  //      chargervoltslv = (rxBuf[5]*.2);
  //      chargerampslv = (rxBuf[4]*256+rxBuf[3])/10.0;
  //      chargertemperature = rxBuf[7]-40;
        Serial.print("hvvolts=");
        Serial.print(chargervoltshv,1);Serial.print(" ");
        Serial.print("amps=");
        Serial.println(chargerampshv,1);Serial.print(" ");
        Serial.print("status=");
        Serial.println(rxBuf[4]);Serial.println();


    }

    rxId = rxId & 0x1fffffff;
//      Serial.println(rxId, HEX);
    if(rxId == 0x1800e5f5)//0x18ffe5f5
    {

     shutoffchargecounter++;

 //       Serial.println("----------------------------------");
 //        Serial.print("Get data from DCDC ID: 0x");
 //        Serial.println(rxId,HEX);
        for(int i = 0; i<len; i++)    // print the data
        {
  //          Serial.print(rxBuf[i], HEX);
  //          Serial.print("\t");
        }
   //     Serial.println();
        chargervoltslv = (rxBuf[5]*.2);
        chargerampslv = (rxBuf[4]*256+rxBuf[3])/10.0;
        chargertemperature = rxBuf[7]-40;
 //       chargervoltshv = (rxBuf[0]*256+rxBuf[1])/10.0;
 //       chargerampshv = (rxBuf[2]*256+rxBuf[3])/10.0;


   //     Serial.print("volts=");
   //     Serial.print(chargervoltslv,1);Serial.print(" ");
   //     Serial.print("amps=");
   //     Serial.print(chargerampslv,1);Serial.print(" ");
   //     Serial.print("temperature=");
   //     Serial.print(chargertemperature);Serial.println("C");
    }
    if (millis() - canbustime >= 1000) {
      canbustime = millis();
   //   Serial.println("sending BGS1 command");
 
      setchargerampslv = ButtonOutput[4];  //when ready to test add these variables in place of hard coded values below
      setchargerampshv = ButtonOutput[3];
      setchargervoltshv = ButtonOutput[8];

   //     Serial.print("setchargerampslv=");
   ///     Serial.print(ButtonOutput[4]);Serial.print(" ");
        Serial.print("setchargerampshv=");
        Serial.print(setchargerampshv);Serial.print(" ");
   //     Serial.print("bmssetting=");
   //     Serial.print(ButtonOutput[1]);Serial.print(" ");
        Serial.print("setchargervoltshv=");
        Serial.print(setchargervoltshv);Serial.print(" ");
//      Serial.print("BMS Set Voltage Calculated:");
//      Serial.println(bmssetting);
if(chargepacket==1){
      SendCanBGS1_Cmd(setchargervoltshv,setchargerampshv); // 140 volts, 3 amps
}
//  chargervoltshv = (rxBuf[0]*256+rxBuf[1])/10.0;  // is charger on?
//      Serial.println("sending VCU command");
      SendCanVCU_Cmd(14,14);  // dc battery 14 volts, 5 amps
    }
}
//canbus loop code end
Serial.print("shutoffcounter: "); Serial.println(shutoffchargecounter); Serial.println("");

if(shutoffchargecounter > 120){
  chargervoltshv = 0;
  chargerampshv = 0;
}

}

//***********************************************************************
void RequestBMSStateBS1(){
unsigned int c;

int i;
    Wire.beginTransmission(SLAVE_ADDRESS);
    Wire.write("BS1 ");
    Wire.write("\r");       // terminate with carriage return delimiter. This is what I will look for when parsing.
    Wire.endTransmission();
    delay(400);//allow time for bms to get new voltage data
    Wire.requestFrom(SLAVE_ADDRESS,38); // now request some data, up to 8 characters
    i=0;
    while (Wire.available()) { // counts down the REQUESTED character count
      c = Wire.read();         // get character
 //   Serial.println(c, HEX);// print the character
      if(c==255) break;        // still needed, break if default char is detected
       bmslights[i++] = c;       // build character string
    }  


    Serial.print("BMS RETURNS: ");
    Serial.println(bmslights);

i=i+1;
    
Wire.beginTransmission(SLAVE_ADDRESS);
    Wire.write("BS2 ");
    Wire.write("\r");       // terminate with carriage return delimiter. This is what I will look for when parsing.
    Wire.endTransmission();
    delay(400);//allow time for bms to get new voltage data
    Wire.requestFrom(SLAVE_ADDRESS,38); // now request some data, up to 8 characters
    while (Wire.available()) { // counts down the REQUESTED character count
      c = Wire.read();         // get character
 //   Serial.println(c, HEX);// print the character
      if(c==255) break;        // still needed, break if default char is detected
       bmslights[i++] = c;       // build character string
    }  
    bmslights[i]=0;      // add the NULL character for string conversion to float

    Serial.print("BMS RETURNS: ");
    Serial.println(bmslights);

    return;           // return float value



}



//***********************************************************************
float RequestVoltage (int CellNumber){
char MyString[10];
unsigned int c;
float f;
int i;
    Wire.beginTransmission(SLAVE_ADDRESS);
    Wire.write("CV ");
    itoa(CellNumber,MyString,10); // convert integer number to ascii characters
    Wire.write(MyString);
    Wire.write("\r");       // terminate with carriage return delimiter. This is what I will look for when parsing.
    Wire.endTransmission();
   //Serial.print("Requesting Data...");
    
    if(CellNumber == 1) delay(400);//allow time for bms to get new voltage data
    Wire.requestFrom(SLAVE_ADDRESS,8); // now request some data, up to 8 characters
    i=0;
    while (Wire.available()) { // counts down the REQUESTED character count
      c = Wire.read();         // get character
      if(c==255) break;        // still needed, break if default char is detected
 //   Serial.println(c, HEX);// print the character
      MyString[i++] = c;       // build character string
    }  

    MyString[i]=0;      // add the NULL character for string conversion to float
    f=atof(MyString);   // convert string to float
    return f;           // return float value
}



void gettemperatures(){
// start another cycle if >400 milliseconds have gone by...
float temptemp = 0;
    Serial.print("Temperatures are: ");
    for(temploop=0; temploop < 13; temploop++){
 //   Serial.print(sensors.getTempCByIndex(11),1); // 1 decimal place of precision
      temptemp=sensors.getTempF(AddrList[temploop]);
      if(temptemp < 80){
        TempPack[temploop]=temptemp;
      }
      Serial.print(TempPack[temploop]);// print with 1 decimal place of precision      
      Serial.print(" ");
    }
    Serial.println(); // cr/lf
    sensors.requestTemperatures(); // Send global command to convert temperatures, no wait
    // You can have more than one IC on the sa()me bus. 
    // 0 refers to the first IC on the wire
    temppacket = 1;
}


void callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param){
  if(event == ESP_SPP_SRV_OPEN_EVT){
    Serial.println("Client Connected");
  }

  if(event == ESP_SPP_CLOSE_EVT ){
    Serial.println("Client disconnected");
   ESP.restart();
  }
}

void setbmslevel(){
  Serial.print("chargervoltshv: ");
  Serial.println(chargervoltshv);
  if(chargervoltshv>100 && chargervoltshv<150){
  SetBMSLimit(minvoltage*10+50);    
  }
  else{
  SetBMSLimit(41900);   
  Serial.println("Not Charging:  BMS SETPOINT raised to 4.19");

  }
}

void keepme(){
    M5.Lcd.setCursor(5, 50);
  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.setTextSize(3);

  M5.Lcd.setCursor(50, 50);
  keepalivecounter = keepalivecounter + 1;
  M5.Lcd.print("K: ");
  M5.Lcd.fillRect(80, 50, 200, 75, TFT_BLACK);
  M5.Lcd.print(keepalivecounter);
  buttonpacket = 1;
}