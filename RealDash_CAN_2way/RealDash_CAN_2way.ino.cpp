# 1 "c:\\users\\jasonw~1\\appdata\\local\\temp\\tmpmgsxsj"
#include <Arduino.h>
# 1 "C:/Users/Jason Wolin/Dropbox/My Hobbies/Electric Car/RealDash_CAN_2way/RealDash_CAN_2way.ino"
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
#define SLAVE_ADDRESS 0x60
#define CAN0_INT 15
#define BGS1_ID 0x1806E5F4
#define VCU_ID 0x1800F5E5
#define READWRITE_PINS 

MCP_CAN CAN0(12);

DeviceAddress AddrList[12] = {
      {0x28, 0xFF, 0xDF, 0x20, 0xB2, 0x17, 0x02, 0xC4},
      {0x28, 0xFF, 0x81, 0x2B, 0xB2, 0x17, 0x02, 0x67},
      {0x28, 0xFF, 0x9E, 0x68, 0xB2, 0x17, 0x01, 0xFE},
      {0x28, 0xFF, 0xAF, 0x9D, 0xB2, 0x17, 0x02, 0xA5},
      {0x28, 0xFF, 0x7C, 0x66, 0xB2, 0x17, 0x01, 0xB2},
      {0x28, 0xFF, 0xD2, 0x6C, 0xC4, 0x17, 0x04, 0x39},
      {0x28, 0xFF, 0x75, 0x59, 0xC4, 0x17, 0x05, 0xCC},
      {0x28, 0xFF, 0xD1, 0x58, 0xC4, 0x17, 0x04, 0x31},
      {0x28, 0xFF, 0x19, 0x7B, 0x90, 0x17, 0x05, 0x6E},
      {0x28, 0xFF, 0x2D, 0x54, 0xB2, 0x17, 0x04, 0x9A},
      {0x28, 0x24, 0x7E, 0x79, 0x97, 0x15, 0x03, 0x23},
      {0x28, 0xD4, 0xBD, 0x79, 0x97, 0x15, 0x03, 0x3D}
};

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
BluetoothSerial SerialBT;
SimpleTimer timer;
int temploop = 0;
unsigned long TemperatureTimer=0, celltimer = 0, temptimer = 0, chargetimer = 0, resettimer = 0, keepalivetimer = 0, keepalivecounter = 0, bmstimer = 0;
int cellpacket = 1, buttonpacket = 1, chargepacket = 1, temppacket = 1, bmssetpacket = 1;
int lastvalue;
float bmssetting,setchargervoltshv, setchargervoltslv, setchargerampshv, setchargerampslv, chargervoltshv, chargervoltslv, chargerampshv, chargervoltstotal, chargerampslv, chargertemperature, bmstemperature, zillatemperature, ambienttemperature;
float CellVoltage, TotalPackVoltage = 0, batteryamps = 0, minvoltage = 0, maxvoltage = 0, deltavoltage = 0;
float TempPack[15] = {0};
float CellVoltages[40] = {3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2,3.2};
char input;
unsigned int priority = 0;
unsigned int rpm = 0;
unsigned int kpa = 992;
unsigned int tps = 965;
unsigned int clt = 80;
unsigned int digitalPins = 0;
unsigned int ButtonOutput[12] = {0};
unsigned int ButtonOutputUpdated[12] = {0};
unsigned int incomingFramePos = 0;
unsigned char stmp[8];
unsigned long canbustime;
const unsigned long serialBlockTag = 0x11223344;
byte incomingFrame[17] = { 0 };
byte data[8] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
INT32U rxId;
INT8U len = 0;
INT8U rxBuf[8];

void init_can();
void setup();
void loop();
void UpdateAmpReading();
void SendCANFramesToSerial();
void SendCANFrameToSerial(unsigned long canFrameId, const byte* frameData);
void ReadIncomingSerialData();
void ProcessIncomingFrame(const byte* frame);
void HandleIncomingSetValueFrame(unsigned long canFrameId, const byte* frameData);
void SendCanBGS1_Cmd(unsigned int volts, unsigned int amps);
void SendCanVCU_Cmd(unsigned int volts, unsigned int amps);
void SetBMSLimit (int HighLimit);
void readcharger();
float RequestVoltage (int CellNumber);
void gettemperatures();
void callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param);
void setbmslevel();
void keepme();
#line 68 "C:/Users/Jason Wolin/Dropbox/My Hobbies/Electric Car/RealDash_CAN_2way/RealDash_CAN_2way.ino"
void setup()
{
delay(5000);
M5.begin();
Serial.begin(115200);
pinMode(26, INPUT);
timer.setInterval(10000, setbmslevel);
timer.setInterval(5000, keepme);
timer.setInterval(5000, gettemperatures);


  EEPROM.begin(512);
    delay(10);


    EEPROM.put(20, 16);





    EEPROM.commit();
    yield();
    EEPROM.end();

sensors.setWaitForConversion(0);
sensors.begin();
sensors.setResolution(11);
EEPROM.begin(EEPROM_SIZE);
EEPROM.get(10, ButtonOutput[2]);
EEPROM.get(20, ButtonOutput[3]);
EEPROM.get(30, ButtonOutput[4]);
EEPROM.get(40, ButtonOutput[5]);
EEPROM.get(50, ButtonOutput[6]);
EEPROM.get(60, ButtonOutput[7]);
EEPROM.get(70, ButtonOutput[8]);
EEPROM.end();

if(ButtonOutput[8]>150){
ButtonOutput[8]= 150;
}

Serial.println("CAN BUS Shield init");

  canbustime = millis();
  init_can();
Serial.println("Test CAN...");
Serial.println("CAN BUS Shield init ok!");


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

timer.run();


if (ButtonOutput[7]==1){
if (millis() - celltimer >= 1000) {
          celltimer = millis();
cellpacket = 1;
}
}

if (ButtonOutput[2]==1){
if (millis() - chargetimer >= 800) {
  chargetimer = millis();
readcharger();
chargepacket = 1;
}
}


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
  float v1 = setchargervoltshv * 10;
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

  v1 = chargervoltshv * 10;
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

  v1 = chargertemperature;
  v2 = minvoltage;
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

 chargepacket = 0;
}

if(cellpacket == 1){

TotalPackVoltage = 0;
int framepacket = 3202;
    for (int i = 1; i < 37; i = i + 4) {

     float v1 = RequestVoltage(i) * 1000;
     float v2 = RequestVoltage(i+1) * 1000;
     float v3 = RequestVoltage(i+2) * 1000;
     float v4 = RequestVoltage(i+3) * 1000;
     if (v1 > 1 && v1 < 5000){
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

  int voltage = (int) TotalPackVoltage / 100;

minvoltage = 50000 ;
for (int i = 1; i<37; i++)
{
    if (CellVoltages[i] < minvoltage) {
         minvoltage = CellVoltages[i];
         Serial.print("min voltage: "); Serial.println(minvoltage);
    }
}

maxvoltage = CellVoltages[1] ;
for (int i = 1; i<37; i++)
{
    if (CellVoltages[i] > maxvoltage) {
         maxvoltage = CellVoltages[i];
    }
}

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

  M5.Lcd.setCursor(5, 10);
  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.setTextSize(3);

  M5.Lcd.setCursor(50, 10);
  M5.Lcd.print("V: ");
  M5.Lcd.fillRect(100, 10, 200, 30, TFT_BLACK);
  M5.Lcd.print(voltage/10);

  cellpacket = 0;
}

if (temppacket == 1){

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




  SerialBT.write((const byte * ) & serialBlockTag, 4);


  SerialBT.write((const byte * ) & canFrameId, 4);


  SerialBT.write(frameData, 8);



}

void ReadIncomingSerialData()
{



 while (SerialBT.available() > 0)
  {






    incomingFrame[incomingFramePos++] = SerialBT.read();


    if (incomingFrame[0] != 0x44)
    {



      memset(incomingFrame, 0, 17);
      incomingFramePos = 0;
    }

    if (incomingFramePos >= 17)
    {

      ProcessIncomingFrame(incomingFrame);


      memset(incomingFrame, 0, 17);
      incomingFramePos = 0;
    }
  }
}

void ProcessIncomingFrame(const byte* frame)
{



  if (frame[0] != 0x44 ||
      frame[1] != 0x33 ||
      frame[2] != 0x22 ||
      frame[3] != 0x11)
  {

    return;
  }


  unsigned long canFrameId = 0;
  memcpy(&canFrameId, frame + 4, 4);





  byte checkByte = 0;
  for (int i=0; i<16; i++)
  {
    checkByte += frame[i];
  }

  if (frame[16] == checkByte)
  {


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

  for(int bx=0; bx<4; bx++){
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


  for(int bx=4; bx<8; bx++){
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



  stmp[0]=(volts*10)>>8;
  stmp[1]=(volts*10)&0x00ff;
  stmp[2]=(amps*10)>>8;
  stmp[3]=(amps*10)&0x00ff;
  stmp[4]=0x00;
  stmp[5]=0x00;
  stmp[6]=0;
  stmp[7]=0;
# 568 "C:/Users/Jason Wolin/Dropbox/My Hobbies/Electric Car/RealDash_CAN_2way/RealDash_CAN_2way.ino"
  CAN0.sendMsgBuf(BGS1_ID,1, 8, stmp);
}


void SendCanVCU_Cmd(unsigned int volts, unsigned int amps)
{


      stmp[0]=0;
      stmp[1]=1;
      stmp[2]=(volts*10)&0x00ff;
      stmp[3]=(volts*10)>>8;
      stmp[4]=(amps*10)&0x00ff;
      stmp[5]=(amps*10)>>8;
      stmp[6]=0;
      stmp[7]=0;
# 594 "C:/Users/Jason Wolin/Dropbox/My Hobbies/Electric Car/RealDash_CAN_2way/RealDash_CAN_2way.ino"
      CAN0.sendMsgBuf(VCU_ID,1, 8, stmp);
}



void SetBMSLimit (int HighLimit){
char MyString[10];
    Wire.beginTransmission(SLAVE_ADDRESS);
    Wire.write("SV 30000 ");
    itoa(HighLimit,MyString,10);
    Wire.write(MyString);
    Wire.write("\r");
    Wire.endTransmission();
    Serial.print("Sent BMS Data...");
  M5.Lcd.fillRect(50, 150, 350, 175, TFT_BLACK);
  M5.Lcd.setCursor(10, 150);
  M5.Lcd.print("BMS:");
  M5.Lcd.setCursor(100, 150);
  M5.Lcd.print(minvoltage);
  Serial.print("BMS Voltage Set To: ");
  Serial.println(minvoltage);
}


void init_can(){

  if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK)
    Serial.println("MCP2515 Initialized Successfully!");
  else
    Serial.println("Error Initializing MCP2515...");
  CAN0.setMode(MCP_NORMAL);
  pinMode(CAN0_INT, INPUT);

}



void readcharger(){


unsigned char len = 0;

if(!digitalRead(CAN0_INT))
{
    CAN0.readMsgBuf(&rxId, &len, rxBuf);



    rxId = rxId & 0x1fffffff;

    if(rxId == 0x18ff50e5)
    {



        for(int i = 0; i<len; i++)
        {


        }

        chargervoltshv = (rxBuf[0]*256+rxBuf[1])/10.0;
        chargerampshv = (rxBuf[2]*256+rxBuf[3])/10.0;






    }
    rxId = rxId & 0x1fffffff;

    if(rxId == 0x1800e5f5)
    {



        for(int i = 0; i<len; i++)
        {


        }

        chargervoltslv = (rxBuf[5]*.2);
        chargerampslv = (rxBuf[4]*256+rxBuf[3])/10.0;
        chargertemperature = rxBuf[7]-40;
# 689 "C:/Users/Jason Wolin/Dropbox/My Hobbies/Electric Car/RealDash_CAN_2way/RealDash_CAN_2way.ino"
    }
    if (millis() - canbustime >= 1000) {
      canbustime = millis();


      setchargerampslv = ButtonOutput[4];
      setchargerampshv = ButtonOutput[3];
      setchargervoltshv = ButtonOutput[8];
# 711 "C:/Users/Jason Wolin/Dropbox/My Hobbies/Electric Car/RealDash_CAN_2way/RealDash_CAN_2way.ino"
      SendCanBGS1_Cmd(setchargervoltshv,setchargerampshv);

      SendCanVCU_Cmd(14,14);
    }
}


}



float RequestVoltage (int CellNumber){
char MyString[10];
unsigned int c;
float f;
int i;
    Wire.beginTransmission(SLAVE_ADDRESS);
    Wire.write("CV ");
    itoa(CellNumber,MyString,10);
    Wire.write(MyString);
    Wire.write("\r");
    Wire.endTransmission();


    if(CellNumber == 1) delay(400);
    Wire.requestFrom(SLAVE_ADDRESS,8);
    i=0;
    while (Wire.available()) {
      c = Wire.read();
      if(c==255) break;

      MyString[i++] = c;
    }

    MyString[i]=0;
    f=atof(MyString);
    return f;
}

void gettemperatures(){

float temptemp = 0;
    Serial.print("Temperatures are: ");
    for(temploop=0; temploop < 13; temploop++){

      temptemp=sensors.getTempF(AddrList[temploop]);
      if(temptemp < 80){
        TempPack[temploop]=temptemp;
      }
      Serial.print(TempPack[temploop]);
      Serial.print(" ");
    }
    Serial.println();
    sensors.requestTemperatures();


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
  SetBMSLimit(minvoltage*10+50);
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
}