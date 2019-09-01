//arduino-cli compile --fqbn esp32:esp32:esp32 "C:\Users\Jason Wolin\Dropbox\Arduino\Electric_VW_Mothership" 
//arduino-cli upload -p COM5 --fqbn esp32:esp32:esp32  "C:\Users\Jason Wolin\Dropbox\Arduino\Electric_VW_Mothership" 
//"C:\Users\Jason Wolin\esptool\esptool.py" --port COM5 write_flash 0x10000 "C:/Users/Jason Wolin/Dropbox/Arduino/Electric_VW_Mothership/Electric_VW_Mothership.esp32.esp32.esp32.bin"
#include <OneWire.h>
#include <DallasTemperature.h>
#include "BluetoothSerial.h"
#define ONE_WIRE_BUS 4

// the 4 byte identifier at the beginning of each CAN frame
// this is required for RealDash to 'catch-up' on ongoing stream of CAN frames
const unsigned long serialBlockTag = 0x11223344;

// Arduino digital and analog pins
unsigned int rpm = 0;
unsigned int cell01volts = 0;
unsigned int cell02volts = 0;
unsigned int kpa = 992; // 99.2
unsigned int tps = 965; // 96.5
unsigned int clt = 80;  // 80 - 100
unsigned int textCounter = 0;
unsigned int tempbattery1 = 0;
unsigned int tempbattery2 = 0;
unsigned int tempbattery3 = 0;
unsigned int tempbattery4 = 0;
unsigned int tempbattery5 = 0;
unsigned int tempbattery6 = 0;
unsigned int tempbattery7 = 0;
unsigned int tempbattery8 = 0;

unsigned int ambienthood = 0;
unsigned int tempmotor = 0;
unsigned int tempbms1 = 0;
unsigned int tempbms2 = 0;

char instring[] = "IC 1, C1:3.7654, C2:3.4555, C3:3.5887, C4:3.4433, C5:3.8774, C6:3.9454, C7:3.3434, C8:3.3434, C9:3.3434, C10:3.3343, C11:3.2324, C12:3.3434, C13:3.3434, C14:3.2322, C15:3.3443, C16:3.3434, C17:3.3434, C18:3.3493";
char delimiters[] = ",:";
char* valPosition;
float icbank1[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};  
float icbank2[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; 
float Celcius=0;
float Fahrenheit=0;

const char *testbank1 = "1";
const char *testbank2 = "2";

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

BluetoothSerial BTserial;
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif



void setup()
{
  Serial.begin(115200);
  BTserial.begin("ESP32test"); //Bluetooth device name
  sensors.begin(); 
  Serial.println("The device started, now you can pair it with bluetooth!");

/*

//This initializes strtok with our string to tokenize
valPosition = strtok(instring, delimiters);

//analyze first byte of packet and see if it IC1 
if (strchr(testbank1, instring[3])){
Serial.println("It's the first IC!");  

//break entire packet into an array
for(int i = 0; i < 38; i++){
    icbank1[i] = atof(valPosition);
    valPosition = strtok(NULL, delimiters);
}
// simplify array
int counter = 0;
for(int i=2; i < 38; i=i+2){
icbank1[counter] = icbank1[i];
Serial.println(icbank1[i],4);
}
}

//analyze first byte of packet and see if it IC2 
if (strchr(testbank2, instring[3])){
Serial.println("It's the second IC!");  

//break entire packet into an array
for(int i = 0; i < 38; i++){
    icbank2[i] = atof(valPosition);
    valPosition = strtok(NULL, delimiters);
}
// simplify array
int counter = 0;
for(int i=2; i < 38; i=i+2){
icbank2[counter] = icbank2[i];
delay(500);
Serial.println(icbank2[i],4);
}
}

*/

}


void loop()
{
  SendCANFramesToSerial();
  
    // just some dummy values for simulated engine parameters
  if (rpm++ > 6750)
  {
    rpm = 0;
  }
  if (cell01volts++ > 4750)
  {
    cell01volts = 0;
  }
   if (cell02volts++ > 3000)
  {
    cell02volts = 0;
  }
  if (kpa++ > 2500)
  {
    kpa = 10;
  }
  if (tps++ > 1000)
  {
    tps = 0;
  }
  if (clt++ > 230)
  {
    // all values in frame are handled as unsigned values. To have negative values,
    // offset actual value and write corresponding conversion to xml file imported to RealDash
    clt = 0;
  }
  if (textCounter++ > 4000)
  {
  
  sensors.requestTemperatures(); 
  Celcius=sensors.getTempCByIndex(0);
  Fahrenheit=sensors.toFahrenheit(Celcius);
  Fahrenheit=Fahrenheit*100;
  tempbattery1 = (int) Fahrenheit;
  Serial.println(tempbattery1);
  Serial.print(" C  ");
  Serial.print(Celcius);
  Serial.print(" F  ");
  Serial.println(Fahrenheit);
  textCounter = 0;
  
  }

  
  delay(5);
}



void SendCANFramesToSerial()
{
  byte buf[8];

  // build & send CAN frames to RealDash.
  // a CAN frame payload is always 8 bytes containing data in a manner
  // described by the RealDash custom channel description XML file
  // all multibyte values are handled as little endian by default.
  // endianess of the values can be specified in XML file if it is required to use big endian values
  // build 1st CAN frame, RPM, MAP, CLT, TPS (just example data)
  memcpy(buf, &rpm, 2);
  memcpy(buf + 2, &kpa, 2);
  memcpy(buf + 4, &clt, 2);
  memcpy(buf + 6, &tps, 2);

  // write first CAN frame to serial
  SendCANFrameToSerial(3200, buf);
  
  memcpy(buf, &cell01volts, 2);
  memcpy(buf + 4, &cell02volts, 2);
  memcpy(buf + 4, &cell01volts, 2);
  memcpy(buf + 6, &cell01volts, 2);

  SendCANFrameToSerial(3201, buf);



  int framepacket = 3202;

  for(int i = 0; i < 12; i=i+5){
  int voltage1 = (int) icbank1[i];
  int voltage2 = (int) icbank1[i+1];
  int voltage3 = (int) icbank1[i+2];
  int voltage4 = (int) icbank1[i+3];
  memcpy(buf, &voltage1, 2);
  memcpy(buf + 2, &voltage2, 2);
  memcpy(buf + 4, &voltage3, 2);
  memcpy(buf + 6, &voltage4, 2);

  SendCANFrameToSerial(framepacket, buf);
  framepacket++;
    
}



}



void SendCANFrameToSerial(unsigned long canFrameId, const byte* frameData)
{
  //Serial.println(digitalPins);
  // the 4 byte identifier at the beginning of each CAN frame
  // this is required for RealDash to 'catch-up' on ongoing stream of CAN frames
  BTserial.write((const byte*)&serialBlockTag, 4);

  // the CAN frame id number
  BTserial.write((const byte*)&canFrameId, 4);

  // CAN frame payload
  BTserial.write(frameData, 8);
}
