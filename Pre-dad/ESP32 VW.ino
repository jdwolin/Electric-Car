//arduino-cli compile --fqbn esp32:esp32:esp32 "C:\Users\Jason Wolin\Dropbox\Arduino\Electric_VW_Mothership" 
//arduino-cli upload -p COM5 --fqbn esp32:esp32:esp32  "C:\Users\Jason Wolin\Dropbox\Arduino\Electric_VW_Mothership" 
//"C:\Users\Jason Wolin\esptool\esptool.py" --port COM5 write_flash 0x10000 "C:/Users/Jason Wolin/Dropbox/Arduino/Electric_VW_Mothership/Electric_VW_Mothership.esp32.esp32.esp32.bin"
//#include <WebOTA.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <M5Stack.h>
#include <FastLED.h>
#include "BluetoothSerial.h"

// CANBUS INITIALIZATION START

#include "mcp_can.h"
#define CAN0_INT 15                              // Set INT to pin 2
MCP_CAN CAN0(12);     // Set CS to pin 10
#define BGS1_ID 0x1806E5F4 // BGS1 message code BGS1
#define VCU_ID  0x1800F5E5 // VCU message code
unsigned char stmp[8];
unsigned int LoopCount=0;
float volts, amps;
unsigned int DwTimer=0, BGS1Timer=0, VcuTimer=0;

// CANBUS INITIALZITION END


#define ONE_WIRE_BUS 4
#define Neopixel_PIN    15
#define NUM_LEDS    10
const unsigned long serialBlockTag = 0x11223344;  // canbus definition for realdash
unsigned long startTime1;  //timer



BluetoothSerial SerialBT;

// Arduino digital and analog pins

unsigned int rpm = 0;
unsigned int cell01volts = 0;
unsigned int cell02volts = 0;
unsigned int kpa = 992; // 99.2
unsigned int tps = 965; // 96.5
unsigned int clt = 80;  // 80 - 100
unsigned int textCounter = 0;
unsigned int ambienthood = 0;
unsigned int tempmotor = 0;
unsigned int keepalive = 0;
unsigned int keepcounter = 0;
int screenfull = 0;

char instring[] = "IC 1, C1:4.8654, C2:2.0000, C3:4.0001, C4:1.4433, C5:1.8774, C6:2.9454, C7:3.3434, C8:3.3434, C9:3.3434, C10:3.3343, C11:3.2324, C12:3.3434, C13:3.3434, C14:3.2322, C15:3.3443, C16:3.3434, C17:3.3434, C18:3.3493, C19:45";
char instring1[] = "IC 2, C1:2.8654, C2:2.0000, C3:2.0001, C4:2.4433, C5:2.8774, C6:2.9454, C7:2.3434, C8:2.3434, C9:2.3434, C10:2.3343, C11:2.2324, C12:2.3434, C13:2.3434, C14:2.2322, C15:2.3443, C16:2.3434, C17:2.3434, C18:2.3493, C19:33";
char tempstring[] = "Temp, T1:30.30, T2:45.00, T3:33.00, T4:23.00, T5:88.00, T6:22.00, T7:10.00, T8: -2.00, T9: 101.00, T10: 23.22, T11: 23.88, T12:99, T14:23"; 
char input;
char delimiters[] = ",:";
char* valPosition;
float icbank1[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};  
float icbank2[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; 
float tempbank[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

const char *testbank1 = "1";  // BMS Board One String
const char *testbank2 = "2";  // BMS Board Two String
const char *testbank3 = "p";  // Temperature String

char name_arr[32];

CRGB leds[NUM_LEDS];
uint8_t gHue = 0;
static TaskHandle_t FastLEDshowTaskHandle = 0;
static TaskHandle_t userTaskHandle = 0;
 

void setup()
{

  Serial.begin(115200); 
  Serial2.begin(9600);
//  neopixelsetup;

//canbus start setup
   startTime1 = millis();
   while (CAN_OK != CAN0.begin(CAN_500KBPS))              // init can bus : baudrate = 500k
    {
        Serial.println("CAN BUS Shield init fail");
        Serial.println(" Init CAN BUS Shield again");
        delay(100);
    }
    Serial.println("CAN BUS Shield init ok!");

//canbus end setup


  FastLED.addLeds<WS2811,Neopixel_PIN,GRB>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(10);
  xTaskCreatePinnedToCore(FastLEDshowTask, "FastLEDshowTask", 2048, NULL, 2, NULL, 1);

  M5.begin();
  M5.Lcd.setTextColor(TFT_WHITE,TFT_BLACK);  
  M5.Lcd.setTextSize(1);


 // M5.Lcd.println(("VW Electric Initialization..."));


  SerialBT.begin("ESP32test"); //Bluetooth device name
  if(!SerialBT.begin("ESP32")){
    Serial.println("An error occurred initializing Bluetooth");
    M5.Lcd.println(("An error occurred initializing Bluetooth"));

  }
//  SerialBT.begin(9600);
  Serial.println("The device started, now you can pair it with bluetooth!");
  M5.Lcd.println(("Pair with bluetooth!"));



if (strchr(testbank1, instring[3])){
Serial.println("It's the first IC!");  
  valPosition = strtok(instring, delimiters);
for(int i = 0; i < 37; i++){
    icbank1[i] = atof(valPosition);
   // delay(100);
    valPosition = strtok(NULL, delimiters);
    Serial.println(i);
    Serial.println(valPosition);
}

// simplify array
int counter = 0;
for(int i=2; i < 37; i=i+2){
icbank1[counter] = icbank1[i];
Serial.println(icbank1[counter],4);
counter++;
}
}


if (strchr(testbank2, instring1[3])){
Serial.println("It's the second IC!");  
valPosition = strtok(instring1, delimiters);
for(int i = 0; i < 37; i++){
    icbank2[i] = atof(valPosition);
   // delay(100);
    valPosition = strtok(NULL, delimiters);
    Serial.println(i);
    Serial.println(valPosition);
}

// simplify array
int counter = 0;
for(int i=2; i < 37; i=i+2){
icbank2[counter] = icbank2[i];
Serial.println(icbank2[counter],4);
counter++;
}
}


//temperature probes
if (strchr(testbank3, tempstring[3])){
Serial.println("It's the temp probes");  
valPosition = strtok(tempstring, delimiters);
for(int i = 0; i < 26; i++){
    tempbank[i] = atof(valPosition);
    valPosition = strtok(NULL, delimiters);
    Serial.println(i);
    Serial.println(valPosition);
}

// simplify array
int counter = 0;
for(int i=2; i < 26; i=i+2){
tempbank[counter] = tempbank[i];
Serial.println(tempbank[counter],4);
counter++;
}
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

//      Serial.print("stmp0=0x");
//      Serial.println(stmp[0],HEX);
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


void loop()
{
//    webota.handle();
  SendCANFramesToSerial();

  if (screenfull == 30)
  {
  //M5.Lcd.fillRect(0,0,320,240, TFT_BLACK);
  M5.Lcd.setCursor(0,10);
  screenfull = 0;
  }
  
    // just some dummy values for simulated engine parameters
  if (rpm++ > 6750)
  {
    rpm = 0;
  }
  if (cell01volts++ > 4750)
  {
    cell01volts = 2000;
  }
   if (cell02volts++ > 3000)
  {
    cell02volts = 2000;
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
  if (keepalive++ > 64000)
{
  M5.Lcd.setCursor(0,30);
  keepalive = 0;
  keepcounter ++; 
  M5.Lcd.print(("Alive for: "));
  M5.Lcd.print((keepcounter));
  M5.Lcd.println((" Cycles          "));
}

  while (Serial2.available()) {
    size_t num_read = Serial2.readBytesUntil('\r', name_arr, sizeof(name_arr)-1 );
    name_arr[num_read] = '\0';
    Serial.println(name_arr);
    Serial.println();
    M5.Lcd.print(("RS232: "));
    M5.Lcd.println((name_arr));
  }

//canbus loop code start

unsigned char len = 0;
unsigned char buf[8];
unsigned char CanStat;
    if(CAN_MSGAVAIL == CAN0.checkReceive()){            // check if data coming 
        CAN0.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf 
        unsigned long canId = CAN0.getCanId();
   //     Serial.print("Canid=");
   //     Serial.println(canId,HEX);
        if(canId == 0x18ff50e5)
        {
            Serial.println("***********************************");
             Serial.print("Get data from CCS1 ID: 0x");
             Serial.println(canId,HEX);
            for(int i = 0; i<len; i++)    // print the data
            {
                Serial.print(buf[i], HEX);
                Serial.print("\t");
            }
            Serial.println();
            volts = (buf[0]*256+buf[1])/10.0;
            amps = (buf[2]*256+buf[3])/10.0;
            Serial.print("volts=");
            Serial.print(volts,1);Serial.print(" ");
            Serial.print("amps=");
            Serial.print(amps,1);Serial.print(" ");
            Serial.print("status=");
            Serial.println(buf[4]);Serial.println();
        }
        if(canId == 0x1800e5f5)//0x1800e5f5
        {
            Serial.println("----------------------------------");
             Serial.print("Get data from DCDC ID: 0x");
             Serial.println(canId,HEX);
            for(int i = 0; i<len; i++)    // print the data
            {
                Serial.print(buf[i], HEX);
                Serial.print("\t");
            }
            Serial.println();
            volts = (buf[5]*.2);
            amps = (buf[4]*256+buf[3])/10.0;
            Serial.print("volts=");
            Serial.print(volts,1);Serial.print(" ");
            Serial.print("amps=");
            Serial.print(amps,1);Serial.print(" ");
            Serial.print("temperature=");
            Serial.print(buf[7]-40);Serial.println("C");
        }
        if (millis() - startTime1 >= 1000) {
          startTime1 = millis();
          Serial.println("sending BGS1 command");
          SendCanBGS1_Cmd(140,3); // 140 volts, 3 amps
          Serial.println("sending VCU command");
          SendCanVCU_Cmd(14,5);  // dc battery 14 volts, 5 amps
        }
    }
//canbus loop code end




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

  for (int i = 0; i < 16; i = i + 4) {
    float v1 = icbank1[i] * 1000;
    float v2 = icbank1[i + 1] * 1000;
    float v3 = icbank1[i + 2] * 1000;
    float v4 = icbank1[i + 3] * 1000;
    int voltage1 = (int) v1;
    int voltage2 = (int) v2;
    int voltage3 = (int) v3;
    int voltage4 = (int) v4;
    memcpy(buf, & voltage1, 2);
    memcpy(buf + 2, & voltage2, 2);
    memcpy(buf + 4, & voltage3, 2);
    memcpy(buf + 6, & voltage4, 2);

    SendCANFrameToSerial(framepacket, buf);
    framepacket++;

  }


  framepacket = 3206;
    float v1 = icbank1[16] * 1000;
    float v2 = icbank1[17] * 1000;
    float v3 = icbank2[0] * 1000;
    float v4 = icbank2[1] * 1000;
    int voltage1 = (int) v1;
    int voltage2 = (int) v2;
    int voltage3 = (int) v3;
    int voltage4 = (int) v4;
    
    memcpy(buf, & voltage1, 2);
    memcpy(buf + 2, & voltage2, 2);
    memcpy(buf + 4, & voltage3, 2);
    memcpy(buf + 6, & voltage4, 2);
   SendCANFrameToSerial(framepacket, buf);
    

framepacket = 3207;
 for (int i = 2; i < 18; i = i + 4) {
    float v1 = icbank2[i] * 1000;
    float v2 = icbank2[i + 1] * 1000;
    float v3 = icbank2[i + 2] * 1000;
    float v4 = icbank2[i + 3] * 1000;
    int voltage1 = (int) v1;
    int voltage2 = (int) v2;
    int voltage3 = (int) v3;
    int voltage4 = (int) v4;
    memcpy(buf, & voltage1, 2);
    memcpy(buf + 2, & voltage2, 2);
    memcpy(buf + 4, & voltage3, 2);
    memcpy(buf + 6, & voltage4, 2);

    SendCANFrameToSerial(framepacket, buf);
    framepacket++;

  }



framepacket = 3211;
 for (int i = 0; i < 14; i = i + 4) {
    float v1 = tempbank[i] * 1000;
    float v2 = tempbank[i + 1] * 1000;
    float v3 = tempbank[i + 2] * 1000;
    float v4 = tempbank[i + 3] * 1000;
    int voltage1 = (int) v1;
    int voltage2 = (int) v2;
    int voltage3 = (int) v3;
    int voltage4 = (int) v4;
    memcpy(buf, & voltage1, 2);
    memcpy(buf + 2, & voltage2, 2);
    memcpy(buf + 4, & voltage3, 2);
    memcpy(buf + 6, & voltage4, 2);

    SendCANFrameToSerial(framepacket, buf);
    framepacket++;

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
}

void FastLEDshowESP32()
{
    if (userTaskHandle == 0) {
        userTaskHandle = xTaskGetCurrentTaskHandle();
        xTaskNotifyGive(FastLEDshowTaskHandle);
        const TickType_t xMaxBlockTime = pdMS_TO_TICKS( 200 );
        ulTaskNotifyTake(pdTRUE, xMaxBlockTime);
        userTaskHandle = 0;
    }
}

void FastLEDshowTask(void *pvParameters)
{
    for(;;) {
        fill_solid(leds, NUM_LEDS, gHue);// rainbow effect
        FastLED.show();// must be executed for neopixel becoming effective
        EVERY_N_MILLISECONDS( 200 ) { gHue++; }
    }
}