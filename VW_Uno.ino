//arduino-cli compile --fqbn esp32:esp32:esp32 "C:\Users\Jason Wolin\Dropbox\Arduino\Electric_VW_Mothership" 
//arduino-cli upload -p COM5 --fqbn esp32:esp32:esp32  "C:\Users\Jason Wolin\Dropbox\Arduino\Electric_VW_Mothership" 
//"C:\Users\Jason Wolin\esptool\esptool.py" --port COM5 write_flash 0x10000 "C:/Users/Jason Wolin/Dropbox/Arduino/Electric_VW_Mothership/Electric_VW_Mothership.esp32.esp32.esp32.bin"

#define ONE_WIRE_BUS 4

#include <Wire.h>

# define SLAVE_ADDRESS 0x60

//#include <SoftwareSerial.h>
//SoftwareSerial BTserial(2, 3); // RX | TX

// the 4 byte identifier at the beginning of each CAN frame
// this is required for RealDash to 'catch-up' on ongoing stream of CAN frames
const unsigned long serialBlockTag = 0x11223344;

// Arduino digital and analog pins
unsigned int rpm = 0;
unsigned int cell01volts = 0;
unsigned int cell02volts = 0;
unsigned int kpa = 992; // 99.2
unsigned int tps = 965; // 96.5
unsigned int clt = 80; // 80 - 100
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

char buffer[40];
boolean receiveFlag = false;

char instring[] = "IC 1, C1:4.8654, C2:2.0000, C3:4.0001, C4:1.4433, C5:1.8774, C6:2.9454, C7:3.3434, C8:3.3434, C9:3.3434, C10:3.3343, C11:3.2324, C12:3.3434, C13:3.3434, C14:3.2322, C15:3.3443, C16:3.3434, C17:3.3434, C18:3.3493";
//char instring1[] = "IC 2, C1:2.8654, C2:2.0000, C3:2.0001, C4:2.4433, C5:2.8774, C6:2.9454, C7:2.3434, C8:2.3434, C9:2.3434, C10:2.3343, C11:2.2324, C12:2.3434, C13:2.3434, C14:2.2322, C15:2.3443, C16:2.3434, C17:2.3434, C18:2.3493";
//char tempstring[] = "Temp, T1:30.30, T2:45.00, T3:33.00, T4:23.00, T5:88.00, T6:22.00, T7:10.00, T8: -2.00, T9: 101.00, T10: 23.22, T11: 23.88, T12:99, T14:23";

char delimiters[] = ",:";
char * valPosition;
float icbank1[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
float icbank2[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
float Celcius = 0;
float Fahrenheit = 0;

byte x = 0;
const char * testbank1 = "1";
const char * testbank2 = "2";

String strTest = "123";

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // BTserial.begin(9600);
  Serial.println("The device started, now you can pair it with bluetooth!");
  //This initializes strtok with our string to tokenize


  
}
  void loop() {
    //SendCANFramesToSerial();

      for(int x = 0; x >36; x++){



    }


  
  }
    delay(500);

  delay(5);
}

void RequestVoltage (int cellnumber){



    Wire.beginTransmission(0x60);
    Wire.write("123");
    Wire.endTransmission();
    
    delay(5);
    Serial.println("Requesting Data");
    Wire.requestFrom(SLAVE_ADDRESS, 250);
    while (Wire.available()) { // slave may send less than requested
      char c = Wire.read(); // receive a byte as character
    if(c==255) break;
    Serial.print(c);         // print the character

}




void SendCANFramesToSerial() {
  byte buf[8];

  // build & send CAN frames to RealDash.
  // a CAN frame payload is always 8 bytes containing data in a manner
  // described by the RealDash custom channel description XML file
  // all multibyte values are handled as little endian by default.
  // endianess of the values can be specified in XML file if it is required to use big endian values
  // build 1st CAN frame, RPM, MAP, CLT, TPS (just example data)
  memcpy(buf, & rpm, 2);
  memcpy(buf + 2, & kpa, 2);
  memcpy(buf + 4, & clt, 2);
  memcpy(buf + 6, & tps, 2);

  // write first CAN frame to serial
  SendCANFrameToSerial(3200, buf);

  memcpy(buf, & cell01volts, 2);
  memcpy(buf + 4, & cell02volts, 2);
  memcpy(buf + 4, & cell01volts, 2);
  memcpy(buf + 6, & cell01volts, 2);

  SendCANFrameToSerial(3201, buf);

  int framepacket = 3202;

  for (int i = 0; i < 20; i = i + 4) {
    float v1 = icbank1[i] * 1000;
    float v2 = icbank1[i + 1] * 1000;
    float v3 = icbank1[i + 2] * 1000;
    float v4 = icbank1[i + 3] * 1000;
    int voltage1 = (int) v1;
    int voltage2 = (int) v2;
    int voltage3 = (int) v3;
    int voltage4 = (int) v4;
    //Serial.println("voltage");
    //Serial.println(voltage1);
    //Serial.println(voltage2);
    //Serial.println(voltage3);
    //Serial.println(voltage4);
    //delay(5000);
    memcpy(buf, & voltage1, 2);
    memcpy(buf + 2, & voltage2, 2);
    memcpy(buf + 4, & voltage3, 2);
    memcpy(buf + 6, & voltage4, 2);

    SendCANFrameToSerial(framepacket, buf);
    framepacket++;

  }

}

void SendCANFrameToSerial(unsigned long canFrameId,
  const byte * frameData) {
  //Serial.println(digitalPins);
  // the 4 byte identifier at the beginning of each CAN frame
  // this is required for RealDash to 'catch-up' on ongoing stream of CAN frames
  Serial.write((const byte * ) & serialBlockTag, 4);

  // the CAN frame id number
  Serial.write((const byte * ) & canFrameId, 4);

  // CAN frame payload
  Serial.write(frameData, 8);
}