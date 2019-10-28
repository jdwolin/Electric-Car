#include <OneWire.h>
#include <DallasTemperature.h>
 
// Data wire is plugged into pin 2 on the Arduino
#define ONE_WIRE_BUS 2

int temploop;
int TemperatureTimer=0;

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

// Setup a oneWire instance to communicate with any OneWire devices 
// (not just Maxim/Dallas temperature ICs) 18B20
OneWire oneWire(ONE_WIRE_BUS);
 
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);
 
void setup(void)
{
  // start serial port
  Serial.begin(115200);
  OCR0A = 0xAF;             // setup for user timer interrupt
  TIMSK0 |= _BV(OCIE0A);    // setup for user timer interrupt
  Serial.println("Dallas Temperature IC Control Library Demo");
  sensors.setWaitForConversion(0); // don't wait for temperature conversion to complete

  // Start up the library
  
  sensors.begin();
  sensors.setResolution(11); // <-****************************************
}  // end of setup

// probe conversion times
//  9 = 93.75ms, 0.5C res
// 10 = 187.5ms  0.25C
// 11 = 375ms    0.125C
// 12 = 750ms    0.0625C res

//User timer interrupt routine
SIGNAL(TIMER0_COMPA_vect){ // 1 ms interrupt routine launched by timer0 ISR
  TemperatureTimer++;      // increment conversion timer
}
 
void loop(void)
{
  if(TemperatureTimer >= 400){ // start another cycle if >400 milliseconds have gone by...
    Serial.print("Temperatures are: ");
    for(temploop=0; temploop < 12; temploop++){
 //   Serial.print(sensors.getTempCByIndex(11),1); // 1 decimal place of precision
      Serial.print(sensors.getTempC(AddrList[temploop]),1); // print with 1 decimal place of precision
      Serial.print(" ");
    }
    Serial.println(); // cr/lf

    Serial.print(" Requesting temperatures...");
    sensors.requestTemperatures(); // Send global command to convert temperatures, no wait
    TemperatureTimer = 0; // reset conversion timer
    // You can have more than one IC on the same bus. 
    // 0 refers to the first IC on the wire
  }
}
