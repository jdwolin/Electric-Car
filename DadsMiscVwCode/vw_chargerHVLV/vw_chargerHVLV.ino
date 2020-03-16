// CAN-BUS Shield
// High Voltage and low voltage driver for Jason's EV battery charger.


#include <SPI.h>
#include "mcp_can.h"

const int SPI_CS_PIN = 10;
#define BGS1_ID 0x1806E5F4 // BGS1 message code BGS1
#define VCU_ID  0x1800F5E5 // VCU message code

MCP_CAN CAN(SPI_CS_PIN);                                    // Set CS pin

unsigned char stmp[8];
unsigned int LoopCount=0;
float volts, amps;
unsigned int DwTimer=0, BGS1Timer=0, VcuTimer=0;


void setup()
{
    Serial.begin(115200);
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);

    while (CAN_OK != CAN.begin(CAN_500KBPS))              // init can bus : baudrate = 500k
    {
        Serial.println("CAN BUS Shield init fail");
        Serial.println(" Init CAN BUS Shield again");
        delay(100);
    }
    Serial.println("CAN BUS Shield init ok!");
}

SIGNAL(TIMER0_COMPA_vect) 
{
  BGS1Timer++; // how often to send BGS command to charger
  VcuTimer++;  // how often to send VCU command to charger
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
      
      CAN.sendMsgBuf(BGS1_ID,1, 8, stmp);
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
      
      CAN.sendMsgBuf(VCU_ID,1, 8, stmp);
}

void loop()
{
unsigned char len = 0;
unsigned char buf[8];
unsigned char CanStat;
    if(CAN_MSGAVAIL == CAN.checkReceive()){            // check if data coming 
        CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf 
        unsigned long canId = CAN.getCanId();
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
        if(BGS1Timer > 1000){
          BGS1Timer = 0;
          SendCanBGS1_Cmd(140,3); // 140 volts, 3 amps
          Serial.println("sending BGS1 command");
        }
        if(VcuTimer > 500){
          VcuTimer = 0;
          Serial.println("sending VCU command");
          SendCanVCU_Cmd(14,5);  // dc battery 14 volts, 5 amps
        }
    }
}

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
