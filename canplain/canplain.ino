#include <M5Stack.h>
#include <mcp_can.h>


/**
 * variable for loop
 */

byte data[8] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};


/**
 * variable for CAN
 */
INT32U rxId;
INT8U extflag = 1;
INT8U len = 0;
INT8U rxBuf[8];
char msgString[128];                        // Array to store serial string

#define CAN0_INT 2                              // Set INT to pin 2
MCP_CAN CAN0(10); 


void init_can();
void test_can();

void setup() {
  M5.begin();
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, 16, 17);

  delay(500);
  M5.Lcd.setTextColor(BLACK);
  // M5.Lcd.setTextSize(1);

  init_can();
  Serial.println("Test CAN...");
}

void loop() {
  if(M5.BtnA.wasPressed())
  {
    M5.Lcd.clear();
    M5.Lcd.printf("CAN Test A!\n");

    init_can();
    Serial.println("Test CAN...");
  }
  test_can();
  M5.update();
}

void init_can(){


  M5.Lcd.printf("CAN Test A!\n");
  M5.Lcd.printf("Receive first, then testing for sending function!\n");

  // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
  if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) == CAN_OK)
    Serial.println("MCP2515 Initialized Successfully!");
  else
    Serial.println("Error Initializing MCP2515...");

  CAN0.setMode(MCP_NORMAL);                     // Set operation mode to normal so the MCP2515 sends acks to received data.

  pinMode(CAN0_INT, INPUT);                            // Configuring pin for /INT input

  Serial.println("MCP2515 Library Receive Example...");
}

void test_can(){
 // unsigned char buf[8];

           //     Serial.print("made it here");
  if(!digitalRead(CAN0_INT))                         // If CAN0_INT pin is low, read receive buffer
  {
           //   Serial.print("made it here");
    CAN0.readMsgBuf(&rxId, &len, rxBuf);      // Read data: len = data length, buf = data byte(s)

//rxId = 0x1800e5f5;

        Serial.println(rxId, HEX);
        Serial.println(len);
    //    Serial.println(rxBuf);
           if(rxId == 0x1800e5f5)//0x1800e5f5
            {

                Serial.println("----------------------------------");
                 Serial.print("Get data from DCDC ID: 0x");
                 Serial.println(rxId,HEX);
                for(int i = 0; i<len; i++)    // print the data
                {
                    Serial.print(rxBuf[i], HEX);
                    Serial.print("\t");
                }
            Serial.println();
            
  }
}

}