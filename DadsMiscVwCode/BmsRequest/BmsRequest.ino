// code for Jason's ev beetle. 10/4/2019
// #include <LTC681x.h>
#include <Arduino.h>
#include <Wire.h>
#include <stdint.h>
#include "UserInterface.h"

// I2C stuff
#define SLAVE_ADDRESS 0x60
float CellVoltage; // for sending cell voltages to mothership
int x;
int GetVoltages = 0;

/*!**********************************************************************
        Inititializes hardware and variables
 ***********************************************************************/
void setup()
{
  Serial.begin(115200);
  // I2C stuff
  Wire.begin(SLAVE_ADDRESS);
  Serial.println("I2C up and running... waiting for I2C communication");
}
/*!*********************************************************************
         main loop
***********************************************************************/
void loop()
{
  if(GetVoltages == 1){
  for(x=1; x <= 36;x++){
      CellVoltage=RequestVoltage(x);
      Serial.print("cell");
      Serial.print(x);
      Serial.print(" voltage=");
      Serial.println(CellVoltage,3);
      delay(10);
    }
  }
  if(Serial.available()){
    char *CmdPtr;
    CmdPtr = read_string();
    CmdPtr = strtok(CmdPtr, " \r");
    if(strcmp(CmdPtr, "sv") == 0){
  char * Arg1Ptr = strtok(0, " \r");
  char * Arg2Ptr = strtok(0, " \r");
      Serial.print(CmdPtr);
      Serial.print(" ");
      Serial.print(Arg1Ptr);
      Serial.print(" ");
      Serial.println(Arg2Ptr);
      
      Wire.beginTransmission(0x60);
      Wire.write("SV ");
      Wire.write(Arg1Ptr);
      Wire.write(" ");
      Wire.write(Arg2Ptr);
      Wire.write("\r");       // terminate with carriage return delimiter.
      Wire.endTransmission();
    }
    if(strcmp(CmdPtr, "s") == 0){
      GetVoltages=0;
    }
    if(strcmp(CmdPtr, "r") == 0){
      GetVoltages=1;
    }
  }
}

//***********************************************************************
float RequestVoltage (int CellNumber){
char MyString[10];
unsigned int c;
float f;
int i;
    Wire.beginTransmission(0x60);
    Wire.write("CV ");
    itoa(CellNumber,MyString,10); // convert integer number to ascii characters
    Wire.write(MyString);
    Wire.write("\r");       // terminate with carriage return delimiter. This is what I will look for when parsing.
    Wire.endTransmission();
    Serial.print("Requesting Data...");
    
    Wire.requestFrom(SLAVE_ADDRESS,8); // now request some data, up to 8 characters
    i=0;
    while (Wire.available()) { // counts down the REQUESTED character count
      c = Wire.read();         // get character
      if(c==255) break;        // still needed, break if default char is detected
      MyString[i++] = c;       // build character string
     // Serial.println(c, DEC) // print the character
    }  
    
    MyString[i]=0;      // add the NULL character for string conversion to float
    f=atof(MyString);   // convert string to float
    return f;           // return float value
}
