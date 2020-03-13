#include <Wire.h>

#define SLAVE_ADDRESS 0x60
byte x = 0x00;
byte JasonMsg[] = "Hi Jason";
void setup()
{
  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  Serial.begin(115200);
  Serial.println("up and running... waiting for I2C communication");
}

void loop()
{
  delay(100);
}

void requestEvent() 
{
  Serial.print("Request from Master. Sending: ");
  Serial.print(x, HEX);
  Serial.print("\n");

  Wire.write(x);
}

void receiveEvent(int bytes)
{
  if(Wire.available() != 0)
  {
    for(int i = 0; i< bytes; i++)
    {
      x = Wire.read();
      Serial.print("Received: ");
      Serial.print(x, HEX);
      Serial.print("\n");
    }
  }
}
