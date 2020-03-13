#include <Wire.h>

// Change this unique address for each I2C slave node
#define NODE_ADDRESS 1

// matches values on master side.
#define TO_MASTER_SIZE 3
#define TO_SLAVE_SIZE 4
#define NODE_READ_DELAY 100

byte messageToMaster[TO_MASTER_SIZE];
byte nodeReceive[TO_SLAVE_SIZE];

void setup() {
  Serial.begin(115200);  
  Serial.print("SLAVE #");
  Serial.println(NODE_ADDRESS);

  Wire.begin(NODE_ADDRESS);  // Activate I2C network
}

void loop() { 
  delay(NODE_READ_DELAY);

  if(Wire.available() == TO_SLAVE_SIZE) {
    readFromMaster();
    sendToMaster();
  }
}

void readFromMaster() {
  for(int i = 0; i < TO_SLAVE_SIZE; i ++){
    nodeReceive[i] = Wire.read();
  }
  Serial.print("Master says ");
  for(int i = 0; i < TO_SLAVE_SIZE; i ++){
    Serial.print(nodeReceive[i]);  
  }
  Serial.println();
}

void sendToMaster() {
  int x0 = analogRead(A0);
  messageToMaster[0] = NODE_ADDRESS;
  messageToMaster[1] = (x0>>8) & 0xff;  // the top byte of x
  messageToMaster[2] = (x0   ) & 0xff;  // the bottom byte of x
  Wire.write(messageToMaster,TO_MASTER_SIZE);  

  Serial.print("Sensor value: ");
  Serial.println(x0);
}
