// Jason's CrashCode

#include <stdio.h> 
#include <string.h>

char instring[] = "IC 1, C1:4.8654, C2:2.0000, C3:4.0001, C4:1.4433, C5:1.8774, C6:2.9454, C7:3.3434, C8:3.3434, C9:3.3434, C10:3.3343, C11:3.2324, C12:3.3434, C13:3.3434, C14:3.2322, C15:3.3443, C16:3.3434, C17:3.3434, C18:3.3493";
char instring1[] = "IC 2, C1:2.8654, C2:2.0000, C3:2.0001, C4:2.4433, C5:2.8774, C6:2.9454, C7:2.3434, C8:2.3434, C9:2.3434, C10:2.3343, C11:2.2324, C12:2.3434, C13:2.3434, C14:2.2322, C15:2.3443, C16:2.3434, C17:2.3434, C18:2.3493";

char delimiters[] = ":,";
char* valPosition;
float icbank1[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};  
float icbank2[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; 

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("The device started, now you can pair it with bluetooth!");

//This initializes strtok with our string to tokenize
valPosition = strtok(instring, delimiters);    // find first comma, throw this part away
valPosition = strtok(NULL, delimiters);        // find first colon, throw this away

//break entire packet into an array
  for(int i = 0; i < 18; i++){
      valPosition = strtok(NULL, delimiters);  // get first numeric string
      icbank1[i] = atof(valPosition);          // convert string to float
      Serial.println(icbank1[i],4);            // print float value
      valPosition = strtok(NULL, delimiters);  // end of comma to next colon. Throw this away    
  }
}


void loop() {
  // put your main code here, to run repeatedly:
     delay(1000);
     Serial.println("in the main loop");
}
