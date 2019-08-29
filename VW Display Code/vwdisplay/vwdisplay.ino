/***************************************************
//Web: http://www.buydisplay.com
EastRising Technology Co.,LTD
Examples for ER-TFTM070-4V2.1(800x480 Pixels) with Resistive Touch Panel
Software SPI,8080 16-bit Interface 5V Power Supply

This program is a quick demo of how create and use buttons.
This program requires the UTFT library.
It is assumed that the display module is connected to an
appropriate shield or that you know how to change the pin 
numbers in the setup.


Tested and worked with:
Arduino Mega 2560,Arduino Due
Works with Arduino 1.6.0 IDE 
****************************************************/

#include <UTFT.h>
#include <UTouch.h>

// Set the pins to the correct ones for your development shield
// ------------------------------------------------------------
// Standard Arduino Mega/Due shield            : <display model>,38,39,40,41

UTFT myGLCD(SSD1963_800480,38,39,40,41);  //(byte model, int RS, int WR, int CS, int RST, int SER)

UTouch  myTouch( 43, 42, 44, 45, 46);  //byte tclk, byte tcs, byte din, byte dout, byte irq


//Graphics myGLCD(10,10,800,480,0xffffff,0xffffff);


// Declare which fonts we will be using
extern uint8_t BigFont[];

int x, y;
char stCurrent[20]="";
int stCurrentLen=0;
char stLast[20]="";

byte rgbtable[34][3]{
  {0,0,255},
  {0,32,255},
  {0,64,255},
  {0,96,255},
  {0,128,255},
  {0,160,255},
  {0,192,255},
  {0,224,255},
  {0,255,255},
  {0,255,224},
  {0,255,192},
  {0,255,160},
  {0,255,128},
  {0,255,96},
  {0,255,64},
  {0,255,32},
  {0,255,0},
  {32,255,0},
  {64,255,0},
  {96,255,0},
  {128,255,0},
  {160,255,0},
  {192,255,0},
  {224,255,0},
  {255,255,0},
  {255,224,0},
  {255,192,0},
  {255,160,0},
  {255,128,0},
  {255,96,0},
  {255,64,0},
  {255,32,0},
  {255,0,0}
  };

/*************************
**   Custom functions   **
*************************/

void drawButtons()
{
// Draw the upper row of buttons
  for (x=0; x<5; x++)
  {
    myGLCD.setColor(0, 0, 255);
    myGLCD.fillRoundRect (10+(x*60), 10, 60+(x*60), 60);
    myGLCD.setColor(255, 255, 255);
    myGLCD.drawRoundRect (10+(x*60), 10, 60+(x*60), 60);
    myGLCD.printNumI(x+1, 27+(x*60), 27);
  }
// Draw the center row of buttons
  for (x=0; x<5; x++)
  {
    myGLCD.setColor(0, 0, 255);
    myGLCD.fillRoundRect (10+(x*60), 70, 60+(x*60), 120);
    myGLCD.setColor(255, 255, 255);
    myGLCD.drawRoundRect (10+(x*60), 70, 60+(x*60), 120);
    if (x<4)
      myGLCD.printNumI(x+6, 27+(x*60), 87);
  }
  myGLCD.print("0", 267, 87);
// Draw the lower row of buttons
  myGLCD.setColor(0, 0, 255);
  myGLCD.fillRoundRect (10, 130, 150, 180);
  myGLCD.setColor(255, 255, 255);
  myGLCD.drawRoundRect (10, 130, 150, 180);
  myGLCD.print("Clear", 40, 147);
  myGLCD.setColor(0, 0, 255);
  myGLCD.fillRoundRect (160, 130, 300, 180);
  myGLCD.setColor(255, 255, 255);
  myGLCD.drawRoundRect (160, 130, 300, 180);
  myGLCD.print("Enter", 190, 147);
  myGLCD.setBackColor (0, 0, 0);
}





void updateStr(int val)
{
  if (stCurrentLen<20)
  {
    stCurrent[stCurrentLen]=val;
    stCurrent[stCurrentLen+1]='\0';
    stCurrentLen++;
    myGLCD.setColor(0, 255, 0);
    myGLCD.print(stCurrent, LEFT, 224);
  }
  else
  {
    myGLCD.setColor(255, 0, 0);
    myGLCD.print("BUFFER FULL!", CENTER, 192);
    delay(500);
    myGLCD.print("            ", CENTER, 192);
    delay(500);
    myGLCD.print("BUFFER FULL!", CENTER, 192);
    delay(500);
    myGLCD.print("            ", CENTER, 192);
    myGLCD.setColor(0, 255, 0);
  }
}

// Draw a red frame while a button is touched
void waitForIt(int x1, int y1, int x2, int y2)
{
  myGLCD.setColor(255, 0, 0);
  myGLCD.drawRoundRect (x1, y1, x2, y2);
  while (myTouch.dataAvailable())
    myTouch.read();
  myGLCD.setColor(255, 255, 255);
  myGLCD.drawRoundRect (x1, y1, x2, y2);
}

//drawColorBar(value 0-32, x coordinate of left of graph, y coordinate of top left corner of graph, size);
void drawColorBar(int value,int spotx, int spoty, int pix){
  for(int i=0;i<33;i++){
    if(i<=value){
    myGLCD.setColor(rgbtable[i][0],rgbtable[i][1],rgbtable[i][2]);
    } else {myGLCD.setColor(VGA_GRAY);}
    myGLCD.fillRect(spotx+(i*pix),spoty,spotx+(i*pix)+pix/2,spoty+18);
  }
 myGLCD.setColor(VGA_BLACK);
 myGLCD.fillRect(spotx+(32*pix)+58,spoty,spotx+(32*pix)+90,spoty+17);
 //myGLCD.setColor(rgbtable[value][0],rgbtable[value][1],rgbtable[value][2]);
 myGLCD.setColor(255,255,255);
 myGLCD.print("V: "+String(value)+"",spotx+(32*pix)+10,spoty+4);
}

/*************************
**  Required functions  **
*************************/

void setup()
{// The following two lines are needed for the  display
// module to enable the backlight. If you are using any other 
// display module these lines should be commented out.
  // -------------------------------------------------------------
  pinMode(8, OUTPUT);  //backlight 
  digitalWrite(8, HIGH);//on
// -------------------------------------------------------------
// Initial setup
  myGLCD.InitLCD();
  myGLCD.clrScr();

  myTouch.InitTouch();
  myTouch.setPrecision(PREC_MEDIUM);

  myGLCD.setFont(BigFont);
  myGLCD.setBackColor(0, 0, 255);
//
//drawButtons();  
}

void loop()

{
  while (true)
  {


   for(int i=0; i<33; i++){
      drawColorBar(i,0,35,4);
      drawColorBar(i,0,70,4);
      drawColorBar(i,0,105,4);
      drawColorBar(i,0,140,4);
      drawColorBar(i,0,175,4);
      drawColorBar(i,0,210,4);
      drawColorBar(i,0,245,4);
      drawColorBar(i,0,280,4);
      drawColorBar(i,0,315,4);
      drawColorBar(i,0,350,4);
      drawColorBar(i,0,385,4);
      drawColorBar(i,0,420,4);
      delay(10);
}

for(int i=32; i>=0; i--){
    drawColorBar(i,0,35,4);
      drawColorBar(i,0,70,4);
      drawColorBar(i,0,105,4);
      drawColorBar(i,0,140,4);
      drawColorBar(i,0,175,4);
      drawColorBar(i,0,210,4);
      drawColorBar(i,0,245,4);
      drawColorBar(i,0,280,4);
      drawColorBar(i,0,315,4);
      drawColorBar(i,0,350,4);
      drawColorBar(i,0,385,4);
      drawColorBar(i,0,420,4);
      delay(10);
}
    
    if (myTouch.dataAvailable())
    {
      myTouch.read();
      x=myTouch.getX();
      y=myTouch.getY();
      
      if ((y>=10) && (y<=60))  // Upper row
      {
        if ((x>=10) && (x<=60))  // Button: 1
        {
          waitForIt(10, 10, 60, 60);
          updateStr('1');
        }
        if ((x>=70) && (x<=120))  // Button: 2
        {
          waitForIt(70, 10, 120, 60);
          updateStr('2');
        }
        if ((x>=130) && (x<=180))  // Button: 3
        {
          waitForIt(130, 10, 180, 60);
          updateStr('3');
        }
        if ((x>=190) && (x<=240))  // Button: 4
        {
          waitForIt(190, 10, 240, 60);
          updateStr('4');
        }
        if ((x>=250) && (x<=300))  // Button: 5
        {
          waitForIt(250, 10, 300, 60);
          updateStr('5');
        }
      }

      if ((y>=70) && (y<=120))  // Center row
      {
        if ((x>=10) && (x<=60))  // Button: 6
        {
          waitForIt(10, 70, 60, 120);
          updateStr('6');
        }
        if ((x>=70) && (x<=120))  // Button: 7
        {
          waitForIt(70, 70, 120, 120);
          updateStr('7');
        }
        if ((x>=130) && (x<=180))  // Button: 8
        {
          waitForIt(130, 70, 180, 120);
          updateStr('8');
        }
        if ((x>=190) && (x<=240))  // Button: 9
        {
          waitForIt(190, 70, 240, 120);
          updateStr('9');
        }
        if ((x>=250) && (x<=300))  // Button: 0
        {
          waitForIt(250, 70, 300, 120);
          updateStr('0');
        }
      }

      if ((y>=130) && (y<=180))  // Upper row
      {
        if ((x>=10) && (x<=150))  // Button: Clear
        {
          waitForIt(10, 130, 150, 180);
          stCurrent[0]='\0';
          stCurrentLen=0;
          myGLCD.setColor(0, 0, 0);
          myGLCD.fillRect(0, 224, 319, 239);
        }
        if ((x>=160) && (x<=300))  // Button: Enter
        {
          waitForIt(160, 130, 300, 180);
          if (stCurrentLen>0)
          {
            for (x=0; x<stCurrentLen+1; x++)
            {
              stLast[x]=stCurrent[x];
            }
            stCurrent[0]='\0';
            stCurrentLen=0;
            myGLCD.setColor(0, 0, 0);
            myGLCD.fillRect(0, 208, 319, 239);
            myGLCD.setColor(0, 255, 0);
            myGLCD.print(stLast, LEFT, 208);
          }
          else
          {
            myGLCD.setColor(255, 0, 0);
            myGLCD.print("BUFFER EMPTY", CENTER, 192);
            delay(500);
            myGLCD.print("            ", CENTER, 192);
            delay(500);
            myGLCD.print("BUFFER EMPTY", CENTER, 192);
            delay(500);
            myGLCD.print("            ", CENTER, 192);
            myGLCD.setColor(0, 255, 0);
          }
        }
      }
    }
  }
}
