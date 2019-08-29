/** \file UTouch.h
 *  \brief mbed library for XPT2046 SPI Touchscreen Controller (TFT_320QVT module).
 *  \copyright GNU Public License, v2. or later
 *
 * A known display with this type of controller chip is the ITDB02-3.2S
 * from http://imall.iteadstudio.com
 *
 * This library is based on the Arduino/chipKIT UTFT library by Henning
 * Karlsen, http://www.rinkydinkelectronics.com/library.php?id=55
 *
 * Pressure sensivity added by Dmitry Shtatnov
 *
 * Copyright (C)2010-2015 Henning Karlsen. All right reserved.
 *
 * Copyright (C)2016 Dmitry Shtatnov. All right reserved.
 * http://modularsynth.ru/
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to:
 *
 * Free Software Foundation, Inc.
 * 51 Franklin St, 5th Floor, Boston, MA 02110-1301, USA
 *
 *********************************************************************/

#include "mbed.h"

#ifndef UTouch_h
#define UTouch_h

#ifdef __cplusplus
extern "C" {
#endif

/** Represents a Touchscreen instance.
 *
 * This is the utility class, through which the XPT2046 touchscreen can be manipulated
 * How to use:
 * \code
 * // If you are using TFT-320QVT TFT LCD module or similar, you also have to add
 * // SSD1289 library by Todor Todorov or modified version by Dmitry Shtatnov
 * 
 * #include "mbed.h"
 * #include "ssd1289.h"
 * #include "UTouch.h"
 * PortOut LCDPA(PortA,0xFF00);
 * PortOut LCDPC(PortC,0x00FF);
 * //                 CS,   REST, RS,   WR,   DATA,      BL, RD
 * SSD1289_LCD myGLCD(PB_3, PB_4, PB_0, PB_1, &LCDPA, &LCDPC, NC, PB_2);
 * 
 * UTouch  myTouch(PB_9, PB_8, PB_7, PB_6, PB_5);
 * 
 * int main() {
 *   myGLCD.Initialize();
 *   myGLCD.SetBackground(COLOR_WHITE);
 *   myGLCD.SetForeground(COLOR_BLACK);
 *   myGLCD.FillScreen(COLOR_WHITE);
 *   myTouch.InitTouch();
 *   myTouch.SetPrecision(PREC_HI);
 *   while(1==1)
 *   {
 *     if (myTouch.DataAvailable())
 *     {
 *       if(myTouch.Read())
 *       {
 *         x = myTouch.GetX();
 *         y = myTouch.GetY();
 *         myGLCD.DrawPixel(x, y, COLOR_BLACK);
 *       }
 *     }
 *   }
 * }
 *
 * \endcode
 * \version 0.1
 * \author Dmitry Shtatnov
 */


#define UTOUCH_VERSION  124

// *** Hardwarespecific defines ***
#define swap(type, i, j) {type t = i; i = j; j = t;}

#define LOW                 0
#define HIGH                1

#define PORTRAIT            0
#define LANDSCAPE           1

#define PREC_LOW            1
#define PREC_MEDIUM         2
#define PREC_HI             3
#define PREC_EXTREME        4

#define byte uint8_t
#define word uint16_t
class UTouch
{
    public:
        int16_t TP_X ,TP_Y, TP_Z1, TP_Z2;

                UTouch(PinName tclk, PinName tcs, PinName tdin, PinName dout, PinName irq);

        void    InitTouch(byte orientation = LANDSCAPE);
        bool    Read();
        bool    DataAvailable();
        int16_t GetX();
        int16_t GetY();
        void    SetPrecision(byte precision);

        void    CalibrateRead();
        int16_t GetPressure();
        void    SetTreshold(int16_t trshLow, int16_t trshHigh);
    
    protected:
        int16_t GetXraw();
        int16_t GetYraw();
        int16_t GetZ1raw();
        int16_t GetZ2raw();
        DigitalOut P_CLK, P_CS, P_DIN;
        DigitalIn P_DOUT;
        DigitalInOut P_IRQ;
        long    _default_orientation;
        byte    orient;
        byte    prec;
        byte    display_model;
        long    disp_x_size, disp_y_size, default_orientation;
        long    touch_x_left, touch_x_right, touch_y_top, touch_y_bottom;

        void    touch_WriteData(byte data);
        word    touch_ReadData();
        int16_t touch_tresholdLow;
        int16_t touch_tresholdHigh;
        int16_t touch_getrawdata(uint8_t code);
};

#ifdef __cplusplus
}
#endif

#endif