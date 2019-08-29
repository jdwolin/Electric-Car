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

#include "UTouch.h"
#include "UTouchCD.h"

void UTouch::touch_WriteData(byte data)
{
    byte temp;

    temp=data;
    P_CLK = LOW;

    for(byte count=0; count<8; count++)
    {
        if(temp & 0x80)
            P_DIN = HIGH;
        else
            P_DIN = LOW;
        temp = temp << 1; 
        P_CLK = LOW;
        P_CLK = HIGH;
    }
}

word UTouch::touch_ReadData()
{
    word data = 0;

    for(byte count=0; count<12; count++)
    {
        data <<= 1;
        P_CLK = HIGH;
        P_CLK = LOW;
        if (P_DOUT)
            data++;
    }
    return(data);
}

UTouch::UTouch(PinName tclk, PinName tcs, PinName din, PinName dout, PinName irq)
    :P_CLK(tclk), P_CS(tcs), P_DIN(din), P_DOUT(dout), P_IRQ(irq)
{
}

void UTouch::InitTouch(byte orientation)
{
    orient                  = orientation;
    _default_orientation    = CAL_S>>31;
    touch_x_left            = (CAL_X>>14) & 0x3FFF;
    touch_x_right           = CAL_X & 0x3FFF;
    touch_y_top             = (CAL_Y>>14) & 0x3FFF;
    touch_y_bottom          = CAL_Y & 0x3FFF;
    disp_x_size             = (CAL_S>>12) & 0x0FFF;
    disp_y_size             = CAL_S & 0x0FFF;
    prec                    = 10;
    
    P_IRQ.output();

    P_CS = HIGH;
    P_CLK = HIGH;
    P_DIN = HIGH;
    P_IRQ = HIGH;
    touch_tresholdLow = -1024;
    touch_tresholdHigh = 2048;

}

int16_t UTouch::touch_getrawdata(uint8_t code) {
    int16_t tmp;
    P_CS = LOW;
    P_IRQ.input();
    if(!P_IRQ) {
        touch_WriteData(code);
        P_CLK = HIGH; P_CLK = LOW;
        tmp = touch_ReadData();
    }
    else {
        tmp = 0;
    }
    P_IRQ.output();
    P_CS = LOW;
    return tmp;
}
    
int16_t UTouch::GetPressure()
{
    int16_t _ry, _rz1, _rz2;
    _ry = GetYraw();
    _rz1 = GetZ1raw();
    _rz2 = GetZ2raw();
    return _ry*_rz1/_rz2-_ry-5000;
}

void UTouch::SetTreshold(int16_t trshLow, int16_t trshHigh)
{
    touch_tresholdLow = trshLow;
    touch_tresholdHigh = trshLow;
}

int16_t UTouch::GetXraw()
{
    return touch_getrawdata(0x90);
}

int16_t UTouch::GetYraw()
{
    return touch_getrawdata(0xD0);
}

int16_t UTouch::GetZ1raw()
{
    return touch_getrawdata(0xC0);
}

int16_t UTouch::GetZ2raw()
{
    return touch_getrawdata(0xB0);
}

bool UTouch::Read()
{
    unsigned long tx=0, temp_x=0;
    unsigned long ty=0, temp_y=0;
    unsigned long minx=99999, maxx=0;
    unsigned long miny=99999, maxy=0;
    int datacount=0;
    uint16_t _press;

        _press = GetPressure();
        if((_press>touch_tresholdLow)&&(_press<touch_tresholdHigh))
        {
    
            P_CS = LOW;

            P_IRQ.input();
            for (int i=0; i<prec; i++)
            {
                if (!P_IRQ)
                {
                    touch_WriteData(0x90);        
                    P_CLK = HIGH; P_CLK = LOW;
                    temp_x=touch_ReadData();
        
                    if (!P_IRQ)
                    {
                        touch_WriteData(0xD0);      
                        P_CLK = HIGH; P_CLK = LOW;
                        temp_y=touch_ReadData();
        
                        if ((temp_x>0) and (temp_x<4096) and (temp_y>0) and (temp_y<4096))
                        {
                            tx+=temp_x;
                            ty+=temp_y;
                            if (prec>5)
                            {
                                if (temp_x<minx)
                                    minx=temp_x;
                                if (temp_x>maxx)
                                    maxx=temp_x;
                                if (temp_y<miny)
                                    miny=temp_y;
                                if (temp_y>maxy)
                                    maxy=temp_y;
                            }
                            datacount++;
                        }
                    }
                }
            }
        }
    P_IRQ.output();

    if (prec>5)
    {
        tx = tx-(minx+maxx);
        ty = ty-(miny+maxy);
        datacount -= 2;
    }

    P_CS = HIGH;
    if ((datacount==(prec-2)) or (datacount==PREC_LOW))
    {
        if (orient == _default_orientation)
        {
            TP_X=ty/datacount;
            TP_Y=tx/datacount;
            return true;
        }
        else
        {
            TP_X=tx/datacount;
            TP_Y=ty/datacount;
            return true;
        }
    }
    else
    {
        TP_X=-1;
        TP_Y=-1;
        return false;
    }
}

bool UTouch::DataAvailable()
{
    bool avail;
    P_IRQ.input();
    avail = !(P_IRQ);
    P_IRQ.output();
    return avail;
}

int16_t UTouch::GetX()
{
    long c;

    if ((TP_X==-1) or (TP_Y==-1))
        return -1;
    if (orient == _default_orientation)
    {
        c = long(long(TP_X - touch_x_left) * (disp_x_size)) / long(touch_x_right - touch_x_left);
        if (c<0)
            c = 0;
        if (c>disp_x_size)
            c = disp_x_size;
    }
    else
    {
        if (_default_orientation == PORTRAIT)
            c = long(long(TP_X - touch_y_top) * (-disp_y_size)) / long(touch_y_bottom - touch_y_top) + long(disp_y_size);
        else
            c = long(long(TP_X - touch_y_top) * (disp_y_size)) / long(touch_y_bottom - touch_y_top);
        if (c<0)
            c = 0;
        if (c>disp_y_size)
            c = disp_y_size;
    }
    return c;
}

int16_t UTouch::GetY()
{
    int c;

    if ((TP_X==-1) or (TP_Y==-1))
        return -1;
    if (orient == _default_orientation)
    {
        c = long(long(TP_Y - touch_y_top) * (disp_y_size)) / long(touch_y_bottom - touch_y_top);
        if (c<0)
            c = 0;
        if (c>disp_y_size)
            c = disp_y_size;
    }
    else
    {
        if (_default_orientation == PORTRAIT)
            c = long(long(TP_Y - touch_x_left) * (disp_x_size)) / long(touch_x_right - touch_x_left);
        else
            c = long(long(TP_Y - touch_x_left) * (-disp_x_size)) / long(touch_x_right - touch_x_left) + long(disp_x_size);
        if (c<0)
            c = 0;
        if (c>disp_x_size)
            c = disp_x_size;
    }
    return c;
}

void UTouch::SetPrecision(byte precision)
{
    switch (precision)
    {
        case PREC_LOW:
            prec=1;     // DO NOT CHANGE!
            break;
        case PREC_MEDIUM:
            prec=12;    // Iterations + 2
            break;
        case PREC_HI:
            prec=27;    // Iterations + 2
            break;
        case PREC_EXTREME:
            prec=102;   // Iterations + 2
            break;
        default:
            prec=12;    // Iterations + 2
            break;
    }
}

void UTouch::CalibrateRead()
{
    unsigned long tx=0;
    unsigned long ty=0;

    P_CS = LOW;

    touch_WriteData(0x90);        
    P_CLK = HIGH; P_CLK = LOW;
    tx=touch_ReadData();

    touch_WriteData(0xD0);      
    P_CLK = HIGH; P_CLK = LOW;
    ty=touch_ReadData();

    P_CS = HIGH;

    TP_X=ty;
    TP_Y=tx;
}

