/*
* Copyright 2012 - Louis McCarthy
* File: bl_lcd.c
* Description: Initializes the LCD3 from BeagleBoardToys
*/
#include "uartStdio.h"
#include "bl_lcd.h"

/*
 * Initialize backlight, LCD data pins, and GPIO for switches
 */
int InitLCD(void)
{
    UARTPuts("Initializing LCD3\r\n", -1);

    return 0;
}
