/**
 * \file  bl_main.c
 *
 * \brief Implements main function for StarterWare bootloader
 *
*/

/*
* Copyright (C) 2012 Texas Instruments Incorporated - http://www.ti.com/
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*    Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the
*    distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "uartStdio.h"
#include "beaglebone.h"
#include "bl_copy.h"
#include "bl_platform.h"
#include "bl.h"
#include "gpio_v2.h"
#include "bl_lcd.h"

/******************************************************************************
**                    External Variable Declararions 
*******************************************************************************/

extern char *deviceType;


/******************************************************************************
**                     Local Function Declararion 
*******************************************************************************/

static void (*appEntry)();


/******************************************************************************
**                     Global Variable Definitions
*******************************************************************************/

unsigned int entryPoint = 0;
unsigned int DspEntryPoint = 0;


/******************************************************************************
**                     Global Function Definitions
*******************************************************************************/
/*
 * \brief This function initializes the system and copies the image. 
 *
 * \param  none
 *
 * \return none 
*/
int main(void)
{
    /* Configures PLL and DDR controller*/
    BlPlatformConfig();

    UARTPuts("\n\rBeagleboot V0.1 - 09/13/2012", -1);
    UARTPuts("\n\rCopyright 2012 - Louis McCarthy (AI0LM)", -1);

    TurnOnLED();
    InitI2C(2);
    InitCapes();
    //LoadCommandInterpreter();

    UARTPuts("\n\rCommand Interpreter Exited...System Halted", -1);
    while(1)
    {
    }
    //return 0;
}

void InitCapes(void)
{
    UARTPuts("\n\rDiscovering Capes...", -1);
    UARTPuts("Done", -1);
}

void InitI2C(int num)
{
    UARTPuts("\n\rInitializing I2C bus ", -1);
	switch(num)
	{
		case 0:
			UARTPuts("0...", -1);
			break;
		case 1:
			UARTPuts("1...", -1);
			break;
		case 2:
			UARTPuts("2...", -1);
			// Setup pinmux
			// Set to reset state
			// Setup clock
			// Setup address of slave
			// Setup interrupts
			// Set mode
			// Start transfer
			// Stop transfer

			// if we find a device, configure DMA mode
			break;
	}
    UARTPuts("Done", -1);
}

void LoadCommandInterpreter(void)
{
	UARTPuts("\n\rSearching for 'APP'\n\r", -1);

	/* Copies application from non-volatile flash memory to RAM */
	ImageCopy();

	UARTPuts("\n\rJumping to StarterWare Application...inconceivable", -1);

	/* Do any post-copy config before leaving boot loader */
	BlPlatformConfigPostBoot();

	/* Giving control to the application */
	appEntry = (void (*)(void)) entryPoint;

	(*appEntry)( );
}

void TurnOnLED(void)
{
    UARTPuts("\n\r\n\rInitializing GPIO 1...", -1);
    GPIO1ModuleClkConfig();
    GPIO1Pin23PinMuxSetup();
    GPIOModuleEnable(SOC_GPIO_1_REGS);
    GPIOModuleReset(SOC_GPIO_1_REGS);
    GPIODirModeSet(SOC_GPIO_1_REGS, 23, GPIO_DIR_OUTPUT);
    GPIOPinWrite(SOC_GPIO_1_REGS, 23, GPIO_PIN_HIGH);
    UARTPuts("Done", -1);
}

void BootAbort(void)
{
    while(1);
}

/******************************************************************************
**                              END OF FILE
*******************************************************************************/
