
#include "msp.h"


/*
 * main.c
 * Author: Kevin Kuwata
 * Date: 5/8/18
 *
 * This file is to be paired with the hardware V2 watch. To develop the device, I will be using the launchpad as the main platform.
 * This means that the pins for the ePaper are going to be the pins that are readily available on the launchpad and not the PCB.
 * Differentiate which code by using the #defines PCB_FW and Launchpad_FW
 */


/*
 * @brief: initEpaper will initialize GPIO and the SPI Pins to communicate to the ePaper Display.
 * The SPI Pins are configured for Primary mode. Because of the hardware on the PCB STE will not be used, but
 * manual GPIO toggling will be required for the CS pin.
 * */
void initEpaper();





void main(void)
{
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer
}

#define LAUNCHPAD_FW
#ifdef LAUNCHPAD_FW
/*
 *  Port 10 for the launchpad
 *  GPIO    10.0        =        Data Command pin, Data is HIGH     == Green Ch5 on Logic Analyzer
 *  GPIO    10.1        =        SCK        ==          yellow wire connected to channel 3 on Logic Analyzer
 *  GPIO    10.2        =       MISO        ==      BLUE - logic analyzer Ch. 8
 *  GPIO    10.3        =       CS      ==  YELLO, channel 3 on Logic analyzer
 * */
void initEpaper(void){

}
#endif
