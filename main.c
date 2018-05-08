
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
 * @name: initEpaper
 * @brief: will initialize GPIO and the SPI Pins to communicate to the ePaper Display.
 *              The SPI Pins are configured for Primary mode.
 * */
void initEpaper(void);

/*
 * @name: sendCommand(byte command)
 * @brief: Sends a command byte to the ePaper
 *              - toggles the CS pin to be low
 *              - sets the command pin to low
 * @inputs: a valid command found in the data sheet and the header file
 * */
void sendCommand(uint8_t command);



/*
 * @name: sendData(byte data)
 * @brief: Sends a command byte to the ePaper
 *              - toggles the CS pin to be low
 *              - sets the command pin to HIGH
 * @inputs: send bytes of data to the ePaper
 *
 * This is better suited for a single data byte instead of multiple say for instance you had to send the LUT
 * this would not work because CS would be toggling between transmits.
     TODO if this proves not correct to initialize the screen I will come back and make one where data is transmitted and CS stays low the entire time
     // looking at other examples of the driver it looks like some keep CS low the entire data transfer of LUT and others don't, ie let it toggle.
*/
void sendData(uint8_t data);


void main(void)
{
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer

	__enable_interrupts();

	initEpaper();

	uint16_t i = 0;

	while(1){
	sendCommand(0xAA);
	for(i = 0; i< 1000; i++);
	sendData(0xBB);
    for(i = 0; i< 1000; i++);
	}

}

#define LAUNCHPAD_FW
#ifdef LAUNCHPAD_FW
/*
 *  Port 10 for the launchpad
 *  GPIO    10.0        =        Data Command pin, Data is HIGH     == Green Ch5 on Logic Analyzer (LA)
 *  PRIMARY    10.1        =        SCK        ==          yellow wire connected to channel 3 on Logic Analyzer
 *  PRIMARY    10.2        =       MISO        ==      BLUE - logic analyzer Ch. 8
 *  GPIO    10.3        =       CS      ==  ORANGE, channel 4 on Logic analyzer
 *
 *  GPIO 7.1    =   ePaper RESET    =   WHITE, CH7 LA //NOTE active LOW
 *  GPIO 7.2    = ePaper BUSY        =  PURPLE, CH4   //NOTE active HIGH
 *
 *  SPI module on UCB3
 *  Polarity: rising edge
 *  clock is low when idle
 *  master only, and STE is not used
 * */
void initEpaper(void){

    EUSCI_B3_SPI->CTLW0 |= UCSWRST; // set to a 1, unlock
    EUSCI_B3_SPI->CTLW0 &= ~(UCCKPH | UCCKPL | UC7BIT | UCMODE0   ); // polarity:0, phase:0, 8 bits, spi mode (vs i2c)
    EUSCI_B3_SPI->CTLW0 |= (UCMSB | UCMST |  UCSYNC | UCSSEL__SMCLK); // MSB, master, sync (vs uart), system clock : 3Mhz
    //TODO: think about using defines like #define CS_PIN   (BIT0) so I don't have to write so much code twice
    //maybe to make better you could use a define for pins and just redefine depending on what you're doing ie LaunchPad or the PCB

    //configure the pins
    P7SEL0 &=~(BIT1 | BIT2);        // busy and reset
    P7SEL1  &= ~(BIT1 | BIT2);

    P10SEL0 &=~(BIT0 | BIT3); // D/C and CS
    P10SEL1 &=~(BIT0 | BIT3);

    P10SEL0 |= (BIT1 | BIT2); //primary mode for MISO and SCK
    P10SEL1 &= ~(BIT1 | BIT2);

    P7DIR &= ~(BIT2); //busy is an input
    P7DIR |= BIT1;
    P10DIR |= BIT0 | BIT1 | BIT2 | BIT3;

    P10OUT &= ~(BIT0 | BIT1 | BIT2 | BIT3); // SET ALL TO LOW
    P7OUT &= ~(BIT1 | BIT7);

    //may need pull down resistor on the Busy pin, and pull up on the RESET

    //TODO: prescaler if too fast for display
    //EUSCI_B3->BRW |= 1; // default reset value is 0... odd.

    EUSCI_B3_SPI->CTLW0 &= ~UCSWRST; // set to a 0 lock

    //enable interrupt for the TX
    EUSCI_B3_SPI ->IFG = 0;     //clear any interrupts
    EUSCI_B3_SPI -> IE |= UCTXIE;

    NVIC_EnableIRQ(EUSCIB0_IRQn);
}
#endif

void sendCommand(uint8_t command){
    P10OUT &= ~BIT0;     // D/C
    P10OUT &= ~BIT3;     // CS
    EUSCI_B3_SPI->TXBUF = command;
    P10OUT |= BIT3;
}

void sendData(uint8_t data){
    P10OUT |=   BIT0;     // D/C
    P10OUT &= ~BIT3;     // CS
    EUSCI_B3_SPI->TXBUF = data;
    P10OUT |= BIT3;

}


