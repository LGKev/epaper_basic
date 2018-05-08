
#include "msp.h"
#include "ePaper.h"

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
	//sendCommand(0xAA);
	//for(i = 0; i< 200; i++);
	//sendData(0xBB);
   // for(i = 0; i< 200; i++);
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

    P10OUT |= (BIT0 | BIT1 | BIT2 | BIT3); // set all high according to data sheet 21/29 good display
    P7OUT &= ~(BIT2);   //busy is active high!
    P7OUT |= (BIT1);      //reset is active low!

    //may need pull down resistor on the Busy pin, and pull up on the RESET

    //TODO: prescaler if too fast for display
    //EUSCI_B3->BRW |= 1; // default reset value is 0... odd.

    EUSCI_B3_SPI->CTLW0 &= ~UCSWRST; // set to a 0 lock

    //enable interrupt for the TX
    EUSCI_B3_SPI ->IFG = 0;     //clear any interrupts
    EUSCI_B3_SPI -> IE |= UCTXIE;

    NVIC_EnableIRQ(EUSCIB3_IRQn);


    //Panel Reset
    uint16_t delay = 0;
    P7OUT &=~BIT1;
    for(delay = 0; delay < 6000; delay++);
    P7OUT |= BIT1;
    for(delay = 0; delay < 6000; delay++); //double check with LA to see if equal or greater than 10mS

    //Gate number and scan order setting
    sendCommand(CMD_DRIVER_OUTPUT_CONTROL);
    sendData((LCD_VERTICAL_MAX - 1) & 0xFF); // Thanks Yehui from Waveshare!
    sendData(((LCD_HORIZONTAL_MAX - 1) >> 8) & 0xFF);// Thanks Yehui from Waveshare!
    sendCommand(0x00);
    //set scan frequency 50hz
    sendCommand(CMD_DUMMY_LINE); //dummy line wdith
    sendData(0x1B); // default value
    sendCommand(CMD_GATE_TIME);
    sendData(0x0B); //default value
    //data entry sequence (y-, x+)/// oof that is gonna trip up my brain
   sendCommand(CMD_DATA_ENTRY);
   sendData(0x01); // note: what would change when we make this bot y+ x+ where would origin be
   //set ram X start/end
   //look on section 8.3 of the IL3820 datasheet shows a diagram of the way data is written
   sendCommand(CMD_X_ADDR_START);
   sendData(0x00);
   sendData(0x18); //specified in the data sheet, interesting, so 200/8 is 25_base10, but 0x19... and 0 to 0x18 is 24
   //set ram Y start end
   sendCommand(CMD_Y_ADDR_START);
   sendData(0xC7); //199 because 0 to 199 is 200 points //oh! because a row is 1 pixel and we have 200 rows?
   sendData(0x00); //why not 25 row?
   sendData(0x00);
   sendData(0x00); //might be one extra, but specified in the datasheet

   //vcom setting
   sendCommand(CMD_VCOM);
   sendData(0xA8); // Thanks Yehui from Waveshare!
   //wavfrom setting
   sendCommand(CMD_WRITE_LUT);
   uint8_t lutCount = 0;
   for(lutCount = 0; lutCount < 30; lutCount++){
       sendData(lut_full_update[lutCount]);
   }
//set ramX address counter
   sendCommand(CMD_X_COUNTER);
   sendData(0x00); //note: from datasheet
//set ramY address counter\
   sendCommand(CMD_Y_COUNTER);
   sendData(0xC7); // note from datasheet
   sendData(0x00);

   //image data download!
   sendCommand(CMD_WRITE_RAM);
   uint16_t fillBlankCount = 0;
   for(fillBlankCount = 0; fillBlankCount < 2000; fillBlankCount++){
       sendData(WHITE);
   }

   //display update sequence setting: use waveform from ram
   sendCommand(CMD_DISPLAY_UPDATE_CTRL2); //note from datasheet
   sendData(0xC7);

   //image update
   sendCommand(CMD_MASTER_ACTV);

   //wait until not busy, or when busy pin goes LOW
   while(P7IN&BIT2);
   sendCommand(CMD_DEEP_SLEEP);
   sendData(0x01); //note specified by datasheet



}
#endif

void sendCommand(uint8_t command){
    while(EUSCI_B3_SPI->IFG & UCTXIFG);
    P10OUT &= ~BIT0;     // D/C
    P10OUT &= ~BIT3;     // CS
    EUSCI_B3_SPI->TXBUF = command;
    P10OUT |= BIT3;
}

void sendData(uint8_t data){
    while(EUSCI_B3_SPI->IFG & UCTXIFG);
    P10OUT |=   BIT0;     // D/C
    P10OUT &= ~BIT3;     // CS
    EUSCI_B3_SPI->TXBUF = data;
    P10OUT |= BIT3;
}

/**/

void EUSCIB3_IRQHandler(void)
{
    if(EUSCI_B3_SPI->IFG & EUSCI_B_IFG_TXIFG){
        EUSCI_B3_SPI->IFG &=~ EUSCI_B_IFG_TXIFG;
    }
}



