/*
 * main.c
 * Author: Kevin Kuwata
 * Date: 5/8/18
 *
 * This file is to be paired with the hardware V2 watch. To develop the device, I will be using the launchpad as the main platform.
 * This means that the pins for the ePaper are going to be the pins that are readily available on the launchpad and not the PCB.
 * Differentiate which code by using the #defines PCB_FW and Launchpad_FW
 */
#include "msp.h"
#include "ePaper.h"


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

/*
 *  @name: void setLut(const unsigned char* lut)
 *  @brief: sends the look up table (LUT) to ePaper
 *  @input: partial or full update LUT ** provided by venord **
 * */
void setLUT(const unsigned char* lut);

/*
 * @name: setMemoryArea(uint8_t x_start, uint8_t y_start, uint8_t x_end, uint8_t y_end)
 * @brief: writes to the specified memory area ( x,y) start, and (xy) end
 * @inputs: x, y start, and xy end, consult an image of the specified area of the graphics display from data sheet
 * */
void setMemoryArea(uint8_t x_start, uint8_t y_start, uint8_t x_end, uint8_t y_end);


/*  @name: setMemoryPointer(uint8_t x, uint8_t y);
 *  @brief: sets the start location in memory
 *  @inputs: x, y starting location
 *
 * */
void setMemoryPointer(uint8_t x, uint8_t y);



/*
 * @name: void clearFrameMemory(unsigned char color) {
 * @brief: writes to the RAM by choosing a color, black or white, 1 or 0.
 * @inputs: color: 1 or 0
 * */
void clearFrameMemory(unsigned char color) ;


/*
 * @name: setFrameMemory(const unsigned char*);
 * @brief: loads wave share default image
 * @input constant unsigned char array (pixel data)
 * */
void setFrameMemory(const unsigned char* image_buffer);

/*
 *  @name: sendToDisplay()
 *  @brief: you must call this after updating the RAM otherwise nothing will be pushed to the display
 *  @inputs: none
 * */
void sendToDisplay(void);


/*
 *  @name: void waitNotBusy(void)
 *  @brief: checks the busy pin with a Port Read, holds us in a while loop until busy pin goes low.
 *  @input: none
 * */
void waitNotBusy(void);

/* == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == *//* == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == */
/* == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == *//* == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == */
//                                                          MAIN.c
/* == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == *//* == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == */
//* == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == *//* == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == */
void main(void)
{
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer

	__enable_interrupts();

	initEpaper();

	clearFrameMemory(0xFF); /// we are getting stuck in the while loop checking busy line here.
	//0x3b command because the phase was wrong! for spi config
	sendToDisplay();

    clearFrameMemory(0xFF);
    sendToDisplay();

	setFrameMemory(IMAGE_DATA);
	sendToDisplay();

	uint32_t i = 0;
	while(1){
	//sendCommand(0xAA);
	//for(i = 0; i< 200; i++);
	//sendData(0xBB);
    for(i = 0; i< 20000; i++);
	    setFrameMemory(IMAGE_DATA);
	    sendToDisplay();

	}

}

/*============================================================================================================*/
/*============================================================================================================*/
/*============================================================================================================*/

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
#define PORT_INIT
#ifdef PORT_INIT
void initEpaper(void){
    EUSCI_B3_SPI->CTLW0 |= UCSWRST; // set to a 1, unlock
       EUSCI_B3_SPI->CTLW0 &= ~(UCCKPL  | UC7BIT | UCMODE0   ); // polarity:0, phase:0, 8 bits, spi mode (vs i2c)
       EUSCI_B3_SPI->CTLW0 |= (UCCKPH | UCMSB | UCMST |  UCSYNC | UCSSEL__SMCLK); // MSB, master, sync (vs uart), system clock : 3Mhz

       /* NOTE:
        * very interesting. I am monitoring with a Logic Analyzer (LA) for the arduino code.
        *       the settings:   MSB, 8 bits, clock polarity is 0, low when idle
        *                           DATA valid on leading edge,
        *                           enable line is active low.
        *    The code appears to be talking to the display (changes color).
        *    Busy line goes high at 0x20 command
        *
        *   But when I use these same settings for the SPI hardware on the MSP
        *   I get the busy line going high in the middle randomly suggesting that the chip is getting a command incorrectly.
        *   I then change the polarity to 1 or data shifted out on the "falling edge" (figure 23-4) and the busy line seemed to
        *   behave correctly, not going high unitl command:0x4E which is the CMD_X_Counter command....
        *
        *
        *   BUT WHEN I CHANGE THIS I have to change the settings in the LA, which doesn't make sense. there is a conflict.
        *   I should be able to communicate the exact same way and not change the LA to see decoding.
        *
        *   so i know that we want the clock to be LOW when idle. meaning looking at figure 23-4 tech ref
        *   the only options are the first one or the 3rd one with phase 1.
        *
        *   so now i don't have to change the logic analyzer and the busy line goes high at 0x20 command!!!
        *
        *   the next issue is the why the send data isn't sending data. >>> ioverflow, too small of type, u8t vs u16t. fixed it.
        *
        *
        *
        * */


       //configure the pins
          P7SEL0 &=~(BIT1 | BIT2);        // busy and reset
          P7SEL1  &= ~(BIT1 | BIT2);

          //pull down on BUSY
          P7REN |= BIT2;
          P7OUT &= ~BIT2; // pulldown

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

          //TODO: prescaler if too fast for display, the waveshare code transmits at 2Mhz
      //    EUSCI_B3->BRW |= BIT1; // default reset value is 0... odd.

          EUSCI_B3_SPI->CTLW0 &= ~UCSWRST; // set to a 0 lock

          //enable interrupt for the TX
          EUSCI_B3_SPI ->IFG = 0;     //clear any interrupts
          EUSCI_B3_SPI -> IE |= UCTXIE;

          NVIC_EnableIRQ(EUSCIB3_IRQn);
          //reset epaper 10ms
          uint16_t delay = 0;
          P7OUT &=~BIT1;
          for(delay = 0; delay < 12000; delay++);
          P7OUT |= BIT1;
          for(delay = 0; delay < 12000; delay++); //double check with LA to see if equal or greater than 10mS

          //ePaper Init Sequence
              sendCommand(CMD_DRIVER_OUTPUT_CONTROL);
              sendData((LCD_VERTICAL_MAX - 1) & 0xFF);
              sendData(((LCD_HORIZONTAL_MAX - 1) >> 8) & 0xFF);
              sendData(0x00);                     // GD = 0; SM = 0; TB = 0;
              sendCommand(CMD_BOOSTER_SOFT_START_CONTROL);
              sendData(0xD7);
              sendData(0xD6);
              sendData(0x9D);
              sendCommand(CMD_VCOM);
              sendData(0xA8);                     // VCOM 7C
              sendCommand(CMD_DUMMY_LINE);
              sendData(0x1A);                     // 4 dummy lines per gate
              sendCommand(CMD_GATE_TIME); // the busy line is being set high here.
              sendData(0x08);                     // 2us per line
              sendCommand(CMD_DATA_ENTRY);
              sendData(0x03);                     // X increment; Y increment

              //send LUT
              setLUT(lut_full_update);

}
#endif




//#define DATASHEET_INIT
#ifdef DATASHEET_INIT
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

    //TODO: prescaler if too fast for display, the waveshare code transmits at 2Mhz
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

    sendCommand(CMD_BOOSTER_SOFT_START_CONTROL);
    sendData(0xD7);
    sendData(0xD6);
    sendData(0x9D);

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
   for(fillBlankCount = 0; fillBlankCount < 200 / 8* 200; fillBlankCount++){
       sendData(WHITE);
   }

   //display update sequence setting: use waveform from ram
   sendCommand(CMD_DISPLAY_UPDATE_CTRL2); //note from datasheet
   sendData(0xC7);

   //image update
   sendCommand(CMD_MASTER_ACTV);

   //wait until not busy, or when busy pin goes LOW
  // while(P7IN&BIT2);
   sendCommand(CMD_DEEP_SLEEP);
   sendData(0x01); //note specified by datasheet



}
#endif

#endif //launchpad firmware

void setMemoryArea(uint8_t x_start, uint8_t y_start, uint8_t x_end, uint8_t y_end){
    sendCommand(CMD_X_ADDR_START);
    sendData((x_start >> 3) & 0xFF);
    sendData((x_end >> 3) & 0xFF); //>>3 is div 8!
        sendCommand(CMD_Y_ADDR_START);
         sendData(y_start & 0xFF);
         sendData((y_start >> 8) & 0xFF);
         sendData(y_end & 0xFF);
         sendData((y_end >> 8) & 0xFF);

         waitNotBusy(); //we get stuck right here. TODO: why do we get stuck after this command? 0x45 start y address
         //wait for busy line... but busy line never goes low!
}

void setMemoryPointer(uint8_t x, uint8_t y){
    sendCommand(CMD_X_COUNTER);
    sendData((x >> 3) & 0xFF);
    sendCommand(CMD_Y_COUNTER);
    sendData(y & 0xFF);
    sendData((y >> 8) & 0xFF);
    waitNotBusy();
}

void setFrameMemory(const unsigned char* image_buffer){
        setMemoryArea(0, 0, LCD_HORIZONTAL_MAX- 1, LCD_VERTICAL_MAX - 1);
        setMemoryPointer(0, 0);
        sendCommand(CMD_WRITE_RAM);
        /* send the image data */
        uint16_t i = 0;
        for (i = 0; i < (200 / 8 ) * 200; i++) {
            sendData(image_buffer[i]);
        }
}


void clearFrameMemory(unsigned char color) {
    setMemoryArea(0, 0, LCD_HORIZONTAL_MAX, LCD_VERTICAL_MAX);
    setMemoryPointer(0, 0);
    sendCommand(CMD_WRITE_RAM);
    /* send the color data */
    uint16_t i =0;
 for (i = 0; i < ((200/8)*200); i++) {
    //for(i=0; i<200; i++)
    sendData(color);
    }
}

void sendToDisplay(){

    sendCommand(CMD_DISPLAY_UPDATE_CTRL2);
    sendData(0xC4);
    sendCommand(CMD_MASTER_ACTV);
    sendCommand(CMD_NOP);

   waitNotBusy();
}

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

void setLUT(const unsigned char* lut){
    sendCommand(CMD_WRITE_LUT);
    uint8_t i = 0;
    for(i = 0; i< 30; i++){
        sendData(lut[i]);
    }
}


void waitNotBusy(){

    uint8_t check = P7IN & BIT2;
    while(check);

}


/* ================================================================ */
//                              ISRs
/* ================================================================ */
/* ================================================================ */

void EUSCIB3_IRQHandler(void)
{
    if(EUSCI_B3_SPI->IFG & EUSCI_B_IFG_TXIFG){
        EUSCI_B3_SPI->IFG &= ~EUSCI_B_IFG_TXIFG;
    }
}



