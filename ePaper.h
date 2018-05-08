/*
 * ePaper200x200_driver.h
 *
 *  Created on: Jan 7, 2018
 *      Author: Kevin Kuwata
 *
 *      This file is essentially the parallel of Crystalfontz128x128_ST7735.h but for the ePaper.
 *		
 */

#ifndef EPAPER_H_
#define EPAPER_H_

#include <stdint.h>
#include "msp.h"

// LCD Screen Dimensions
#define LCD_VERTICAL_MAX                   200
#define LCD_HORIZONTAL_MAX                 200

//TODO: check that this is truly White  OR BLACK
#define BLACK   0XFF
#define WHITE    0X00

#define LCD_ORIENTATION_UP    0
#define LCD_ORIENTATION_LEFT  1
#define LCD_ORIENTATION_DOWN  2
#define LCD_ORIENTATION_RIGHT 3

//ePaper 200x200 LCD controller Command Table
#define CMD_DRIVER_OUTPUT_CONTROL 0X01
#define CMD_BOOSTER_SOFT_START_CONTROL 0X0C
#define CMD_GATE_SCAN_SP            0X0F
#define CMD_DEEP_SLEEP          0X10
#define CMD_DATA_ENTRY  0X11
#define CMD_SW_RESET     0X12
#define CMD_MASTER_ACTV  0X20
#define CMD_WRITE_RAM 0X24
#define CMD_VCOM    0X2C
#define CMD_WRITE_LUT 0X32
#define CMD_DUMMY_LINE 0X3A
#define CMD_GATE_TIME 0X3B //default is 50hZ!
#define CMD_BORDER 0X3C
#define CMD_DISPLAY_UPDATE_CTRL2    0X22
#define CMD_DISP_UPDATE_SEQ1            0XC4 // ENABLE CLOCK, ENABLE CP, TO PATTERN DISPLAY.

#define CMD_X_ADDR_START 0X44
#define CMD_Y_ADDR_START 0X45
#define CMD_X_COUNTER 0X4E
#define CMD_Y_COUNTER 0X4F
#define CMD_NOP 0XFF //GOOD FOR ENDING A COMMAND    /   DATA





#endif /* EPAPER_H_ */
