#ifndef __LCD_H_
#define __LCD_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/**
 * @brief  Number of columns & rows on the LCD display.
 */
#define _LCD_COLS      20  // Number of columns on the LCD display
#define _LCD_ROWS      4  // Number of rows on the LCD display

/**
 * @brief  LCD commands.
 */
#define LCD_CLEARDISPLAY      0x01  // Clear the display
#define LCD_RETURNHOME        0x02  // Return the cursor to the home position
#define LCD_ENTRYMODESET      0x04  // Set the entry mode
#define LCD_DISPLAYCONTROL    0x08  // Control the display
#define LCD_CURSORSHIFT       0x10  // Shift the cursor
#define LCD_FUNCTIONSET       0x20  // Set the function
#define LCD_SETCGRAMADDR      0x40  // Set the CGRAM address
#define LCD_SETDDRAMADDR      0x80  // Set the DDRAM address

/**
 * @brief  Flags for display entry mode.
 */
#define LCD_ENTRYRIGHT          0x00  // Entry mode: right
#define LCD_ENTRYLEFT           0x02  // Entry mode: left
#define LCD_ENTRYSHIFTINCREMENT 0x01  // Entry mode: shift increment
#define LCD_ENTRYSHIFTDECREMENT 0x00  // Entry mode: shift decrement

/**
 * @brief  Flags for display on/off control.
 */
#define LCD_DISPLAYON        0x04  // Display on
#define LCD_CURSORON         0x02  // Cursor on
#define LCD_BLINKON          0x01  // Blink on

/**
 * @brief  Flags for display/cursor shift.
 */
#define LCD_DISPLAYMOVE      0x08  // Display move
#define LCD_CURSORMOVE       0x00  // Cursor move
#define LCD_MOVERIGHT        0x04  // Move right
#define LCD_MOVELEFT         0x00  // Move left

/**
 * @brief  Flags for function set.
 */
#define LCD_8BITMODE        0x10  // 8-bit mode
#define LCD_4BITMODE        0x00  // 4-bit mode
#define LCD_2LINE          0x08  // 2-line display
#define LCD_1LINE          0x00  // 1-line display
#define LCD_5x10DOTS        0x04  // 5x10 dots
#define LCD_5x8DOTS         0x00  // 5x8 dots

/**
 * @brief  Struct to hold LCD options.
 */
typedef struct {
   uint8_t DisplayControl;  // Display control flags
   uint8_t DisplayFunction;  // Display function flags
   uint8_t DisplayMode;    // Display mode flags
   uint8_t currentX;      // Current X position
   uint8_t currentY;      // Current Y position
} LcdOption;

/**
 * @brief  Delay for a specified number of microseconds.
 * @param  us The number of microseconds to delay.
 */
void LcdDelay_us(uint16_t us);

/**
 * @brief  Delay for a specified number of milliseconds.
 * @param  ms The number of milliseconds to delay.
 */
void LcdDelay_ms(uint8_t ms);

/**
 * @brief  Set the reset pin low.
 */
void LcdRstLow(void);

/**
 * @brief  Set the reset pin high.
 */
void LcdRstHigh(void);

/**
 * @brief  Set the enable pin low.
 */
void LcdEnLow(void);

/**
 * @brief  Set the enable pin high.
 */
void LcdEnHigh(void);

/**
 * @brief  Enable blinking.
 */
void LcdEnBlink(void);

/**
 * @brief  Set the backlight.
 */
void LcdBackLightSet(void);


/**
 * @brief  Initialize the LCD.
 */
void LcdInit(void);

/**
 * @brief  Clear the display.
 */
void LcdClear(void);

/**
 * @brief  Turn the display on.
 */
void LcdDisplayOn(void);

/**
 * @brief  Turn the display off.
 */
void LcdDisplayOff(void);

/**
 * @brief  Print a string to the display.
 * @param  x The X position to start printing.
 * @param  y The Y position to start printing.
 * @param  str The string to print.
 */
void LcdPuts(uint8_t x, uint8_t y, char const *str);

/**
 * @brief  Turn blinking on.
 */
void LcdBlinkOn(void);

/**
 * @brief  Turn blinking off.
 */
void LcdBlinkOff(void);

/**
 * @brief  Turn the cursor on.
 */
void LcdCursorOn(void);

/**
 * @brief  Turn the cursor off.
 */
void LcdCursorOff(void);

/**
 * @brief  Scroll the display left.
 */
void LcdScrollLeft(void);

/**
 * @brief  Scroll the display right.
 */
void LcdScrollRight(void);

/**
 * @brief  Create a custom character.
 * @param  location The location of the custom character.
 * @param  data The data for the custom character.
 */
void LcdCreateChar(uint8_t location, uint8_t const *data);

/**
 * @brief  Print a custom character.
 * @param  x The X position to print the character.
 * @param  y The Y position to print the character.
 * @param  location The location of the custom character.
 */
void LcdPutCustom(uint8_t x, uint8_t y, uint8_t location);

/**
 * @brief  Print a single character.
 * @param  Data The character to print.
 */
void LcdPut(uint8_t Data);

/**
 * @brief  Read a string from the display.
 * @param  str The buffer to store the read string.
 * @param  x The X position to start reading.
 * @param  y The Y position to start reading.
 * @param  length The length of the string to read.
 */
//void LcdReadStr(char* str, uint8_t x, uint8_t y, uint8_t length);

/**
 * @brief  Send a command to the LCD.
 * @param  cmd The command to send.
 */
void LcdCmd(uint8_t cmd);

/**
 * @brief  Send a 4-bit command to the LCD.
 * @param  cmd The 4-bit command to send.
 */
void LcdCmd4bit(uint8_t cmd);

/**
 * @brief  Send data to the LCD.
 * @param  data The data to send.
 */
void LcdData(uint8_t data);

/**
 * @brief  Set the cursor position.
 * @param  col The column position.
 * @param  row The row position.
 */
void LcdCursorSet(uint8_t col, uint8_t row);

/**
 * @brief  Get the width of the display.
 * @return The width of the display.
 */
int16_t LcdGetWidth(void);

/**
 * @brief  Get the height of the display.
 * @return The height of the display.
 */
int16_t LcdGetHeight(void);

/**
 * @brief  Draw a bitmap on the display.
 * @param  x The X position to start drawing.
 * @param  y The Y position to start drawing.
 * @param  image The bitmap image to draw.
 * @param  mem8 The memory address to store the bitmap data.
 */
//void DrawBitmap(char x, char y, const uint8_t *image, char mem8);


#ifdef __cplusplus
}
#endif

#endif  /* __LCD_H */

