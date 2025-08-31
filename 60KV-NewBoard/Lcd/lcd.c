
#include "lcd.h"

LcdOption Options;

/**
 * @brief  Set the reset pin low.
 */
void LcdRstLow(void){ HAL_GPIO_WritePin(LcdRs_GPIO_Port, LcdRs_Pin, GPIO_PIN_RESET); }

/**
 * @brief  Set the reset pin high.
 */
void LcdRstHigh(void){ HAL_GPIO_WritePin(LcdRs_GPIO_Port, LcdRs_Pin, GPIO_PIN_SET); }

/**
 * @brief  Set the enable pin low.
 */
void LcdEnLow(void){ HAL_GPIO_WritePin(LcdEn_GPIO_Port, LcdEn_Pin, GPIO_PIN_RESET); }

/**
 * @brief  Set the enable pin high.
 */
void LcdEnHigh(void){ HAL_GPIO_WritePin(LcdEn_GPIO_Port, LcdEn_Pin, GPIO_PIN_SET); }

/**
 * @brief  Enable blinking.
 */
void LcdEnBlink(void){ LcdEnHigh(); LcdDelay_us(100); LcdEnLow(); LcdDelay_us(100); }

/**
 * @brief  Set the backlight.
 */
void LcdBackLightSet(void){ HAL_GPIO_WritePin(LcdBackLight_GPIO_Port, LcdBackLight_Pin, GPIO_PIN_SET); };

/**
 * @brief  Delay for a specified number of microseconds.
 * @param  us The number of microseconds to delay.
 */
void LcdDelay_us(uint16_t us)
{
  uint32_t  Div = (SysTick->LOAD+1)/1000;
  uint32_t  StartMicros = HAL_GetTick()*1000 + (1000- SysTick->VAL/Div);
  while((HAL_GetTick()*1000 + (1000-SysTick->VAL/Div)-StartMicros < us));  
}

/**
 * @brief  Delay for a specified number of milliseconds.
 * @param  ms The number of milliseconds to delay.
 */
void LcdDelay_ms(uint8_t ms)
{
  //#if _LCD_USE_FREERTOS==1
  //osDelay(ms);
  //#else
  HAL_Delay(ms);
  //#endif
}

/**
 * @brief  Initialize the LCD.
 */
void LcdInit(void)
{
	while(HAL_GetTick()<200)
	   LcdDelay_ms(1);
	/* Set cursor pointer to beginning for LCD */
	Options.currentX = 0;
	Options.currentY = 0;
	Options.DisplayFunction = LCD_4BITMODE | LCD_5x8DOTS | LCD_1LINE;
	if (_LCD_ROWS > 1)
		Options.DisplayFunction |= LCD_2LINE;
	/* Try to set 4bit mode */
	LcdCmd4bit(0x03);
	LcdDelay_ms(5);
	/* Second try */
	LcdCmd4bit(0x03);
	LcdDelay_ms(5);
	/* Third goo! */
	LcdCmd4bit(0x03);
	LcdDelay_ms(5);
	/* Set 4-bit interface */
	LcdCmd4bit(0x02);
	LcdDelay_ms(5);
	/* Set # lines, font size, etc. */
	LcdCmd(LCD_FUNCTIONSET | Options.DisplayFunction);
	/* Turn the display on with no cursor or blinking default */
	Options.DisplayControl = LCD_DISPLAYON;
	LcdDisplayOn();
	LcdClear();
	/* Default font directions */
	Options.DisplayMode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
	LcdCmd(LCD_ENTRYMODESET | Options.DisplayMode);
	LcdDelay_ms(5);
}

/**
 * @brief  Clear the display.
 */
void LcdClear(void)
{
	LcdDelay_ms(5);
	LcdCmd(LCD_CLEARDISPLAY);
	LcdDelay_ms(5);
}

/**
 * @brief  Turn the display on.
 */
void LcdDisplayOn(void)
{
	Options.DisplayControl |= LCD_DISPLAYON;
	LcdCmd(LCD_DISPLAYCONTROL | Options.DisplayControl);
}

/**
 * @brief  Turn the display off.
 */
void LcdDisplayOff(void)
{
	Options.DisplayControl &= ~LCD_DISPLAYON;
	LcdCmd(LCD_DISPLAYCONTROL | Options.DisplayControl);
}

/**
 * @brief  Print a string to the display.
 * @param  x The X position to start printing.
 * @param  y The Y position to start printing.
 * @param  str The string to print.
 */
void LcdPuts(uint8_t x, uint8_t y, char const *str)
{
	LcdCursorSet(x, y);
	while (*str)
    {
		if (Options.currentX >= _LCD_COLS)
       {
			Options.currentX = 0;
			Options.currentY++;
			LcdCursorSet(Options.currentX, Options.currentY);
		}
		if (*str == '\n')
        {
			Options.currentY++;
			LcdCursorSet(Options.currentX, Options.currentY);
		}
		else if (*str == '\r')
		{
			LcdCursorSet(0, Options.currentY);
		}
		else
		{
			LcdData(*str);
			Options.currentX++;
		}
		str++;
	}
}

/**
 * @brief  Turn blinking on.
 */
void LcdBlinkOn(void)
{
	Options.DisplayControl |= LCD_BLINKON;
	LcdCmd(LCD_DISPLAYCONTROL | Options.DisplayControl);
}

/**
 * @brief  Turn blinking off.
 */
void LcdBlinkOff(void)
{
	Options.DisplayControl &= ~LCD_BLINKON;
	LcdCmd(LCD_DISPLAYCONTROL | Options.DisplayControl);
}

/**
 * @brief  Turn the cursor on.
 */
void LcdCursorOn(void)
{
	Options.DisplayControl |= LCD_CURSORON;
	LcdCmd(LCD_DISPLAYCONTROL | Options.DisplayControl);
}

/**
 * @brief  Turn the cursor off.
 */
void LcdCursorOff(void)
{
	Options.DisplayControl &= ~LCD_CURSORON;
	LcdCmd(LCD_DISPLAYCONTROL | Options.DisplayControl);
}

/**
 * @brief  Scroll the display left.
 */
void LcdScrollLeft(void)
{
	LcdCmd(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
}

/**
 * @brief  Scroll the display right.
 */
void LcdScrollRight(void)
{
	LcdCmd(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
}

/**
 * @brief  Create a custom character.
 * @param  location The location of the custom character.
 * @param  data The data for the custom character.
 */
void LcdCreateChar(uint8_t location, uint8_t const *data)
{
	uint8_t i;
	/* We have 8 locations available for custom characters */
	location &= 0x07;
	LcdCmd(LCD_SETCGRAMADDR | (location << 3));
	
	for (i = 0; i < 8; i++) {
		LcdData(data[i]);
	}
}

/**
 * @brief  Print a custom character.
 * @param  x The X position to print the character.
 * @param  y The Y position to print the character.
 * @param  location The location of the custom character.
 */
void LcdPutCustom(uint8_t x, uint8_t y, uint8_t location)
{
	LcdCursorSet(x, y);
	LcdData(location);
}

/**
 * @brief  Print a single character.
 * @param  Data The character to print.
 */
void LcdPut(uint8_t data)
{
	LcdData(data);
}

/**
 * @brief  Send a command to the LCD.
 * @param  cmd The command to send.
 */
void LcdCmd(uint8_t cmd)
{
	LcdRstLow();
	LcdCmd4bit(cmd >> 4);
	LcdCmd4bit(cmd & 0x0F);
}

/**
 * @brief  Send a 4-bit command to the LCD.
 * @param  cmd The 4-bit command to send.
 */
void LcdCmd4bit(uint8_t cmd)
{
	HAL_GPIO_WritePin(Lcd7_GPIO_Port, Lcd7_Pin, (GPIO_PinState)(cmd & 0x08));
	HAL_GPIO_WritePin(Lcd6_GPIO_Port, Lcd6_Pin, (GPIO_PinState)(cmd & 0x04));
	HAL_GPIO_WritePin(Lcd5_GPIO_Port, Lcd5_Pin, (GPIO_PinState)(cmd & 0x02));
	HAL_GPIO_WritePin(Lcd4_GPIO_Port, Lcd4_Pin, (GPIO_PinState)(cmd & 0x01));
	LcdEnBlink();
}

/**
 * @brief  Send data to the LCD.
 * @param  data The data to send.
 */
void LcdData(uint8_t data)
{
	LcdRstHigh();
	LcdCmd4bit(data >> 4);
	LcdCmd4bit(data & 0x0F);
}

/**
 * @brief  Set the cursor position.
 * @param  col The column position.
 * @param  row The row position.
 */
void LcdCursorSet(uint8_t col, uint8_t row)
{
	uint8_t row_offsets[] = {0x00, 0x40, 0x14, 0x54};
	if (row >= _LCD_ROWS)
		row = 0;
	Options.currentX = col;
	Options.currentY = row;
	LcdCmd(LCD_SETDDRAMADDR | (col + row_offsets[row]));
}

/**
 * @brief  Get the width of the display.
 * @return The width of the display.
 */
int16_t LcdGetWidth(void) { return _LCD_COLS; }

/**
 * @brief  Get the height of the display.
 * @return The height of the display.
 */
int16_t LcdGetHeight(void) { return _LCD_ROWS; }

/**
 * @brief  Draw a bitmap on the display.
 * @param  x The X position to start drawing.
 * @param  y The Y position to start drawing.
 * @param  image The bitmap image to draw.
 * @param  mem8 The memory address to store the bitmap data.
 */
/*void LcddrawBitmap(char x, char y, const uint8_t *image, char mem8)
{
	CreateChar(mem8, image);
	CursorSet(x, y);
	Data(mem8);
}*/

