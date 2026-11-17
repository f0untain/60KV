#ifndef __EEPROM_H
#define __EEPROM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdbool.h>
#include "math.h"
#include "string.h"


/**
 * @brief  I2C handle declaration.
 */
extern I2C_HandleTypeDef hi2c1;

/**
 * @brief  I2C handle definition for EEPROM communication.
 */
#define EEPROM_I2C_HANDLE &hi2c1

/**
 * @brief  EEPROM I2C address (8 bits). The device address is 7 bits, with the MSB set to 0 for write operations.
 * @details A0 bit is used when the address pin is grounded.
 */
#define EEPROM_I2C_ADDRESS 0xA0

/**
 * @brief  Page size of the EEPROM in bytes.
 */
#define EEPROM_PAGE_SIZE 64  // Size in bytes

/**
 * @brief  Total number of pages in the EEPROM.
 */
#define EEPROM_TOTAL_PAGES 512  // Number of pages

/**
* @brief  Calculates the number of bytes to write based on the size and offset.
* @param  size The size of the data to write in bytes.
* @param  offset The offset within the page where writing begins.
* @return The number of bytes to write.
*/
uint16_t CalculateBytesToWrite(uint16_t size, uint16_t offset);

/**
	* @brief  Writes data to the EEPROM.
	* @param  page The page number to write to.
	* @param  offset The offset within the page to start writing.
	* @param  data Pointer to the data buffer to be written.
	* @param  size The size of the data in bytes.
	*/
void WriteToEeprom(uint16_t page, uint16_t offset, uint8_t* data, uint16_t size);

/**
	* @brief  Converts a floating-point number to a byte array.
	* @param  byte_array Pointer to the array where the converted bytes will be stored.
	* @param  float_value The floating-point value to convert.
	*/
void FloatToByteArray(uint8_t* byte_array, float float_value);

/**
	* @brief  Converts a byte array to a floating-point number.
	* @param  byte_array Pointer to the byte array containing the data.
	* @return The floating-point number obtained from the byte array.
	*/
float ByteArrayToFloat(uint8_t* byte_array);

/**
	* @brief  Writes a floating-point number to the EEPROM.
	* @param  page The page number to write to.
	* @param  offset The offset within the page to start writing.
	* @param  data The floating-point number to be written.
	*/
void WriteFloatToEeprom(uint16_t page, uint16_t offset, float data);

/**
	* @brief  Reads a floating-point number from the EEPROM.
	* @param  page The page number to read from.
	* @param  offset The offset within the page to start reading.
	* @return The floating-point number read from the EEPROM.
	*/
float ReadFloatFromEeprom(uint16_t page, uint16_t offset);

/**
	* @brief  Reads data from the EEPROM.
	* @param  page The page number to read from.
	* @param  offset The offset within the page to start reading.
	* @param  data Pointer to the buffer where the read data will be stored.
	* @param  size The number of bytes to read.
	*/
void ReadFromEeprom(uint16_t page, uint16_t offset, uint8_t* data, uint16_t size);

/**
	* @brief  Erases a page in the EEPROM.
	* @param  page The page number to erase.
	*/
void EraseEepromPage(uint16_t page);


#ifdef __cplusplus
}
#endif

#endif /* __EEPROM_H */
