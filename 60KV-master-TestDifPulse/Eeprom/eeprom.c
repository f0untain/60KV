#include "eeprom.h"

uint8_t tempByteArray[4];

/**
 * @brief  Determines the number of bytes to write based on the size and offset.
 * @param  size The size of the data to write in bytes.
 * @param  offset The starting byte offset within the page.
 * @return The number of bytes to write.
 */
uint16_t CalculateBytesToWrite(uint16_t size, uint16_t offset)
{
    return ((size + offset) < EEPROM_PAGE_SIZE) ? size : (EEPROM_PAGE_SIZE - offset);
}

/**
 * @brief  Writes data to the EEPROM.
 * @param  page The starting page number (0 to EEPROM_TOTAL_PAGES-1).
 * @param  offset The starting byte offset within the page (0 to EEPROM_PAGE_SIZE-1).
 * @param  data Pointer to the data buffer to be written.
 * @param  size The size of the data in bytes.
 */
void WriteToEeprom(uint16_t page, uint16_t offset, uint8_t* data, uint16_t size)
{
    // Calculate the bit position where page addressing starts
    int pageAddressShift = log(EEPROM_PAGE_SIZE) / log(2);

    // Calculate the start and end page numbers
    uint16_t startPage = page;
    uint16_t endPage = page + ((size + offset) / EEPROM_PAGE_SIZE);

    // Number of pages to be written
    uint16_t numberOfPages = (endPage - startPage) + 1;
    uint16_t dataPosition = 0;

    // Write data to EEPROM
    for (int i = 0; i < numberOfPages; i++)
    {
        // Calculate memory address by combining page and byte addresses
        uint16_t memoryAddress = (startPage << pageAddressShift) | offset;
        uint16_t bytesRemaining = CalculateBytesToWrite(size, offset);

        HAL_I2C_Mem_Write(EEPROM_I2C_HANDLE, EEPROM_I2C_ADDRESS, memoryAddress, 2, &data[dataPosition], bytesRemaining, 1000);

        // Move to the next page
        startPage++;
        offset = 0;  // Reset offset for new page
        size -= bytesRemaining;  // Reduce size by bytes written
        dataPosition += bytesRemaining;  // Update data buffer position

        HAL_Delay(5);  // Write cycle delay (5 ms)
    }
}

/**
 * @brief  Converts a floating-point number to a byte array.
 * @param  byteArray Pointer to the array where the converted bytes will be stored.
 * @param  floatValue The floating-point value to convert.
 */
void FloatToByteArray(uint8_t* byteArray, float floatValue)
{
    union {
        float floatVal;
        uint8_t bytes[4];
    } floatUnion;

    floatUnion.floatVal = floatValue;

    for (uint8_t i = 0; i < 4; i++)
    {
        byteArray[i] = floatUnion.bytes[i];
    }
}

/**
 * @brief  Converts a byte array to a floating-point number.
 * @param  byteArray Pointer to the byte array containing the data.
 * @return The floating-point number obtained from the byte array.
 */
float ByteArrayToFloat(uint8_t* byteArray)
{
    union {
        float floatVal;
        uint8_t bytes[4];
    } floatUnion;

    for (uint8_t i = 0; i < 4; i++)
    {
        floatUnion.bytes[i] = byteArray[i];
    }

    return floatUnion.floatVal;
}

/**
 * @brief  Writes a floating-point number to the EEPROM.
 * @param  page The starting page number (0 to EEPROM_TOTAL_PAGES-1).
 * @param  offset The starting byte offset within the page (0 to EEPROM_PAGE_SIZE-1).
 * @param  data The floating-point number to be written.
 */
void WriteFloatToEeprom(uint16_t page, uint16_t offset, float data)
{
    FloatToByteArray(tempByteArray, data);
    WriteToEeprom(page, offset, tempByteArray, 4);
}

/**
 * @brief  Reads a floating-point number from the EEPROM.
 * @param  page The starting page number (0 to EEPROM_TOTAL_PAGES-1).
 * @param  offset The starting byte offset within the page (0 to EEPROM_PAGE_SIZE-1).
 * @return The floating-point number read from the EEPROM.
 */
float ReadFloatFromEeprom(uint16_t page, uint16_t offset)
{
    uint8_t buffer[4];
    ReadFromEeprom(page, offset, buffer, 4);
    return ByteArrayToFloat(buffer);
}

/**
 * @brief  Reads data from the EEPROM.
 * @param  page The starting page number (0 to EEPROM_TOTAL_PAGES-1).
 * @param  offset The starting byte offset within the page (0 to EEPROM_PAGE_SIZE-1).
 * @param  data Pointer to the buffer where the read data will be stored.
 * @param  size The number of bytes to read.
 */
void ReadFromEeprom(uint16_t page, uint16_t offset, uint8_t* data, uint16_t size)
{
    int pageAddressShift = log(EEPROM_PAGE_SIZE) / log(2);

    uint16_t startPage = page;
    uint16_t endPage = page + ((size + offset) / EEPROM_PAGE_SIZE);

    uint16_t numberOfPages = (endPage - startPage) + 1;
    uint16_t dataPosition = 0;

    for (int i = 0; i < numberOfPages; i++)
    {
        uint16_t memoryAddress = (startPage << pageAddressShift) | offset;
        uint16_t bytesRemaining = CalculateBytesToWrite(size, offset);
	    HAL_I2C_Mem_Read(EEPROM_I2C_HANDLE, EEPROM_I2C_ADDRESS, memoryAddress, 2, &data[dataPosition], bytesRemaining, 1000);

        startPage++;
        offset = 0;
        size -= bytesRemaining;
        dataPosition += bytesRemaining;
    }
}

/**
 * @brief  Erases a page in the EEPROM memory.
 * @param  page The page number to erase.
 * @note   To erase multiple pages, call this function in a loop.
 */
void EraseEepromPage(uint16_t page)
{
    // Calculate memory address based on the page number
    int pageAddressShift = log(EEPROM_PAGE_SIZE) / log(2);
    uint16_t memoryAddress = page << pageAddressShift;

    // Create a buffer filled with 0xFF to reset the EEPROM page
    uint8_t eraseBuffer[EEPROM_PAGE_SIZE];
    memset(eraseBuffer, 0xFF, EEPROM_PAGE_SIZE);

    // Write the erase buffer to the EEPROM
	HAL_I2C_Mem_Write(EEPROM_I2C_HANDLE, EEPROM_I2C_ADDRESS, memoryAddress, 2, eraseBuffer, EEPROM_PAGE_SIZE, 1000);

    HAL_Delay(5);  // Write cycle delay
}
