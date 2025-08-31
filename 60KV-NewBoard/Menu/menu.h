#ifndef __MENU_H_
#define __MENU_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include "button.h"
#include "lcd.h"
#include "eeprom.h"


// Macro definitions
#define NUMBER_OF_VARIABLES			5
#define NUMBER_OF_BLD_VARIABLES		6

#define LCD_LINE_RECENT		4  /*Represent recent cursor line*/
#define LCD_LINE1			0
#define LCD_LINE2			1
#define LCD_LINE3			2
#define LCD_LINE4			3

#define LCD_DownLine		0
#define LCD_RightLine		1
#define LCD_UpLine			2
#define LCD_LeftLine		3

#define LCD_Mem0			0
#define LCD_Mem1			1
#define LCD_Mem2			2
#define LCD_Mem3			3
#define LCD_Mem4			4
#define LCD_Mem5			5
#define LCD_Mem6		    6
#define LCD_Mem7			7

#define LCD_X               15

#define LCD_PROGRESS_BAR_LINE  100

//Variable type to show in LCD
#define ET_FLT				1
#define ET_DBL				2
#define ET_INT				3
#define ET_FSTR				4 /* string saved in flash */
#define ET_ULNG				5
#define ET_NULL				6 // Nothing to display

// Menu state machine states
#define ST_STANDBY                      20

/*----------------------------------------------*/
#define ST_MAIN_STATE					30
/*
\_______________  ________________________/
 				\/								*/
#define VA_FIRST_VARIABLE               31
#define VA_SECOND_VARIABLE		        32
#define VA_THIRD_VARIABLE			    33
#define VA_FORTH_VARIABLE			    34
#define VA_FIFTH_VARIABLE			    35

/*----------------------------------------------*/
#define ST_BLD_INFO_STATE				40		/*
\_______________  ________________________/
 				\/								*/
#define VA_BLD_FIRST_VARIABLE           41
#define VA_BLD_SECOND_VARIABLE			42
#define VA_BLD_THIRD_VARIABLE			43
#define VA_BLD_FORTH_VARIABLE           44
#define VA_BLD_FIFTH_VARIABLE           45
#define VA_BLD_SIXTH_VARIABLE           46


#define ST_INT_ADJUST					50

#define ST_FLT_ADJUST                   60

#define ST_STR_CHANGE					70

#define ST_CHANGE_MENU			    	80

#define ST_ERRORS						90

#define ST_CONTACT_US				    100

#define ST_VOLTAGE_SHOW					110

#define VA_FREQUENCY		    VA_FIRST_VARIABLE
#define VA_TIME_ON			    VA_SECOND_VARIABLE
#define VA_DC_VOLTAGE			VA_THIRD_VARIABLE
#define VA_KV_VOLTAGE		    VA_FORTH_VARIABLE
#define CONTACT_US		        VA_FIFTH_VARIABLE

#define VA_BLD_OFFSET			VA_BLD_FIRST_VARIABLE
#define VA_BLD_CALIBRATION		VA_BLD_SECOND_VARIABLE
#define VA_BLD_DIVIDER			VA_BLD_THIRD_VARIABLE
#define VA_BLD_RATIO		    VA_BLD_FORTH_VARIABLE
#define VA_BLD_TON_PRE		    VA_BLD_FIFTH_VARIABLE
#define VA_BLD_DELAY		    VA_BLD_SIXTH_VARIABLE

//On/off LEDs commands
#define Led1(x) HAL_GPIO_WritePin(Led1_GPIO_Port, Led1_Pin, x)
#define Led2(x) HAL_GPIO_WritePin(Led2_GPIO_Port, Led2_Pin, x)
#define Led3(x) HAL_GPIO_WritePin(Led3_GPIO_Port, Led3_Pin, x)
#define Led4(x) HAL_GPIO_WritePin(Led4_GPIO_Port, Led4_Pin, x)
#define Buzzer(x) HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, x)

//variables
extern uint16_t errorVar, warningVar; // Each bit of this variable corresponds to an error state.
extern uint8_t cursorLine; // holds the cursor line, according to "menu.h"

extern int TimeOn;
extern int Frequency;
extern float AdcVoltage;
/**
 * @brief This variables use to calculate frequency and width of PWM
 */
#define Freq_CLK    72000000            //72 MHz
#define PWM_PSC     72
#define PWM_CLK     Freq_CLK/(PWM_PSC)  //1MHz / 100,000 -> 10 Hz

#define ADC_VOLTAGE_REFRENCE  3.000f
#define ADC_STEP 		      4095.000f
/**
 * @brief This variables for create delay with timer
 */
extern volatile bool	flagRefreshValue500ms,
	 			        flcd100ms,
	 			        f2lcd100ms,
	 			        flcd500ms,
						flag1ms,
						flag1s,
						flag1sStart,
						flagCapCh,
						flagRythm,
						flagRampStart,
						flagStopPwm;

extern TIM_HandleTypeDef htim2;
//#define PwmCh   TIM_CHANNEL_1

extern ADC_HandleTypeDef hadc2;

/* Array to determine that the buzzer need to toggle.
 * this array consists of buzzer[0] = number of toggles
 *					  and buzzer[1] = delay between each toggle.
 * this array must be initialized when need to buzzer toggled.
 */
extern volatile uint8_t buzzer[2];
#define BUZZ_NUM     0
#define BUZZ_DELAY   1

typedef struct {
	union {
		float		fval;
		const char*		pText[4]; /* 4: maximum number of states; ie "non" ,"LOW", "MED" and "HIGH". */
		int			ival;
		//uint32_t	ulval;
	};
	int8_t strNum; /* variable that determine witch string should be displayed.
		 			* using int8_t insteed of uint8_t because if strNum == 0 and strNum--
		 			* strNum ==-1 not 255. (to flow control)
		 			*/
	float downLimit; /* These variables store maximum and minimum of variable or strNum.*/
	float upLimit;
} ValueStruct;

typedef struct /*PROGMEM*/
{
	uint8_t variable;
	uint8_t input;
	uint8_t nextVariable;
	uint8_t cursorLine; // LCD Line which cursor is pointed to it.
} STATE_NEXTVARIABLE;

typedef struct /*PROGMEM*/
{
	uint8_t variable;
	/*PGM_P*/ const char* pText;
	uint8_t adjustState;
	ValueStruct *pValueStruct;
	uint8_t elementType;
	uint8_t IndexMenu; //follow the menu and sub menus

} STATE_VARIABLE;

typedef struct /*PROGMEM*/
{
	unsigned char state;
	/*PGM_P*/ const char* pText;
	char(*pFunc)(char input);
} MENU_STATE;

typedef enum {
	STANDBY,
	RUN,
	STOP,
} mode_state_t;

typedef enum {
	SHORTCIRCUIT,
	HV,
	IDLE,
} error_state_t;

extern mode_state_t mode_state;

void SetErrorState(error_state_t errorstate);

error_state_t GetErrorState(void);

void LedUpdateBlink(void);

void ReadVoltageADC(void);

/**
 * @brief This function change frequency of your PWM.
 * @param frequency Is period of the PWM.
 */
void SetFrequency(int frequency);

/**
 * @brief This function change Ton/Width of the PWM.
 * @param Ton Is width that set for PWM.
 * @param TonPre Is width that set for PrePWM.
 */
void SetDutyCycle_us(int Ton, int TonPre, int Delay);

void StartPWMs(void);

void StopPWMs(void);

/**
 * @brief It draws lines in lcd.
 */
void HLine(char x, char y, uint8_t KindofLine, char mem8);

/**
 * @brief It runs when the user select contact us menu
 * @param input is buttons state after debouncig.
 * @return nextState is next state according to the current state and button input.
 */
char ContactUs(char input);

/**
 * @brief It shows weld animation.
 */
void WeldDisplay(void);

/**
 * @brief This function displays LCD texts.
 * @note This function displays LCD texts each 500 mS or if when a change in data occurs.
 */
void DisplayRunMenu(void);

/**
 * @brief This function shifts between the different variables.
 * @param stimuli is Button input.
 * @return nextState is next state according to the current state and button input.
 */
unsigned char VariableMachine(unsigned char stimuli);

/**
 * @brief This function run when is in standby mode.
 * @param input is buttons state after debouncig.
 * @return nextState is next state according to the current state and button input.
 */
char StandBy(char input);

/**
 * @brief This function run when is in menue or submenu untill one of the values choose.
 * @param input is buttons state after debouncig.
 * @return nextState is next state according to the current state and button input.
 */
char MainMenu(char input);

/**
 * @brief This function adjusts integer values.
 * @param input is buttons state after debouncig.
 * @return nextState is next state according to the current state and button input.
 */
char IntAdjust(char input);

/**
 * @brief This function adjusts float values.
 * @param input is buttons state after debouncig.
 * @return nextState is next state according to the current state and button input.
 */
char FloatAdjust(char input);

/**
 * @brief This function adjusts string values.
 * @param input is buttons state after debouncig.
 * @return nextState is next state according to the current state and button input.
 */
char StringChange(char input);

/**
 * @brief Writes a single ValueStruct to the EEPROM.
 * @param page The page number (0 to EEPROM_TOTAL_PAGES-1).
 * @param offset The starting byte offset within the page (0 to EEPROM_PAGE_SIZE-1).
 * @param data Pointer to the ValueStruct to be written.
 */
void WriteValueStructToEeprom(uint16_t page, uint16_t offset, ValueStruct* data);

/**
 * @brief Reads a single ValueStruct from the EEPROM.
 * @param page The page number (0 to EEPROM_TOTAL_PAGES-1).
 * @param offset The starting byte offset within the page (0 to EEPROM_PAGE_SIZE-1).
 * @param data Pointer to the ValueStruct where the read data will be stored.
 */
void ReadValueStructFromEeprom(uint16_t page, uint16_t offset, ValueStruct* data);

/**
 * @brief Writes an array of ValueStruct to the EEPROM.
 * @param startPage The starting page number (0 to EEPROM_TOTAL_PAGES-1).
 * @param data Pointer to the array of ValueStruct to be written.
 * @param arraySize The number of ValueStruct elements in the array.
 */
void WriteValueStructArrayToEeprom(uint16_t startPage, ValueStruct* data, uint16_t arraySize);

/**
 * @brief Reads an array of ValueStruct from the EEPROM.
 * @param startPage The starting page number (0 to EEPROM_TOTAL_PAGES-1).
 * @param data Pointer to the array where the read ValueStruct elements will be stored.
 * @param arraySize The number of ValueStruct elements to read.
 */
void ReadValueStructArrayFromEeprom(uint16_t startPage, ValueStruct* data, uint16_t arraySize);

/**
 * @brief Write init values once in every external eeprom.
 */
void InitValueWriteToEeprom(void);

/**
 * @brief read init values from external eeprom.
 */
void InitValueReadFromEeprom(void);

/**
 * @brief Converts ADC final value of a NTC resistor table to its equivalent Temperature.
 * @param AdcValue Current ADC value of a NTC.
 * @return temperature Equivalent Temperature from the NTC resistor table.
 */
float NtcTemp(unsigned long int AdcValue);

char Warnings(void);
char ErrorHandler(char input);

#ifdef __cplusplus
}
#endif

#endif   /* __MENU_H */
