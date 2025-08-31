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
#define NUMBER_OF_VARIABLES			7/*8*/
	//#define NUMBER_OF_MAIN_VARIABLES    7

	/*#define ALFA_PAGE_ADD	   10
	#define BETA_PAGE_ADD	   20
	#define TETA_PAGE_ADD	   30
	#define DELTA_PAGE_ADD     40
	#define GAMA_PAGE_ADD      50
	#define CUSTOM_PAGE_ADD    60*/

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

#define LCD_PROGRESS_BAR_LINE  100

	//Variable type to show in LCD
#define ET_FLT				1
#define ET_DBL				2
#define ET_INT				3
#define ET_FSTR				4 /* string saved in flash */
#define ET_ULNG				5
#define ET_NULL				6 // Nothing to display

	// Voltage Level
#define VOL_LEVEL_NONE	        0
#define VOL_LEVEL_LOW	    	1
#define VOL_LEVEL_MED	        2
#define VOL_LEVEL_HIGH	        3

	// Menu state machine states
#define ST_STANDBY                      20

	/*----------------------------------------------*/
#define ST_MAIN_STATE					30
	/*
	\_______________  ________________________/
	 				\/								*/
#define VA_FIRST_VARIABLE               1
#define VA_SECOND_VARIABLE		        2
#define VA_THIRD_VARIABLE			    3
#define VA_FOURTH_VARIABLE              4
#define VA_FIFTH_VARIABLE               5
#define VA_SIXTH_VARIABLE               6
#define VA_SEVENTH_VARIABLE             7
#define VA_EIGHTH_VARIABLE              8
	 				/*#define VA_NINTH_VARIABLE               9
	 				#define VA_TENTH_VARIABLE               10
	 				#define VA_ELEVENTH_VARIABLE            11
	 				#define VA_TWELFTH_VARIABLE             12
	 				#define VA_THIRTEENTH_VARIABLE          13*/
	 				/*----------------------------------------------*/

#define ST_INT_ADJUST					50

#define ST_FLT_ADJUST                   60

#define ST_STR_CHANGE					70

#define ST_CHANGE_MENU				    80

#define ST_ERRORS						90

#define ST_CONTACT_US				    100

#define VA_VOLTAGE			    VA_FIRST_VARIABLE
#define VA_FREQUENCY			VA_SECOND_VARIABLE
#define VA_TIME				    VA_THIRD_VARIABLE
#define VA_RYTHM_ON			    VA_FOURTH_VARIABLE
#define VA_RYTHM_OFF			VA_FIFTH_VARIABLE
#define VA_RUN		            VA_SIXTH_VARIABLE
#define CONTACT_US		        VA_SEVENTH_VARIABLE
	 				/*#define VA_RAMP			        VA_FOURTH_VARIABLE
	 				#define VA_RYTHM_ON			    VA_FIFTH_VARIABLE
	 				#define VA_RYTHM_OFF		    VA_SIXTH_VARIABLE
	 				#define VA_RUN		            VA_SEVENTH_VARIABLE
	 				#define CONTACT_US			    VA_EIGHTH_VARIABLE*/

	 				//On/off LEDs commands
#define Led1(x) HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, x)
#define Led2(x) HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, x)
#define Led3(x) HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, x)
#define Led4(x) HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, x)
#define Buzzer(x) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, x)

#define RealyLow(x) HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, x)
#define RealyMed(x) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, x)
#define RealyHigh(x) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, x)

	 				//variables
	extern char enterFunction;
	extern uint16_t errorVar, warningVar; // Each bit of this variable corresponds to an error state.
	extern uint8_t cursorLine; // holds the cursor line, according to "menu.h"
	extern int Ramp;
	extern float TimeOn;
	extern int Frequency;
	extern int RythmOn;
	extern int RythmOff;
	extern int RunTime;
	extern uint16_t NumberOfRampPulse;
	extern float StepRamp;
	extern int8_t Intensity;
	extern uint8_t Count1sStart;
	extern uint16_t CountRunTime;
	extern float CountRamp;
	extern uint16_t PeriodCount;
	extern bool PwmActive;

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

	/**
	 * @brief This variables use to calculate frequency and width of PWM
	 */
#define Freq_CLK    72000000            //72 MHz
#define PWM_PSC     7200                //7200
#define PWM_CLK     Freq_CLK/(PWM_PSC)  //10 KHz

	 /**
	  * @brief This variables for create delay with timer
	  */
	extern volatile bool	flagRefreshValue500ms,
		 			        flcd100ms,
		 			        f2lcd100ms,
		 			        flcd500ms,
							flag1ms,
							flag1sDly,
							flagCapCh,
							flagRythm,
							flagRampStart,
							flagStopPwm;

	extern TIM_HandleTypeDef htim3;
#define PwmCh   TIM_CHANNEL_3

	/* Array to determine that the buzzer need to toggle.
	 * this array consists of buzzer[0] = number of toggles
	 *					  and buzzer[1] = delay between each toggle.
	 * this array must be initialized when need to buzzer toggled.
	 */
	extern volatile uint8_t buzzer[2];
#define BUZZ_NUM     0
#define BUZZ_DELAY   1

	/**
	 * @brief This function change frequency of your PWM.
	 * @param frequency Is period of the PWM.
	 */
	void SetFrequency(int frequency);

	/**
	 * @brief This function change Ton/Width of the PWM.
	 * @param Ton Is width that set for PWM.
	 */
	void SetDutyCycle(float Ton);

	/**
	 * @brief Start and stop the PWM with ramp.
	 */
	void SetRamp(void);

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
