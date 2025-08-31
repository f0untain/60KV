#include <menu.h>

//Variables
//char enterFunction = 1;
uint8_t variable;
uint8_t cursorLine; // holds the cursor line, according to "menu.h"
const char* statetext[4]; // Holds 4 const char pointer to display in lines of LCD.
uint8_t elementType[4]; // Holds 4 types for values of each variable that display in lines of LCD.
uint8_t reWarning = 1; //for exhibition of warnings and DisplayRunMenu.
uint8_t refreshLcd; //for refreshing LCD.
uint16_t errorVar = 0x00, warningVar = 0x00; // Each bit of this variable corresponds to an error state.
static char count1;
uint8_t led_blink_timeout = 0;
bool blink_enable =false;

//Set PWM's variable set
int Frequency;
int TimeOn;
int AdcOffset;
float AdcCalib;
int VolDivider;
int VolRatio;
int TimeOnPre;
int PhaseDelay;
uint32_t count1ms = 0;
bool one_time_excute = true;

mode_state_t  mode_state = STANDBY;
error_state_t  error_state = IDLE;

unsigned char const UpDownCharacter[8] = { 0x04, 0x0E, 0x1F, 0x00, 0x00, 0x1F, 0X0E, 0x04 }; // "<>" character
uint8_t const arcSymbol[5][8] = {
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x18 },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x10, 0x1C },
	{ 0x00, 0x00, 0x00, 0x00, 0x12, 0x14, 0x18, 0x1E },
	{ 0x00, 0x00, 0x08, 0x11, 0x12, 0x14, 0x19, 0x1E },
	{ 0x00, 0x00, 0x08, 0x11, 0x12, 0x14, 0x19, 0x1E }
};
uint8_t const Line[4][8] = {
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F },
	// line down
	{ 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10 },
	// line right
	{ 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
	// line up
	{ 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01 }
	// line left
};
uint8_t const BarGraph[6][8] = {
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
	{ 0x00, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10 },
	{ 0x00, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18 },
	{ 0x00, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C },
	{ 0x00, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E },
	{ 0x00, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F }
};

const char NON[]	= "None";
const char LOW[]	= "LOW ";
const char MED[] 	= "MED ";
const char HIGH[]   = "HIGH";

/*! \USER MENU */
const char MT_FIRST_VARIABLE[]	  = " 1.Freq(HZ)";
const char MT_SECOND_VARIABLE[]	  = " 2.Ton(uS)";
const char MT_THIRD_VARIABLE[]    = " 3.V-DC(V)";
const char MT_FORTH_VARIABLE[]    = " 4.V-HV(KV)";
const char MT_FIFTH_VARIABLE[]    = " 5.Contact Us";

/*! \BUILDER MENU */
const char MT_BLD_FIRST_VARIABLE[]	 = " 6.Offset";
const char MT_BLD_SECOND_VARIABLE[]	 = " 7.CALIB.";
const char MT_BLD_THIRD_VARIABLE[]	 = " 8.DIV.";
const char MT_BLD_FORTH_VARIABLE[]	 = " 9.Ratio";
const char MT_BLD_FIFTH_VARIABLE[]	 = " 10.Ton-Pre";
const char MT_BLD_SIXTH_VARIABLE[]	 = " 11.delay";

ValueStruct *pValues[4]; // Holds 4 ValueStruct pointer that point to a value for each variable.
ValueStruct values[NUMBER_OF_VARIABLES + 1];
ValueStruct bldValues[ NUMBER_OF_BLD_VARIABLES+1 ];

// Initial value for sub menu 1
ValueStruct EEValues[ NUMBER_OF_VARIABLES+1 ] = {
//	VARIABLE_VALUE						STRING_NUMBER				DOWN_LIMIT					  UP_LIMIT
	{.pText = {NULL, NULL, NULL, NULL},	.strNum = 0,				.downLimit = 0,				  .upLimit = 0},         /* This line is added for correction of difference between variable numbers(one based) and values arrays(zero based).*/
	{.ival = 10,						.strNum = 20,			    .downLimit = 1,	              .upLimit = 100},		 /* Frq*/
	{.ival = 10,						.strNum = 20,				.downLimit = 5,			      .upLimit = 30},	     /* time-us*/
	{.ival = 0,							.strNum = 20,				.downLimit = 0,				  .upLimit = 0},         /* DC voltage*/
	{.fval = 0.00f,						.strNum = 10,				.downLimit = 0,				  .upLimit = 0},         /* High voltage*/

};

ValueStruct EEBldValues[ NUMBER_OF_BLD_VARIABLES+1 ] = {
	{.pText = {NULL, NULL, NULL, NULL},	.strNum = 0,				.downLimit = 0,				  .upLimit = 0},         /* This line is added for correction of difference between variable numbers(one based) and values arrays(zero based).*/
	{.ival = 0,							.strNum = 20,				.downLimit = 0,				  .upLimit = 50},        /* ADC offset*/
	{.ival = 1,							.strNum = 20,				.downLimit = 1,				  .upLimit = 99},        /* ADC calibration*/
	{.ival = 260,						.strNum = 20,				.downLimit = 200,			  .upLimit = 400},       /* Divider*/
	{.ival = 60,						.strNum = 20,				.downLimit = 50,			  .upLimit = 200},       /* Ratio*/
	{.ival = 40,						.strNum = 20,				.downLimit = 1,			  	  .upLimit = 500},       /* PrePulseTon*/
	{.ival = 5,							.strNum = 20,				.downLimit = 1,			  	  .upLimit = 500},       /* PhaseDelay*/
};
	
STATE_NEXTVARIABLE StateNextVariable[21] = {
	//	VARIABLE		   		INPUT		    	NEXT VARIABLE				CURSOR LINE
	{ VA_FIRST_VARIABLE,   		ROTARY_CW_DIR,   	VA_SECOND_VARIABLE,  		LCD_LINE2 },

	{ VA_SECOND_VARIABLE,  		ROTARY_CW_DIR,   	VA_THIRD_VARIABLE,   		LCD_LINE3 },
	{ VA_SECOND_VARIABLE,  		ROTARY_CCW_DIR,  	VA_FIRST_VARIABLE,   		LCD_LINE1 },

	{ VA_THIRD_VARIABLE,   		ROTARY_CW_DIR,   	VA_FORTH_VARIABLE,   		LCD_LINE3 },
	{ VA_THIRD_VARIABLE,   		ROTARY_CCW_DIR,  	VA_SECOND_VARIABLE,  		LCD_LINE2 },

	{ VA_FORTH_VARIABLE,   		ROTARY_CW_DIR,   	VA_FIFTH_VARIABLE,   		LCD_LINE4 },
	{ VA_FORTH_VARIABLE,   		ROTARY_CCW_DIR,  	VA_THIRD_VARIABLE,   		LCD_LINE3 },

	{ VA_FIFTH_VARIABLE,   		KEY_BACK,   		VA_BLD_FIRST_VARIABLE,  	LCD_LINE4 },
	{ VA_FIFTH_VARIABLE,   		ROTARY_CCW_DIR,  	VA_FORTH_VARIABLE,   		LCD_LINE3 },
	 /*******************************	Builder menu	*******************************/
	{VA_BLD_FIRST_VARIABLE,		ROTARY_CW_DIR,		VA_BLD_SECOND_VARIABLE,		LCD_LINE4},
	{VA_BLD_FIRST_VARIABLE,		ROTARY_CCW_DIR,		VA_FIFTH_VARIABLE,			LCD_LINE3},

	{VA_BLD_SECOND_VARIABLE,	ROTARY_CW_DIR,		VA_BLD_THIRD_VARIABLE,		LCD_LINE4},
	{VA_BLD_SECOND_VARIABLE,	ROTARY_CCW_DIR,		VA_BLD_FIRST_VARIABLE,		LCD_LINE3},

	{VA_BLD_THIRD_VARIABLE,		ROTARY_CW_DIR,		VA_BLD_FORTH_VARIABLE,		LCD_LINE4},
	{VA_BLD_THIRD_VARIABLE,		ROTARY_CCW_DIR,		VA_BLD_SECOND_VARIABLE,		LCD_LINE3},

	{VA_BLD_FORTH_VARIABLE,		ROTARY_CW_DIR,		VA_BLD_FIFTH_VARIABLE,		LCD_LINE4},
	{VA_BLD_FORTH_VARIABLE,		ROTARY_CCW_DIR,		VA_BLD_THIRD_VARIABLE,		LCD_LINE3},

	{VA_BLD_FIFTH_VARIABLE,		ROTARY_CW_DIR,		VA_BLD_SIXTH_VARIABLE,		LCD_LINE4},
	{VA_BLD_FIFTH_VARIABLE,		ROTARY_CCW_DIR,		VA_BLD_FORTH_VARIABLE,		LCD_LINE3},

	{VA_BLD_SIXTH_VARIABLE,		ROTARY_CCW_DIR,		VA_BLD_FIFTH_VARIABLE,		LCD_LINE3},
	{ 0,                   		0,               	0,                   			0 }
};

STATE_VARIABLE StateVariable[13] = {
	//	VARIABLE      VARIABLE TEXT		    			ADJUST_STATE     	P_VALUE_STR   	Element TYPE
	{ VA_FREQUENCY,   		MT_FIRST_VARIABLE,    		ST_INT_ADJUST,   	&values[1],   	ET_INT},
	{ VA_TIME_ON,     		MT_SECOND_VARIABLE,   		ST_INT_ADJUST,   	&values[2],   	ET_INT},
	{ VA_DC_VOLTAGE,  		MT_THIRD_VARIABLE,    		ST_VOLTAGE_SHOW, 	&values[3],   	ET_INT},
	{ VA_KV_VOLTAGE,  		MT_FORTH_VARIABLE,    		ST_VOLTAGE_SHOW, 	&values[4],   	ET_FLT},
	{ CONTACT_US,     		MT_FIFTH_VARIABLE,    		ST_CONTACT_US,   	&values[5],   	ET_NULL},
	/*******************************	Builder menu	*******************************/
	{VA_BLD_OFFSET,			MT_BLD_FIRST_VARIABLE,		ST_INT_ADJUST,  	&bldValues[1],	ET_INT},
	{VA_BLD_CALIBRATION,	MT_BLD_SECOND_VARIABLE,		ST_INT_ADJUST,		&bldValues[2],	ET_INT},
	{VA_BLD_DIVIDER,		MT_BLD_THIRD_VARIABLE,		ST_INT_ADJUST,		&bldValues[3],	ET_INT},
	{VA_BLD_RATIO,			MT_BLD_FORTH_VARIABLE,		ST_INT_ADJUST,		&bldValues[4],	ET_INT},
	{VA_BLD_TON_PRE,		MT_BLD_FIFTH_VARIABLE,		ST_INT_ADJUST,		&bldValues[5],	ET_INT},
	{VA_BLD_DELAY,			MT_BLD_SIXTH_VARIABLE,		ST_INT_ADJUST,		&bldValues[6],	ET_INT},

	{ 0,              		NULL,                 		0,               	NULL,           0     }
};

static uint8_t nextVariable = VA_FIRST_VARIABLE;

void SetErrorState(error_state_t errorstate)
{
	error_state = errorstate;
}

error_state_t GetErrorState(void)
{
	return error_state;
}

void LedUpdateBlink(void)
{
	if (led_blink_timeout > 0)
	{
		if (error_state == SHORTCIRCUIT)
			HAL_GPIO_TogglePin(Led3_GPIO_Port, Led3_Pin);
		else if (error_state == HV)
			HAL_GPIO_TogglePin(Led4_GPIO_Port, Led4_Pin);
		led_blink_timeout--;
		if (led_blink_timeout == 0)
		{
			if (error_state == SHORTCIRCUIT)
				Led3(1);
			else if (error_state == HV)
				Led4(1);
			blink_enable = false;
			//Led3(0);
			//Led4(0);
			error_state = IDLE;
			one_time_excute = true;
		}
	}
}

void ReadVoltageADC(void)
{
	HAL_ADC_Start(&hadc2);
	if (HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY) == HAL_OK)
	{
		uint32_t adc_value = HAL_ADC_GetValue(&hadc2);
		float voltage = (((float)adc_value - AdcOffset) * ADC_VOLTAGE_REFRENCE * AdcCalib) / ADC_STEP;
		//values[3].ival = voltage;
		values[3].ival = (int)(VolDivider*voltage);
		values[4].fval = (float)((VolRatio*VolDivider*voltage)/1000.00);
		//values[4].fval = voltage;
		if(values[3].ival >= 820)
		{
			error_state = HV;
		}
	}
	HAL_ADC_Stop(&hadc2);
}

/**
 * @brief This function change frequency of your PWM.
 * @param frequency Is period of the PWM.
 */
void SetFrequency(int frequency)
{
	/* Frequency_PWM = Frequency_CLK/((APRx+1)*(PSCx+1))
	 * Frequency_PWM = It is the input of function that user want to set
	 * Frequency_CLK = 72 MHz
	 * APRx = Auto-reload register
	 * PSCx = The value of prescaler */
	volatile static uint32_t PWM_APR;
	PWM_APR = (PWM_CLK/frequency) /*- 1*/ ;
	TIM2->ARR = PWM_APR;
}

/**
 * @brief This function change Ton/Width of the PWM.
 * @param Ton Is width that set for PWM.
 */
void SetDutyCycle_us(int Ton)
{
	// If using Channel 1 change it to CCR1
	//TIM2->CCR1 = Ton;
	TIM2->CCR1 = TimeOnPre;
	TIM2->CCR2 = TIM2->CCR1 + PhaseDelay;
	TIM2->CCR3 = Ton + TIM2->CCR2;
}

/**
 * @brief It draws lines in lcd.
 */
void HLine(char x, char y, uint8_t KindofLine, char mem8)
{
	LcdCreateChar(mem8, Line[KindofLine]);
	LcdCursorSet(x, y);
	LcdData(mem8);
}

/**
 * @brief It runs when the user select contact us menu
 * @param input is buttons state after debouncig.
 * @return nextState is next state according to the current state and button input.
 */
char ContactUs(char input)
{
	static uint8_t enterFunction = 1;
	if (enterFunction == 1)
	{
		LcdClear();
		LcdPuts(1, LCD_LINE1, "Tel:021-66735875");
		LcdPuts(1, LCD_LINE2, "Tel:09191143937");
		LcdPuts(1, LCD_LINE3, "website:");
		LcdPuts(2, LCD_LINE4, "Nekoorad.com");
	}
	enterFunction = 0;
	if (input == KEY_STANDBY)
	{
		//enterFunction = 1;
		//return ST_STANDBY;

		count1 = 19;
		nextVariable = VA_FIRST_VARIABLE;
		cursorLine = LCD_LINE1;
		//variable = VA_SECOND_VARIABLE;
		variable = ST_CHANGE_MENU;
		return ST_STANDBY;
	}
	else if (input == KEY_ACCEPT || input == KEY_BACK)
	{
		refreshLcd = true; // atention to DisplyRunMenu	function
		variable = ST_CHANGE_MENU;
		enterFunction = 1;
		return ST_MAIN_STATE;
	}

	return ST_CONTACT_US;
}

/**
 * @brief It shows weld animation.
 */
void WeldDisplay(void)
{
	uint8_t ArcNumber = 0;
	while (ArcNumber < 5)
	{
		if (flcd100ms)
		{
			LcdCreateChar(LCD_Mem7, arcSymbol[ArcNumber]);
			LcdCursorSet(10, LCD_LINE1);
			LcdData(LCD_Mem7);
			ArcNumber++;
			flcd100ms = false;
		}

	}
}

/**
 * @brief This function displays LCD texts.
 * @note This function displays LCD texts each 500 mS or if when a change in data occurs.
 */
void DisplayRunMenu(void)
{
	char lcdBuffer[12];
	if ( flcd500ms == true )
	{

		for (uint8_t n = 0; n < 4; n++)
		{
			// refresh
			LcdPuts(1, n, statetext[n]);

			// Display values
			switch (elementType[n])
			{
			case ET_FLT:	sprintf(lcdBuffer, "%0.2f", pValues[n]->fval);
			                LcdPuts(LCD_X, n, lcdBuffer);
				            break;

			case ET_INT:	sprintf(lcdBuffer, "%-3d", pValues[n]->ival);
			                LcdPuts(LCD_X, n, lcdBuffer);
				            break;

			case ET_FSTR:	LcdPuts(LCD_X, n, pValues[n]->pText[pValues[n]->strNum]);
				            break;
			}
		}
		flcd500ms = false;
	}
	// Plain menu text
	if (refreshLcd == true)
	{
		//LcdEnHigh;
		LcdClear();
		LcdCursorSet(0, cursorLine);
		LcdData(0x7E); //  "->" character

		for (uint8_t n = 0; n < 4; n++)
		{
			// refresh 
			LcdPuts(1, n, statetext[n]);

			/* Display values */
			switch (elementType[n])
			{
			case ET_FLT:	sprintf(lcdBuffer, "%0.2f", pValues[n]->fval);
			                LcdPuts(LCD_X, n, lcdBuffer);
				            break;

			case ET_INT:	sprintf(lcdBuffer, "%-3d", pValues[n]->ival);
			                LcdPuts(LCD_X, n, lcdBuffer);
				            break;

			case ET_FSTR:	LcdPuts(LCD_X, n, pValues[n]->pText[pValues[n]->strNum]);
				            break;

				/*case ET_ULNG:	sprintf(lcdBuffer, "%lu", pValues[n]->ulval);
				                LCD_Puts(LCD_X,n, lcdBuffer);
								break;*/
			}
		}
		refreshLcd = false;
	}
}

/**
 * @brief This function shifts between the different variables.
 * @param stimuli is Button input.
 * @return nextState is next state according to the current state and button input.
 */
unsigned char VariableMachine(unsigned char stimuli)
{
	unsigned char nextVariable = variable; // Default stay in same state
	unsigned char i, j;
	uint8_t cursorBuffer;

	for (i = 0; (j = StateNextVariable[i].variable); i++)
	{
		if (j == variable &&
				StateNextVariable[i].input == stimuli)

		{
			nextVariable = StateNextVariable[i].nextVariable;
			cursorBuffer = StateNextVariable[i].cursorLine;

			if (cursorBuffer != LCD_LINE_RECENT)
				cursorLine = cursorBuffer;

			break;
		}
	}
	return nextVariable;
}

/**
 * @brief This function run when is in standby mode.
 * @param input is buttons state after debouncig.
 * @return nextState is next state according to the current state and button input.
 */
char StandBy(char input)
{
	Led1(0);
	Led3(0);
	Led4(0);
	led_blink_timeout=0;
	buzzer[BUZZ_NUM] = 0;
	buzzer[BUZZ_DELAY] = 0;
	blink_enable = false;
	TIM2->CNT = 0;
	SetDutyCycle_us(0);
	HAL_TIM_Base_Stop_IT(&htim2);
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
	HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_2);
	HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_3);
	HAL_GPIO_WritePin(Relay_GPIO_Port, Relay_Pin, GPIO_PIN_RESET);
	mode_state = STANDBY;
	error_state = IDLE;
	static char x1_dot = 0/*,count1=0*/;
	static uint8_t showARMWELD = 1; // ARMWELD animation in standBy.
	static uint8_t	enterFunction = 1;

	if (enterFunction == 1)
	{
		buzzer[BUZZ_NUM] = 1;
		buzzer[BUZZ_DELAY] = 255;
		enterFunction = 0;
	}


	if (reWarning == 0 || warningVar == 0)	//need for show after some second warning occured or not any warnings.
	{
		if (f2lcd100ms)
		{

			// display animation of medinalTeb first time when turn on after that show "STANDBY"
			if (x1_dot >= 6 && count1 < 19)
			{
				count1++;
				if (count1 <= 5)
				{
					HLine(9 + count1, LCD_LINE3, LCD_UpLine, LCD_Mem0);
					HLine(10 - count1, LCD_LINE3, LCD_UpLine, LCD_Mem0);
				}
				else if (count1 == 6)
				{
					HLine(9 + count1, LCD_LINE2, LCD_RightLine, LCD_Mem1);
					HLine(10 - count1, LCD_LINE2, LCD_LeftLine, LCD_Mem2);
				}
				else if (count1 <= 11)
				{
					HLine(21 - count1, LCD_LINE1, LCD_DownLine, LCD_Mem3);
					HLine(count1 - 2, LCD_LINE1, LCD_DownLine, LCD_Mem3);
				}
				else if (count1 <= 12)
					WeldDisplay();

			}
			else if (count1 == 19)
			{
				LcdClear();
				LcdPuts(7, LCD_LINE2, "STANDBY");
				showARMWELD = false;
				//					WELD_FAN_OFF;   // turn off fan when ARMWELD animation finish.
				count1++; // it runs just first time this (if).
			}
			f2lcd100ms = false;
		}

		if (flcd500ms)
		{
			if (showARMWELD == true)
			{
				//					WELD_FAN_ON;   // turn on fan when the device  is turning on.
				if (x1_dot < 6)
				{
					LcdClear();
					x1_dot++;
				}
				LcdPuts(x1_dot, 1, "NEKOO");
				LcdPuts((17 - x1_dot), 1, "RAD");
			}
			flcd500ms = false;
		}

	}
	if (input == KEY_STANDBY)
	{
		//enterFunction = 1;
		showARMWELD = false; // because of don't show ARMWELD when come from RunMeno.
		mode_state = STOP;
		return ST_MAIN_STATE;
	}

	return ST_STANDBY;
}

/**
 * @brief This function run when is in menue or submenu untill one of the values choose.
 * @param input is buttons state after debouncig.
 * @return nextState is next state according to the current state and button input.
 */
char MainMenu(char input)
{
	uint8_t j;
	//static uint8_t nextVariable = VA_FIRST_VARIABLE;
	static uint8_t adjustState = ST_MAIN_STATE;
	static char enterFunction = 1;

	//if (HAL_GPIO_ReadPin(ShortCircuitDetect_GPIO_Port, ShortCircuitDetect_Pin) == GPIO_PIN_RESET)
	//{
		//error_state = SHORTCIRCUIT;
	//}
	if (error_state != IDLE && one_time_excute == true)
	{
		HAL_GPIO_WritePin(Relay_GPIO_Port, Relay_Pin, GPIO_PIN_RESET);
		HAL_TIM_Base_Stop_IT(&htim2);
		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
		HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_2);
		HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_3);
		Led1(0);
		enterFunction = 1;
		mode_state = STOP;
		blink_enable = true;
		led_blink_timeout = 10 ;
		buzzer[BUZZ_NUM] = 15;
		buzzer[BUZZ_DELAY] = 200;
		one_time_excute = false;
	}

	if (nextVariable != variable)
	{
		variable = nextVariable;
		refreshLcd = true; // atention to DisplyRunMenu	function
		for (uint8_t currentVariable = 0; (j = StateVariable[currentVariable].variable); currentVariable++)
		{
			/* macro to determine which variable must be displayed in first line in  */
            #define FIRST_LINE_VAR		currentVariable-cursorLine

			if (j == variable)
			{
				adjustState = StateVariable[currentVariable].adjustState;
				for (uint8_t n = 0; n < 4; n++)
				{
					statetext[n]  = StateVariable[FIRST_LINE_VAR + n].pText;
					elementType[n] = StateVariable[FIRST_LINE_VAR + n].elementType;
					pValues[n] = (ValueStruct*)StateVariable[FIRST_LINE_VAR + n].pValueStruct;
				}
				break;
			}
		}
	}
	if (reWarning == 0 || warningVar == 0)	//need for show after some second warning occurred or not any warnings.
		DisplayRunMenu();

	if (input == KEY_STANDBY && error_state == IDLE)
	{
		count1 = 19;
		nextVariable = VA_FIRST_VARIABLE;
		cursorLine = LCD_LINE1;
		variable = VA_SECOND_VARIABLE;
		return ST_STANDBY;
	}
	else if (input == KEY_ACCEPT)
	{
		return adjustState;
	}
	/*else if (input == KEY_BACK)
	{
		//nextVariable = cursorLine+1;
		nextVariable = VA_FIRST_VARIABLE;
		return ST_MAIN_STATE;
	}*/
	else if (input == KEY_START && error_state == IDLE)
	{
		// Act like Start Buttom
		if (enterFunction == 1)  // If it is the first time the button is pushed and the variables of frequency and time-on(duty cycle) are set
		{
			TIM2->CNT = 0;
			SetFrequency(Frequency);
			SetDutyCycle_us(TimeOn);
			HAL_TIM_Base_Start(&htim2);
			HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
			HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_2);
			HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_3);
			HAL_GPIO_WritePin(Relay_GPIO_Port, Relay_Pin, GPIO_PIN_SET);
			Led1(1);
			enterFunction = 0;
			mode_state = RUN;
			Led3(0);
			Led4(0);
		}
		/* Act like Stop Buttom */
		else if (enterFunction == 0)
		{
			HAL_TIM_Base_Stop_IT(&htim2);
			HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
			HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_2);
			HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_3);
			HAL_GPIO_WritePin(Relay_GPIO_Port, Relay_Pin, GPIO_PIN_RESET);
			Led1(0);
			enterFunction = 1;
			mode_state = STOP;
		}
		return ST_MAIN_STATE;
	}
	else if (input != KEY_NULL)
	{
		// Plain menu, clock the state machine
		nextVariable = VariableMachine(input);
	}

	return ST_MAIN_STATE;
}

/**
 * @brief This function adjusts integer values.
 * @param input is buttons state after debouncig.
 * @return nextState is next state according to the current state and button input.
 */
char IntAdjust(char input)
{
	int16_t valueIA;
	//uint8_t j = 1;
	static char enterFunction = 1;

	char lcdBufferIA[15];

	if (enterFunction == 1)
	{
		//pValues[cursorLine]->ival=values[cursorLine+1].ival;
		TIM1->CNT = (pValues[cursorLine]->ival)*ROTARY_SPC; //The amount of variable(that cursor point it)  return to CNT of timer or QDEC .
		enterFunction = 0;
	}
	LcdCreateChar(0, UpDownCharacter); // "<>" character
	LcdCursorSet(13, cursorLine);
	LcdData(0); //  "<>" character


	/* Check timer valueIA to be in range. */
	valueIA = (int16_t)TIM1->CNT / ROTARY_SPC; // int16_t to avoid of roll over timer valueIA from bottom to top.

	if (valueIA > (int16_t)pValues[cursorLine]->upLimit)
		TIM1->CNT = (int16_t)pValues[cursorLine]->upLimit*ROTARY_SPC;

	else if (valueIA < (int16_t)pValues[cursorLine]->downLimit)
		TIM1->CNT = (int16_t)pValues[cursorLine]->downLimit*ROTARY_SPC;

	// Ensure TIM1->CNT is never negative
	if ((int16_t)TIM1->CNT < 0) {
	    TIM1->CNT = 0;
	}

	sprintf(lcdBufferIA, "%-3d", TIM1->CNT / ROTARY_SPC);
	LcdPuts(LCD_X, cursorLine, lcdBufferIA);

	switch (input)
	{
	case KEY_ACCEPT:
		pValues[cursorLine]->ival = TIM1->CNT / ROTARY_SPC;
		if (variable == VA_FREQUENCY)
		{
			Frequency = pValues[cursorLine]->ival;
		}
		else if (variable == VA_TIME_ON)
		{
			TimeOn = pValues[cursorLine]->ival;
		}
		else if (variable == VA_BLD_OFFSET)
		{
			AdcOffset = pValues[cursorLine]->ival;
		}
		else if (variable == VA_BLD_CALIBRATION)
		{
			AdcCalib = (float)((pValues[cursorLine]->ival * 0.001f) + 1.000f);
		}
		else if (variable == VA_BLD_DIVIDER)
		{
			VolDivider = pValues[cursorLine]->ival;
		}
		else if (variable == VA_BLD_RATIO)
		{
			VolRatio = pValues[cursorLine]->ival;
		}
		else if (variable == VA_BLD_TON_PRE)
		{
			TimeOnPre = pValues[cursorLine]->ival;
		}
		else if (variable == VA_BLD_DELAY)
		{
			PhaseDelay = pValues[cursorLine]->ival;
		}
		WriteValueStructArrayToEeprom(0,values, NUMBER_OF_VARIABLES+1);
		WriteValueStructArrayToEeprom(50,bldValues, NUMBER_OF_BLD_VARIABLES+1);

		enterFunction = 1;
		LcdCursorSet(13, cursorLine);
		LcdData(0x20); //  "blank" character
		return ST_MAIN_STATE;

	case KEY_BACK:
		enterFunction = 1;
		LcdCursorSet(13,cursorLine);
		LcdData(0x20);  //  "blank" character
		return ST_MAIN_STATE;

	case KEY_STANDBY:
		/*enterFunction = 1;
		LcdCursorSet(13, cursorLine);
		LcdData(0x20); //  "blank" character
		return ST_STANDBY;*/
		count1 = 19;
		nextVariable = VA_FIRST_VARIABLE;
		cursorLine = LCD_LINE1;
		//variable = VA_SECOND_VARIABLE;
		variable = ST_CHANGE_MENU;
		return ST_STANDBY;
	}
	return ST_INT_ADJUST;
}

/**
 * @brief This function adjusts string values.
 * @param input is buttons state after debouncig.
 * @return nextState is next state according to the current state and button input.
 */
char StringChange(char input)
{
	static char enterFunction = 1;
	static int8_t strNumTemp;
	if (enterFunction == 1)
	{
		strNumTemp = pValues[cursorLine]->strNum;
		enterFunction = 0;
	}

	switch (input)
	{
	case ROTARY_CW_DIR:		strNumTemp++; break;

	case ROTARY_CCW_DIR:	strNumTemp--; break;

	case KEY_ACCEPT:

		// store the temporary adjusted value to the global variable
		pValues[cursorLine]->strNum = strNumTemp;

		/*Intensity = pValues[cursorLine]->strNum;

		if (variable == VA_VOLTAGE && pValues[cursorLine]->strNum==VOL_LEVEL_LOW)
		{
		}

		WriteValueStructArrayToEeprom(0,values, NUMBER_OF_VARIABLES+1);*/

		enterFunction = 1;
		LcdCursorSet(13, cursorLine);
		LcdData(0x20); //  "blank" character

		return ST_MAIN_STATE;

		case KEY_BACK:
			enterFunction = 1;
			LcdCursorSet(13,cursorLine);
			LcdData(0x20);  //  "blank" character
			return ST_MAIN_STATE;

	case KEY_STANDBY:
		/*enterFunction = 1;
		LcdCursorSet(13, cursorLine);
		LcdData(0x20); //  "blank" character
		return ST_STANDBY;*/
		count1 = 19;
		nextVariable = VA_FIRST_VARIABLE;
		cursorLine = LCD_LINE1;
		//variable = VA_SECOND_VARIABLE;
		variable = ST_CHANGE_MENU;
		return ST_STANDBY;
	}

	LcdCreateChar(0, UpDownCharacter); // "<>" character
	LcdCursorSet(13, cursorLine);
	LcdData(0); //  "<>" character


	if (strNumTemp > pValues[cursorLine]->upLimit)
	{
		strNumTemp = pValues[cursorLine]->upLimit;
	}
	else if (strNumTemp < pValues[cursorLine]->downLimit)
	{
		strNumTemp = pValues[cursorLine]->downLimit;
	}
	LcdPuts(LCD_X, cursorLine, pValues[cursorLine]->pText[strNumTemp]);

	return ST_STR_CHANGE;
}

/**
 * @brief This function adjusts float values.
 * @param input is buttons state after debouncig.
 * @return nextState is next state according to the current state and button input.
 */
char FloatAdjust(char input)
{
	#define INTEGER_PART	0
	#define DECIMAL_PART	1
	static uint8_t mode = INTEGER_PART;
	static int FloatTemp[2];
	static char enterFunction = 1;
	char lcdBufferIA[15];

	if (enterFunction == 1)
	{
		//FloatTemp[DECIMAL_PART] = modf(pValues[cursorLine]->fval, &FloatTemp[INTEGER_PART]);
		FloatTemp[INTEGER_PART] = (int) pValues[cursorLine]->fval;
		FloatTemp[DECIMAL_PART] = round((pValues[cursorLine]->fval - FloatTemp[INTEGER_PART]) * 10);

		//UpLimit_DEC = round((pValues[cursorLine]->upLimit - (int)pValues[cursorLine]->upLimit)*10);
		//DownLimit_DEC = round((pValues[cursorLine]->downLimit - (int)pValues[cursorLine]->downLimit) * 10);

		if (mode == INTEGER_PART)
		{
			TIM1->CNT = (int16_t)(FloatTemp[INTEGER_PART])*ROTARY_SPC;
		}
		/*else if ( mode == DECIMAL_PART)
		{
			TIM1->CNT = ((int16_t)(FloatTemp[DECIMAL_PART])*10)*ROTARY_SPC;
		}*/
		enterFunction = 0;
	}
	if (mode == INTEGER_PART)
	{
		FloatTemp[INTEGER_PART] = (int16_t)TIM1->CNT / ROTARY_SPC;
		if (FloatTemp[INTEGER_PART] > (int16_t)pValues[cursorLine]->upLimit)
			TIM1->CNT = (int16_t)pValues[cursorLine]->upLimit*ROTARY_SPC;

		else if (FloatTemp[INTEGER_PART] < (int16_t)pValues[cursorLine]->downLimit)
			TIM1->CNT = (int16_t)pValues[cursorLine]->downLimit*ROTARY_SPC;

		LcdCreateChar(0, UpDownCharacter); // "<>" character
		   
		LcdCursorSet(19, cursorLine);
		LcdData(0x20); //  "blank" character
		   
		LcdCursorSet(13, cursorLine);
		LcdData(0); //  "<>" character
	}
	else if (mode == DECIMAL_PART)
	{
		FloatTemp[DECIMAL_PART] = (int16_t)TIM1->CNT / ROTARY_SPC;
		if (FloatTemp[DECIMAL_PART] > 9)
			TIM1->CNT = 9*ROTARY_SPC;

		else if (FloatTemp[DECIMAL_PART] < 0)
			TIM1->CNT = 0*ROTARY_SPC;

        //The value of the float number should not be below downlimit of the value
		//else if (FloatTemp[DECIMAL_PART] == 0 && FloatTemp[INTEGER_PART] == 0)
			//TIM1->CNT = DownLimit_DEC*ROTARY_SPC;

		if (FloatTemp[INTEGER_PART] < (int16_t)pValues[cursorLine]->downLimit ||
		    (FloatTemp[INTEGER_PART] == (int16_t)pValues[cursorLine]->downLimit &&
		     FloatTemp[DECIMAL_PART] < (int16_t)((pValues[cursorLine]->downLimit - (int16_t)pValues[cursorLine]->downLimit) * 10)))
		{
		    FloatTemp[INTEGER_PART] = (int16_t)pValues[cursorLine]->downLimit;
		    FloatTemp[DECIMAL_PART] = (int16_t)((pValues[cursorLine]->downLimit - FloatTemp[INTEGER_PART]) * 10);
		    TIM1->CNT = (FloatTemp[INTEGER_PART] * 10 + FloatTemp[DECIMAL_PART]) * ROTARY_SPC;
		}
		LcdCreateChar(0, UpDownCharacter); // "<>" character

		LcdCursorSet(19, cursorLine);
		LcdData(0); //  "<>" character

		LcdCursorSet(13, cursorLine);
		LcdData(0x20); //  "blank" character
	}

	// store the temporary adjusted value to the global variable
	pValues[cursorLine]->fval = (FloatTemp[INTEGER_PART] + (FloatTemp[DECIMAL_PART] / 10.0f));
	sprintf(lcdBufferIA, "%-4.1f ", pValues[cursorLine]->fval);
	LcdPuts(LCD_X, cursorLine, lcdBufferIA);

	switch (input)
	{
	case ROTARY_CW_DIR:  FloatTemp[mode]++; break;

	case ROTARY_CCW_DIR:  FloatTemp[mode]--; break;

	case KEY_ACCEPT:

		pValues[cursorLine]->fval = (FloatTemp[INTEGER_PART] + (FloatTemp[DECIMAL_PART] / 10.0));
		if (mode == INTEGER_PART)
		{
			mode++;
			TIM1->CNT = (int16_t)(FloatTemp[DECIMAL_PART])*ROTARY_SPC;
			break;
		}
		else if (mode == DECIMAL_PART)
		{
			/*if (variable == VA_TIME_ON)
			{
				TimeOn = (pValues[cursorLine]->fval / 1.000);
			}

			WriteValueStructArrayToEeprom(0,values, NUMBER_OF_VARIABLES+1);*/

			mode = INTEGER_PART;
			enterFunction = 1;
			LcdCursorSet(19, cursorLine);
			LcdData(0x20); //  "blank" character
			return ST_MAIN_STATE;
		}

    case KEY_BACK:
		if (mode == DECIMAL_PART)
		{
			mode=INTEGER_PART;
			TIM1->CNT = (int16_t)(FloatTemp[INTEGER_PART])*ROTARY_SPC;
			break;
		}

		else
		{
			enterFunction = 1;
			LcdCursorSet(13,cursorLine);
			LcdData(0x20);  //  "blank" character
			return ST_MAIN_STATE;
		}

	case KEY_STANDBY:
		/*enterFunction = 1;
		LcdCursorSet(13, cursorLine);
		LcdData(0x20); //  "blank" character
		return ST_STANDBY;*/
		count1 = 19;
		nextVariable = VA_FIRST_VARIABLE;
		cursorLine = LCD_LINE1;
		//variable = VA_SECOND_VARIABLE;
		variable = ST_CHANGE_MENU;
		return ST_STANDBY;
	}

	return ST_FLT_ADJUST;
}

/**
 * @brief Writes a single ValueStruct to the EEPROM.
 * @param page The page number (0 to EEPROM_TOTAL_PAGES-1).
 * @param offset The starting byte offset within the page (0 to EEPROM_PAGE_SIZE-1).
 * @param data Pointer to the ValueStruct to be written.
 */
void WriteValueStructToEeprom(uint16_t page, uint16_t offset, ValueStruct* data)
{
	/*if (data == NULL)
	    return;

	if (page >= EEPROM_TOTAL_PAGES)
	    return;*/

    uint16_t currentOffset = offset;

    // Write strNum
    WriteToEeprom(page, currentOffset, (uint8_t*)&data->strNum, sizeof(int8_t));
    currentOffset += sizeof(int8_t);

    // Write downLimit
    WriteFloatToEeprom(page, currentOffset, data->downLimit);
    currentOffset += sizeof(float);

    // Write upLimit
    WriteFloatToEeprom(page, currentOffset, data->upLimit);
    currentOffset += sizeof(float);

    // Write union data based on strNum
    if (data->strNum >= 0 && data->strNum <= 3) {
        // Write pText array
        for (int i = 0; i < 4; i++) {
            if (data->pText[i] != NULL) {
                uint8_t len = strlen(data->pText[i]);
                WriteToEeprom(page, currentOffset, &len, sizeof(uint8_t));
                currentOffset += sizeof(uint8_t);
                WriteToEeprom(page, currentOffset, (uint8_t*)data->pText[i], len);
                currentOffset += len;
            } else {
                uint8_t len = 0;
                WriteToEeprom(page, currentOffset, &len, sizeof(uint8_t));
                currentOffset += sizeof(uint8_t);
            }
        }
    } else if (data->strNum == 10) {
        // Write fval
        WriteFloatToEeprom(page, currentOffset, data->fval);
    } else if (data->strNum == 20){
        // Write ival
        WriteToEeprom(page, currentOffset, (uint8_t*)&data->ival, sizeof(int));
    }
}

/**
 * @brief Reads a single ValueStruct from the EEPROM.
 * @param page The page number (0 to EEPROM_TOTAL_PAGES-1).
 * @param offset The starting byte offset within the page (0 to EEPROM_PAGE_SIZE-1).
 * @param data Pointer to the ValueStruct where the read data will be stored.
 */
void ReadValueStructFromEeprom(uint16_t page, uint16_t offset, ValueStruct* data)
{
    uint16_t currentOffset = offset;

    // Read strNum
    ReadFromEeprom(page, currentOffset, (uint8_t*)&data->strNum, sizeof(int8_t));
    currentOffset += sizeof(int8_t);

    // Read downLimit
    data->downLimit = ReadFloatFromEeprom(page, currentOffset);
    currentOffset += sizeof(float);

    // Read upLimit
    data->upLimit = ReadFloatFromEeprom(page, currentOffset);
    currentOffset += sizeof(float);

    // Read union data based on strNum
    if (data->strNum >= 0 && data->strNum <= 3) {
        // Read pText array
        for (int i = 0; i < 4; i++) {
            uint8_t len;
            ReadFromEeprom(page, currentOffset, &len, sizeof(uint8_t));
            currentOffset += sizeof(uint8_t);

            if (len > 0) {
                char* temp = (char*)malloc(len + 1);
                ReadFromEeprom(page, currentOffset, (uint8_t*)temp, len);
                temp[len] = '\0';
                currentOffset += len;

                // Match the read string with the constants
                if (strcmp(temp, NON) == 0) data->pText[i] = NON;
                else if (strcmp(temp, LOW) == 0) data->pText[i] = LOW;
                else if (strcmp(temp, MED) == 0) data->pText[i] = MED;
                else if (strcmp(temp, HIGH) == 0) data->pText[i] = HIGH;
                else data->pText[i] = NULL;

                free(temp);
            } else {
                data->pText[i] = NULL;
            }
        }
    } else if (data->strNum == 10) {
        // Read fval
        data->fval = ReadFloatFromEeprom(page, currentOffset);
    } else if (data->strNum == 20){
        // Read ival
        ReadFromEeprom(page, currentOffset, (uint8_t*)&data->ival, sizeof(int));
    }
}

/**
 * @brief Writes an array of ValueStruct to the EEPROM.
 * @param startPage The starting page number (0 to EEPROM_TOTAL_PAGES-1).
 * @param data Pointer to the array of ValueStruct to be written.
 * @param arraySize The number of ValueStruct elements in the array.
 */
void WriteValueStructArrayToEeprom(uint16_t startPage, ValueStruct* data, uint16_t arraySize)
{
    uint16_t currentPage = startPage;
    uint16_t currentOffset = 0;

    for (uint16_t i = 0; i < arraySize; i++)
    {
        WriteValueStructToEeprom(currentPage, currentOffset, &data[i]);

        // Move to the next page if we're close to the end of the current page
        currentOffset += sizeof(ValueStruct);
        if (currentOffset + sizeof(ValueStruct) > EEPROM_PAGE_SIZE)
        {
            currentPage++;
            currentOffset = 0;
        }
    }
}

/**
 * @brief Reads an array of ValueStruct from the EEPROM.
 * @param startPage The starting page number (0 to EEPROM_TOTAL_PAGES-1).
 * @param data Pointer to the array where the read ValueStruct elements will be stored.
 * @param arraySize The number of ValueStruct elements to read.
 */
void ReadValueStructArrayFromEeprom(uint16_t startPage, ValueStruct* data, uint16_t arraySize)
{
    uint16_t currentPage = startPage;
    uint16_t currentOffset = 0;

    for (uint16_t i = 0; i < arraySize; i++)
    {
        ReadValueStructFromEeprom(currentPage, currentOffset, &data[i]);

        // Move to the next page if we're close to the end of the current page
        currentOffset += sizeof(ValueStruct);
        if (currentOffset + sizeof(ValueStruct) > EEPROM_PAGE_SIZE)
        {
            currentPage++;
            currentOffset = 0;
        }
    }
}

/**
 * @brief Write init values once in every external eeprom.
 */
void InitValueWriteToEeprom(void)
{
	WriteValueStructArrayToEeprom(0,EEValues, NUMBER_OF_VARIABLES+1);
	WriteValueStructArrayToEeprom(50,EEBldValues, NUMBER_OF_BLD_VARIABLES+1);
}

/**
 * @brief read init values from external eeprom.
 */
void InitValueReadFromEeprom(void)
{
	ReadValueStructArrayFromEeprom(0,values, NUMBER_OF_VARIABLES+1);
	Frequency = values[1].ival;
	TimeOn = values[2].ival;
	ReadVoltageADC();
	ReadValueStructArrayFromEeprom(50,bldValues, NUMBER_OF_BLD_VARIABLES+1);
	AdcOffset = bldValues[1].ival;
	AdcCalib = (float)((bldValues[2].ival * 0.001f) + 1.000f);
	VolDivider = bldValues[3].ival;
	VolRatio = bldValues[4].ival;
	TimeOnPre = bldValues[5].ival;
	PhaseDelay = bldValues[6].ival;

}

const unsigned int Ntc5K3520[153] =
{
	0xFFFF,	    //  -32	 000
	65406,  	//  -31	 001
	61940,  	//  -30	 002
	58680,  	//  -29	 003
	55611,  	//  -28	 004
	52722,  	//  -27	 005
	50001,	    //  -26	 006
	47437,  	//  -25	 007
	45020,  	//  -24  008
	42742,  	//  -23  009
	40592,  	//  -22  010
	38564,  	//  -21  011
	36650,  	//  -20  012
	34842,  	//  -19  013
	33134,  	//  -18  014
	31520,  	//  -17  015
	29994,  	//  -16  016
	28552,  	//  -15  017
	27187,  	//  -14  018
	25896,  	//  -13  019
	24673,  	//  -12  020
	23516,  	//  -11  021
	22420,  	//  -10  022
	21381, 		//  -09  023
	20369,  	//  -08  024
	19463,  	//  -07  025
	18577,  	//  -06  026
	17737,  	//  -05  027
	16940,  	//  -04  028
	16183,  	//  -03  029
	15465,  	//  -02  030
	14782,  	//  -01  031
	14134,  	//   00  032
	13517,  	//   01  033
	12931,  	//   02  034
	12374,  	//   03  035
	11844,  	//   04  036
	11339,  	//   05  037
	10859,  	//   06  038
	10402,  	//   07  039
	9966,  		//   08  040
	9552,  		//   09  041
	9157,  		//   10  042
	8780,  		//   11  043
	8421,  		//   12  044
	8079,  		//   13  045
	7752,  		//   14  046
	7440,  		//   15  047
	7143,  		//   16  048
	6859,  		//   17  049
	6588,  		//   18  050
	6329,  		//   19  051
	6082,  		//   20  052
	5846,  		//   21  053
	5620,  		//   22  054
	5404,  		//   23  055
	5197,  		//   24  056
	5000,  		//   25  057
	4811,  		//   26  058
	4630,  		//   27  059
	4457,  		//   28  060
	4291,  		//   29  061
	4132,  		//   30  062
	3981,  		//   31  063
	3835,  		//   32  064
	3696,  		//   33  065
	3562,  		//   34  066
	3434,  		//   35  067
	3312,  		//   36  068
	3194,  		//   37  069
	3081,  		//   38  070
	2973,  		//   39  071
	2869,  		//   40  072
	2769,  		//   41  073
	2674,  		//   42  074
	2582,  		//   43  075
	2494,  		//   44  076
	2409,  		//   45  077
	2327,  		//   46  078
	2249,  		//   47  079
	2174,  		//   48  080
	2102,  		//   49  081
	2032,  		//   50  082
	1965,  		//   51  083
	1901,  		//   52  084
	1839,  		//   53  085
	1779,  		//   54  086
	1722,  		//   55  087
	1667,  		//   56  088
	1613,  		//   57  089
	1562,  		//   58  090
	1513,  		//   59  091
	1465,  		//   60  092
	1419,  		//   61  093
	1375,  		//   62  094
	1333,  		//   63  095
	1292, 		//   64  096
	1252,		//   65  097
	1214,		//   66  098
	1177,		//   67  099
	1141,		//   68  100
	1107,		//   69  101
	1074,		//   70  102
	1042, 		//   71  103
	1011,		//   72  104
	981,  		//   73  105
	952,  		//   74  106
	924,  		//   75  107
	898,  		//   76  108
	872,  		//   77  109
	846,  		//   78  110
	822,  		//   79  111
	799, 		//   80  112
	776,		//   81  113
	754,		//   82  114
	733,		//   83  115
	712,		//   84  116
	692,		//   85  117
	673,		//   86  118
	654,		//   87  119
	636,		//   88  120
	619,		//   89  121
	602,		//   90  122
	586,		//   91  123
	570, 		//   92  124
	554,		//   93  125
	539,		//   94  126
	525,		//   95  127
	511,		//   96  128
	497,		//   97  129
	484,		//   98  130
	472,		//   99  131
	459,		//  100  132
	447,		//  101  133
	436,		//  102  134
	424,		//  103  135
	413,		//  104  136
	403,		//  105  137
	392,		//  106  138
	382,		//  107  139
	373,		//  108  140
	363,		//  109  141
	354,		//  110  142
	345,		//  111  143
	337,		//  112  144
	328,		//  113  145
	320,		//  114  146
	312,		//  115  147
	305,		//  116  148
	297,		//  117  149
	290,		//  118  150
	283,		//  119  151
	276  		//  120  152
};

/**
 * @brief Converts ADC final value of a NTC resistor table to its equivalent Temperature.
 * @param AdcValue Current ADC value of a NTC.
 * @return temperature Equivalent Temperature from the NTC resistor table.
 */
float NtcTemp(unsigned long int AdcValue)
{
	// NTC pull up Resistor = 5000 Ohm %1
	#define NTC_PULL_RES			5000

	/* for 11 bit resolution(signed mode) -> 2047
	 * for 12 bit resolution			  -> 4095
	 * for 8 bit resolution				  -> 255*/
	#define ADC_TOP_VALUE			4095

	int16_t	Low = 0,
			Mid,
			High = 152;

	float	Rntc = 0,
			Scale = 0,
			Delta = 0,
			temperature = 0;

//..............................................................

// NTC pullup Resistor = 5000 Ohm %1
// NTC Ref. Voltage = AVcc(2.48 Volt)
// A2D Vref. Voltage = 1.21 mVolt  (ADC Ref. 2.48V)/4095)

	Rntc =((AdcValue * NTC_PULL_RES)/(ADC_TOP_VALUE - AdcValue));

	if(Rntc > 61940)	return -30;
	else if(Rntc < 276) return 120;
	else
	{
		while(1) // NTC Array values are up/down
		{
			Mid =(Low + High)/2;

// ---------------------------------------------------------------------------------
			if(Rntc < Ntc5K3520[Mid])
			{
				Low = (Mid+1);
				if(Rntc == Ntc5K3520[Low])
				{
				   temperature = (Low - 32);
				   return temperature;
				}
				else if(Rntc > Ntc5K3520[Low])
			    {
					Scale = ((Ntc5K3520[Mid] - Ntc5K3520[Low])/10);
					Delta = ((Rntc - Ntc5K3520[Low])/Scale);
					temperature  = ((Low - 32)-(Delta*0.1));
					return temperature;
				}
			}
// ---------------------------------------------------------------------------------
			else if(Rntc > Ntc5K3520[Mid])
			{
				High = (Mid-1);
				if(Rntc == Ntc5K3520[High])
				{
				   temperature = (High - 32);
				   return temperature;
				}
				else if(Rntc < Ntc5K3520[High])
				{
					Scale = ((Ntc5K3520[High] - Ntc5K3520[Mid])/10);
					Delta = ((Rntc - Ntc5K3520[Mid])/Scale);
					temperature  = ((Mid - 32)-(Delta*0.1));
					return temperature;
				}
			}
// ---------------------------------------------------------------------------------
			else if(Rntc == Ntc5K3520[Mid])
			{
				temperature = (Mid - 32);
				return temperature;
			}
// ---------------------------------------------------------------------------------
		}
	}
}

