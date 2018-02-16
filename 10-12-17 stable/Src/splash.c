#include "stm32746g_discovery_lcd.h"
#include "stm32746g_discovery_ts.h"

static TS_StateTypeDef  TS_State;
void drawsplash (void)
{
	int screenpress = 0;
	
	//Initialise the LCD
	BSP_LCD_SelectLayer(0);
	BSP_LCD_Clear(0);
	BSP_LCD_SetFont(&LCD_DEFAULT_FONT);

	//Draw splash screen
	BSP_LCD_DrawHLine(20,20,440);
	BSP_LCD_DrawHLine(20,60,440);
	BSP_LCD_DrawHLine(20,270,440);
	BSP_LCD_DrawVLine(20,20,250);
	BSP_LCD_DrawVLine(460,20,250);
	BSP_LCD_DisplayStringAt(0,30,(uint8_t*)"STM32F746 Discovery DSO", CENTER_MODE);
	BSP_LCD_DisplayStringAt(0,130,(uint8_t*)"Build date: 10/12/17",CENTER_MODE);
	BSP_LCD_DisplayStringAt(0,210,(uint8_t*)"Initialisation Complete",CENTER_MODE);
	BSP_LCD_DisplayStringAt(0,230,(uint8_t*)"Touch to continue...",CENTER_MODE);

	//Leave the splash screen displayed -
	//Poll the touchscreen until user press then return to main.c
	while (screenpress == 0)
	{
		BSP_TS_GetState(&TS_State);
      if(TS_State.touchDetected)
			{
				screenpress++;
				BSP_LCD_Clear(0);
				HAL_Delay(200);		//Delay so user can release ts
			}	
	}
}

