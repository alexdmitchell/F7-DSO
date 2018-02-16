#include "stm32746g_discovery_lcd.h"
#include "gui.h"

extern float time_div[8]; //stores t/div shift
extern uint32_t time_div_selected; //stores which t/div mode is currently selected
extern uint32_t v_shift; //stores vertical offset, (20+)221 = centre of trace area
extern uint32_t h_shift; //stores horizontal offset
uint32_t trig_lcd_ypos = 0;
extern uint32_t pga_gain[8];
extern uint16_t pga_gain_selected;


extern uint32_t maxadcval;
uint32_t groundyval_prev;
	uint32_t scaled_val;
	uint32_t scaled_val_prev;
	uint32_t val;
	uint32_t time_div_selected_prev;

extern uint32_t attenuation;
extern uint32_t coup_mode;
//uint32_t yval;
uint32_t yval_prev;

/* Function to draw the gridlines on the main page of the GUI */
void drawmain (void)
{
	BSP_LCD_SelectLayer(0);
	BSP_LCD_DisplayStringAt(4, 247, (uint8_t*)"V/-", LEFT_MODE);
	BSP_LCD_DisplayStringAt(63, 247, (uint8_t*)"V/+", LEFT_MODE);
	BSP_LCD_DisplayStringAt(123, 247, (uint8_t*)"T/-", LEFT_MODE);
	BSP_LCD_DisplayStringAt(183, 247, (uint8_t*)"T/+", LEFT_MODE);
	BSP_LCD_DisplayStringAt(260, 245, (uint8_t*)"<", LEFT_MODE);
	BSP_LCD_DisplayStringAt(320, 245, (uint8_t*)">", LEFT_MODE);
	BSP_LCD_DisplayStringAt(380, 250, (uint8_t*)"^", LEFT_MODE);
	BSP_LCD_DisplayStringAt(440, 245, (uint8_t*)"v", LEFT_MODE);

	BSP_LCD_SetFont(&Font12);
	BSP_LCD_DisplayStringAt(245, 1, (uint8_t*)"QUICK", LEFT_MODE);
	BSP_LCD_DisplayStringAt(240, 10, (uint8_t*)"MEASURE", LEFT_MODE);
	BSP_LCD_SetFont(&Font24);
	if (pga_gain_selected != 0)
	{
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
		BSP_LCD_FillRect(460,90,20,90);
		BSP_LCD_SetTextColor(LCD_COLOR_WHITE);	
	}
	else
	{
		BSP_LCD_DisplayStringAt(460, 90, (uint8_t*)"A", LEFT_MODE);
		BSP_LCD_DisplayStringAt(460, 110, (uint8_t*)"U", LEFT_MODE);
		BSP_LCD_DisplayStringAt(460, 130, (uint8_t*)"T", LEFT_MODE);
		BSP_LCD_DisplayStringAt(460, 150, (uint8_t*)"O", LEFT_MODE);
	}
}
	
/*Function to draw the current status, i.e. V/div, T/div*/	
void drawstatus ()
{
	BSP_LCD_SelectLayer(0);
	BSP_LCD_SetFont(&Font20);
	char buf [100]; //buffer for lcd
	val = pga_gain[pga_gain_selected];
	switch (val)
	{
		case 0: scaled_val=500;
		break;
	  case 1: scaled_val=250;
		break;
		case 2: scaled_val=125;
		break;
		case 3: scaled_val=100;
		break;
		case 4: scaled_val=70;
		break;
		case 5: scaled_val=50;
		break;
		case 6: scaled_val=16;
		break;
		case 7: scaled_val=32;
		break;
	}

	if (attenuation == 10)
	{
		scaled_val = scaled_val * 10;
	}

	if (scaled_val_prev!= scaled_val)
	{
		clearstatus();
	}
	
	scaled_val_prev=scaled_val;
	
	if (scaled_val >= 1000)
	{
		sprintf(buf, "%dV/", scaled_val/1000);
	}
	else
	{
		sprintf(buf, "%dmV/", scaled_val);
	}
	BSP_LCD_DisplayStringAt(40, 1, (uint8_t*)buf, LEFT_MODE);
	
	if (time_div_selected_prev != time_div_selected)
	{
		clearstatus();
	}
	time_div_selected_prev = time_div_selected;
	
	if ((time_div[time_div_selected]*20 >= 1000))
	{
		sprintf(buf, "%.0fms/",((time_div[time_div_selected]*20)/1000));
	}
	else
	{
		sprintf(buf, "%.0fus/",(time_div[time_div_selected]*20));
	}
	
	//Display current time/div
	
	BSP_LCD_DisplayStringAt(150,1,(uint8_t*)buf,LEFT_MODE);
	
	//Display attenuation status button
	BSP_LCD_SetFont(&Font12);
	if (attenuation == 1)
	{
	
	BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
	BSP_LCD_DisplayStringAt(323,1, (uint8_t*)"1:1", LEFT_MODE);
	BSP_LCD_SetTextColor(LCD_COLOR_RED);
	BSP_LCD_DisplayStringAt(320,10, (uint8_t*)"1:10", LEFT_MODE);
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	}
	
	else
	{
	BSP_LCD_SetTextColor(LCD_COLOR_RED);
	BSP_LCD_DisplayStringAt(323,1, (uint8_t*)"1:1", LEFT_MODE);
	BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
	BSP_LCD_DisplayStringAt(320,10, (uint8_t*)"1:10", LEFT_MODE);
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	}
	
	//Display ac/dc coupling status button
	if (coup_mode == 0)
	{
	BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
	BSP_LCD_DisplayStringAt(384,1, (uint8_t*)"DC COUPLING", LEFT_MODE);
	BSP_LCD_SetTextColor(LCD_COLOR_RED);
	BSP_LCD_DisplayStringAt(384,10, (uint8_t*)"AC COUPLING", LEFT_MODE);
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	}
	else
	{
	BSP_LCD_SetTextColor(LCD_COLOR_RED);
	BSP_LCD_DisplayStringAt(384,1, (uint8_t*)"DC COUPLING", LEFT_MODE);
	BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
	BSP_LCD_DisplayStringAt(384,10, (uint8_t*)"AC COUPLING", LEFT_MODE);
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	}
	
	
BSP_LCD_SetFont(&Font24);
	
}

/* Function to clear the trace area on the LCD */	
void cleartrace (void)
{
	BSP_LCD_SelectLayer(0);
	//Clear the trace by drawing black rectangle in area
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	BSP_LCD_FillRect(19,20,441,215);
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
}


/* Function to clear the status area on the LCD */
void clearstatus(void)
{
		BSP_LCD_SelectLayer(0);
	//Clear the status area
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	BSP_LCD_FillRect(0,0,480,20);
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
}


/* Function to show RUN state in the L/H corner */
void drawrun (void)
{
	BSP_LCD_SelectLayer(0);
	//Display green circle
	BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
	BSP_LCD_FillCircle(9,9,8);
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
}

/* Function to display STOP state in the L/H corner */
void drawstop (void)
{
	//Display red circle
	BSP_LCD_SetTextColor(LCD_COLOR_RED);
	BSP_LCD_FillCircle(9,9,8);
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
}

/* Function to display control at limit warning */
void drawcontrollimit (void)
{
	uint32_t i;
	
	for (i=0; i<250; i++)
	{
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
	BSP_LCD_DrawRect(80,110,320,70);
	BSP_LCD_DrawRect(81,111,318,68);
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	BSP_LCD_DisplayStringAt(0,125, (uint8_t*)"The control is", CENTER_MODE);
	BSP_LCD_DisplayStringAt(0,150, (uint8_t*)"at its limit", CENTER_MODE);
	}
	
}

/* Function to display control at limit warning */
void drawnosignal (void)
{
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	BSP_LCD_DrawRect(80,110,320,70);
	BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
	BSP_LCD_DrawHLine(35,110, 400);
	BSP_LCD_DrawVLine(35,110, 70);
	BSP_LCD_DrawHLine(35,180,400);
	BSP_LCD_DrawVLine(435,110, 70);
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	BSP_LCD_DisplayStringAt(0,125, (uint8_t*)"No signal detected:", CENTER_MODE);
	BSP_LCD_DisplayStringAt(0,150, (uint8_t*)"check probe connection", CENTER_MODE);
}



void drawground (int yval)
{

	if (yval_prev == yval)
	{
		;//do nothing
	}
	else
	{
	
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	BSP_LCD_FillRect(0,20,20,229);
	
	//if the ground value is off the top end of the screen
	if (yval < 30 && pga_gain_selected == 0)
	{
		
		BSP_LCD_SetTextColor(LCD_COLOR_RED);
		//if there is zero gain, draw an upwards arrow
		
			BSP_LCD_DrawVLine(10,50,10);
			BSP_LCD_DrawLine(8,52,10,50);
			BSP_LCD_DrawLine(12,52,10,50);
		
		//draw red ground symbol with x
		BSP_LCD_DrawVLine(10,30,7);
		BSP_LCD_DrawHLine(5,30+7,10);
		BSP_LCD_DrawHLine(8,30+9,5);
		BSP_LCD_DrawHLine(9,30+11,2);
		BSP_LCD_DrawLine(12,29,15,31);
		BSP_LCD_DrawLine(12,31,15,29);
		BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	}
	
	//if the ground value is off the bottom end of the screen
	if (yval > 230 && pga_gain_selected == 0)
	{
		BSP_LCD_SetTextColor(LCD_COLOR_RED);
		//if there is zero gain, draw a downward arrow
		
			BSP_LCD_DrawVLine(10,195,10);
			BSP_LCD_DrawLine(8,203,10,205);
			BSP_LCD_DrawLine(12,203,10,205);
		
		//draw red ground symbol with x
		BSP_LCD_DrawVLine(10,215,7);
		BSP_LCD_DrawHLine(5,215+7,10);
		BSP_LCD_DrawHLine(8,215+9,5);
		BSP_LCD_DrawHLine(9,215+11,2);
		BSP_LCD_DrawLine(12,214,15,216);
		BSP_LCD_DrawLine(12,216,15,214);
		BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	}
	
	
	//if the ground value is within screen range
	if (yval >= 30 && yval <= 230 && pga_gain_selected == 0)
	{
		//draw green ground symbol with connecting line
		BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
		BSP_LCD_DrawHLine(10,yval,8);
		BSP_LCD_DrawVLine(10,yval,7);
		BSP_LCD_DrawHLine(5,yval+7,10);
		BSP_LCD_DrawHLine(8,yval+9,5);
		BSP_LCD_DrawHLine(9,yval+11,2);
		BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	}
	
	}
	yval_prev = yval;
}


/* Function to display fatal error has occured */
void drawfault (void)
{
	while(1)
	{
	//disable the background
  BSP_LCD_SetLayerVisible(1,DISABLE);
	//draw red error box
	BSP_LCD_SetTextColor(LCD_COLOR_RED);
	BSP_LCD_DrawHLine(10,110, 460);
	BSP_LCD_DrawVLine(10,110, 70);
	BSP_LCD_DrawHLine(10,180,460);
	BSP_LCD_DrawVLine(470,110, 70);
	BSP_LCD_DisplayStringAt(0,125, (uint8_t*)"A fatal error has occured!", CENTER_MODE);
	BSP_LCD_DisplayStringAt(0,150, (uint8_t*)"Press reset to continue.. ", CENTER_MODE);	
	}
}

void drawquickmeasure (float vmin, float vmax, float freq)
{
	BSP_LCD_SetFont(&Font24);
	char vpp_buf[100];
	char vmax_buf[100];
	char vmin_buf[100];
	char freq_buf[100];
	
	sprintf(vpp_buf, "Vp-p = %.2f V", vmax-vmin);
	sprintf(vmax_buf, "Vmax = %.2f V", vmax);
	sprintf(vmin_buf, "Vmin = %.2f V", vmin);
	sprintf(freq_buf, "Freq = %.2f Hz", freq);
	
	
	uint32_t i;
	
	for (i=0; i<500; i++)
	{
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	BSP_LCD_DisplayStringAt(0,100, (uint8_t*)vpp_buf, CENTER_MODE);
	BSP_LCD_DisplayStringAt(0,125, (uint8_t*)vmax_buf, CENTER_MODE);
	BSP_LCD_DisplayStringAt(0,150, (uint8_t*)vmin_buf, CENTER_MODE);
	BSP_LCD_DisplayStringAt(0,175, (uint8_t*)freq_buf, CENTER_MODE);
	}
	printf("quickmeas complete\n");
}
