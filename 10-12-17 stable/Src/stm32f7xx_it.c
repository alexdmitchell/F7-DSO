/**
  ******************************************************************************
  * @file    stm32f7xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"
#include "stm32f7xx.h"
#include "stm32f7xx_it.h"

/* USER CODE BEGIN 0 */
#include "stm32746g_discovery_lcd.h"
#include "stm32746g_discovery_ts.h"
#include "gui.h"
#include "math.h"
#include "spi.h"
#include "main.h"


uint32_t DMA_compl = 0;
extern uint32_t no_sample_points; //number of sample points in ADC array
extern uint32_t ADC3ConvertedValues[80000];
extern uint32_t run_state; //variable used to store start/stop state 0=stopped, 1=running
extern float time_div[7]; //stores t/div shift
extern uint32_t time_div_selected; //stores which t/div mode is currently selected
extern int32_t v_shift; //stores vertical offset
extern uint32_t h_shift; //stores horizontal offset
extern uint32_t i_min; //stores the start point of data to plot
extern uint32_t i_max; //stores the end point of data to plot
extern uint32_t pga_gain[8]; //array to store gain values
static TS_StateTypeDef  TS_State;	//touch screen handle
uint32_t trigpoint; //stores the point to trigger from
extern int32_t trig_level; //stores the trigger level
extern uint16_t x,y;	//stores the touchscreen co-ordinates
uint32_t sum;	//used for v_avg
uint32_t average; //used for v_avg
uint32_t i; //while loop position counter
int32_t yval; //stores the corresponding y value for every x
int32_t difference; //used for autoset, stores difference between yval and edge of viewing area
uint32_t i,j,sum,average=0;
uint32_t min = 255;	//used to calculate v_min
uint32_t max = 0; //used to calculate v_max
uint32_t v_avg;	//used to store v_avg
uint32_t first_rise,first_fall,second_rise,second_fall;	//rising/falling points in array
uint32_t initial_autoset = 0; //used to store whether autoset has initially been actioned

extern uint16_t pga_gain_selected;
extern uint32_t maxadcval;
extern ADC_HandleTypeDef hadc3;
extern DMA_HandleTypeDef hdma_adc3;
extern uint32_t attenuation;
extern uint32_t coup_mode;
void autoset(void);
void quickmeasure (void);

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc3;

/******************************************************************************/
/*            Cortex-M7 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */
	BSP_LCD_Clear(0);
	drawfault();
	printf("Hardfault occured!"); 
  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
  /* USER CODE BEGIN HardFault_IRQn 1 */

  /* USER CODE END HardFault_IRQn 1 */
}

/**
* @brief This function handles Memory management fault.
*/
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */
	BSP_LCD_Clear(0);
	drawfault();
	printf("Memory management fault occured!");
  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
  /* USER CODE BEGIN MemoryManagement_IRQn 1 */

  /* USER CODE END MemoryManagement_IRQn 1 */
}

/**
* @brief This function handles Pre-fetch fault, memory access fault.
*/
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */
	BSP_LCD_Clear(0);
	drawfault();
	printf("Memory access fault occured!");
  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
  /* USER CODE BEGIN BusFault_IRQn 1 */

  /* USER CODE END BusFault_IRQn 1 */
}

/**
* @brief This function handles Undefined instruction or illegal state.
*/
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */
	BSP_LCD_Clear(0);
	drawfault();
	printf("Undefined instruction or illegal state occured!");
  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
  /* USER CODE BEGIN UsageFault_IRQn 1 */

  /* USER CODE END UsageFault_IRQn 1 */
}

/**
* @brief This function handles System service call via SWI instruction.
*/
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
* @brief This function handles Debug monitor.
*/
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
* @brief This function handles Pendable request for system service.
*/
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f7xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles EXTI line[15:10] interrupts.
*/
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */

	if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_11) != RESET) //USER button pressed
	{
		//toggle the DSO run state
		if (run_state == 0)
		{
			run_state = 1;	
		}
		else
		{
			run_state = 0;
		}
		
		printf("user button pressed\n");
		printf("run_state = %d\n",run_state);
	}

	
	if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_13) != RESET) //touchscreen pressed
	{
		BSP_TS_GetState(&TS_State);
    if(TS_State.touchDetected)
    {
      /* Get X and Y position of the touch post calibrated */
      x = TS_State.touchX[0];
      y = TS_State.touchY[0];
				
			/* run/stop pressed */
			if (x <=20 && y <= 20)
			{
				//toggle the run_state
				if (run_state == 1)
				{
					run_state = 0;
				}
				else
				{
					run_state = 1;
				}
				//delay so user can release button
				HAL_Delay(200);
			}
				
			/* v- pressed */
			if (x <= 60 && y >= 250)
			{
				pga_increment_gain();
				dac_set_level(0);
				v_shift=250;
				//delay so user can release button
				HAL_Delay(200);
			}
					
			/* v+ pressed */	
			if (x >=60 && x <= 120 && y >=250)
			{
				pga_decrement_gain();
				dac_set_level(0);
				v_shift=250;
				//delay so user can release button
				HAL_Delay(200);
				if (pga_gain_selected == 0)
				{
				autoset();
				}
			}
				
			/* T- pressed */
			if (x >= 120 && x <= 180 && y >= 250)
			{
				//if out of range, display warning
				if (time_div_selected == 0)
				{
					drawcontrollimit();
				}
				//otherwise decrease t/
				else
				{
					time_div_selected--;
				}
			//delay so user can release button
			HAL_Delay(200);		
			}
				
			/* T+ pressed */
			if (x >= 180 && x <= 240 && y >=250)
			{
				//if out of range, display warning
				if (time_div_selected == 7)
				{
					drawcontrollimit();
				}
				//otherwise increase t/
				else
				{
					time_div_selected++;
				}
			//delay so user can release button
			HAL_Delay(200);
			}
					
			/* < pressed */
			if (x >= 240 && x <= 300 && y >=250)
			{
				if (h_shift == 0)
				{
					drawcontrollimit();
				}
				else
				h_shift=h_shift-3;
			}
						
				
			/* > pressed */
			if (x >= 300 && x <= 360 && y >=250)
			{
				h_shift=h_shift+3;				
			}
				
			/* ^ pressed */
			if (x >= 360 && x <= 420 && y >=250)
			{
				if (pga_gain_selected == 0)
				{
					v_shift--;
				}
				  
				if (pga_gain_selected != 0)
				{
					dac_decrement_level();
				}
			//delay so user can release button
			HAL_Delay(50);
			}
				
			/* v pressed */
			if (x >= 420 && y >=250)
			{
				if (pga_gain_selected == 0)
				{
					v_shift++;
				}
					
				if (pga_gain_selected != 0)
				{
					dac_increment_level();
				}
			//delay so user can release button
			HAL_Delay(50);
			}
			
			/* attenuation toggle pressed */
			if (y <=20 && x > 320 && x < 360)
			{
				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
				if (attenuation == 1)
				{
					attenuation = 10;
			
				}
				else
				{
					attenuation = 1;
					
				}
			//delay so user can release button
			HAL_Delay(200);		
			}
			
			/* ac/dc coup toggle pressed */
			if (y <= 20 && x > 384 && x < 460)
			{
				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
				if (coup_mode == 0)
				{
					coup_mode = 1;
				}
				else
				{
					coup_mode = 0;
				}
			//delay so user can release button
			HAL_Delay(200);
			}				
				
			/* autoset pressed */
			if (y >=20 && y <= 250 && x > 460)
			{
				//only allow autoset if there is zero gain
				if (pga_gain_selected == 0)
				{
					autoset();
					//delay so user can release button
					HAL_Delay(200);
				}		
			}
			
			/*quick measure pressed*/
			if (x >= 240 && x <= 290 && y <= 20)
			{
				quickmeasure();
				//delay so user can release button
				HAL_Delay(200);
				
			}
			
		}
	}

	
  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_11);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
  /* USER CODE END EXTI15_10_IRQn 1 */
}

/**
* @brief This function handles DMA2 stream1 global interrupt.
*/
void DMA2_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream1_IRQn 0 */
  //uint32_t j;

	int32_t groundyval; 
	uint32_t adcvalselect;
	uint32_t xval=0; //stores the current position in ADC3ConvertedValues array
	uint32_t yval_prev=0;
	uint32_t yval_prev_set=0;
	
  /* USER CODE END DMA2_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc3);
  /* USER CODE BEGIN DMA2_Stream1_IRQn 1 */


	if (((HAL_DMA_GetState(&hdma_adc3)) == 1) && run_state == 1)
	{
		//start the plotting process

		//initially autoset the DSO so trace is initially viewable on screen
		if (initial_autoset == 0)
		{
		autoset();
		initial_autoset++;
		}
		
		//Calculate the average voltage to use as the trigger level
		min = 255;
		max = 0;
		
		for (i=0; i<no_sample_points/2; i++)
		{
			if (ADC3ConvertedValues[i] < min)
			{
				min = ADC3ConvertedValues[i];
			}
			if (ADC3ConvertedValues[i] > max)
			{
				max = ADC3ConvertedValues[i];
			}
		}	
		v_avg = (max + min) / 2;
		trig_level = v_avg;

		//if the average voltage is 0, no signal so just plot from the beginning of the array
		if (v_avg == 0)
		{
			printf("waiting for signal..\n");
			xval=1;
		}
		//otherwise find a point to trigger from
		else
		{
		
		for (i=1; i<no_sample_points/2; i++)
		{
			//find the first point in ADC array greater than the previously calculated average V
			//(first rising edge)
			adcvalselect=i*time_div[time_div_selected];
			if (ADC3ConvertedValues[adcvalselect] > trig_level)
			{	
				goto firstfall;
			}
		}	
		//find the next point in ADC array that is less than the average V
		//(first falling edge)
		firstfall:
		{
			for (j=i; i<no_sample_points; j++)
			{
				adcvalselect=j*time_div[time_div_selected];
				if (ADC3ConvertedValues[adcvalselect] < trig_level)
				{
					//initial x value is now set at the triggering point, j
					xval = j;
					break;
				}
			}
		}
		}	

		//if trigger point has not been found in first 480 values,
		//disable trigger by forcing trigger point = 1
		if (xval > 480)
		{
			xval=1;
	//		printf("cannot trigger!\n");
		}
		

	//clear the trace area before replotting
	cleartrace();

	//uncomment the following line to disable the trigger
	//	xval=1;	
		
		
	//plot from left to right
	// viewable area 20 - 460(x), 20 - 230(y)
	for (i=20; i<460; i++)
	{	
		//if user scrolls left beyond the start of the array
		if (xval*time_div[time_div_selected]+h_shift > no_sample_points)
		{
		;//catch the error, do nothing
		}
		
		else
		{
			//calculate the value from the array to select based on 
			//triggering point (xval), time_div and horizontal shift
			adcvalselect=xval*time_div[time_div_selected]+h_shift;	
			
			//obtain the value in the array at position adcvalselect then divide by 19.5 to scale into the
			//viewable area 20-230 (255/(230-20) = 1.214)
			//add 20 to shift the values to the edge of the measurement area then subtract v_shift to
			//shift the value up/down as set by the user
			//add 250 to the calculation to make the y value positive
			//subtract the entire calculation from 250, so that it is inverted
			yval=250-(((ADC3ConvertedValues[adcvalselect]/1.214)+20-v_shift)+250);
			groundyval=250-(1/1.21+20-v_shift+250);
			drawground(groundyval);
	
			if (yval_prev_set == 0)
			{
				yval_prev = yval;
				yval_prev_set = 1;
			}
			
			//if the trace y value is out of bounds
			if (yval < 21 || yval > 230)
			{
				goto end;
			}
			else
			{
				//if yval has changed, draw a line between the previous and present y
				//to create a continuous trace
				if (yval != yval_prev)
				{
					BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
					
					if (yval <=230 && yval >=20 && yval_prev <=230 && yval_prev >=20)
					{
						BSP_LCD_DrawLine(i-1,yval_prev,i,yval);
					}
					
				BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
				}
				else
				{
					//plot the trace whose x,y co-ordinates on lcd = i,yval
					BSP_LCD_DrawPixel(i,yval,LCD_COLOR_GREEN); 
				}
	
				yval_prev = yval;
					
				end:
			
			//increment the xval
			xval++;
			}
		}
	}
}
  /* USER CODE END DMA2_Stream1_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)	{
	//uint32_t j;
	DMA_compl = 1;
	
//	printf("ADC_ConvCpltCallback triggered at %d\n\r",HAL_GetTick());	
	//for (j=2046;j<2048;j++)
	//for (j=0;j<3;j++)
	//	printf( "CB_Sample[%d]=%u\r\n",j,ADC3ConvertedValues[j]);	
//	printf( "CB_Sample[%d]=%d\r\n",j,ADC3ConvertedValues[j]);	
}

void autoset(void)
{
		printf("autoset pressed\n");
	  //calculate the average value of adc3convertedvalues
		sum=0;
		for (i=0; i<no_sample_points; i++)
		{			
			sum=sum+(ADC3ConvertedValues[i]/1.21)+20;
		}		
		average = sum / no_sample_points;
		//then adjust v_shift to bring the trace back into the visible area
		v_shift = average + 125;
		printf("average=%d\n",average);
}

/* Function to provide basic signal measurements */
void quickmeasure (void)
{
	uint32_t no_signal = 1;
	uint32_t period_samples;
	double period_ms;
	float freq_hz;
	
		printf("quick meas pressed\n");
	
		min = 255;
		max = 0;
		
		//calculate the min and max values of adc3convertedvalues
		for (i=0; i<no_sample_points; i++)
			{
				if (ADC3ConvertedValues[i] < min)
				{
					min = ADC3ConvertedValues[i];
				}
				if (ADC3ConvertedValues[i] > max)
				{
					max = ADC3ConvertedValues[i];
				}
			}
		//display values to debug viewer
		printf("Min V = %.3f\n",min*0.0129);
		printf("Max V = %.3f\n",max*0.0129);
		printf("Vp-p = %.3f\n",(max*0.0129)-(min*0.0129));
		v_avg = (max + min) / 2;	
		printf("v_avg=%d\n",v_avg);
			
		//determine the point where the first rising edge occurs
		for (i=100; i<no_sample_points; i++)
			{
				//only use values which are being displayed on-screen
				//as a result of changing time/div
				if (ADC3ConvertedValues[i] > v_avg)
					{
						first_rise=i; //the i value where this occurs becomes the trigger point
						printf("first rising edge = %d\n",first_rise);
						no_signal = 0;
						break;
					}
			}	
			
		//if a rising edge was found previously,
		if (no_signal == 0)
		{
		//determine the point where the first falling edge occurs
		//using v_avg as threshold
		for (i=first_rise; i<no_sample_points; i++)
			{
				if (ADC3ConvertedValues[i] < v_avg)
					{
						first_fall=i; //the i value where this occurs becomes the trigger point
						printf("first falling edge = %d\n",first_fall);
						break;
					}
			}	
			
			//determine the point where the second rising edge occurs
			//using v_avg as threshold
			for (i=first_fall; i<no_sample_points; i++)
			{
				if (ADC3ConvertedValues[i] > v_avg)
					{
						second_rise=i; //the i value where this occurs becomes the trigger point
						printf("second rising edge = %d\n",second_rise);
						break;
					}
			}	
			//determine the point where the second falling edge occurs
			//using v_avg as threshold
			for (i=second_rise; i<no_sample_points; i++)
			{
				if (ADC3ConvertedValues[i] < v_avg)
					{
						second_fall=i; //the i value where this occurs becomes the trigger point
						printf("second falling edge = %d\n",second_rise);
						break;
					}
			}	
		}
		else
		{
			printf("no signal found..\n");
		}
		
		//calculate period
		period_samples = second_fall - first_fall;
		//Period between samples = 0.407us (0.000000407s) at 2.54 MSPS 
		period_ms = period_samples*0.000000407;
		freq_hz = 1/period_ms;
		printf("Frequency=%.2f\n",freq_hz);
		//use drawquickmeasure to output to LCD
		drawquickmeasure (min*0.0129, max*0.0129, freq_hz);
	
}
	



/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
