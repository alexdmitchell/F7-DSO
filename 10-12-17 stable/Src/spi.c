//spi.c

#include "stm32f7xx_hal.h"
#include "stm32f7xx.h"
#include "main.h"
#include "gui.h"
extern SPI_HandleTypeDef hspi2;

uint16_t spi_data_tx;
uint16_t dac_current_level = 16;
uint16_t gain = 0;
extern uint32_t pga_gain[8]; //Gain=1,2,4,5,8,10,16,32
extern uint16_t pga_gain_selected;
extern int32_t v_shift;

/*Function to control the MCP4801 DAC*/
/*Instructions issued in format: CCCC DDDD DDDD XXXX where C=Control, D=Data, X=don't care bits*/
/*Min level = 16, max level = 4080, step = 16*/
void dac_set_level(int level)
{
	printf("dac level %d requested\n",level);
		//Check that level entered is valid
		if (level < 0 || level > 4080)
		{
			 printf("Invalid DAC level set\n");// do nothing, invalid value
		}
	
		else
		{
			//Bring the CS pin low to enable DAC
			HAL_GPIO_WritePin(GPIOI, GPIO_PIN_3, GPIO_PIN_RESET);
		
			//0x1000 = 0001 0000 0000 0000 = Write to DAC register, Output gain=2x, Vout Active
			//ADD value between 16-4080 to change data bits in steps of 16 (to ignore don't care bits)
			spi_data_tx=0x1000 + level;
	
			//Transmit the level to the DAC
			HAL_SPI_Transmit(&hspi2,(uint8_t*)(&spi_data_tx),1,50);
			//DAC should now be set, deselect the DAC by setting CS high
			HAL_GPIO_WritePin(GPIOI, GPIO_PIN_3, GPIO_PIN_SET);
	
			//Set the dac_current_level to that which has been just set
			dac_current_level = level;
		}
}

void dac_increment_level(void)
{
		//Check that level can be decreased
		if (dac_current_level + 16 > 4080)
			{
				drawcontrollimit();
			}
	
		else
			{
			//Bring the CS pin low to enable DAC
			HAL_GPIO_WritePin(GPIOI, GPIO_PIN_3, GPIO_PIN_RESET);
				
			//0x1000 = 0001 0000 0000 0000 = Write to DAC register, Output gain=2x, Vout Active
			//ADD 16 to the existing level
			dac_current_level = dac_current_level + 16;
			spi_data_tx=0x1000 + dac_current_level;
	
			//Transmit the new level to the DAC
			HAL_SPI_Transmit(&hspi2,(uint8_t*)(&spi_data_tx),1,50);
			//DAC should now be set, deselect the DAC by setting CS high
			HAL_GPIO_WritePin(GPIOI, GPIO_PIN_3, GPIO_PIN_SET);
				
			printf("dac level = %d\n",dac_current_level);
			}
}


void dac_decrement_level(void)
{
		//Check that level can be decreased
		if (dac_current_level - 16 < 16)
			{
				drawcontrollimit();
			}
		else
			{
			//Bring the CS pin low to enable DAC
			HAL_GPIO_WritePin(GPIOI, GPIO_PIN_3, GPIO_PIN_RESET);
				
			//0x1000 = 0001 0000 0000 0000 = Write to DAC register, Output gain=2x, Vout Active
			//ADD 16 to the existing level
			dac_current_level = dac_current_level - 16;
			spi_data_tx=0x1000 + dac_current_level;
	
			//Transmit the new level to the DAC
			HAL_SPI_Transmit(&hspi2,(uint8_t*)(&spi_data_tx),1,50);
			//DAC should now be set, deselect the DAC by setting CS high
			HAL_GPIO_WritePin(GPIOI, GPIO_PIN_3, GPIO_PIN_SET);
				printf("dac level = %d\n",dac_current_level);
			}	
}

/*Function to control the MCP6S21 PGA*/
/*Instructions issued in format: CCCXXXXC XXXXXDDD where C=Control, D=Data, X=don't care bits*/
void pga_control(int gain)
{
	//Bring the CS pin low to ENABLE PGA
	HAL_GPIO_WritePin(GPIOI, GPIO_PIN_2, GPIO_PIN_RESET);
	
	//010XXXX0  = Write to PGA register, addresses the Gain register
	spi_data_tx = 0x4000 + gain;
	//Transmit the new level to the PGA
	HAL_SPI_Transmit(&hspi2,(uint8_t*)(&spi_data_tx),1,50);
	
	//PGA should now be set, deselect the PGA by setting CS high
	HAL_GPIO_WritePin(GPIOI, GPIO_PIN_2, GPIO_PIN_SET);
	
	printf("pga gain requested = %d\n",gain);
}

void pga_increment_gain(void)
{
	if (pga_gain_selected+1 > 5)
	{
		drawcontrollimit();
	}
	else
	{
		pga_gain_selected++;
	
		//Bring the CS pin low to ENABLE PGA
		HAL_GPIO_WritePin(GPIOI, GPIO_PIN_2, GPIO_PIN_RESET);
	
		//010XXXX0  = Write to PGA register, addresses the Gain register
		spi_data_tx = 0x4000 + pga_gain[pga_gain_selected];
		//Transmit the new level to the PGA
		HAL_SPI_Transmit(&hspi2,(uint8_t*)(&spi_data_tx),1,50);
	
		//PGA should now be set, deselect the PGA by setting CS high
		HAL_GPIO_WritePin(GPIOI, GPIO_PIN_2, GPIO_PIN_SET);
	}
	printf("gain=%d\n",pga_gain[pga_gain_selected]);
}

void pga_decrement_gain(void)
{
	if (pga_gain_selected-1 < 0)
	{
		drawcontrollimit();
	}
	else
	{
		pga_gain_selected--;
	
		//Bring the CS pin low to ENABLE PGA
		HAL_GPIO_WritePin(GPIOI, GPIO_PIN_2, GPIO_PIN_RESET);
	
		//010XXXX0  = Write to PGA register, addresses the Gain register
		spi_data_tx = 0x4000 + pga_gain[pga_gain_selected];
		//Transmit the new level to the PGA
		HAL_SPI_Transmit(&hspi2,(uint8_t*)(&spi_data_tx),1,50);
	
		//PGA should now be set, deselect the PGA by setting CS high
		HAL_GPIO_WritePin(GPIOI, GPIO_PIN_2, GPIO_PIN_SET);
	}
	printf("gain=%d\n",pga_gain[pga_gain_selected]);
}
