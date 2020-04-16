
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2020 STMicroelectronics
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
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */

#include	"define.h"
#include 	"stdio.h"
#include	"stdlib.h"


/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
int phy_to_dll_rx_bus=0;
int phy_to_dll_rx_bus_valid=0;
int dll_to_phy_tx_bus=0;
int dll_to_phy_tx_bus_valid=0;
int phy_tx_busy=0;
int rx_data=0;
int tx_data='FUCK';//0x4655434B
int delay=0;
uint16_t	txShiftCount=128;
uint16_t rxShiftCount=256;
uint8_t state=0;
uint8_t thing=0;
int check=0;
int prevTX=0;
int prevRX=0;
int prevINTC=0;
int ctxbit=0;
int IgnoreSignal=0;
int curr=0;
static uint8_t interfaceClock;
static uint8_t isAlive;
static uint8_t Busy;
int sentcheck=2;
int recievecheck=2;
int txbustransfer=0;
int rxbustransfer=0;
uint8_t BIGCHEATS=0;
uint8_t BIGHACKS=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/*
void dll_RX(){
	if(phy_to_dll_rx_bus_valid){
		rx_data=phy_to_dll_rx_bus;
		phy_to_dll_rx_bus=0;
		phy_to_dll_rx_bus_valid=0;
	}
}

void dll_TX(){
	if((dll_to_phy_tx_bus_valid==0) && (phy_tx_busy==0)){
		if(!delay){
			dll_to_phy_tx_bus=tx_data;
			dll_to_phy_tx_bus_valid=1;
			phy_tx_busy=1;
			delay=rand()%9;
		}else if(delay>0){
			delay--;
		}
	}
}
*/

void phy_TX(){
	state=0;
	check=HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13);
	int negative=(int)txShiftCount;
	if(check && (check!=prevTX)){
		phy_tx_busy=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_3)||phy_tx_busy;
		if((phy_tx_busy==1) && (txShiftCount>0)){
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET);
				state=HAL_GPIO_ReadPin(GPIOA,txShiftCount);
				ctxbit--;
			txShiftCount>>=1;
			if(state==1){
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,GPIO_PIN_SET);
			}else{
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,GPIO_PIN_RESET);
			}
			//HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,GPIO_PIN_SET);
		}else{
			//HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,GPIO_PIN_RESET);
			txShiftCount=128;
			sentcheck=check;
		}
	}
	if((sentcheck!=check) && (sentcheck!=2)){
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);
		sentcheck=2;
	}
	prevTX=check;
}

void phy_RX(){
	thing=0;
	int hhh=(int)HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_7);
	int chk=HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_14);
	if((chk==0) && (chk!=prevRX) ){
		if(hhh==0 && rxShiftCount>=256 && rxShiftCount<=327688){
			thing=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0);
			if(thing==1){
				HAL_GPIO_WritePin(GPIOB,rxShiftCount,GPIO_PIN_SET);
				rx_data=HAL_GPIO_ReadPin(GPIOB,rxShiftCount);
			}else{
				HAL_GPIO_WritePin(GPIOB,rxShiftCount,GPIO_PIN_RESET);
			}
			rxShiftCount<<=1;
		}else{
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_SET);
			rxShiftCount=256;
			recievecheck=chk;
		}
		if((recievecheck!=chk) && (recievecheck!=2)){
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_RESET);
			recievecheck=2;
		}

	}
	prevRX=chk;
}

/*
void dll_layer(){
	dll_TX();
	dll_RX();
}
*/

void phy_layer(){
	phy_TX();
	phy_RX();
}

void interface(){
	curr=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_4);
	txbustransfer=0;
	rxbustransfer=0;
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_5));// send alive to dll
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_4));//Send clock to dll
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_6));// phy tx busy
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_7));// dll to phy tx bus valid
	txbustransfer=HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_7);
	if(txbustransfer==1){
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0));
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1));
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_2));
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_3));
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_4));
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_5));
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_6));
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_7));
		BIGHACKS=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0);
		BIGHACKS=BIGHACKS*2;
		BIGHACKS|=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1);
		BIGHACKS=BIGHACKS*2;
		BIGHACKS|=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_2);
		BIGHACKS=BIGHACKS*2;
		BIGHACKS|=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_3);
		BIGHACKS=BIGHACKS*2;
		BIGHACKS|=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_4);
		BIGHACKS=BIGHACKS*2;
		BIGHACKS|=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_5);
		BIGHACKS=BIGHACKS*2;
		BIGHACKS|=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_6);
		BIGHACKS=BIGHACKS*2;
		BIGHACKS|=(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_7));
	}
	/*if(prevINTC==curr){//check if the clock state is static
		if(curr==1){
			IgnoreSignal=1;
		}else{
			IgnoreSignal=0;
		}
	}else if(curr==1){//rising edge
		if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_3)==1){
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET);
		}else{
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_RESET);
		}
	}else if(curr==0){*///dont trust it
		rxbustransfer=(int)HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_7);
		if(rxbustransfer==1){
			IgnoreSignal=0;
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_8));
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_9));
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_10));
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_11));
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_12));
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_13,HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_13));
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_14,HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_14));
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_15));
			BIGCHEATS=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_8);
			BIGCHEATS*=2;
			BIGCHEATS|=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_9);
			BIGCHEATS*=2;
			BIGCHEATS|=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_10);
			BIGCHEATS*=2;
			BIGCHEATS|=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_11);
			BIGCHEATS*=2;
			BIGCHEATS|=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_12);
			BIGCHEATS*=2;
			BIGCHEATS|=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_13);
			BIGCHEATS*=2;
			BIGCHEATS|=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_14);
			BIGCHEATS*=2;
			BIGCHEATS|=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_15);
			
			

		}else{
			IgnoreSignal=1;
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_RESET);

		}
	//}
	prevINTC=curr;
	
	
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
			
			

  while (1){
		interfaceClock=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_4);
		isAlive=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_5);
		Busy=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_6);
//		dll_layer();
		phy_layer();
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_SET);
		interface();
		
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		
  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV4;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 9999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_DOWN;
  htim2.Init.Period = 50;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 39999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_DOWN;
  htim3.Init.Period = 50;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, phy_tx_clock_Pin|phy_tx_data_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, phy_tx_data_bus_bit_0_Pin|phy_tx_data_bus_bit_1_Pin|phy_tx_data_bus_bit_2_Pin|phy_tx_data_bus_bit_3_Pin 
                          |phy_tx_data_bus_bit_4_Pin|phy_tx_data_bus_bit_5_Pin|phy_tx_data_bus_bit_6_Pin|phy_tx_data_bus_bit_7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, phy_tx_data_bus_valid_Pin|pin_interface_clock_Pin|pin_phy_alive_Pin|pin_phy_tx_busy_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : phy_tx_clock_Pin phy_tx_data_Pin */
  GPIO_InitStruct.Pin = phy_tx_clock_Pin|phy_tx_data_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : phy_rx_clock_Pin */
  GPIO_InitStruct.Pin = phy_rx_clock_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(phy_rx_clock_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : phy_tx_data_bus_bit_0_Pin phy_tx_data_bus_bit_1_Pin phy_tx_data_bus_bit_2_Pin phy_tx_data_bus_bit_3_Pin 
                           phy_tx_data_bus_bit_4_Pin phy_tx_data_bus_bit_5_Pin phy_tx_data_bus_bit_6_Pin phy_tx_data_bus_bit_7_Pin */
  GPIO_InitStruct.Pin = phy_tx_data_bus_bit_0_Pin|phy_tx_data_bus_bit_1_Pin|phy_tx_data_bus_bit_2_Pin|phy_tx_data_bus_bit_3_Pin 
                          |phy_tx_data_bus_bit_4_Pin|phy_tx_data_bus_bit_5_Pin|phy_tx_data_bus_bit_6_Pin|phy_tx_data_bus_bit_7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : phy_rx_data_Pin phy_rx_data_bus_bit_2_Pin phy_rx_data_bus_bit_3_Pin phy_rx_data_bus_bit_4_Pin 
                           phy_rx_data_bus_bit_5_Pin phy_rx_data_bus_bit_6_Pin phy_rx_data_bus_bit_7_Pin phy_rx_data_bus_valid_Pin 
                           phy_rx_data_bus_bit_0_Pin phy_rx_data_bus_bit_1_Pin */
  GPIO_InitStruct.Pin = phy_rx_data_Pin|phy_rx_data_bus_bit_2_Pin|phy_rx_data_bus_bit_3_Pin|phy_rx_data_bus_bit_4_Pin 
                          |phy_rx_data_bus_bit_5_Pin|phy_rx_data_bus_bit_6_Pin|phy_rx_data_bus_bit_7_Pin|phy_rx_data_bus_valid_Pin 
                          |phy_rx_data_bus_bit_0_Pin|phy_rx_data_bus_bit_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : phy_tx_data_bus_valid_Pin pin_interface_clock_Pin pin_phy_alive_Pin pin_phy_tx_busy_Pin */
  GPIO_InitStruct.Pin = phy_tx_data_bus_valid_Pin|pin_interface_clock_Pin|pin_phy_alive_Pin|pin_phy_tx_busy_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
