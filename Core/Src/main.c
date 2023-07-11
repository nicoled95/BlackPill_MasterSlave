/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Different patterns for signal multiplexing
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * @contact mail:  nicoledupuy95@gmail.com
  * ***************************************************************************
  * The following program allows control ADG1206 multiplexers.
  * These have their logic control pins connected to the output
  * of the SN74LV595A Shift Registers in cascade.
  * This set up is found on the development PCB:
  * 		"Multiplexer for BIMMS"
  * 			"Nicole Dupuy"
  * 			   "05/2023"
  * Only half of the board was considered.
  * 6 multiplexers connected to 3 shift registers on the right side or BIMMS1
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint8_t dato_tx[1] ; //variable to be transmitted to the shift registers
uint8_t dato_rx[1]; //data to receive of a single value from the Master
uint8_t repetitions = 3; //number of times the cycle is done

HAL_StatusTypeDef stado;//to know if the data is received correctly

/*-------------------------------------------------------------*/
/***************Different types of configurations***************/
/***************************************************************/

uint8_t block1_6[3]={// Array of 3 bytes for transmit 24 bits
		0x01, // First byte 	0000 0001 || Stm-_E1 |	Stm+_E2
		0x23, // Second byte 	0010 0011 || CH2-_E3 |	CH2+_E4
		0x45, // Third byte 	0100 0101 || CH1-_E5 |	CH1+_E6
};

uint8_t block6_1[3]={// Array of 3 bytes for transmit 24 bits
		0x54, // First byte 	0101 0100 || Stm+_E6 |	Stm-_E5
		0x32, // Second byte 	0011 0010 || CH2-_E4 |	CH2+_E3
		0x10, // Third byte 	0001 0000 || CH1-_E2 |	CH1+_E1
};
uint8_t allOdd[8] ={
		0x02, //0000 0000 0000 0000 0000 0010 || X1_E1 |	X2_E3
		0x24, //0000 0000 0000 0000 0010 0100 || X1_E3 |	X2_E5
		0x46, //0000 0000 0000 0000 0100 0110 || X1_E5 |	X2_E7
		0x68, //0000 0000 0000 0000 0110 1000 || X1_E7 |	X2_E9
		0x8A, //0000 0000 0000 0000 1000 1010 || X1_E9 |	X2_E11
		0xAC, //0000 0000 0000 0000 1010 1100 || X1_E11|	X2_E13
		0xCE, //0000 0000 0000 0000 1100 1110 || X1_E13|	X2_E15
		0xE0  //0000 0000 0000 0000 1110 0000 || X1_E15|	X2_E1
};
uint8_t odd_ch1[8][3] ={
	{0x00,0x00,0x02},//(0000 0000) (0000 0000) (0000 0010) || CH1-_E1 |	CH1+_E3
	{0x00,0x00,0x24},//(0000 0000) (0000 0000) (0010 0100) || CH1-_E3 |	CH1+_E5
	{0x00,0x00,0x46},//(0000 0000) (0000 0000) (0100 0110) || CH1-_E5 |	CH1+_E7
	{0x00,0x00,0x68},//(0000 0000) (0000 0000) (0110 1000) || CH1-_E7 |	CH1+_E9
	{0x00,0x00,0x8A},//(0000 0000) (0000 0000) (1000 1010) || CH1-_E9 |	CH1+_E11
	{0x00,0x00,0xAC},//(0000 0000) (0000 0000) (1010 1100) || CH1-_E11|	CH1+_E13
	{0x00,0x00,0xCE},//(0000 0000) (0000 0000) (1100 1110) || CH1-_E13|	CH1+_E15
	{0x00,0x00,0xE0} //(0000 0000) (0000 0000) (1110 0000) || CH1-_E15|	CH1+_E1
};
uint8_t stim_even[8][3] ={
	{0x13,0x00,0x00},//(0001 0011) (0000 0000) (0000 0000) || Stim+_E2 |	Stim-_E4
	{0x35,0x00,0x00},//(0011 0101) (0000 0000) (0000 0000) || Stim+_E4 |	Stim-_E6
	{0x57,0x00,0x00},//(0101 0111) (0000 0000) (0000 0000) || Stim+_E6 |	Stim-_E8
	{0x79,0x00,0x00},//(0111 1001) (0000 0000) (0000 0000) || Stim+_E8 |	Stim-_E10
	{0x9B,0x00,0x00},//(1001 1011) (0000 0000) (0000 0000) || Stim+_E10|	Stim-_E12
	{0xBD,0x00,0x00},//(1011 1101) (0000 0000) (0000 0000) || Stim+_E12|	Stim-_E14
	{0xDF,0x00,0x00},//(1101 1111) (0000 0000) (0000 0000) || Stim+_E14|	Stim-_E16
	{0xF1,0x00,0x00} //(0000 0000) (0000 0000) (0000 0000) || Stim+_E16|	Stim-_E2
};
uint16_t from1to16ch1[16] = { //1 from first to last
		0b00000000, //1
		0b00010001, //2
		0b00100010, //3
		0b00110011, //4
		0b01000100, //5
		0b01010101, //6
		0b01100110, //7
		0b01110111, //8
		0b10001000, //9
		0b10011001, //10
		0b10101010, //11
		0b10111011, //12
		0b11001100, //13
		0b11011101, //14
		0b11101110, //15
		0b11111111, //16
};
uint16_t from16to1ch1 [16] = { // 1 from last to first
		0b11111111, //16
		0b11101110, //15
		0b11011101, //14
		0b11001100, //13
		0b10111011, //12
		0b10101010, //11
		0b10011001, //10
		0b10001000, //9
		0b01110111, //8
		0b01100110, //7
		0b01010101, //6
		0b01000100, //5
		0b00110011, //4
		0b00100010, //3
		0b00010001, //2
		0b00000000, //1
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*-------------------------------------------------------------*/
/******************Functions for the Patterns*******************/
/***************************************************************/
void Blocked1_6(){
	/*This function blocks the channels on the electrodes from 1 to 6 in order.*/
	/*Stim- Stim+ CH2- CH2+ CH1- CH1+*/
		 HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, 0); //selector of slave, active on LOW state
		 HAL_SPI_Transmit(&hspi2, (uint8_t*)(block1_6),3, HAL_MAX_DELAY); //(SPI used , buffer, size of data, time of transmission)
		 HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, 1); // disable the slave with a HIGH state
}
void Blocked6_1(){
	/*This function blocks the channels on the electrodes from 6 to 1 in order.*/
	/*CH1+ CH1- CH2+ CH2- Stim+ Stim-*/
		 HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, 0); //selector of slave, active on LOW state
		 HAL_SPI_Transmit(&hspi2, (uint8_t*)(block6_1),3, HAL_MAX_DELAY); //(SPI used , buffer, size of data, time of transmission)
		 HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, 1); // disable the slave with a HIGH state
}
void Odd_All_Chs(){
	/*This function allows injecting two signals into any channels
	 *  by opening the electrodes in pairs:
	 *  1-3 || 3-5 || 5-7 || 7-9 || 9-11 || 11-13 || 13-15 || 15-1*/
	for (int i=0;i<=7;i++){
		HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, 0);//selector of slave, active on LOW state
		dato_tx[0] = allOdd[i];
		HAL_SPI_Transmit(&hspi2, (uint8_t*)(dato_tx),1, HAL_MAX_DELAY); //(SPI used , buffer, size of data, time of transmission)
		HAL_Delay(200); //delay for being abble to see the signals. comment if you dont need it
		HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, 1); // disable the slave with a HIGH state
	}}
void Odd_CH1(){
	/*This function allows injecting two signals into CH1+ AND CH1-
		 *  by opening the electrodes in pairs:
		 *  1-3 || 3-5 || 5-7 || 7-9 || 9-11 || 11-13 || 13-15 || 15-1 .*/
	 for (int i=0;i<=8;i++){
		 	 for (int j=0;j<=3;j++){
		 		 	 dato_tx[j] = odd_ch1[i][j];
		 HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, 0); //selector of slave, active on LOW state
		 HAL_SPI_Transmit(&hspi2, (uint8_t*)(dato_tx),3, HAL_MAX_DELAY); //(SPI used , buffer, size of data, time of transmission)
		 HAL_Delay(200);
		 HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, 1); // disable the slave with a HIGH state
	}		}}
void Even_Stim(){
	/*This function allows injecting two signals into Stim+ AND Stim-
		 *  by opening the electrodes in pairs:
		 *  2-4 || 4-6 || 6-8 || 8-10 || 10-12 || 12-14 || 14-16 || 16-2 .*/
	 for (int i=0;i<=8;i++){
		 	 for (int j=0;j<=3;j++){
		 		 	 dato_tx[j] = stim_even[i][j];
		HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, 0);//selector of slave, active on LOW state
		HAL_SPI_Transmit(&hspi2, (uint8_t*)(dato_tx),3, HAL_MAX_DELAY); //(SPI used , buffer, size o data, time of transmission)
		HAL_Delay(200);
		HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, 1); // disable the slave with a HIGH state
	}	}}
void  Rising(){
	/*This function enables the electrodes in order, from 1 to 16,
	with any channel.*/
	for (int i=0;i<=15;i++){ //Display from 1 to 16
		HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, 0); //selector of slave, active on LOW state
		dato_tx[0] = from1to16ch1[i];
		HAL_SPI_Transmit(&hspi2, (uint8_t*)(dato_tx),1, HAL_MAX_DELAY); //(SPI used , buffer, size o data, time of transmission)
		HAL_Delay(200);
		HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, 1); // disable the slave with a HIGH state
	}	}
void Falling(){
	/*This function enables the electrodes in order, from 16 to 1,
	with any channel.*/
	for (int i=0;i<=15;i++){ //Display from 0 to 16
		HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, 0);//selector of slave, active on LOW state
		dato_tx[0] = from16to1ch1[i];
		HAL_SPI_Transmit(&hspi2, (uint8_t*)(dato_tx),1, HAL_MAX_DELAY); //(SPI used , buffer, size o data, time of transmission)
		HAL_Delay(200);
		HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, 1); // disable the slave with a HIGH state
	}	}


/* USER CODE END 0 */

/** MAIN FUNCTION*********************************************************************************************************************
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_SPI1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  	  	  HAL_GPIO_WritePin(GPIOC, LED_pin13_Pin,GPIO_PIN_SET); //LED Incorporated
  	  	  HAL_GPIO_WritePin(GPIOC, ENABLE_B1_Pin,GPIO_PIN_RESET);//set ENABLE B1
  	  	  HAL_GPIO_WritePin(GPIOC, ENABLE_B2_Pin,GPIO_PIN_RESET);//set ENABLE B2
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_GPIO_WritePin(GPIOC, ENABLE_B1_Pin,GPIO_PIN_SET);//ENABLE B1 ON
	  HAL_GPIO_WritePin(GPIOC, ENABLE_B2_Pin,GPIO_PIN_SET);//ENABLE B2 ON

	stado = HAL_SPI_Receive(&hspi1,dato_rx, 1, HAL_MAX_DELAY);//Control variable
		if(stado == HAL_OK){//if the data is well received
			 HAL_GPIO_WritePin(GPIOC, LED_pin13_Pin,GPIO_PIN_RESET);//LED BLUE BLACKPILL ON
			switch (dato_rx[0]){ // Analysis of the command received from the master.
			   case 0x01:
				   for (int i = 0; i < repetitions ; i++) {
					   Blocked1_6(); }
					/*This function blocks the channels on the electrodes from 1 to 6 in order.*/
					/*Stim- Stim+ CH2- CH2+ CH1- CH1+*/
				   break;
			   case 0x02:
				   for (int i = 0; i < repetitions ; i++) {
					   Blocked6_1(); }
					/*This function blocks the channels on the electrodes from 6 to 1 in order.*/
					/*CH1+ CH1- CH2+ CH2- Stim+ Stim-*/
					break;
			   case 0x03:
				   for (int i = 0; i < repetitions ; i++) {
					   Odd_All_Chs() ; }
					/*This function allows injecting two signals into any channels
					 *  by opening the electrodes in pairs:
					 *  1-3 || 3-5 || 5-7 || 7-9 || 9-11 || 11-13 || 13-15 || 15-1*/
					break;
			   case 0x04:
				   for (int i = 0; i < repetitions ; i++) {
					   Odd_CH1(); }
					/*This function allows injecting two signals into CH1+ AND CH1-
						 *  by opening the electrodes in pairs:
						 *  1-3 || 3-5 || 5-7 || 7-9 || 9-11 || 11-13 || 13-15 || 15-1 .*/
					break;
			   case 0x05:
				   for (int i = 0; i < repetitions ; i++) {
					   Even_Stim(); }
					/*This function allows injecting two signals into Stim+ AND Stim-
						 *  by opening the electrodes in pairs:
						 *  2-4 || 4-6 || 6-8 || 8-10 || 10-12 || 12-14 || 14-16 || 16-2 .*/
					break;
			   case 0x06:
				   for (int i = 0; i < repetitions ; i++) {
				  Rising(); }
					/*This function enables the electrodes in order, from 1 to 16,
					with any channel.*/
					break;
			   case 0x07:
				   for (int i = 0; i < repetitions ; i++) {
			      Falling(); }
					/*This function enables the electrodes in order, from 16 to 1,
					with any channel.*/
					break;

			   default : //default block
				  break;
			}
		}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		HAL_GPIO_WritePin(GPIOC, LED_pin13_Pin,GPIO_PIN_SET);//LED BLUE BLACKPILL OFF
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
