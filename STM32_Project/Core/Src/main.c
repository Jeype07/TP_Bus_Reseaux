/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct BMP280_CalibDataNames{ // structure containing calibration registers names (datasheet p.21)
	uint16_t dig_T1;  //0x88/0x89
	int16_t dig_T2;   //0x8A/0x8B
	int16_t dig_T3;
	uint16_t dig_P1;
	int16_t dig_P2;
	int16_t dig_P3;
	int16_t dig_P4;
	int16_t dig_P5;
	int16_t dig_P6;
	int16_t dig_P7;
	int16_t dig_P8;  //0x9C/0x9D
	int16_t dig_P9;  //0x9E/0x9F
} BMP280_CalibDataNames;

typedef int32_t BMP280_S32_t;
typedef uint32_t BMP280_U32_t;
typedef int64_t BMP280_S64_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BMP_ADDR 0x77<<1 // BMP280 address
#define BMP_ID_REG 0xD0 // adress of the ID register

#define BMP_ADDR_MODE 0xF4 // address of the "ctrl_meas" reg to set the modes/config
#define BMP_MODE 0b01010111 // 010 oversampling t x2  101 oversampling p x16	11 mode normal

#define BMP_CALIB_REG 0x88    // 1st calibration register address
#define BMP_CALIB_DATA_LENGTH 24 // size in bytes of calibration data

#define BMP_TEMP_PRESS_REG 0xF7 // 1st press register address
#define BMP_TEMP_PRESS_DATA_LENGTH 3 // size of press + temp registers

#define SIZE_OF_USART4_BUF 7
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t data;
uint8_t UART4_rxBuffer[SIZE_OF_USART4_BUF];

// Variables initialization, buffers
//I2C
uint8_t id_buf[1];
uint8_t data_config[2];
uint8_t calib_reg[1];
uint8_t calib_data[BMP_CALIB_DATA_LENGTH];
BMP280_CalibDataNames *calib_names;
uint8_t temp_reg = 0xFA;
uint8_t press_reg = 0xF7;
//int32_t raw_temp;
//int32_t raw_press;
int32_t t_fine;

// Variables CAN
CAN_TxHeaderTypeDef   TxHeader;
uint8_t               TxData[3];
uint32_t              TxMailbox;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN1_Init(void);
static void MX_UART4_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

void User_UartCompleteCallback(UART_HandleTypeDef *huart);

void read_calibration_data();

void read_raw_t_p(int32_t *raw_press, int32_t *raw_temp);

void query_ID_BMP();

void query_Config_BMP();

void query_Calib_BMP();

void conf_CAN();

void get_BMP_meas();

void rotate_motor_90d();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void query_ID_BMP(){
	//question réponse capteur avec I2C pour ID capteur
	id_buf[0]= BMP_ID_REG;
	HAL_I2C_Master_Transmit(&hi2c1,BMP_ADDR,id_buf,1,HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(&hi2c1,BMP_ADDR,id_buf,1,HAL_MAX_DELAY);
	printf("BMP280 ID : %x\r\n",id_buf[0]);
}

void query_Config_BMP(){
	//Configuration et vérification du capteur
	data_config[0]= BMP_ADDR_MODE;
	data_config[1]= BMP_MODE;
	printf("Register : %x\r\n",data_config[0]);
	HAL_I2C_Master_Transmit(&hi2c1,BMP_ADDR,data_config,2,HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(&hi2c1,BMP_ADDR,data_config,2,HAL_MAX_DELAY);
	printf("Mode : %x\r\n",data_config[0]);
}

void query_Calib_BMP(){
	// Retrieving of calibration Data
	calib_reg[0] = BMP_CALIB_REG;
	HAL_I2C_Master_Transmit(&hi2c1, BMP_ADDR, calib_reg, 1, HAL_MAX_DELAY);
	if (HAL_I2C_Master_Receive(&hi2c1, BMP_ADDR, calib_data, BMP_CALIB_DATA_LENGTH, HAL_MAX_DELAY)== HAL_OK){
		calib_names->dig_T1 = (uint16_t)((calib_data[1] << 8) | calib_data[0]);
		calib_names->dig_T2 = (int16_t)((calib_data[3] << 8) | calib_data[2]);
		calib_names->dig_T3 = (int16_t)((calib_data[5] << 8) | calib_data[4]);
		calib_names->dig_P1 = (uint16_t)((calib_data[7] << 8) | calib_data[6]);
		calib_names->dig_P2 = (int16_t)((calib_data[9] << 8) | calib_data[8]);
		calib_names->dig_P3 = (int16_t)((calib_data[11] << 8) | calib_data[10]);
		calib_names->dig_P4 = (int16_t)((calib_data[13] << 8) | calib_data[12]);
		calib_names->dig_P5 = (int16_t)((calib_data[15] << 8) | calib_data[14]);
		calib_names->dig_P6 = (int16_t)((calib_data[17] << 8) | calib_data[16]);
		calib_names->dig_P7 = (int16_t)((calib_data[19] << 8) | calib_data[18]);
		calib_names->dig_P8 = (int16_t)((calib_data[21] << 8) | calib_data[20]);
		calib_names->dig_P9 = (int16_t)((calib_data[23] << 8) | calib_data[22]);

		printf("Calibration Data:\r\n");
		printf("dig_T1: %u\r\n", calib_names->dig_T1);
		printf("dig_T2: %d\r\n", calib_names->dig_T2);
		printf("dig_T3: %d\r\n", calib_names->dig_T3);
		printf("dig_P1: %u\r\n", calib_names->dig_P1);
		printf("dig_P2: %d\r\n", calib_names->dig_P2);
		printf("dig_P3: %d\r\n", calib_names->dig_P3);
		printf("dig_P4: %d\r\n", calib_names->dig_P4);
		printf("dig_P5: %d\r\n", calib_names->dig_P5);
		printf("dig_P6: %d\r\n", calib_names->dig_P6);
		printf("dig_P7: %d\r\n", calib_names->dig_P7);
		printf("dig_P8: %d\r\n", calib_names->dig_P8);
		printf("dig_P9: %d\r\n", calib_names->dig_P9);
	}
	else{
		printf("Erreur de calibration\r\n");
	}


}
/*
void read_raw_t_p(int32_t *raw_press, int32_t *raw_temp){
	uint8_t raw_data[BMP_TEMP_PRESS_DATA_LENGTH];
	uint8_t reg = BMP_TEMP_PRESS_REG;

	if (HAL_I2C_Master_Transmit(&hi2c1, BMP_ADDR, &reg, 1, HAL_MAX_DELAY)==HAL_OK){
		HAL_I2C_Master_Receive(&hi2c1, BMP_ADDR, raw_data, BMP_TEMP_PRESS_DATA_LENGTH, HAL_MAX_DELAY);
		*raw_press = (int32_t)(((raw_data[0] << 16) | (raw_data[1] << 8) | raw_data[2]) >> 4);
		*raw_temp = (int32_t)(((raw_data[3] << 16) | (raw_data[4] << 8) | raw_data[5]) >> 4);
	}
	else{
		printf("Erreur de communication sur le bus bus I2C\r\n");
	}
}

*/

int bmp280_get_raw_temp(void){
	uint8_t raw_data[BMP_TEMP_PRESS_DATA_LENGTH];
	if(HAL_I2C_Master_Transmit(&hi2c1,BMP_ADDR, &temp_reg, 1, HAL_MAX_DELAY) != HAL_OK){
		printf("Erreur de communication sur le bus bus I2C (temp)\r\n");
	}

	else{
		HAL_I2C_Master_Receive(&hi2c1, BMP_ADDR, raw_data, 3, HAL_MAX_DELAY);
	}
	int raw_temp = raw_data[0]<<12|raw_data[1]<<4|raw_data[2]>>4;
	return raw_temp;
}

int bmp280_get_raw_press(void){
	uint8_t raw_data[BMP_TEMP_PRESS_DATA_LENGTH];
	if(HAL_I2C_Master_Transmit(&hi2c1,BMP_ADDR, &press_reg, 1, HAL_MAX_DELAY) != HAL_OK){
		printf("Erreur de communication sur le bus bus I2C (press)\r\n");
	}
	else{
		HAL_I2C_Master_Receive(&hi2c1, BMP_ADDR, raw_data, 3, HAL_MAX_DELAY);
	}
	int raw_pres = raw_data[0]<<12|raw_data[1]<<4|raw_data[2]>>4;
	return raw_pres;
}


// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123�? equals 51.23 DegC.
// t_fine carries fine temperature as global value

BMP280_S32_t bmp280_compensate_T_int32(BMP280_S32_t adc_T,BMP280_CalibDataNames *calib_names, int32_t *t_fine)
{
	BMP280_S32_t var1, var2, T;
	var1 = ((((adc_T>>3)-((BMP280_S32_t)calib_names->dig_T1<<1))) * ((BMP280_S32_t)calib_names->dig_T2)) >> 11;
	var2 = (((((adc_T>>4)-((BMP280_S32_t)calib_names->dig_T1)) * ((adc_T>>4)-((BMP280_S32_t)calib_names->dig_T1))) >> 12) * ((BMP280_S32_t)calib_names->dig_T3)) >> 14;
	*t_fine = var1 + var2;
	T = (*t_fine * 5 + 128) >> 8;
	return T;
}

// Returns pressure in Pa as unsigned 32 bit integer. Output value of “96386�? equals 96386 Pa = 963.86 hPa
BMP280_U32_t bmp280_compensate_P_int64(BMP280_S32_t adc_P, BMP280_CalibDataNames *calib_names, int32_t t_fine)
{
	BMP280_S64_t var1, var2, p;
	var1 = ((BMP280_S64_t)t_fine)-128000;
	var2 = var1 * var1 * (BMP280_S64_t)calib_names->dig_P6;
	var2 = var2 + ((var1*(BMP280_S64_t)calib_names->dig_P5)<<17);
	var2 = var2 + (((BMP280_S64_t)calib_names->dig_P4)<<35);
	var1 = ((var1 * var1 * (BMP280_S64_t)calib_names->dig_P3)>>8) + ((var1 * (BMP280_S64_t)calib_names->dig_P2)<<12);
	var1 = (((((BMP280_S64_t)1)<<47)+var1))*((BMP280_S64_t)calib_names->dig_P1)>>33;
	if (var1 == 0)
	{
		return 0; // avoid exception caused by division by zero
	}
	p = 1048576-adc_P;
	p = (((p<<31)-var2)*3125)/var1;
	var1 = (((BMP280_S64_t)calib_names->dig_P9) * (p>>13) * (p>>13)) >> 25;
	var2 = (((BMP280_S64_t)calib_names->dig_P8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((BMP280_S64_t)calib_names->dig_P7)<<4);
	return (BMP280_U32_t)p;
}

void get_BMP_meas(){
	//Retrieving the raw temp and press values
	int32_t raw_temp = bmp280_get_raw_temp();
	int32_t raw_press = bmp280_get_raw_press();

	// Compensated temperature and pressure
	int32_t temp = bmp280_compensate_T_int32(raw_temp, calib_names, &t_fine);
	int32_t press = bmp280_compensate_P_int64(raw_press, calib_names, t_fine);
	printf("Compensated Temp = %ld C\r\nCompensated Press = %ld Pa\r\n", temp/100, press/256);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_UART_Receive_IT (&huart4, (uint8_t*)UART4_rxBuffer, SIZE_OF_USART4_BUF);
	printf("%s\r\n",UART4_rxBuffer);
}

void conf_CAN(){
	// Motor pilot : +90 degree
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.StdId = 0x61;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = 2;
	TxHeader.TransmitGlobalTime = DISABLE;

	TxData[0] = 90; 	//angle
	TxData[1] = 0x00;	//positive
	//TxData[2] = 0xA0;	//speed


	HAL_CAN_Start(&hcan1);
}

void rotate_motor_90d(){
	if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox)!= HAL_OK){
		printf("Erreur de communication sur le bus CAN\r\n");
	}
	else{
		printf("Commnication CAN etablie\r\n");
	}

	if(TxData[1]==1){
		TxData[1] = 0;
	}
	else{
		TxData[1]=1;
	}
	HAL_Delay(1000);
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
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
	MX_USART2_UART_Init();
	MX_CAN1_Init();
	MX_UART4_Init();
	MX_TIM2_Init();
	MX_I2C1_Init();
	/* USER CODE BEGIN 2 */
	HAL_UART_RegisterCallback(&huart4, HAL_UART_RX_COMPLETE_CB_ID, User_UartCompleteCallback);
	HAL_TIM_Base_Start_IT(&htim2);
	conf_CAN();
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	printf("==== TP BUS & NETWORK ====\r\n");

	// get id, calib, and config data
	query_ID_BMP();
	query_Config_BMP();
	query_Calib_BMP();

	HAL_UART_Receive_IT (&huart4, (uint8_t *) UART4_rxBuffer, SIZE_OF_USART4_BUF); //usart rasp pi


	while (1)
	{
		/* USER CODE END WHILE */
		get_BMP_meas();
		rotate_motor_90d();
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
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 180;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Activate the Over-Drive mode
	 */
	if (HAL_PWREx_EnableOverDrive() != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief CAN1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN1_Init(void)
{

	/* USER CODE BEGIN CAN1_Init 0 */

	/* USER CODE END CAN1_Init 0 */

	/* USER CODE BEGIN CAN1_Init 1 */

	/* USER CODE END CAN1_Init 1 */
	hcan1.Instance = CAN1;
	hcan1.Init.Prescaler = 5;
	hcan1.Init.Mode = CAN_MODE_NORMAL;
	hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan1.Init.TimeSeg1 = CAN_BS1_15TQ;
	hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
	hcan1.Init.TimeTriggeredMode = DISABLE;
	hcan1.Init.AutoBusOff = DISABLE;
	hcan1.Init.AutoWakeUp = DISABLE;
	hcan1.Init.AutoRetransmission = DISABLE;
	hcan1.Init.ReceiveFifoLocked = DISABLE;
	hcan1.Init.TransmitFifoPriority = DISABLE;
	if (HAL_CAN_Init(&hcan1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN CAN1_Init 2 */

	/* USER CODE END CAN1_Init 2 */

}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void)
{

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 180-1;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 1000000-1;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

/**
 * @brief UART4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_UART4_Init(void)
{

	/* USER CODE BEGIN UART4_Init 0 */

	/* USER CODE END UART4_Init 0 */

	/* USER CODE BEGIN UART4_Init 1 */

	/* USER CODE END UART4_Init 1 */
	huart4.Instance = UART4;
	huart4.Init.BaudRate = 115200;
	huart4.Init.WordLength = UART_WORDLENGTH_8B;
	huart4.Init.StopBits = UART_STOPBITS_1;
	huart4.Init.Parity = UART_PARITY_NONE;
	huart4.Init.Mode = UART_MODE_TX_RX;
	huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart4.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart4) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN UART4_Init 2 */

	/* USER CODE END UART4_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LD2_Pin */
	GPIO_InitStruct.Pin = LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void User_UartCompleteCallback(UART_HandleTypeDef *huart)
{
	printf("j'ai reçu des datas\r\n");
}



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
