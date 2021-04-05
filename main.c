/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CTRL 0b000;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

ETH_HandleTypeDef heth;

SPI_HandleTypeDef hspi4;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
//SPI adresses Motor Driver DRV8711
const uint8_t CTRL_addr = 0x0;
const uint8_t TORQUE_addr = 0x1;
const uint8_t OFF_addr = 0x2;
const uint8_t BLANK_addr = 0x3;
const uint8_t DECAY_addr = 0x4;
const uint8_t STALL_addr = 0x5;
const uint8_t DRIVE_addr = 0x6;
const uint8_t STATUS_addr = 0x7;
//SPI default values Motor Driver DRV8711
const uint16_t CTRL_defaultValue = 0b110000010000;
const uint16_t TORQUE_defaultValue = 0b000111111111;
const uint16_t OFF_defaultValue = 0b000000110000;
const uint16_t BLANK_defaultValue = 0b000010000000;
const uint16_t DECAY_defaultValue = 0b000100010000;
const uint16_t STALL_defaultValue = 0b000001000000;
const uint16_t DRIVE_defaultValue = 0b101001011001;
const uint16_t STATUS_defaultValue = 0b000000000000;
//SPI slaves
const uint8_t motor_1 = 1;
const uint8_t motor_2 = 2;
const uint8_t motor_3 = 3;
const uint8_t motor_4 = 4;
const uint8_t motor_5 = 5;
const uint8_t motor_6 = 6;
const uint8_t adc_1 = 7;
const uint8_t adc_2 = 8;
const uint8_t adc_3 = 9;
const uint8_t adc_4 = 10;
const uint8_t adc_5 = 11;
const uint8_t adc_6 = 12;
/*
const uint8_t motor_2 = 2;
const uint8_t motor_3 = 3;
const unsigned char motor_4 = 4;
const unsigned char motor_5 = 5;
const unsigned char motor_6 = 6;
*/
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_SPI4_Init(void);
/* USER CODE BEGIN PFP */
// Writes a high or low value to the direction pin to specify what direction to turn the motor.
  void motor_set_direction(unsigned char motor, unsigned char dir){
    // The STEP pin must not change for at least 200 nanoseconds before and after changing the DIR pin.
	HAL_Delay(1);
	  if(motor == 1){
		  if(dir == 1){
			  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET);
		  }else{
			  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET);
		  }
	  }else if(motor == 2){
		  if(dir == 1){
			  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET);
		  }else{
			  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET);
		  }
	  }else{
		  //all other motors go here
	  }
    HAL_Delay(1);
  }

  void motor_do_steps(unsigned char motor, unsigned int numberOfSteps){
	  for(unsigned int x = 0; x < numberOfSteps; x++){
		// The STEP minimum high pulse width is 1.9 microseconds.
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, GPIO_PIN_SET);
		HAL_Delay(1);
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, GPIO_PIN_RESET);
		HAL_Delay(1);
	  }
  }

  void uart_send(char* uart_buffer, char message[50]){
	  int uart_buffer_length;
	  uart_buffer_length = sprintf(uart_buffer, message);
	  HAL_UART_Transmit(&huart3, (uint8_t *)uart_buffer, uart_buffer_length, 100);
  }

  ///select an SPI slave to communicate with it
  void slave_select(uint8_t slave, uint8_t select){
	  //deselect all slaves
	  //for motor drivers set CS low to deselect
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
	  //for ADU set CS high to deselect
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
	  if(select == 1){
		  if(slave == motor_1){
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET); //select motor - set CS high
		  }else if(slave == adc_1){
			  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET); //select adu - set CS low
		  }else{
			  //all slaves stay deselected
		  }
	  }else{
		  //all slaves stay deselected
	  }
  }

  /// Reads the register at the given address and returns its raw value.
  uint16_t readReg(uint8_t slave, uint8_t address){
    uint8_t COMMAND = ( (0x8 | (address & 0b111)) );
	char spi_buf_rx[2];
	spi_buf_rx[0] = 0x00;
	spi_buf_rx[1] = 0x00;
	spi_buf_rx[2] = 0x00;
	slave_select(slave, 1);
    HAL_SPI_Transmit(&hspi4, (uint8_t *)&COMMAND, 1, 100);
    HAL_SPI_Receive(&hspi4, (uint8_t *)spi_buf_rx, 3, 100);
    slave_select(slave, 0);
    uint16_t dataOut = ( ((unsigned int)spi_buf_rx[0] << 8) |
    		(unsigned int)spi_buf_rx[1] << 4 | (unsigned int)spi_buf_rx[2]);
    return dataOut & 0xFFF;
  }
  /// Reads the register at the given address and returns its raw value.
  void readReg_to_uart(uint8_t slave, uint8_t address){
    uint8_t COMMAND = ( (0x8 | (address & 0b111)) );
	char spi_buf_rx[2];
	//spi_buf_rx[0] = 0x00;
	//spi_buf_rx[1] = 0x00;
	//spi_buf_rx[2] = 0x00;
	slave_select(slave, 1);
    HAL_SPI_Transmit(&hspi4, (uint8_t *)&COMMAND, 1, 100);
    HAL_SPI_Receive(&hspi4, (uint8_t *)spi_buf_rx, 3, 100);
    slave_select(slave, 0);
    uint16_t dataOut = ( ((unsigned int)spi_buf_rx[0] << 8) |
    		(unsigned int)spi_buf_rx[1] << 4 | (unsigned int)spi_buf_rx[2]);
    //UART for debugging
    char uart_buf[50];
    int uart_buf_len;
    uart_buf_len = sprintf(uart_buf,"slave %x, reg %x reads: %x \r\n",
    		(unsigned int)slave, (unsigned int)address,
			(unsigned int)dataOut & 0b111111111111);
    HAL_UART_Transmit(&huart3, (uint8_t *)uart_buf, uart_buf_len, 100);
  }

  /// Writes the specified value to a specified register.
  void writeReg(unsigned char slave, uint8_t address, uint16_t data){
	//4 bit address
	uint8_t ADDR = ( (address & 0b111) );
	//12 bit data
	uint8_t DATA_3 = (uint8_t)((data & 0xF00) >> 8);
	uint8_t DATA_2 = (uint8_t)((data & 0x0F0) >> 4);
	uint8_t DATA_1 = (uint8_t)((data & 0x00F) );
	slave_select(slave, 1);
	HAL_SPI_Transmit(&hspi4, (uint8_t *)&ADDR, 1, 100);
	HAL_SPI_Transmit(&hspi4, (uint8_t *)&DATA_3, 1, 100);
	HAL_SPI_Transmit(&hspi4, (uint8_t *)&DATA_2, 1, 100);
	HAL_SPI_Transmit(&hspi4, (uint8_t *)&DATA_1, 1, 100);
	slave_select(slave, 0);
  }

  void motor_enable(unsigned char motor, unsigned char enable){
	uint16_t valueToSend;
	//build the needed command
	if(enable == 1){
		valueToSend = readReg(motor, CTRL_addr) | 0b000000000001; //to enable motor
	}else{
		valueToSend = readReg(motor, CTRL_addr) & 0b111111111110; //to disable motor
	}
	//send command via SPI
	writeReg(motor, CTRL_addr, valueToSend);
  }

  uint8_t motorIsEnabled(unsigned char motor){
	  uint8_t dataOut;
	  dataOut = readReg(motor, CTRL_addr) & 0b000000000001;
	  return dataOut & 0xFF;
  }

  //write default values to motor driver registers
  void motor_set_default(unsigned char motor){
	  writeReg(motor, CTRL_addr, CTRL_defaultValue);
	  writeReg(motor, TORQUE_addr, TORQUE_defaultValue);
	  writeReg(motor, OFF_addr, OFF_defaultValue);
	  writeReg(motor, BLANK_addr, BLANK_defaultValue);
	  writeReg(motor, DECAY_addr, DECAY_defaultValue);
	  writeReg(motor, STALL_addr, STALL_defaultValue);
	  writeReg(motor, DRIVE_addr, DRIVE_defaultValue);
	  writeReg(motor, STATUS_addr, STATUS_defaultValue);
  }

  void motor_set_microstep(unsigned char motor, unsigned int microstepDivisor){
	  //default is 0010: 1/4 step
	  //1001 – 1111 is Reserved
	  uint16_t bits;
	  uint16_t valueToSend;
	  if(microstepDivisor == 1){
		  bits = 0b0000; //0000: Full-step, 71% current
	  }else if(microstepDivisor == 2){
		  bits = 0b0001; //0001: Half step
	  }else if(microstepDivisor == 4){
		  bits = 0b0010; //0010: 1/4 step
	  }else if(microstepDivisor == 8){
		  bits = 0b0011; //0011: 1/8 step
	  }else if(microstepDivisor == 16){
		  bits = 0b0100; //0100: 1/16 step
	  }else if(microstepDivisor == 32){
		  bits = 0b0101; //0101: 1/32 step
	  }else if(microstepDivisor == 64){
		  bits = 0b0110; //0110: 1/64 step
	  }else if(microstepDivisor == 128){
		  bits = 0b0111; //0111: 1/128 step
	  }else if(microstepDivisor == 256){
		  bits = 0b1000; //1000: 1/256 step
	  }else{
		  bits = 0b0010; //set default - 0010: 1/4 step
	  }
	  valueToSend = readReg(motor, CTRL_addr) & 0b111110000111; //erase this bits
	  valueToSend = valueToSend | (bits << 3); //put there new bits
	  //send to UART
	  /*
		char uart_buf[50];
		int uart_buf_len;
		uart_buf_len = sprintf(uart_buf,"motor_set_microstep: %x \r\n", (unsigned int)valueToSend);
		HAL_UART_Transmit(&huart3, (uint8_t *)uart_buf, uart_buf_len, 100);
		*/
		//send to SPI
	  writeReg(motor, CTRL_addr, valueToSend);
  }

  ///set current 0...255
  void motor_set_current(unsigned char motor, unsigned int current){
	  uint16_t valueToSend;
	  if(current > 255){
		  current = 255;
	  }
	  valueToSend = readReg(motor, TORQUE_addr) & 0b111100000000; //erase this bits
	  valueToSend = valueToSend | current; //put there new bits
	  writeReg(motor, TORQUE_addr, valueToSend);
  }



/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  char uart_buf[50];
  int uart_buf_len;
  char spi_buf[20];
  char spi_buf_rx[20];

  uint16_t regValue;
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
  MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_SPI4_Init();
  /* USER CODE BEGIN 2 */
  uart_send(uart_buf, "start\r\n");

  //rotate
  motor_enable(motor_1, 1);

  motor_set_current(motor_1, 250);
  readReg_to_uart(motor_1, TORQUE_addr);
  readReg_to_uart(motor_1, CTRL_addr);
  motor_set_microstep(motor_1, 256);
  motor_do_steps(motor_1, 1000);
  HAL_Delay(1000);

  motor_enable(motor_1, 0);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1){

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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure LSE Drive Capability 
  */
  HAL_PWR_EnableBkUpAccess();
  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

   uint8_t MACAddr[6] ;

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  heth.Init.AutoNegotiation = ETH_AUTONEGOTIATION_ENABLE;
  heth.Init.PhyAddress = LAN8742A_PHY_ADDRESS;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.RxMode = ETH_RXPOLLING_MODE;
  heth.Init.ChecksumMode = ETH_CHECKSUM_BY_HARDWARE;
  heth.Init.MediaInterface = ETH_MEDIA_INTERFACE_RMII;

  /* USER CODE BEGIN MACADDRESS */
    
  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 7;
  hspi4.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi4.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0|USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PG0 USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_0|USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PD1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
