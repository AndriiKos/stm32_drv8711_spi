#include "main.h"
#include <stdio.h>

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

int main(void)
{
  char uart_buf[50];
  int uart_buf_len;
  char spi_buf[20];
  char spi_buf_rx[20];

  uint16_t regValue;
  
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

  while (1){

  }
}
