/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "usbgpib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define false (0)
#define true  (1)

#define buf_size (235)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
const unsigned int version = 5;

char cmd_buf[10], buf[buf_size +20];
unsigned int buf_out = 0;
unsigned int buf_in = 0;

int partnerAddress = 1;
int myAddress;

char eos = 10; // Default end of string character.
char eos_string[3] = "";
char eos_code = 3;
char eoiUse = 1; // By default, we are using EOI to signal end of
                 // msg from instrument
char debug = 0; // enable or disable read&write error messages

uint8_t strip = 0;
char autoread = 1;
char eot_enable = 1;
char eot_char = 13; // default CR
char listen_only = 0;
char mode = 1;
char save_cfg = 1;
unsigned int status_byte = 0;

uint32_t timeout = 1000;
uint32_t seconds = 0;

// Variables for device mode
uint8_t device_talk = false;
uint8_t device_listen = false;
uint8_t device_srq = false;

// EEPROM variables
const char VALID_EEPROM_CODE = 0xAA;

#define WITH_TIMEOUT
#define WITH_WDT
//#define VERBOSE_DEBUG
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void GPIO_Output_High(GPIO_TypeDef *port, uint32_t pin);
void GPIO_Output_Low(GPIO_TypeDef *port, uint32_t pin);
void GPIO_Output_Float(GPIO_TypeDef *port, uint32_t pin);
void prep_gpib_pins(void);
void gpib_init(void);
char gpib_controller_assign(int address);
char gpib_cmd(char *bytes, int length);
char gpib_write(char *bytes, int length, uint8_t useEOI);
char _gpib_write(char *bytes, int length, uint8_t attention, uint8_t useEOI);
/**
 *  @see https://github.com/STMicroelectronics/STM32CubeF4/blob/master/Projects/STM32F401RE-Nucleo/Examples/UART/UART_Printf/Src/main.c
 */
#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
 set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

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
  /* USER CODE BEGIN 2 */
  printf("GPIB USB\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DC_Pin|SC_Pin|PE_Pin|TE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DIO1_Pin DIO2_Pin DIO3_Pin DIO4_Pin
                           DIO5_Pin DIO6_Pin DIO7_Pin DIO8_Pin */
  GPIO_InitStruct.Pin = DIO1_Pin|DIO2_Pin|DIO3_Pin|DIO4_Pin
                          |DIO5_Pin|DIO6_Pin|DIO7_Pin|DIO8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : DC_Pin SC_Pin PE_Pin TE_Pin */
  GPIO_InitStruct.Pin = DC_Pin|SC_Pin|PE_Pin|TE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SRQ_Pin ATN_Pin */
  GPIO_InitStruct.Pin = SRQ_Pin|ATN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : EOI_Pin DAV_Pin BOOT1_Pin NRFD_Pin
                           NDAC_Pin IFC_Pin REN_Pin */
  GPIO_InitStruct.Pin = EOI_Pin|DAV_Pin|BOOT1_Pin|NRFD_Pin
                          |NDAC_Pin|IFC_Pin|REN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/**
 * @brief  Retargets the C library printf function to the USART.
 * @param  None
 * @retval None
 */
PUTCHAR_PROTOTYPE {
	/* Place your implementation of fputc here */
	/* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
	HAL_UART_Transmit(&huart2, (uint8_t *) &ch, 1, 0xFFFF);

	return ch;
}

void GPIO_Output_High(GPIO_TypeDef *port, uint32_t pin){
	static GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
}

void GPIO_Output_Low(GPIO_TypeDef *port, uint32_t pin){
	static GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
}

void GPIO_Output_Float(GPIO_TypeDef *port, uint32_t pin){
	static GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void prep_gpib_pins(void){
	GPIO_Output_Low(TE_GPIO_Port, TE_Pin); // Disables talking on data and handshake lines
	GPIO_Output_Low(PE_GPIO_Port, PE_Pin);

    if (mode) {
	    GPIO_Output_High(SC_GPIO_Port, SC_Pin); // Allows transmit on REN and IFC
	    GPIO_Output_Low(DC_GPIO_Port, SC_Pin); // Transmit ATN and receive SRQ
	}
	else {
	    GPIO_Output_Low(SC_GPIO_Port, SC_Pin);
	    GPIO_Output_High(DC_GPIO_Port, DC_Pin);
	}

	GPIO_Output_Float(DIO1_GPIO_Port, DIO1_Pin);
	GPIO_Output_Float(DIO2_GPIO_Port, DIO2_Pin);
	GPIO_Output_Float(DIO3_GPIO_Port, DIO3_Pin);
	GPIO_Output_Float(DIO4_GPIO_Port, DIO4_Pin);
	GPIO_Output_Float(DIO5_GPIO_Port, DIO5_Pin);
	GPIO_Output_Float(DIO6_GPIO_Port, DIO6_Pin);
	GPIO_Output_Float(DIO7_GPIO_Port, DIO7_Pin);
	GPIO_Output_Float(DIO8_GPIO_Port, DIO8_Pin);

	if (mode) {
	    GPIO_Output_High  (ATN_GPIO_Port, ATN_Pin);
	    GPIO_Output_Float (EOI_GPIO_Port, EOI_Pin);
	    GPIO_Output_Float (DAV_GPIO_Port, DAV_Pin);
	    GPIO_Output_Low   (NRFD_GPIO_Port, NRFD_Pin);
	    GPIO_Output_Low   (NDAC_GPIO_Port, NDAC_Pin);
	    GPIO_Output_High  (IFC_GPIO_Port, IFC_Pin);
	    GPIO_Output_Float (SRQ_GPIO_Port, SRQ_Pin);
	    GPIO_Output_Low   (REN_GPIO_Port, REN_Pin);
	}
	else {
	    GPIO_Output_Float(ATN_GPIO_Port, ATN_Pin);
	    GPIO_Output_Float(EOI_GPIO_Port, EOI_Pin);
	    GPIO_Output_Float(DAV_GPIO_Port, DAV_Pin);
	    GPIO_Output_Float(NRFD_GPIO_Port, NRFD_Pin);
	    GPIO_Output_Float(NDAC_GPIO_Port, NDAC_Pin);
	    GPIO_Output_Float(IFC_GPIO_Port, IFC_Pin);
	    GPIO_Output_Float(SRQ_GPIO_Port, SRQ_Pin);
	    GPIO_Output_Float(REN_GPIO_Port, REN_Pin);
	}
}

void gpib_init() {
	prep_gpib_pins(); // Put all the pins into high-impedance mode
	//GPIO_Output_Low(NRFD_GPIO_Port, NRFD_Pin); // ?? Needed ??
	GPIO_Output_High(NDAC_GPIO_Port, NDAC_Pin); // ?? Needed ??
}

char gpib_controller_assign(int address) {
	myAddress = address;
	GPIO_Output_Low(IFC_GPIO_Port, IFC_Pin); // Assert interface clear. Resets bus and makes it
	                 // controller in charge.
	HAL_Delay(200);
	GPIO_Output_Float(IFC_GPIO_Port, IFC_Pin); // Finishing clearing interface

	GPIO_Output_Low(REN_GPIO_Port, REN_Pin); // Put all connected devices into "remote" mode
	cmd_buf[0] = CMD_DCL;
	return gpib_cmd(cmd_buf, 1); // Send GPIB DCL cmd, clear all devices on bus
}

char gpib_cmd(char *bytes, int length) {
    // Write a GPIB CMD byte to the bus
	return _gpib_write(bytes, length, 1, 0);
}

char gpib_write(char *bytes, int length, uint8_t useEOI) {
    // Write a GPIB data string to the bus
	return _gpib_write(bytes, length, 0, useEOI);
}

char _gpib_write(char *bytes, int length, uint8_t attention, uint8_t useEOI) {
    /*
    * Write a string of bytes to the bus
    * bytes: array containing characters to be written
    * length: number of bytes to write, 0 if not known.
    * attention: 1 if this is a gpib command, 0 for data
    */
	char a; // Storage variable for the current character
	int i; // Loop counter variable

	GPIO_Output_High(PE_GPIO_Port, PE_Pin);

	if(attention) // If byte is a gpib bus command
	{
		GPIO_Output_Low(ATN_GPIO_Port, ATN_Pin); // Assert the ATN line, informing all
		                 // this is a cmd byte.
	}

	if(length==0) // If the length was unknown
	{
		length = strlen((char*)bytes); // Calculate the number of bytes to
		                               // be sent
	}

	GPIO_Output_High(TE_GPIO_Port, TE_Pin); // Enable talking

	GPIO_Output_High(EOI_GPIO_Port, EOI_Pin);
	GPIO_Output_High(DAV_GPIO_Port, DAV_Pin);
	GPIO_Output_Float(NRFD_GPIO_Port, NRFD_Pin);
	GPIO_Output_Float(NDAC_GPIO_Port, NDAC_Pin);

	// Before we start transfering, we have to make sure that NRFD is high
	// and NDAC is low
    #ifdef WITH_TIMEOUT
	seconds = 0;
	/** @todo fix interrupt issue */
	// enable_interrupts(INT_TIMER2);
	// while((input(NDAC) || !(input(NRFD))) && (seconds <= timeout)) {
	while(0){
	    //restart_wdt();
		if(seconds >= timeout) {
		    if (debug == 1) {
			    printf("Timeout: Before writing %c %x ", bytes[0], bytes[0]);
			}
			device_talk = false;
			device_srq = false;
			prep_gpib_pins();
			return 1;
		}
	}
	// disable_interrupts(INT_TIMER2);
    #else
	while(input(NDAC)){}
    #endif



	for(i = 0;i < length;i++) { //Loop through each character, write to bus
		a = bytes[i]; // So I don't have to keep typing bytes[i]

		#ifdef VERBOSE_DEBUG
		printf("Writing byte: %c %x %c", a, a, eot_char);
		#endif

		// Wait for NDAC to go low, indicating previous bit is now done with
    #ifdef WITH_TIMEOUT
		seconds = 0;
		/** @todo fix interrupt issue */
		// enable_interrupts(INT_TIMER2);
		// while(input(NDAC) && (seconds <= timeout)) {
		while (0) {
		    restart_wdt();
			if(seconds >= timeout) {
			    if (debug == 1) {
				    printf("Timeout: Waiting for NDAC to go low while writing%c", eot_char);
				}
				device_talk = false;
				device_srq = false;
				prep_gpib_pins();
				return 1;
			}
		}
		// disable_interrupts(INT_TIMER2);
    #else
		while(input(NDAC)){}
    #endif

		// Put the byte on the data lines
		a = a^0xff;
		/** @todo Do this with HAL library */
		// output_b(a);

		GPIO_Output_Float(NRFD_GPIO_Port, NRFD_Pin);

		// Wait for listeners to be ready for data (NRFD should be high)
    #ifdef WITH_TIMEOUT
		seconds = 0;
		/** @todo fix interrupt issue */
		// enable_interrupts(INT_TIMER2);
		// while(!(input(NRFD)) && (seconds <= timeout)) {
		while(0){
		    restart_wdt();
			if(seconds >= timeout) {
			    if (debug == 1) {
				    printf("Timeout: Waiting for NRFD to go high while writing%c", eot_char);
			    }
			    device_talk = false;
			    device_srq = false;
			    prep_gpib_pins();
				return 1;
			}
		}
		// disable_interrupts(INT_TIMER2);
    #else
		while(!(input(NRFD))){}
    #endif

		if((i==length-1) && (useEOI)) { // If last byte in string
			GPIO_Output_Low(EOI_GPIO_Port, EOI_Pin); // Assert EOI
		}

		GPIO_Output_Low(DAV_GPIO_Port, DAV_Pin); // Inform listeners that the data is ready to be read


		// Wait for NDAC to go high, all listeners have accepted the byte
    #ifdef WITH_TIMEOUT
		seconds = 0;
		/** @todo fix interrupt issue */
		// enable_interrupts(INT_TIMER2);
		// while(!(input(NDAC)) && (seconds <= timeout)) {
		while(0){
		    restart_wdt();
			if(seconds >= timeout) {
			    if (debug == 1) {
			        printf("Timeout: Waiting for NDAC to go high while writing%c", eot_char);
			    }
			    device_talk = false;
			    device_srq = false;
			    prep_gpib_pins();
				return 1;
			}
		}
		// disable_interrupts(INT_TIMER2);
    #else
		while(!(input(NDAC))){}
    #endif

		GPIO_Output_High(DAV_GPIO_Port, DAV_Pin); // Byte has been accepted by all, indicate
		                   // byte is no longer valid

	} // Finished outputing all bytes to listeners

	GPIO_Output_Low(TE_GPIO_Port, TE_Pin); // Disable talking on datalines

	// Float all data lines
	GPIO_Output_Float(DIO1_GPIO_Port, DIO1_Pin);
	GPIO_Output_Float(DIO2_GPIO_Port, DIO2_Pin);
	GPIO_Output_Float(DIO3_GPIO_Port, DIO3_Pin);
	GPIO_Output_Float(DIO4_GPIO_Port, DIO4_Pin);
	GPIO_Output_Float(DIO5_GPIO_Port, DIO5_Pin);
	GPIO_Output_Float(DIO6_GPIO_Port, DIO6_Pin);
	GPIO_Output_Float(DIO7_GPIO_Port, DIO7_Pin);
	GPIO_Output_Float(DIO8_GPIO_Port, DIO8_Pin);

	if(attention) { // If byte was a gpib cmd byte
		GPIO_Output_High(ATN_GPIO_Port, ATN_Pin); // Release ATN line
	}

	GPIO_Output_Float(DAV_GPIO_Port, DAV_Pin);
	GPIO_Output_Float(EOI_GPIO_Port, EOI_Pin);
	GPIO_Output_High(NDAC_GPIO_Port, NDAC_Pin);
	GPIO_Output_High(NRFD_GPIO_Port, NRFD_Pin);

	GPIO_Output_Low(PE_GPIO_Port, PE_Pin);

	return 0;
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
