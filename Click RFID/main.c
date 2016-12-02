/*
 * main_bis.c
 *
 *  Created on: 1 déc. 2016
 *      Author: leChat
 */

/**
 ******************************************************************************
 * File Name          : main.c
 * Description        : Main program body
 ******************************************************************************
 *
 * COPYRIGHT(c) 2016 STMicroelectronics
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
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#define CLICK1
#ifdef CLICK1
#define IRQ_IN_PIN	GPIO_PIN_10
#define IRQ_IN_PORT	GPIOB
#define SSI_0_PIN	GPIO_PIN_1
#define SSI_0_PORT	GPIOC
#define SSI_1_PIN	GPIO_PIN_10
#define SSI_1_PORT	GPIOE
#endif
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

typedef enum //Appli_StatusTypeDef
{
	Appli_OK       = 0x00U,
	Appli_ERROR    = 0x01U,
	Appli_BUSY     = 0x02U,
	Appli_TIMEOUT  = 0x03U
} Appli_StatusTypeDef;

unsigned char buffer2 = 0 ;//1 config de réception pour la co au PC
unsigned int SizeTrans2 = 1 ; //1octet
signed int TimeoutTrans2 = 0 ;

unsigned char Reception[10];
unsigned char TmpRxZonne;
static unsigned char *bufferRx3 = &TmpRxZonne ;//1 config de réception pour la co au CR95HF
unsigned int SizeRes3 = 1 ; //1octet
signed int TimeoutRes3 = 0 ; //1ms


static unsigned char bufferTx3 = 0 ;//1 config d'émission pour la co au CR95HF
unsigned int SizeTrans3 = 1 ; //1 octet
signed int TimeoutTrans3 = 0 ;//1 ms
//57600 bauds -> 1bit en 17.4us -> 8bits = 139.2us -> 11bits en 191.4us

Appli_StatusTypeDef Echo_Status = Appli_BUSY;
Appli_StatusTypeDef IDN_Status  = Appli_BUSY;

static unsigned char bufferError[] = "NoInit" ; //message d'erreur
static unsigned char bufferOK1[]   = "echoOK" ; //acknowledge 1
static unsigned char bufferOK2[]   = "IDN_OK" ; //acknowledge 1

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
static HAL_StatusTypeDef UART3_Receive_Delay(unsigned char *);
Appli_StatusTypeDef IDN_CR95HF(unsigned char *);
static Appli_StatusTypeDef UART3_Receive_CR95HF(unsigned char *);
Appli_StatusTypeDef Echo_CR95HF_bis(uint32_t Delay);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

	/* USER CODE BEGIN 1 */
	GPIOE->BSRR=0x00000000; //PE10
	GPIOB->BSRR=0x00000000;	//PB10
	GPIOC->BSRR=0x00000000;	//PC1
	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART2_UART_Init();
	//MX_USART3_UART_Init();

	/* USER CODE BEGIN 2 */

	/* Si régénération auto du code :**************************************************
	 * ci-dessus :Initialize all configured peripherals
	 * et vérifier les valeurs de baudrate et bit de stops....
	 * MX_GPIO_Init();				//gpio générale par hal plaquant à 0 SSI_1 et SSI_0.
	 * MX_USART2_UART_Init(); 		//uart avec le pc
	 * //MX_USART3_UART_Init(); 	//pas maintenant.
	 *
	 * Dans la fonction MX_USART2_UART_Init,
	 * si régenération de code, rajouter :
	 * huart2.pTxBuffPtr = &buffer2 ;
	 * huart2.pRxBuffPtr = huart2.pTxBuffPtr; //utilisation du même buffer.
	 */

	//séquence de démarrage
	//passage à l'état haut des broches à la mise sous tension,
	//magnifique plaquage des proches SSI_0 , SSI_1 et IRQ_IN à 0 par MX_GPIO_Init
	HAL_Delay(1); //temps mort
	HAL_GPIO_WritePin(IRQ_IN_PORT, IRQ_IN_PIN, GPIO_PIN_SET); //remonté phénoménale de IRQ_IN
	HAL_Delay(1); //temps mort

	/* Initialisation complète de l'UART3 CR95HF
	 * si régenération de code, rajouter :
	 * huart3.pTxBuffPtr = &bufferTx3 ;
	 * huart3.pRxBuffPtr = bufferRx3 ;
	 */
	MX_USART3_UART_Init();

	Echo_Status = Echo_CR95HF( 200 ); //watchdog à 201ms

	if(Echo_Status==Appli_OK){
		HAL_UART_Transmit(&huart2, &bufferOK1[0], 6, 10); 	//alerte à l'erreur sur le terminal du pc.
	}else{
		HAL_UART_Transmit(&huart2, &bufferError[0], 6, 10); 	//alerte à l'erreur sur le terminal du pc.
	}
	buffer2= 0x0 ; //valeur par défaut de la réception uart : "1"

	IDN_Status = IDN_CR95HF(&Reception[0]);
	if(IDN_Status==Appli_OK){
		HAL_UART_Transmit(&huart2, &bufferOK2[0], 6, 10); 	//alerte à l'erreur sur le terminal du pc.
	}else{
		HAL_UART_Transmit(&huart2, &bufferError[0], 6, 10); 	//alerte à l'erreur sur le terminal du pc.
	}
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		//double fonction : pour le pc et le CR95HF
		HAL_UART_Receive (&huart2, &buffer2, SizeTrans2, TimeoutTrans2); //lecture en boucle ; pas de gestion de l'état de lecture
		if(buffer2 != 0){		//si le buffer est rempli
			HAL_UART_Transmit(&huart2, &buffer2, SizeTrans2, TimeoutTrans2); // transmission
			HAL_UART_Transmit(&huart3, &bufferTx3, SizeTrans3, TimeoutTrans3); 	// transmission
			buffer2 =0 ; 			//raz buffer
		}
		HAL_UART_Receive (&huart3, bufferRx3, SizeRes3, TimeoutRes3);
		if (bufferRx3 !=0){				//si réception depuis le CR95HF
			HAL_UART_Transmit(&huart2, bufferRx3, SizeTrans2, TimeoutTrans2); //transmisoin vers le pc
			bufferRx3 = 0;
		}


	}
	/* USER CODE END 3 */

}

/** System Clock Configuration
 */
void SystemClock_Config(void)
{

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;

	/**Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();

	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 168;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/**Initializes the CPU, AHB and APB busses clocks
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

	/**Configure the Systick interrupt time
	 */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

	/**Configure the Systick
	 */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

	huart2.Instance = USART2;
	huart2.Init.BaudRate = 9600;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	//config perso :  **********************************************
	//	huart2.pTxBuffPtr = &buffer2 ;
	//	huart2.pRxBuffPtr = huart2.pTxBuffPtr;
	//**************************************************************
	if (HAL_UART_Init(&huart2) != HAL_OK)
	{
		Error_Handler();
	}

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{
	//ici on reconfigure à la main PB10 en broche pour l'UART.
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = GPIO_PIN_10; 			//Tx_UART3
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;	//opendrain
	GPIO_InitStruct.Pull = GPIO_PULLUP;		//en pull up
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF7_USART3; //broche dédiée à la fonction alternative UART3
	HAL_GPIO_Init(IRQ_IN_PORT, &GPIO_InitStruct); //on réinitialise

	huart3.Instance = USART3;
	huart3.Init.BaudRate = 57600;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_2;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	//config perso :  **********************************************
	//	huart3.pTxBuffPtr = &bufferRx3 ;
	//	huart3.pRxBuffPtr = &bufferRx3 ;
	//**************************************************************
	if (HAL_UART_Init(&huart3) != HAL_OK)
	{
		Error_Handler();
	}

}

/** Configure pins as
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
     PC3   ------> I2S2_SD
     PA4   ------> I2S3_WS
     PA5   ------> SPI1_SCK
     PA6   ------> SPI1_MISO
     PA7   ------> SPI1_MOSI
     PB10   ------> I2S2_CK
     PC7   ------> I2S3_MCK
     PA9   ------> USB_OTG_FS_VBUS
     PA10   ------> USB_OTG_FS_ID
     PA11   ------> USB_OTG_FS_DM
     PA12   ------> USB_OTG_FS_DP
     PC10   ------> I2S3_CK
     PC12   ------> I2S3_SD
     PB6   ------> I2C1_SCL
     PB9   ------> I2C1_SDA
 */
static void MX_GPIO_Init(void)
{

	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	/*__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();


	//Configure GPIO pin Output Level
	HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

	//Configure GPIO pin Output Level
	HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

	//Configure GPIO pin Output Level
	HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
			|Audio_RST_Pin, GPIO_PIN_RESET);
	 */

	//ici on initialise à la main les GPIO.*******************************
	//ici on initialise à la main PC1.
	GPIO_InitStruct.Pin = SSI_0_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;	//opendrain
	GPIO_InitStruct.Pull = GPIO_PULLUP;		//en pull up
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(SSI_0_PORT, &GPIO_InitStruct);
	//HAL_GPIO_WritePin(SSI_0_PORT, SSI_0_PIN, GPIO_PIN_RESET); //SSI_0 plaquée à 0

	//ici on initialise à la main PE10.
	GPIO_InitStruct.Pin = SSI_1_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;	//opendrain
	GPIO_InitStruct.Pull = GPIO_PULLUP;		//en pull up
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(SSI_1_PORT, &GPIO_InitStruct);
	//HAL_GPIO_WritePin(SSI_1_PORT, SSI_1_PIN, GPIO_PIN_RESET); //SSI_1 plaquée à 0

	//ici on initialise à la main PB10.
	GPIO_InitStruct.Pin = GPIO_PIN_10; 			//IRQ_IN
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;	//opendrain
	GPIO_InitStruct.Pull = GPIO_PULLUP;		//en pull up
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(IRQ_IN_PORT, &GPIO_InitStruct);

	//***************************************************************************
	/*
	//Configure GPIO pin : CS_I2C_SPI_Pin
	GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

	//Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin
	GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

	//Configure GPIO pin : PDM_OUT_Pin
	GPIO_InitStruct.Pin = PDM_OUT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
	HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

	//Configure GPIO pin : B1_Pin
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	//Configure GPIO pin : PA4
	GPIO_InitStruct.Pin = GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	//Configure GPIO pins : PA5 PA6 PA7
	GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	//Configure GPIO pin : BOOT1_Pin
	GPIO_InitStruct.Pin = BOOT1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);
	//
  //Configure GPIO pin : CLK_IN_Pin *****DANGER***** config broche PB10
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

	//Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin
	GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
			|Audio_RST_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	//Configure GPIO pins : PC7 I2S3_SCK_Pin PC12
	GPIO_InitStruct.Pin = GPIO_PIN_7|I2S3_SCK_Pin|GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	//Configure GPIO pin : VBUS_FS_Pin
	GPIO_InitStruct.Pin = VBUS_FS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(VBUS_FS_GPIO_Port, &GPIO_InitStruct);

	//Configure GPIO pins : OTG_FS_ID_Pin OTG_FS_DM_Pin OTG_FS_DP_Pin
	GPIO_InitStruct.Pin = OTG_FS_ID_Pin|OTG_FS_DM_Pin|OTG_FS_DP_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	//Configure GPIO pin : OTG_FS_OverCurrent_Pin
	GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

	//Configure GPIO pins : Audio_SCL_Pin Audio_SDA_Pin
	GPIO_InitStruct.Pin = Audio_SCL_Pin|Audio_SDA_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	//Configure GPIO pin : MEMS_INT2_Pin
	GPIO_InitStruct.Pin = MEMS_INT2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);
	 */

}

/* USER CODE BEGIN 4 */

//OLD WAY TO USE VARIABLE I'M SHURE IT'S TOOO HEAVY !
Appli_StatusTypeDef Echo_CR95HF_bis(uint32_t Delay)
{	//fonction s'occupant de demander , recevoir, et vérifier la réception d'un echo avec le CR95HF.
	//paramêtre : le watchdog de l'echo. valeur effective max sz watchdog: paramêtre + 1.
	unsigned char TmpEcho = 0;		//création du char
	TmpEcho = 0x55; 				//valeur du char d'émission
	uint32_t tickstart = 0U;		//fonction HAL de delay...
	tickstart = HAL_GetTick();
	while(((HAL_GetTick() - tickstart) < Delay) && (TmpEcho == 0) )
	{
		HAL_UART_Transmit(&huart3, &TmpEcho, SizeTrans3, TimeoutTrans3); 	// transmission
		TmpEcho = 0;
		if( UART3_Receive_Delay(&TmpEcho) == HAL_BUSY ){
			return Appli_BUSY;
		}
	}
	if((TmpEcho == 0x55)){
		return Appli_OK ;
	}else if((HAL_GetTick() - tickstart) < Delay){
		return Appli_TIMEOUT ;
	}else{
		return Appli_ERROR ;
	}
}

static HAL_StatusTypeDef UART3_Receive_Delay(unsigned char *pointeur)
{//fonction d'attente attentive
	HAL_StatusTypeDef Uart3_Rx_Status = HAL_BUSY ;
	uint32_t tickstart = 0U;
	tickstart = HAL_GetTick();
	while(((HAL_GetTick() - tickstart) < 1) && (Uart3_Rx_Status != HAL_OK) )
	{
		Uart3_Rx_Status = HAL_UART_Receive (&huart3, pointeur, SizeRes3, TimeoutRes3); //reception
	}
	return Uart3_Rx_Status;
}

static Appli_StatusTypeDef UART3_Receive_CR95HF(unsigned char *pointeur){
	//recoit une donnée dans un espace mémoire dont la première adresse
	//est donnée en paramètre. Gardez-vous bien de réserver un espace trop exigue !
	unsigned char TmpChar = 0;		 //nb de caractères à recevoir encore.
	unsigned char index = 0;		 //index de la position-1 dans la trame
	for(TmpChar=2; 					 //permet la réception des deux premiers octets
			TmpChar>0; TmpChar-- ){  //condition d'écoute
		*pointeur = 0;				 //reset du buffer de reception
		if (UART3_Receive_Delay(pointeur) == HAL_BUSY){ //réception d'un caractère
			return Appli_BUSY;
		}

		if(TmpChar == 0 && index == 1){ //si on vient de recevoir l'info de longueur de data
			if(*(pointeur-1) != 0x00){	//si (enplus) le résult code recu n'est pas zéro
				if(*(pointeur-1) == 0x82){		//si (enplus) le résult code recu est 0x82
					//alors A COMPLETER
				}
				if(*(pointeur-1) == 0x83){		//si (enplus) le résult code recu est 0x83
					//alors A COMPLETER
				}
				return Appli_ERROR;				//lancer un code d'alerte
			}							//sinon,
			TmpChar = (*pointeur)-2 ;	//relancer la boucle pour la trame à venir
		}
		pointeur++;					//incrémenter l'adresse de réception
		index++;					//incrémenter le nombre total de bits recus
	}
	return Appli_OK;
}
Appli_StatusTypeDef IDN_CR95HF(unsigned char *pointeur){
	// retourne la longeur de la trame recue
	// prends en paramêtre un pointeur vers la première case
	// d'un tableau d'unsigned char. c'est dans ce tableau
	// qu'est écrite la trame intéressante.
	unsigned char TmpZonne = 0;
	//******Vérif espace mémoire suffisant réservé*******
	if(*(pointeur + 0x0E) != "\0"){
		return Appli_ERROR;
	}
	//****** Emission*****
	TmpZonne = 0x01; 	//1ere partie de la commande
	TmpZonne=HAL_UART_Transmit(&huart3, &bufferTx3, SizeTrans3, TimeoutTrans3); 	// transmission
	if(TmpZonne){
		return TmpZonne; //transmet le code d'erreur de HAL_UART_Transmit
	}
	TmpZonne = 0x00;	//2eme partie de la commande
	TmpZonne=HAL_UART_Transmit(&huart3, &bufferTx3, SizeTrans3, TimeoutTrans3); 	// transmission
	if(TmpZonne){
		return TmpZonne; //transmet le code d'erreur de HAL_UART_Transmit
	}
	//****** Réception*****
	UART3_Receive_CR95HF(pointeur);
	return Appli_OK;
}


/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler */
	/* User can add his own implementation to report the HAL error return state */
	while(1)
	{
		HAL_UART_Transmit(&huart2, &bufferError[0], 6, 10); 	//alerte à l'erreur sur le terminal du pc.

	}
	/* USER CODE END Error_Handler */
}

#ifdef USE_FULL_ASSERT

/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */

}

#endif

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
