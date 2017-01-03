/*
 * STM32_RFIDClick_lib.c
 *
 *  Created on: 11 déc. 2016
 *      Author: leChat
 */

//#define USE_HAL_DRIVER //stm32f4xx inclue stm32f4xx_hal
//#include "stm32f407xx.h"
#include "stm32f4xx_hal.h"
#include "STM32_RFIDClick_lib.h"

//appel des variables à importer
#ifdef DEBUG
extern UART_HandleTypeDef huart2;
#endif //DEBUG
extern UART_HandleTypeDef huart3;

//Privates Variables
Appli_StatusTypeDef Start_Status 	= Appli_NONE;
Appli_StatusTypeDef Echo_Status 	= Appli_NONE;
Appli_StatusTypeDef Idle_Status 	= Appli_NONE;
Appli_StatusTypeDef Idle_Init_Status= Appli_NONE;
Appli_StatusTypeDef IDN_Status  	= Appli_NONE;

Idle_Handle_TypeDef Idle_Handle1_Union ;

unsigned char Reception[20];

//le DEBUG
//TENTER DES CONST !!
#ifdef DEBUG
const unsigned char bufferError[]	= "Not Ok|" ; //message d'erreur
const unsigned char bufferOK[] 	= "all OK|" ; //acknowledge 1
const unsigned char tab = 0x09;
#endif /* DEBUG */


Appli_StatusTypeDef My_CODE_1(void){
	//vérifie les états par défaut des broches de la condition de départ
	GPIOE->BSRR=0x00000000; //PE10
	GPIOB->BSRR=0x00000000;	//PB10
	GPIOC->BSRR=0x00000000;	//PC1
	return Appli_OK;
}
Appli_StatusTypeDef My_CODE_2(void){
	/* Si régénération auto du code :**************************************************
	 * ci-dessus :Initialize all configured peripherals
	 * et vérifier les valeurs de baudrate et bit de stops....
	 * MX_GPIO_Init();				//gpio générale par hal plaquant à 0 SSI_1 et SSI_0.
	 * MX_USART2_UART_Init(); 		//uart avec le pc
	 * //MX_USART3_UART_Init(); 	//pas maintenant.
	 */
	//finalisation de la condition de départ
	Start_Status = Start_CR95HF();
	//############################################# ECHO
	if(!(Echo_Status=Echo_CR95HF(50))){
#ifdef DEBUG
		PrintDebug(Echo_Status); 						//etape 0 =Echo Ok
#endif // DEBUG
		//		//############################################# IDN
		//				IDN_Status 			= IDN_CR95HF(&Reception[0]);//ranger l'idn dans Reception
		//		//si DEBUG : affiche l'IDN
		//		#ifdef DEBUG
		//				PrintDebug(IDN_Status); 						//etape 1 =IDN Ok
		//		#endif // DEBUG
		//############################################# Idle Init
		Idle_Init_Status 	= RM_Idle_Init(&Idle_Handle1_Union.Idle_Handle_struct);
#ifdef DEBUG
		PrintDebug(Idle_Init_Status); 					//etape 2 =InitIdle Ok
#endif // DEBUG
		while(Idle_Status){
			//############################################# Idle
			Idle_Status 		= Idle_CR95HF(&Idle_Handle1_Union);
#ifdef DEBUG
			PrintDebug(Idle_Status); 						//etape 3 = Idle Ok
#endif // DEBUG
		}
	}
	return Appli_OK;
}
Appli_StatusTypeDef My_CODE_3(void){
#ifdef DEBUG
	//pour le while(1)
	unsigned char buffer2 = 0 ;//1 config de réception pour la co au PC
	unsigned int SizeTrans2 = 1 ; //1 octet
	signed int TimeoutTrans2 = 0 ; //one shoot
	unsigned int SizeRes3 = 1 ; //1 octet
	signed int TimeoutRes3 = 1 ; //1ms
	unsigned int SizeTrans3 = 1 ; //1 octet
	signed int TimeoutTrans3 = 0;// one shoot
	unsigned char bufferRx3 = 0; //
	unsigned char bufferTx3 = 0;

	//double fonction echo : pour le pc et le CR95HF
	HAL_UART_Receive (&huart2, &buffer2, SizeTrans2, TimeoutTrans2); //lecture en boucle ; pas de gestion de l'état de lecture
	if(buffer2 != 0){		//si le buffer est rempli
		HAL_UART_Transmit(&huart2, &buffer2, SizeTrans2, TimeoutTrans2); // transmission
		HAL_UART_Transmit(&huart3, &bufferTx3, SizeTrans3, TimeoutTrans3); 	// transmission
		buffer2 =0 ; 			//raz buffer
	}
	//transmission du "U" si echoCR95HF ok (0x55)
	HAL_UART_Receive (&huart3, &bufferRx3, SizeRes3, TimeoutRes3);
	if (bufferRx3 !=0){				//si réception depuis le CR95HF
		HAL_UART_Transmit(&huart2, &bufferRx3, SizeTrans2, TimeoutTrans2); //transmisoin vers le pc
		bufferRx3 = 0;
	}
#endif //DEBUG
	return Appli_OK;
}



//déclaration de fonctions_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_
#ifdef DEBUG
void PrintDebug(unsigned char Status_Debug){
	unsigned char *DebugPointeur = &tab ;
	static unsigned char Etape = 0x0A ;
	if(Etape == 0x0A){
	HAL_UART_Transmit(&huart2, &Etape, 1 , 20); //revenir à la marge
	Etape = 0x0D;
	HAL_UART_Transmit(&huart2, &Etape, 1 , 20); //sauter une ligne
	Etape = 0x30;								//init de l'identifiant
	}
	if(Etape <= NbDebugStage){
		HAL_UART_Transmit(&huart2, DebugPointeur , 1 , 20); //tabulation
		HAL_UART_Transmit(&huart2, &Etape, 1 , 20);			//identification
		if(Status_Debug==Appli_OK){
			DebugPointeur = &bufferOK;
			HAL_UART_Transmit(&huart2, DebugPointeur, 7, 10); 		// Acknowledge sur le terminal du pc.
			HAL_GPIO_WritePin(GPIOD, Led_Verte_Est, GPIO_PIN_SET); 		// allume la led Verte
			HAL_GPIO_WritePin(GPIOD, Led_Rouge_Ouest, GPIO_PIN_RESET); 	// éteinds la led rouge
		}else{
			DebugPointeur = &bufferError;
			HAL_UART_Transmit(&huart2, DebugPointeur, 7, 10); 	// alerte à l'erreur sur le terminal du pc.
			HAL_GPIO_WritePin(GPIOD, Led_Rouge_Ouest, GPIO_PIN_SET); 		// allume la led Rouge
			HAL_GPIO_WritePin(GPIOD, Led_Verte_Est, GPIO_PIN_RESET); 		// éteinds la led Verte
		}
		Etape ++ ;										//incrément de l'identifiant
	}else if(Status_Debug==Appli_OK){		//si le programme est fini, et Appli_OK,
		DebugPointeur = &bufferOK;
		HAL_UART_Transmit(&huart2, DebugPointeur, 7, 10); 		// Acknowledge sur le terminal du pc.
		HAL_GPIO_WritePin(GPIOD, Led_Verte_Est, GPIO_PIN_SET); 	// allume la led Verte

	}
}
#endif //debug

Appli_StatusTypeDef RM_Idle_Init(Idle_Struct_typeDef *pointeur){
	//#define WU_PERIOD 500 ;
	//#define MAX_SLEEP 0x28;

	pointeur->WUsource.WUsource_struct.TagDetection 	= 1 ;	//réveil du CR95HF par détection de Tg
	pointeur->WUsource.WUsource_struct.TimeOut 		= 1 ; 		//et réveil du CR95HF par minuterie
	pointeur->EnterControlH	=__Enter_TagDetectorCalibration;	//REM,   A100  Initial Dac Compare
	pointeur->EnterControlL =0x00;								//invariablement
	pointeur->WUControlH	=__ControlH_TagDetectorCalibration;	//REM,   F801  Initial Dac Compare
	pointeur->WUControlL	=__ControlL_TagDetector_Or_Calibration;
	pointeur->LeaveControlL	=0x18;								//1800 HFO
	pointeur->LeaveControlH	=0x00;
	pointeur->WUPeriod	 	=0x20;	//REM,   20 Wup Period 32 Inactivity period = 256ms (LFO @ 32kHz)
	pointeur->OscStart	 	=0x60;								//REM,   60 Osc  3ms   (LFO @ 32kHz)
	pointeur->DACStart	 	=0x60;								//REM,   60 Dac  3ms   (LFO @ 32kHz)
	pointeur->DACDataL	 	=0x00;
	pointeur->DACDataH	 	=0x00; //valeur à calibrer
	pointeur->SwingCount	=0x3F;								//REM,   3F  Swing 13.56  4.6 us
	pointeur->MaxSleep		=0x05;								//REM,   01 Maximum number of Sleep before Wakeup 2 !
	return Appli_OK;
}

Appli_StatusTypeDef Idle_CR95HF(Idle_Handle_TypeDef *p_handle){
	// prends en paramêtre un pointeur vers une union
	// d'un tableau d'unsigned char. c'est dans ce tableau
	// qu'est écrite la trame intéressante. il est
	// uni à une structure pour faciliter sa configuration

	unsigned char retour = 0;
	unsigned char TmpChar = 0;

	// si la commande demandée est un tag detection's calibration
	if(p_handle->Idle_Handle_struct.EnterControlH==__Enter_TagDetectorCalibration
			&& p_handle->Idle_Handle_struct.WUControlH==__ControlH_TagDetectorCalibration
			&& p_handle->Idle_Handle_struct.WUControlL==__ControlL_TagDetector_Or_Calibration
			&& p_handle->Idle_Handle_struct.WUPeriod==__WUPeriod_standard)
	{

		if(p_handle->Idle_Handle_struct.DACDataH!=0)//accès plus rapide en lecture
			p_handle->Idle_Handle_struct.DACDataH=0;					//pour DataH = 0
		if((retour=Idle_talk(&(p_handle->Idle_Handle_tab[0])))==UPPER_DATAH){	//vérifier la réponse "tag detect" soit "Upper_dataH"
			p_handle->Idle_Handle_struct.DACDataH=DICHO_START;			//pour DataH = valeur_max
		}
		else													//sinon
			return Appli_ERROR;									//error

		// BOUCLE DICHOTOMIQUE
		TmpChar = 0x80;						// masque 0b 1000 0000
		while(!(DICHO_START&TmpChar)){TmpChar=TmpChar>>1;}	//copie le bit de poids fort à 1 de DICHO_START dans TmpChar.
		while(TmpChar>= DICHO_STOP){ 						//pour TmpChar jusqu'a DICHO_STOP par division par deux
			if((retour = Idle_talk(&(p_handle->Idle_Handle_tab[0])))==LOWER_DATAH){ //si l'idle réponds "trop bas"
				p_handle->Idle_Handle_struct.DACDataH=-TmpChar;	//diminuer DacDataH de f
			}
			else					//si c'est "trop haut"
				p_handle->Idle_Handle_struct.DACDataH=+TmpChar;	//augmenter DacDataH de f

			TmpChar=TmpChar>>1;
		}
		//FIN DE BOUCLE DICHOTOMIQUE
		//ENCADREMENT DE LA VALEUR
		if(retour == UPPER_DATAH){										// last Wake-up event = Tag Detect (0x02)
			p_handle->Idle_Handle_struct.DACDataL=p_handle->Idle_Handle_struct.DACDataH;	//on commence par assigner DacDataL
		}
		//APPLICATION DES MARGES
		else if(retour == LOWER_DATAH){									// last Wake-up event = Timeout (0x01)
			p_handle->Idle_Handle_struct.DACDataL=p_handle->Idle_Handle_struct.DACDataH,
					p_handle->Idle_Handle_struct.DACDataH=+DICHO_STOP;
		}else{
			return Appli_BUSY;
		}
	}
	return Idle_talk(&(p_handle->Idle_Handle_tab[0]));
}

unsigned char Idle_talk(unsigned char *pointeur){
	unsigned char retour = 0x42;
	unsigned char TmpZonne[2] = {0x07,0x0E};
	HAL_UART_Transmit	(&huart3, &TmpZonne[0], 2, 100);			//commande
	HAL_UART_Transmit	(&huart3, pointeur, TmpZonne[1], 100);		//trame
	HAL_UART_Receive	(&huart3, &TmpZonne[0] , 2, 10);			//commande
	HAL_UART_Receive 	(&huart3, &retour, TmpZonne[1], 20);		//trame
	return retour;
}
Appli_StatusTypeDef Start_CR95HF(void){
	//séquence de démarrage
	//passage à l'état haut des broches à la mise sous tension,
	//magnifique plaquage des proches SSI_0 , SSI_1 et IRQ_IN à 0 par MX_GPIO_Init
	HAL_Delay(1); 					//attente
	HAL_GPIO_WritePin(IRQ_IN_PORT, IRQ_IN_PIN, GPIO_PIN_SET); //remonté phénoménale de IRQ_IN
	HAL_Delay(1); 					//temps mort

	MX_USART3_UART_Init(); //init de l'uart3 et réaffectation de PB10
#ifdef DEBUG
	MX_USART2_UART_Init(); //init de l'uart2
	HAL_GPIO_WritePin(GPIOD, Led_Bleu_Sud, GPIO_PIN_SET); 	// allume la led bleu
#endif
	return Appli_OK;
}

/* USART3 init function */
void MX_USART3_UART_Init(void){
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
	if (HAL_UART_Init(&huart3))
	{
#ifdef DEBUG
		Error_Handler();
#endif
	}
}

#ifdef DEBUG
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
	if (HAL_UART_Init(&huart2) != HAL_OK)
	{
		Error_Handler();
	}
}
#endif

void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler */
	/* User can add his own implementation to report the HAL error return state */
	while(1)
	{
#ifdef	DEBUG
		HAL_UART_Transmit(&huart2, &bufferError[0], 7, 10); 	//alerte à l'erreur sur le terminal du pc.
		HAL_GPIO_WritePin(GPIOD, LD5_Pin, GPIO_PIN_SET);		//allume la led rouge
#endif //DEBUG
	}
	/* USER CODE END Error_Handler */
}
//echo rev3
Appli_StatusTypeDef Echo_CR95HF(unsigned char iteration)
{//fonction s'occupant de demander , recevoir, et vérifier la réception d'un echo avec le CR95HF.
	unsigned char TmpBuffer = 0x55 ;//1 config d'émission pour la co au CR95HF
	do{
		if(TmpBuffer != 0x55) //temps d'accès en lecture plus rapide
			TmpBuffer = 0x55;
		HAL_UART_Transmit(&huart3, &TmpBuffer, 1, 2); 	// transmission
		iteration --;
	}while((HAL_UART_Receive(&huart3, &TmpBuffer, 1, 2) != HAL_OK) && (iteration > 0) );
#ifdef DEBUG
	HAL_UART_Transmit(&huart2, &TmpBuffer, 1, 20);
#endif //DEBUG
	if(TmpBuffer == 0x55){
		return Appli_OK ;
	}else
		if(TmpBuffer !=0 ){
			return Appli_BUSY ;
		}if(iteration == 0){
			return Appli_TIMEOUT ;
		}else{
			return Appli_ERROR ;
		}
}

Appli_StatusTypeDef IDN_CR95HF(unsigned char *pointeur){
	//pointeur vers adresse de retour de l'IDN
	//gare aux espaces de moins de moins de 14octets
	unsigned char TmpZonne[2] = {0x01,0x00};
	if( HAL_UART_Transmit(&huart3, &TmpZonne[0], 2, 1)) 	//	Emission
		return Appli_ERROR; //transmet le code d'erreur de HAL_UART_Transmit
	if(	HAL_UART_Receive(&huart3, &TmpZonne[0] , 2, 10))	//	Réception
		return Appli_ERROR; //transmet le code d'erreur de HAL_UART_Receive
#ifdef DEBUG
	Appli_StatusTypeDef RETOUR_RECEPTION_IDN = Appli_NONE;
	RETOUR_RECEPTION_IDN = HAL_UART_Receive (&huart3, pointeur, TmpZonne[1], 20);
	HAL_UART_Transmit(&huart2, pointeur, 0x0E, 20);
	return RETOUR_RECEPTION_IDN;
#endif
#ifndef DEBUG
	return HAL_UART_Receive (&huart3, pointeur, (TmpZonne[1]-2), 20);
#endif
}
