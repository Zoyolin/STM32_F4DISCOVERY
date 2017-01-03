/*
 * STM32_RFIDClick_lib.h

 *
 *  Created on: 11 déc. 2016
 *      Author: leChat
 */

#ifndef APPLICATION_USER_STM32_RFIDCLICK_LIB_H_
#define APPLICATION_USER_STM32_RFIDCLICK_LIB_H_

#define DEBUG //compilation conditionelle des codes de debug
//c'est à dire des retours d'affichage des leds, et de l'affichage pc. pas des codes de retour.
#ifdef DEBUG
#define NbDebugStage 0x32 //0x30=une étape nommée '0' 0|//0x32=trois étapes nommées '0,1,2'
#define Led_Bleu_Sud	LD6_Pin
#define Led_Rouge_Ouest	LD5_Pin
#define Led_Verte_Est	LD4_Pin
#define Led_Orange_Nord	LD3_Pin
#endif //DEBUG

#define CLICK1	//click placé sur l'emplacement 1
#ifdef CLICK1
#define IRQ_IN_PIN	GPIO_PIN_10
#define IRQ_IN_PORT	GPIOB
#define SSI_0_PIN	GPIO_PIN_1
#define SSI_0_PORT	GPIOC
#define SSI_1_PIN	GPIO_PIN_10
#define SSI_1_PORT	GPIOE
#endif /* CLICK1*/

//procedure IDLE/Tag_Detection_Calibration :
		//Dichotomide depuis la valeur maximale
#define DICHO_START 0xFC
		// afiné à une résolution de
#define DICHO_STOP	0x04
		//réponses de dichotomie :
#define UPPER_DATAH	0x02
#define LOWER_DATAH 0x01
//tout le monde
typedef enum //Appli_StatusTypeDef //format des retours de fonction
{
	Appli_OK       = 0x00 ,
	Appli_ERROR    = 0x01 ,
	Appli_BUSY     = 0x02 ,
	Appli_TIMEOUT  = 0x03 ,
	Appli_NONE	   = 0x04
} Appli_StatusTypeDef;

/*
	struct {
		unsigned char power_up 		:1 ;
		unsigned char hibernate		:1 ;
		unsigned char tag_detector	:1 ;
		unsigned char sleep			:1 ;
		unsigned char reader		:1 ;
		unsigned char RFU			:1 ;
	};  */


//Idle only :
typedef union{
	unsigned char WuSource;
	struct {
		unsigned char TimeOut			:1 ;
		unsigned char TagDetection		:1 ;
		unsigned char Undefined			:1 ;
		unsigned char LowPulseIRQ_IN	:1 ;
		unsigned char Undefined2		:4 ;
	} WUsource_struct;
}WUsource_typedef;

typedef enum {
	__Enter_Sleep		=0x01,
	__Enter_Hibernate	=0x04,
	__Enter_TagDetector =0x21,
	__Enter_TagDetectorCalibration = 0xA2 //ou A1 selon la doc utilisée
}EnterControlH_typedef;

typedef enum {
	__ControlH_Hibernate	=0x04,
	__ControlH_Sleep		=0x38,
	__ControlH_TagDetector =0x79,
	__ControlH_TagDetectorCalibration = 0xF8 //ou A1 selon la doc utilisée
}WUControlH_typedef;
typedef enum {
	__ControlL_Hibernate_Or_Sleep,
	__ControlL_TagDetector_Or_Calibration //ou A1 selon la doc utilisée
}WUControlL_typedef;
typedef enum {
	__WUPeriod_standard = 0x20	//valeur à ajuster avec WUPERIOD_CORRECTION
}WUPeriod_typedef;
#define WUPERIOD_CORRECTION 0

typedef struct{
	WUsource_typedef 		WUsource; //indice 0
	EnterControlH_typedef 	EnterControlH;
	unsigned char 			EnterControlL;
	WUControlH_typedef 		WUControlH;
	WUControlL_typedef 		WUControlL;
	unsigned char 			LeaveControlL;
	unsigned char 			LeaveControlH;
	WUPeriod_typedef		WUPeriod;
	unsigned char 			OscStart;
	unsigned char 			DACStart;
	unsigned char 			DACDataL; //valeur à calibrer
	unsigned char 			DACDataH;
	unsigned char 			SwingCount;
	unsigned char 			MaxSleep; //indice E
} Idle_Struct_typeDef;

typedef union {
	unsigned char Idle_Handle_tab[0x0E];
	Idle_Struct_typeDef Idle_Handle_struct ;
} Idle_Handle_TypeDef;

/*Attention -------
 * si régénération du code insérer parmis les prototypes
 * #ifdef DEBUG
 * void Error_Handler(void);
 * #endif
 * et à la fin des structures d'init d'uart :
 * #ifdef DEBUG
 * Error_Handler();
 * #endif
 * retirer "static" du prototype et de la déclaration de MX_USART3_UART_Init.
 */
Appli_StatusTypeDef My_CODE_1(void);
Appli_StatusTypeDef My_CODE_2(void);
Appli_StatusTypeDef My_CODE_3(void);

Appli_StatusTypeDef Start_CR95HF(void);
Appli_StatusTypeDef Echo_CR95HF(unsigned char );
Appli_StatusTypeDef IDN_CR95HF(unsigned char *);
Appli_StatusTypeDef RM_Idle_Init(Idle_Struct_typeDef *);
Appli_StatusTypeDef Idle_CR95HF(Idle_Handle_TypeDef *);
unsigned char Idle_talk(unsigned char *);

 void MX_USART3_UART_Init(void);

void Error_Handler(void);

#ifdef DEBUG
static void MX_USART2_UART_Init(void);
#endif

#ifdef DEBUG
void PrintDebug(unsigned char );
#endif /* DEBUG */


#endif /* APPLICATION_USER_STM32_RFIDCLICK_LIB_H_ */
