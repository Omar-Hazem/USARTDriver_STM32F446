#ifndef UART_DRIVER_H
#define UART_DRIVER_H

#include "STM32F446xx.h"

//stop bits
#define USART_STOP_BITS_1                          (uint32_t)0x01
#define USART_STOP_BITS_2                          (uint32_t)0x02

//Baud rate
#define USART_BAUD_9600                   				9600
#define USART_BAUD_115200                 				115200

//parity bits
#define USART_PARITY_NONE													(uint32_t)0x00
#define USART_PARITY_EVEN													(uint32_t)0x01
#define USART_PARITY_ODD													(uint32_t)0x02

//enable clocks for USARTs 
#define USART1_CLOCK_ENABLE()										(RCC->APB2ENR) |= (RCC_APB2ENR_USART1EN)
#define USART1_CLOCK_DISABLE()										(RCC->APB2ENR) &= ~(RCC_APB2ENR_USART1EN)

#define UART2_CLOCK_ENABLE()											(RCC->APB1ENR) |= (RCC_APB1ENR_USART2EN)
#define UART2_CLOCK_DISABLE()										(RCC->APB1ENR) &= ~(RCC_APB1ENR_USART2EN)

#define UART3_CLOCK_ENABLE()											(RCC->APB1ENR) |= (RCC_APB1ENR_USART3EN)
#define UART3_CLOCK_DISABLE()										(RCC->APB1ENR) &= ~(RCC_APB1ENR_USART3EN)

#define UART4_CLOCK_ENABLE()											(RCC->APB1ENR) |= (RCC_APB1ENR_UART4EN)
#define UART4_CLOCK_DISABLE()											(RCC->APB1ENR) &= ~(RCC_APB1ENR_UART4EN)

#define UART5_CLOCK_ENABLE()											(RCC->APB1ENR) |= (RCC_APB1ENR_UART5EN)
#define UART5_CLOCK_DISABLE()											(RCC->APB1ENR) &= ~(RCC_APB1ENR_UART5EN)

#define USART6_CLOCK_ENABLE()											(RCC->APB2ENR) |= (RCC_APB2ENR_USART6EN)
#define USART6_CLOCK_DISABLE()										(RCC->APB2ENR) &= ~(RCC_APB2ENR_USART6EN)


//********Enumerations
typedef enum{
	OVERSAMPLING_16_ENABLE = (uint32_t)0X00,
	OVERSAMPLING_8_ENABLE  = (uint32_t)0X01
}OVERSAMPLING;

typedef enum
{
	USART_STATE_RESET             = (uint32_t)0x00,    /*!< Peripheral is not yet Initialized                  */
  USART_STATE_READY             = (uint32_t)0x01,    /*!< Peripheral Initialized and ready for use           */
  USART_STATE_BUSY              = (uint32_t)0x02,    /*!< an internal process is ongoing                     */   
  USART_STATE_BUSY_TX           = (uint32_t)0x12,    /*!< Data Transmission process is ongoing               */ 
  USART_STATE_BUSY_RX           = (uint32_t)0x22,    /*!< Data Reception process is ongoing                  */
  USART_STATE_BUSY_TX_RX        = (uint32_t)0x32,    /*!< Data Transmission and Reception process is ongoing */        
}USART_STATES;

typedef enum
{
	USART_ERROR_NONE              = (uint32_t)0x00,    /*!< No error                  */
  USART_ERROR_PE                = (uint32_t)0x01,    /*!< Parity error              */
  USART_ERROR_NE                = (uint32_t)0x02,    /*!< Noise error               */   
  USART_ERROR_FE                = (uint32_t)0x04,    /*!< Frame error               */ 
  USART_ERROR_ORE               = (uint32_t)0x08,    /*!< Overrun error             */
  USART_ERROR_DMA               = (uint32_t)0x10,    /*!< DMA transfer error        */        
}USART_ERRORS;


//********Structures

typedef struct
{
	uint32_t BaudRate;                  /* This member configures the UART communication baud rate */
																 
	uint32_t WordLength;                /* Specifies the number of data bits transmitted or received in a frame */
																 
	uint32_t StopBits;                  /* Specifies the number of stop bits transmitted */
																
	uint32_t Parity;                    /* Specifies the parity mode. */
											

	uint32_t Mode;                      /*  Specifies whether the Receive or Transmit mode is enabled or disabled */

	uint32_t OverSampling;  	/*  Specifies whether the Over sampling 8 is enabled or disabled */

}USART_INIT;


//UART handle structure
typedef struct
{

	USART_TypeDef * USARTx;
	USART_INIT  Init;
	USART_STATES TX_State;
	USART_STATES RX_State;
	uint32_t Error;
	uint8_t * TX_Buffer;
	uint16_t TX_Counter;
	uint16_t TX_Size;
	uint8_t * RX_Buffer;
	uint16_t RX_Counter;
	uint16_t RX_Size;
//	TX_CallBack * TX_FinishCallBack;
//	RX_CallBack * RX_FinishCallBack;

}USART_HANDLE;

//pointer to fuctions typedefs (Call Back functions)
// void (*TX_CallBack) (USART_HANDLE * instance);		//to call a function in the upper layer 
// void (*RX_CallBack) (USART_HANDLE * instance);		//to call a function in the upper layer
//	

//Function prototypes

void USART_Enable(USART_HANDLE * handle);
void USART_Disable(USART_HANDLE * handle);
void USART_TX_EnableDisable(USART_TypeDef * uartx,uint8_t TE);
void USART_RX_EnableDisable(USART_TypeDef * uartx,uint8_t RE);
void USART_WordLengthConfig(USART_HANDLE * handle);
void USART_StopBitsConfig(USART_HANDLE * handle);
void USART_BaudRateConfig(USART_HANDLE * handle);
void USART_OversamplingConfig(USART_HANDLE *handle);
void USART_TXInterruptEnable(USART_TypeDef *uartx, uint32_t txe_en);
void USART_RxInterruptEnable(USART_TypeDef *uartx, uint32_t rxne_en);
void USART_ConfigErrorInterrupt(USART_HANDLE * handle , uint8_t enable);
void USART_Init(USART_HANDLE * handle);
void USART_ConfigParityError(USART_HANDLE * handle , uint8_t enable);
void USART_TX(USART_HANDLE * handle,uint8_t * buffer,uint16_t length);
uint8_t USART_RX(USART_HANDLE * handle,uint8_t * buffer,uint16_t length);
void USART_ClearErrorFlag(USART_HANDLE * handle);
void USART_TXEInterruptHandle(USART_HANDLE * handle);
void USART_RXNEInterruptHandle(USART_HANDLE * handle);
void USART_TXCompleteInterrupt(USART_HANDLE * handle);
void USART_RXNEInterruptHandle(USART_HANDLE * handle);
void USART_InterruptHandle(USART_HANDLE * handle);

#endif
