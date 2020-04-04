#include "UART_DRIVER.h"
#include "stm32f4xx.h"

/*
Enable the given USART peripheral 
*/
void USART_Enable(USART_HANDLE * handle)
{
	if (handle->USARTx == USART1)
	{
		USART1_CLOCK_ENABLE();
		handle->USARTx->CR1 |=  (1<<13);
	}
	else if (handle->USARTx == USART2)
	{
		UART2_CLOCK_ENABLE();
		handle->USARTx->CR1 |=  (1<<13);
	}
	else if (handle->USARTx == USART3)
	{
		UART3_CLOCK_ENABLE();
		handle->USARTx->CR1 |=  (1<<13);
	}
	else if (handle->USARTx == UART4)
	{
		UART4_CLOCK_ENABLE();
		handle->USARTx->CR1 |=  (1<<13);
	}
	else if (handle->USARTx == UART5)
	{
		UART5_CLOCK_ENABLE();
		handle->USARTx->CR1 |=  (1<<13);
	}
	else if (handle->USARTx == USART6)
	{
		USART6_CLOCK_ENABLE();
		handle->USARTx->CR1 |=  (1<<13);
	}
}

void USART_Disable(USART_HANDLE * handle)
{
	if (handle->USARTx == USART1)
	{
		USART1_CLOCK_DISABLE();
	}
	else if (handle->USARTx == USART2)
	{
		UART2_CLOCK_DISABLE();
	}
	else if (handle->USARTx == USART3)
	{
		UART3_CLOCK_DISABLE();
	}
	else if (handle->USARTx == UART4)
	{
		UART4_CLOCK_DISABLE();
	}
	else if (handle->USARTx == UART5)
	{
		UART5_CLOCK_DISABLE();
	}
	else if (handle->USARTx == USART6)
	{
		USART6_CLOCK_DISABLE();
	}
}

void USART_TX_EnableDisable(USART_TypeDef * uartx,uint8_t TE)
{
	if(ENABLE)
	{
		uartx->CR1 |= (1<<3);
	}
	else if(~ENABLE)
	{
		uartx->CR1 &= ~(1<<3);
	}
}

void USART_RX_EnableDisable(USART_TypeDef * uartx,uint8_t RE)
{
	if(RE)
	{
		uartx->CR1 |= (1<<2);
	}
	else
	{
		uartx->CR1 &= ~(1<<2);
	}
}

void USART_WordLengthConfig(USART_HANDLE * handle)
{
	if(handle->Init.WordLength == 0x08)
	{
		handle->USARTx->CR1 &= ~(1<<12);
	}
	else if(handle->Init.WordLength == (uint32_t)0x09)
	{
		handle->USARTx->CR1 |= (1<<12);
	}
	else
	{
	
	}
}

void USART_StopBitsConfig(USART_HANDLE * handle)
{
	
	switch(handle->Init.StopBits)
	{
		case USART_STOP_BITS_1 : handle->USARTx->CR2 &= ~(0x03<<13);
		break;
		case USART_STOP_BITS_2 : handle->USARTx->CR2 |= (0x02<<13);
		break;
	}
}

void USART_BaudRateConfig(USART_HANDLE * handle)
{
	switch(handle->Init.BaudRate)
	{
		case(USART_BAUD_9600):	
			handle->USARTx->BRR = (uint32_t)0x0D03	;
			break;
		case(USART_BAUD_115200):
			handle->USARTx->BRR = (uint32_t)0x0113;
			break;
	}
}

void USART_OversamplingConfig(USART_HANDLE * handle)
{
	if(handle->Init.OverSampling == 8)
	{
		handle->USARTx->CR1 |= (1<<15);	//	1: oversampling by 8
	}
	else if(handle->Init.OverSampling == 16)
	{
		handle->USARTx->CR1 &= ~(1<<15);	//	0: oversampling by 16
	}
	
}

void USART_TXInterruptEnable(USART_TypeDef *uartx, uint32_t txe_en)
{
	if(txe_en)
		uartx->CR1 |= (1<<7);
	else
		uartx->CR1 &= ~(1<<7);
}

void USART_RxInterruptEnable(USART_TypeDef *uartx, uint32_t rxne_en)
{
	if(rxne_en)
		uartx->CR1 |= (1<<5);
	else
		uartx->CR1 &= ~(1<<5);
}



void USART_ConfigErrorInterrupt(USART_HANDLE * handle , uint8_t enable)
{
if(enable)
{
	handle->USARTx->CR3 |= (1<<0);
}
else
{
handle->USARTx->CR3 &= ~(1<<0);
}
}


void USART_Init(USART_HANDLE * handle)
{
	USART_Enable(handle);
	handle->TX_State = USART_STATE_BUSY;
	USART_WordLengthConfig(handle);
	USART_StopBitsConfig(handle);
	USART_BaudRateConfig(handle);
	
	USART_TX_EnableDisable(handle->USARTx,ENABLE);	//TX enabled for this app , otherwise use DISABLED
	handle->TX_State = USART_STATE_READY;
	
	/* In asynchronous mode, the following bits must be kept cleared:
     - LINEN and CLKEN bits in the USART_CR2 register,
     - SCEN, HDSEL and IREN  bits in the USART_CR3 register.*/
  CLEAR_BIT(handle->USARTx->CR2, (USART_CR2_LINEN | USART_CR2_CLKEN));
  CLEAR_BIT(handle->USARTx->CR3, (USART_CR3_SCEN | USART_CR3_HDSEL | USART_CR3_IREN));
	
	USART_RX_EnableDisable(handle->USARTx,ENABLE);	//RX enabled for this app , otherwise use DISABLED
	USART_OversamplingConfig(handle);
	handle->RX_State = USART_STATE_READY;
	
	handle->Error = USART_ERROR_NONE;
}


void USART_ConfigParityError(USART_HANDLE * handle , uint8_t enable)
{
	if (enable)
	{
	handle->USARTx->CR1 |= (1<<8); 
	}
	else
	{
		handle->USARTx->CR1 &= ~(1<<8);
	}
}



void USART_TX(USART_HANDLE * handle,uint8_t * data,uint16_t size)
{
	uint8_t TCFlag = handle->USARTx->SR & USART_SR_TC;
	while(!TCFlag)
	{
//	0: Transmission is not complete
//	1: Transmission is complete
	}
	handle->TX_State = USART_STATE_BUSY_TX;
	handle->TX_Buffer = data;
	handle->TX_Counter = size;
	handle->TX_Size = size;
	//USART_TXInterruptEnable(handle->USARTx, ENABLE);
	while(handle->TX_Counter > 0u)
	{
		handle->TX_Counter--;
		handle->USARTx->DR = ((*data++) &((uint8_t)0xff));
	}
		handle->TX_State = USART_STATE_READY;
}


uint8_t USART_RX(USART_HANDLE * handle , uint8_t * data , uint16_t size)
{
	static uint32_t value ;
	if(handle->RX_State == USART_STATE_READY)
	{
		handle->Error = USART_ERROR_NONE;
		handle->RX_Counter = size;
		handle->RX_Size = size;
		handle->RX_State = USART_STATE_BUSY_RX;
		while(handle->RX_Counter > 0u)
		{
			handle->RX_Counter--;
			*data++ = handle->USARTx->DR & (uint8_t)0xff;
			
		}
		handle->RX_State = USART_STATE_READY;
	}
	handle->RX_Buffer = data;
	handle->RX_Counter = size;
	handle->RX_Size = size;
	handle->RX_State = USART_STATE_BUSY_RX;
	USART_Enable(handle);	//maybe disabled by some other code
	USART_TXInterruptEnable(handle->USARTx, ENABLE);
	USART_ConfigErrorInterrupt(handle,ENABLE);
	USART_ConfigParityError(handle,ENABLE);
	USART_RxInterruptEnable(handle->USARTx, ENABLE);
	value = handle->USARTx->DR;
	return value;
}

void USART_ClearErrorFlag(USART_HANDLE * handle)
{
	uint32_t temp= 0;
	temp = handle->USARTx->SR;
	temp = handle->USARTx->DR;
}

void USART_TXEInterruptHandle(USART_HANDLE * handle)
{
	if(handle->TX_State == USART_STATE_BUSY_TX)
	{
		handle->USARTx->DR = (*(handle->TX_Buffer) & (uint8_t)0xFF);
		handle->TX_Buffer++;
		handle->TX_Counter--;
		if(handle->TX_Counter == 0)
		{
			handle->USARTx->CR1 |= USART_CR1_TCIE;//enable TX complete interrupt
			handle->USARTx->CR1 &= ~USART_CR1_TXEIE;//disable TX interrupt
		}
		
	}
}


void USART_RXNEInterruptHandle(USART_HANDLE * handle)
{
		if (handle->RX_State == USART_STATE_BUSY_RX)
	{
		if(handle->Init.Parity == 0)
		{
			*(handle->RX_Buffer) = (uint8_t)(handle->USARTx->DR) & (uint8_t)0xff;
			handle->RX_Buffer ++;
		}
		else
		{
			*(handle->RX_Buffer) = (uint8_t)(handle->USARTx->DR) & (uint8_t)0x7f;
			handle->RX_Buffer ++;
		}
		handle->RX_Counter--;
		if(handle->RX_Counter == 0)
		{
			handle->USARTx->CR1 &= ~USART_CR1_RXNEIE;
			handle->USARTx->CR1 &= ~USART_CR1_PEIE;
			handle->USARTx->CR1 &= ~USART_CR3_EIE;
			handle->RX_State = USART_STATE_READY;
		}
	}
}

void USART_TXCompleteInterrupt(USART_HANDLE * handle)
{
	handle->USARTx->CR1 &= ~USART_CR1_TCIE;
	handle->TX_State = USART_STATE_READY;
}

void USART_InterruptHandle(USART_HANDLE * handle)
{
	uint32_t temp1 = 0, temp2 = 0;

	temp1 = handle->USARTx->SR & USART_SR_PE;
  temp2 = handle->USARTx->CR1 & USART_CR1_PEIE;
  /* UART parity error interrupt occurred ------------------------------------*/
  if((temp1) && (temp2))
  { 
		USART_ClearErrorFlag(handle);
    
    handle->Error |= USART_ERROR_PE;
  }
  
	temp1 = handle->USARTx->SR & USART_SR_FE;
	temp2 = handle->USARTx->CR3 & USART_CR3_EIE;
  /* UART frame error interrupt occurred -------------------------------------*/
  if((temp1 ) && (temp2 ))
  { 
    USART_ClearErrorFlag(handle);
    
    handle->Error |= USART_ERROR_FE;
  }
  
  
	temp1 = handle->USARTx->SR & USART_SR_NE;
  temp2 = handle->USARTx->CR3 & USART_CR3_EIE;
  /* UART noise error interrupt occurred -------------------------------------*/
  if((temp1 ) && (temp2 ))
  { 
     USART_ClearErrorFlag(handle);
    
    handle->Error |= USART_ERROR_NE;
  }
  
 
	temp1 = handle->USARTx->SR & USART_SR_ORE;
  temp2 = handle->USARTx->CR3 & USART_CR3_EIE;
  /* UART Over-Run interrupt occurred ----------------------------------------*/
  if((temp1) && (temp2))
  { 
     USART_ClearErrorFlag(handle);
    
    handle->Error |= USART_ERROR_ORE;
  }
  
 
	 temp1 = handle->USARTx->SR & USART_SR_RXNE;
   temp2 = handle->USARTx->CR1 & USART_CR1_RXNEIE;
  /* UART in mode Receiver ---------------------------------------------------*/
  if((temp1 ) && (temp2))
  { 
    USART_RXNEInterruptHandle(handle);
  }
  
  
	temp1 = handle->USARTx->SR & USART_SR_TXE;
  temp2 = handle->USARTx->CR1 & USART_CR1_TXEIE;
  /* UART in mode Transmitter ------------------------------------------------*/
  if((temp1 ) && (temp2))
  {
    USART_TXEInterruptHandle(handle);
  }
  
	temp1 = handle->USARTx->SR & USART_SR_TC;
	temp2 = handle->USARTx->CR1 & USART_CR1_TCIE;
  /* UART in mode Transmitter end --------------------------------------------*/
  if((temp1 ) && (temp2))
  {
    USART_TXCompleteInterrupt(handle);
  }

 // if(handle->Error != USART_ERROR_NONE)
  {
    /* Set the UART state ready to be able to start again the process */
   /* handle->TX_State = USART_STATE_READY;
		handle->RX_State = USART_STATE_READY;
    
    hal_uart_error_cb(handle);*/
  }  
}


