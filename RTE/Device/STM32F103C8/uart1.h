#include "stm32f10x.h"

char receivedChar;

void uart1_init()
{
	RCC->APB2ENR |= ( (1<<2) | (1<<0) | (1<<14) );  //all clocks enable
	GPIOA->CRH |= ( (1<<7) | (1<<4) | (1<<5) );  //Alternate function push-
	GPIOA->CRH &= ~(1<<6);   //pull and output max 50Mhz
	
	USART1->CR1 &= ~(1<<12);  //8bit data
	USART1->CR2 &= ~( (1<<13) | (1<<12) );  //1 Stop bit
	USART1->BRR = 0x1D4C;   //9600 baud rate
	USART1->CR1 |=  ( (1<<13) | (1<<3) );  //Tx and UART Enabled
	USART1-> CR1 |= (1<<2);  //Rx enabled
}

void transmit(char data)
{
	USART1->DR = data;
	while(! (USART1->SR & 0x0080) );  //wait for TXE to 1
}

void transmitString(char* data)
{
	while(*data != '\0')
	{
		transmit(*data++);
	}
}
 
char uart1_receive() {
	while(! (USART1->SR & 0x0020) );  //wait for RXEN to 1
	receivedChar = USART1->DR;
	return receivedChar;
}


