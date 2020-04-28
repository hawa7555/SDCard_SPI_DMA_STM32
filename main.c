#include "stm32f10x.h"
#include "delay.h"
#include "uart1.h"

uint8_t sdData[513];
int counter = 0;

void b2a(uint8_t data)
{
	int div = 100;
	while(div)
	{
		char sendChar = (data/div) + '0';
		transmit(sendChar);
		data = data % div;
		div = div / 10;
	}
}

void spiInit() {
	
	RCC->APB2ENR |= (1<<12);
	RCC->APB2ENR |= (1<<2); 
	RCC->APB2ENR |= (1<<0);
	
	GPIOA->CRL |= (1<<16) | (1<<17);         //gpio4 output 50Mhz
	GPIOA->CRL &= ~( (1<<18) | (1<<19) );   //push-pull
	
	GPIOA->CRL |= (1<<20) | (1<<21) | (1<<23);   //gpio5 output 50Mhz
	GPIOA->CRL &= ~(1<<22);                      //af push pull
	
	GPIOA->CRL |= (1<<28) | (1<<29) | (1<<31);   //gpio7 output 50Mhz
	GPIOA->CRL &= ~(1<<30);                      //af push pull
	
	//GPIOA->ODR &= ~(1<<4);          //slave select o/p to gnd
	
	GPIOA->CRL &= ~( (1<<24) |(1<<25) | (1<<27) );   //gpio6 input mode
	GPIOA->CRL |= (1<<26);           //floating input
	
	//SPI1->CR1 |= (1<<15) | (1<<14);  //bidirection and transmit only
	//SPI1->CR1 |= (1<<5) | (1<<3);    // freq/64
	SPI1->CR1 |= (1<<4);         // freq/8
	SPI1->CR1 |= (1<<2);         //master config
	//SPI1->CR1 |= (1<<10);           //rx only in 2-line unidirectional mode
	
	SPI1->CR2 |= (1<<2);    //no multimaster
	//SPI1->CR2 |= (1<<6);     //receive interrupt
	
	//SPI1->CR1 |= (1<<6);    //enable spi
	//GPIOA->ODR &= ~(1<<4);          //slave select o/p to gnd
	//delay_ms(1);
}

void spiSend(uint8_t spiData) {
	SPI1->DR = spiData;
	while(! (SPI1->SR & 0x0002) );
}

uint8_t spiReceive() {
	while(! (SPI1->SR & 0x0001) );
	
	uint8_t data = SPI1->DR;
	return data;
}

/**
void SPI1_IRQHandler(void) {
	if( SPI1->SR & (1<<0) ) {
		data = SPI1->DR;
	}
} **/

void sendByte(uint8_t data, int len)
{
  for(int i = 0; i<len ; i++)
  {
    spiSend(data);
  }
 }

 uint8_t getResponse()
 {
	 uint8_t response;
	 for(int i = 0; i<30000; i++)
	 {
		 spiSend(0xff);
		 response = spiReceive();
		 if(response != 0xff) return response;
	 }
	 return 0xff;
 }
 
 uint8_t sendCommand(uint8_t cmnd, uint32_t argument, uint8_t crc)
{
	sendByte(0xff, 1);     //dummy byte
	
	spiSend(cmnd + 64);          
	for(int i = 24; i>=0; i-= 8)
	{
		spiSend(argument>>i);   //msb of argument should be sent first but spi transfers lsb
		}
	spiSend(crc);        //crc
		
	return getResponse();
}

void send_cmd55()
{
	do
    {
      spiSend(0x77);         //cmd55 command
      sendByte(0x00,4);      //arg
      spiSend(0xff);         //crc
     }while(getResponse() == 0xff);
}

void fillData()
{
	sdData[0] = 0xfc;             //start token for every sector write
	for(int i = 1; i <513; i++)   //512 bytes of data
	{
		sdData[i] = i-1;
	}
}

void dmaInit() {
	
	RCC->AHBENR |= (1<<0);    //dma1 clock
	SPI1->CR2 |= (1<<1);    //DMA enabled for spi
	
	DMA1_Channel3->CCR |= (1<<13) | (1<<12);   //very high priority
	
	//(spi was set for 8 bit format so this bit was not creating any changes)
	//DMA1_Channel3->CCR = (1<<8);  //PSize is 16bit 
	
	DMA1_Channel3->CCR |= (1<<7);    //memory increment
	DMA1_Channel3->CCR |= (1<<5);    //circular mode
	
	DMA1_Channel3->CCR |= (1<<4);     //read from memory
	//DMA1_Channel3->CCR &= (1<<4);     //read from peripheral
	
	DMA1_Channel3->CCR |= (1<<1);      //interrupt enable
	
	DMA1_Channel3->CNDTR = 513;    //no of transfers before interrupt
	
	DMA1_Channel3->CPAR = (uint32_t) &SPI1->DR;
	DMA1_Channel3->CMAR = (uint32_t) sdData;
}

void DMA1_Channel3_IRQHandler() {
	
	if( DMA1->ISR & (1<<9) )
	{
		DMA1->IFCR |= (1<<9);     //clear flag bit
		DMA1_Channel3->CCR &= ~(1<<0);    //stop DMA
		//b2a( getResponse() );
		getResponse();
		
		do{
			spiSend(0xff);
		}while(spiReceive() != 0x00);       //wait till card doesn't issue busy token
		
		do{
			spiSend(0xff);
		}while(spiReceive() == 0x00);       //wait till card is busy in writing
		
		counter++;
		if(counter == 3)
		{
			//DMA1_Channel3->CCR &= ~(1<<0);    //stop DMA
			/**do{
				spiSend(0xff);
				 }while(spiReceive() != 0x00);       //wait till card doesn't issue busy token
		
			do{
				spiSend(0xff);
				 }while(spiReceive() == 0x00);       //wait till card is busy in writing**/
		
			sendCommand(13, 0x00000000, 0xff);   //cmd13 stop token
			getResponse();
			//transmitString(": Stop token Response\r\n");
	
			transmitString("Data Write Completed\r\n");
			return;
		}
	/**	
		do{
			spiSend(0xff);
		}while(spiReceive() != 0x00);       //wait till card doesn't issue busy token
		
		do{
			spiSend(0xff);
		}while(spiReceive() == 0x00);       //wait till card is busy in writing
		
		sendCommand(13, 0x00000000, 0xff);   //cmd13 stop token
		getResponse();
		transmitString(": Stop token Response\r\n");
	
	transmitString("Data Write Completed\r\n");
	**/
	//DMA1->IFCR |= (1<<9);     //clear flag bit
  sendByte(0xff, 20);         //dummy clocks		
	DMA1_Channel3->CCR |= (1<<0);   //enable DMA
  }
}

int main()
{
	uart1_init();
	//transmitString("Starting...");
	
	fillData();
	
	tim3_init();
	spiInit();
	SPI1->CR1 |= (1<<6);    //enable spi
	
	dmaInit();
	NVIC_EnableIRQ(DMA1_Channel3_IRQn);
	
	GPIOA->ODR &= ~(1<<4);          //slave select o/p to GND
	delay_ms(1);
	
	sendByte(0xff, 20);    //dummy bytes
	
	sendCommand(0, 0x00000000, 0x95);   //cmd0 command and response
	sendCommand(8, 0x000001aa, 0x87);   //cmd8 command
	
  for(int i = 0; i<4; i++)    //further response of cmd 8 cmmnd
	{
		getResponse();
	}
	
	do{
    send_cmd55();																		      //cmd55 before every acmd command
  }while( sendCommand(41, 0x40000000, 0xff) != 0x00);    // acmd41, to initialize sdCard
	
	sendCommand(16, 0x00000200, 0xff);    //cmd16, argument define 512 bytes as sector size
	
	//sendCommand(24, 0x00000000, 0xff);    //cmd24, single write initialize
	sendCommand(25, 0x00000000, 0xff);    //cmd25, multi write initialize
	
	sendByte(0xff, 20);                   //dummy clocks
	
	DMA1_Channel3->CCR |= (1<<0);        //enable dma channel
	
	while(1);
}


