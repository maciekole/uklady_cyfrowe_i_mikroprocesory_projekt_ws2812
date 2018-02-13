
//includes------------------------------------------------------<>
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_usart.h"
#include "misc.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

//defines-------------------------------------------------------<>
#define	GREEN_LED1	GPIOD,	GPIO_Pin_12		// dioda zielona
#define	RED_LED2	GPIOD,	GPIO_Pin_13		// dioda czerwona
#define	ORANGE_LED3	GPIOD,	GPIO_Pin_14		// dioda pomaranczowa
#define	BLUE_LED4	GPIOD,	GPIO_Pin_15		// dioda niebieska

#define	BUTTON	GPIOA,	GPIO_Pin_0			// przycisk uzytkownika

//-----------------------------------------------ws2812{}
/*
 * --------------- WS2812 manual ----------------
 * power supply voltage: +3.5 ~ +5.3 V
 *
 * data transfer time = TH + TL = 1.25us +/-150ns
 * 		T0H - 0, high voltage - 0.35us - +/-150ns
 * 		T0L - 0, low voltage  - 0.9us  - +/-150ns
 * 		T1H - 1, high voltage - 0.9us  - +/-150ns
 * 		T1L - 1, low voltage  - 0.35us - +/-150ns
 * 		RES - low voltage     - above 50us
 *
 * 		0 code - T0H + T0L
 * 		1 code - T1H + T1L
 *
 * data order is GRB !
 * 3 * 8 bit = 24 bit per LED =~ 30us
 * Transmission pause =~ 2 * LED
 *
 * TIM3 = 2*APB1 => TIM_CLK = 84MHz
 * PWM 	= TIM_CLK/ (period + 1)/(prescale + 1)
 * ----------------------------------------------
 *
 *   PRESCALER    0	84 MHz (11.9ns)
 *   PERIOD     104	80 kHz (1.25us)
 *   LO_TIME     29	29 * 11.9ns = 0.34us
 *   HI_TIME     76	76 * 11.9ns = 0.90us
 */
#define  LED_QTY    8		// ilosc diod ws2812 w panelu
#define  LED_MAX    8		// masksymalna ilosc diod

#define  PRESCALER    0		// wartosc preskalera
#define  PERIOD		104 	// wartosc okresu
#define  LO_TIME	 29		// czas trwania stanu niskiego
#define  HI_TIME	 76		// czas trwania stanu wysokiego

#define  TIMER_BUF_LENGTH1   (LED_QTY + 2) * 24		// dlugosc bufora dla bitow kazdej diody
#define  TIMER_BUF_LENGTH    (LED_MAX + 3) * 24		// (3*8rgb + pauza) * diody w panelu

typedef struct {
  uint8_t red;
  uint8_t green;
  uint8_t blue;
}RGB_t;			// typ RGB_t jest odwzorowaniem kolorow dla jednej diody

/*
 * 	Colour codes
 * 	------------------------------
 * 	3 x 8 bit, RGB
 */
#define RGB_COLOUR_OFF      (RGB_t) {0x00,0x00,0x00}		// off : 0,0,0
#define RGB_COLOUR_BLUE     (RGB_t) {0x00,0x00,0xFF}		// niebieski : 0,0,255
#define RGB_COLOUR_GREEN    (RGB_t) {0x00,0xFF,0x00}		// zielony : 0,255,0
#define RGB_COLOUR_RED      (RGB_t) {0xFF,0x00,0x00}		// czerwony : 255,0,0
#define RGB_COLOUR_WHITE    (RGB_t) {0xFF,0xFF,0xFF}		// bialy : 255,255,255
#define RGB_COLOUR_CYAN     (RGB_t) {0x00,0xFF,0xFF}		// cyan : 0,255,255
#define RGB_COLOUR_MAGENTA  (RGB_t) {0xFF,0x00,0xFF}		// magenta : 255,0,255
#define RGB_COLOUR_YELLOW   (RGB_t) {0xFF,0xFF,0x00}		// yellow : 255,255,0
//-----------------------------------------------ws2812{}

//-----------------------------------------------usart{}
#define BUFFER_SIZE	128		// dlugosc bufora dla danych przesylanych przez usart
//-----------------------------------------------usart{}

#define FRAME_SIZE 100		// wielkosc ramki dla komendy


//global variables----------------------------------------------<>
//-----------------------------------------------ws2812{}
uint32_t dma_status;					// status dma
uint16_t TIMER_BUF[TIMER_BUF_LENGTH];	// bufor wypelnienia sygnalu "1" i "0" (HI_TIME i LO_TIME)
uint8_t  channel;						// kanal timera = 1, mozliwosc rozbudowania programu do podlaczeniu wielu paneli do uC
uint32_t ledmax;						// zmienna odpowiadajaca maksymalnej ilosc diod w panelu
uint32_t pos;							// zmienna pozycji dla bufora TIMER_BUF
RGB_t *str[LED_QTY];								// dowolna dioda przedstawiona za pomoca kolorow
RGB_t LED_BUF[LED_QTY];					// tablica diod
//-----------------------------------------------ws2812{}

//-----------------------------------------------usart{}
int rxEmpty = 0;								// ...
int rxBusy = 0;									// zmienne pozycji bufora
volatile char received_buffer[BUFFER_SIZE];		// bufor danych odbieranych

int txEmpty = 0;								// ...
int txBusy = 0;									// zmienne pozycji bufora
volatile char send_buffer[BUFFER_SIZE];			// bufor danych wysylanych

char fBuff[30];

char test[8] = {'B','I','C','E','P','S','\n','\r'};
//-----------------------------------------------usart{}
char frame[FRAME_SIZE];				// ramka komendy do analizy
int frame_position = 0;				// pozycja w ramce
uint8_t ordinary_number = 0;
typedef enum
{
	IDLE,
	START,
	STOP
}status;							// status protokolu

status protocol_status2 = IDLE;		// domyslnie oczekujacy

int protocol_timer = 0;				// ?

char led_array[8];
int led_array_pointer = 0;

//functions-----------------------------------------------------<>

//-----------------------------------------------ws2812{}
/*
 * ws2812_Init_gpio
 * inicjalizacja portu GPIO dla wejscia panelu diod ws2812
 *
 * GPIOC
 * DIN -> PC6
 */
void ws2812_Init_gpio(void)
{
	GPIO_InitTypeDef gpio_struct;							// dekalaracja struktury

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);	// uruchomienie zegara dla GPIO

	gpio_struct.GPIO_Pin = GPIO_Pin_6;						// konfiguracja pinu 6
	gpio_struct.GPIO_Mode = GPIO_Mode_AF;					// tryb alternate function, pin jako sygnal od timera TIM3
	gpio_struct.GPIO_Speed = GPIO_Speed_100MHz;				// predkosc
	gpio_struct.GPIO_OType = GPIO_OType_PP;					// ...
	gpio_struct.GPIO_PuPd = GPIO_PuPd_UP ;					// ...
	GPIO_Init(GPIOC, &gpio_struct);							// inicjalizacja struktury

	GPIOC->BSRRH = GPIO_Pin_6;								// ustawiamy PC6 w stan wysoki

	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM3);	// konfiguracja pinu jako AF dla TIM3
}

/*
 * ws2812_Init_tim
 * inicjalizacja timera TIM3
 */
void ws2812_Init_tim(void)
{
	TIM_TimeBaseInitTypeDef  time_base_struct;
	TIM_OCInitTypeDef  oc_struct;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

	time_base_struct.TIM_Period = PERIOD;					// okres = 104
	time_base_struct.TIM_Prescaler = PRESCALER;				// prescaler = 0
	time_base_struct.TIM_ClockDivision = TIM_CKD_DIV1;
	time_base_struct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &time_base_struct);

	oc_struct.TIM_OCMode = TIM_OCMode_PWM1;					// timer w trybie PWM
	oc_struct.TIM_OutputState = TIM_OutputState_Enable;
	oc_struct.TIM_Pulse = 0;
	oc_struct.TIM_OCPolarity = TIM_OCPolarity_High;

    TIM_OC1Init(TIM3, &oc_struct);
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM3, ENABLE);						// enable timer
}

/*
 * ws2812_Init_dma
 * inicjalizacja dma DMA1_Stream4
 */
void ws2812_Init_dma(void)
{
	DMA_InitTypeDef dma_struct;

    DMA_Cmd(DMA1_Stream4, DISABLE);										// dezaktywacja dma, bedziemy aktywowac dla kazdej zmiany kolorow
    DMA_DeInit(DMA1_Stream4);

    dma_struct.DMA_Channel = DMA_Channel_5;								// kanal 5
    dma_struct.DMA_PeripheralBaseAddr = (uint32_t) &(TIM3->CCR1);		// adres bazowy pod ktory ladujemy dane
    dma_struct.DMA_Memory0BaseAddr = (uint32_t)TIMER_BUF;				// wartosci do zaladowania
    dma_struct.DMA_DIR = DMA_DIR_MemoryToPeripheral;					// kierunek
    dma_struct.DMA_BufferSize = TIMER_BUF_LENGTH;									// wielkosc bufora, 24 bity * 8 led
    dma_struct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dma_struct.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma_struct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; // 16bit
    dma_struct.DMA_MemoryDataSize = DMA_PeripheralDataSize_HalfWord;
    dma_struct.DMA_Mode = DMA_Mode_Normal;
    dma_struct.DMA_Priority = DMA_Priority_VeryHigh;
    dma_struct.DMA_FIFOMode = DMA_FIFOMode_Disable;
    dma_struct.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
    dma_struct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    dma_struct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA1_Stream4, &dma_struct);
}

/*
 * DMA1_Stream4_IRQHandler
 * obsluga przerwania od dma
 */
void DMA1_Stream4_IRQHandler(void)
{
	if (DMA_GetITStatus(DMA1_Stream4, DMA_IT_TCIF4))		// sprzawdz, czy dma zakonczyl transfer
	{
		DMA_ClearITPendingBit(DMA1_Stream4, DMA_IT_TCIF4);	// reset flagi przerwania

		TIM_Cmd(TIM3, DISABLE);								// wylacz timer
		DMA_Cmd(DMA1_Stream4, DISABLE);						// wylacz dma

		dma_status = 0;										// ustaw status na 'ready'
	}
}

/*
 *
 */
void ws2812_DMA_Start(void)
{
	dma_status = 1;									// ustaw status dma na 'busy'

	ws2812_Init_dma();

   DMA_ITConfig(DMA1_Stream4, DMA_IT_TC, ENABLE);	// przerwanie Transfer_Complete aktywowane
    DMA_Cmd(DMA1_Stream4, ENABLE);					// wlacz dma

    TIM_Cmd(TIM3, ENABLE);							// wlacz timer
}

/*
 * ws2812_Init_nvic
 * inicjalizacja przerwan
 */
void ws2812_Init_nvic(void)
{
	NVIC_InitTypeDef nvic_struct;

    TIM_DMACmd(TIM3, TIM_DMA_CC1, ENABLE);			 	// wlaczenie zapytan timera do dma (?)

    nvic_struct.NVIC_IRQChannel = DMA1_Stream4_IRQn;	// przerwania globalne dla DMA1_Stream4
    nvic_struct.NVIC_IRQChannelPreemptionPriority = 0;
    nvic_struct.NVIC_IRQChannelSubPriority = 0;
    nvic_struct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic_struct);
}

/*
 * SetChannel(uint8_t ch)
 * ustawienie kanalu timera
 *
 * wartosc przyjeta jest rowna zawsze channel = 1
 *
 * *zaimplementowana jest wersja tylko dla jednego panelu ws2812, w razie potrzeby mozliwe jest rozszerzenie o dodatkowe
 *  panele i podpiecie ich do innych portow
 */
void ws2812_SetChannel(uint8_t ch)
{
    if(ch == 1) {
      channel = 1;
      str[ch] = &LED_BUF[0];
      ledmax = LED_QTY;
    }
    else	return;
}

/*
 * Fill_Timer_Buf
 * wypelnia bufor timera wartosciami rgb danej diody
 */
void ws2812_Fill_Timer_Buf(void)
{
	uint32_t no = 0;
	uint8_t mask = 0x80;	//	maska = 0x80 = 1000 0000
	RGB_t led;
    pos=0;
    TIMER_BUF[pos++]=0;
    TIMER_BUF[pos++]=0;
	for(no = 0; no < ledmax; no++)
	{
		led.blue = str[no]->blue;
		led.green=str[no]->green;
		led.red=str[no]->red;

		mask = 0x80;
		while(mask != 0)	//	green 8-bit
		{
			TIMER_BUF[pos] = LO_TIME;
			if((led.green&mask) != 0) TIMER_BUF[pos] = HI_TIME;
			pos++;
			mask >>= 1;													// 000000000 timer_buf 00000
		}
		mask = 0x80;

		while(mask != 0)	//	red 8-bit
        {
        	TIMER_BUF[pos] = LO_TIME;
        	if((led.red&mask) != 0) TIMER_BUF[pos] = HI_TIME;
        	pos++;
        	mask >>= 1;
        }
        mask = 0x80;

        while(mask != 0)	//	blue 8-bit
        {
        	TIMER_BUF[pos] = LO_TIME;
        	if((led.blue&mask) != 0) TIMER_BUF[pos] = HI_TIME;
        	pos++;
        	mask >>= 1;
        }

	}
	TIMER_BUF[pos++]=0;
	pos = 0;
}

/*
 * Refresh
 * odswiezenie\aktualizacja kolorow na panelu
 *
 * umozliwia zapalenie lub zgaszenie diody
 */
void ws2812_Refresh(void)
{
	while(dma_status != 0); // DMA status busy?

	ws2812_Fill_Timer_Buf();

	ws2812_DMA_Start();
}

/*
 * All_LED_RGB(RGB_t rgb, uint8_t refresh)
 * rgb 		- kod rgb
 * refresh	- 1 jezeli chcemy zaktualizowac panel
 *
 * ustawia kolor na wszystkich diodach w panelu
 */
void ws2812_All_LED_RGB(RGB_t *rgb, uint8_t refresh)
{
	uint32_t n;

	for(n = 0; n < ledmax; n++)
	{
		str[n] = rgb;
	}
	if(refresh == 1) ws2812_Refresh();
}

/*
 * One_LED_RGB(uint32_t on, RGB_t rgb, uint8_t refresh)
 * on 		- numer diody
 * rgb 		- kod rgb
 * refresh	- 1 jezeli chcemy zaktualizowac panel
 *
 * ustawia kolor zadanej diody w panelu
 */
void ws2812_One_LED_RGB(uint32_t on, RGB_t *rgb, uint8_t refresh)
{
	if(on < ledmax)
	{
		str[on] = rgb;
		if(refresh == 1) ws2812_Refresh();
	}
}

/*
 * Reset_LEDs
 * wylacz\zresetuj wszystkie diody
 */
void ws2812_Reset_LEDs(void)
{
	ws2812_SetChannel(1);

    while(dma_status != 0);
    ws2812_DMA_Start();

    ws2812_SetChannel(1);
    ws2812_All_LED_RGB(&RGB_COLOUR_OFF, 1);
}

/*
 * Init
 * inicjalizacja peryferiow potrzebnych do obslugi ws2812
 */
void ws2812_Init(void)
{
	uint32_t n;

	dma_status = 0;
	channel = 0;
	ledmax = 0;

	for(n = 0; n < TIMER_BUF_LENGTH; n++)
	{
		TIMER_BUF[n] = 0;					// utworzenie bufora wartosci czasowych i wypelnienie go '0'
	}

    for(n = 0; n < LED_QTY; n++)
    {
    	LED_BUF[n] = RGB_COLOUR_OFF;		// utworzenie tablic kolorow dla kazdej diody
    }
    channel = 1;

    ws2812_SetChannel(channel);				// wybor kanalu (TIM3 CH1 - PC6)

    ws2812_Init_gpio();
    ws2812_Init_tim();
    ws2812_Init_nvic();
    ws2812_Init_dma();

    ws2812_Reset_LEDs();  					// Reset wszystkich diod w panelu
}

//-----------------------------------------------ws2812{}

//-----------------------------------------------usart{}
/*
 * usart_Init_gpio
 * inicjalizacja portu GPIO dla usart
 *
 * GPIOB
 * PB6 - TXD
 * PB7 - RXD
 */
void usart_Init_gpio(void)
{
	GPIO_InitTypeDef gpio;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);	// uruchomienie zegara

	// PB6 - TX | PB7 - RX
	gpio.GPIO_Mode = GPIO_Mode_AF;							// tryb AF
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;				// piny PB6 i PB7
	gpio.GPIO_PuPd = GPIO_PuPd_UP;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOB, &gpio);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1); // ustawienie AF na usart
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1); // --||--
}

/*
 * usart_Init_usart
 * inicjalizacja usart i ustawienie parametrow
 */
void usart_Init_usart(void)
{
	USART_InitTypeDef usart;
	NVIC_InitTypeDef  nvic;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);				// zegar

	usart.USART_BaudRate = 9600;										// predkosc transmisji na 9600
	usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	// brak kontroli przeplywu
	usart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					// praca w trybie nadawczo - obdiorczym
	usart.USART_Parity = USART_Parity_No;								// kontrola parzystosci
	usart.USART_StopBits = USART_StopBits_1;							// ilosc bitow stopu
	usart.USART_WordLength = USART_WordLength_8b;						// dlugosc przesylanego slowa
	USART_Init(USART1, &usart);
	USART_Cmd(USART1, ENABLE);

	//USART1 -> CR1 |= USART_CR1_RXNEIE; // USART1 Receive Interrupt Enable
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);						// aktywacja przerwan usart - odbior
	//USART_ITConfig(USART1, USART_IT_TXE, ENABLE);

	nvic.NVIC_IRQChannel = USART1_IRQn;									// konfiguracja przerwania dla USART1
	nvic.NVIC_IRQChannelCmd = ENABLE;
	nvic.NVIC_IRQChannelPreemptionPriority = 0;
	nvic.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&nvic);
}

/*
 * USART1_IRQHandler
 * obsluga przerwan usart
 */
void USART1_IRQHandler(void)
{
	int i;
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)			// jezeli przerwanie wywolanie przez odbior danych
	{
		if(rxBusy <= BUFFER_SIZE)									// odbieramy ...
		{
			received_buffer[rxBusy] = USART_ReceiveData(USART1);
			rxBusy++;
		}
		if(rxEmpty >= BUFFER_SIZE) rxEmpty = 0;						// sprawdzenie przepelnienia bufora
		if(rxEmpty == rxBusy) rxBusy++;
		if(rxBusy >= BUFFER_SIZE) rxBusy = 0;
	}

	/*
	 * ver 2
	 */
	if(USART_GetITStatus(USART1, USART_IT_TXE) != RESET)			// jezeli przerwanie wywolane przez wysylanie danych
	{
		if(txBusy != txEmpty)										// sprawdzamy czy jest cos do wyslania
		{
			USART_SendData(USART1, send_buffer[txBusy]);			// wysylamy ...
			//send_buffer[txBusy] = 0;
			txBusy++;
			if(txEmpty >= BUFFER_SIZE) txEmpty = 0;					// sprawdzenie czy nie przekroczylismy granicy bufora
			if(txBusy >= BUFFER_SIZE) txBusy = 0;
		}
		else														// jezeli nic nie ma do wyslania
		{
			USART_ITConfig(USART1, USART_IT_TXE, DISABLE);			// blokujemy przerwanie
		}
	}
	/*
	 * ver 2
	 */
}

/*
 * ! zle !
 * nie uzywac !!
 */
void usart_Send(volatile char *c, ...)
{
	while(*c)
	{
		while( !(USART1 -> SR & 0x00000040));

		USART_SendData(USART1, *c);

		*c++;
	}
}

/*
 * usart_uSend(char* format, ...)
 * funkcja wysylajace dane
 * przyjmuje rozna liczbe argumentow przy wywolaniu
 */
void usart_uSend(char* format, ...)
{
	char tmp_buf[128];						// bufor pomocniczy
	__IO int index;
	int i;

	va_list arglist;						// tworzymy liste argumentow
	va_start(arglist, format);				// z elementow przekazanych do funkcji
	vsprintf(tmp_buf, format, arglist);		// i zapisujemy do bufora pomocniczego
	va_end(arglist);

	index = txEmpty;
	for(i = 0; i < strlen(tmp_buf); i++)	// uzupelniamy bufor nadawczy
	{
		send_buffer[index] = tmp_buf[i];
		index++;
		if(index >= BUFFER_SIZE)			// uwaga na przekroczenie bufora
		{
			index = 0;
		}
	}

	__disable_irq();						// zablokowanie przerwan uC

	if((txEmpty == txBusy) && (USART1 -> SR & USART_FLAG_TXE)) // kiedy mamy dane do wyslania
	{
		txEmpty = index;
		USART_SendData(USART1, send_buffer[txBusy]);			// rozpoczynamy nadawanie
		txBusy++;
		if(txBusy >= BUFFER_SIZE)
		{
			txBusy = 0;
		}
		USART_ITConfig(USART1, USART_IT_TXE, ENABLE);			// aktywujemy przerwanie od transmisji
	}
	else
	{
		txEmpty = index;
	}

	__enable_irq();												// odblokowanie przerwan
}

/*
 *
 */
void usart_Send_Char(volatile char c)
{
	while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
	USART_SendData(USART1, c);
}

/*
 * usart_Get_Char
 * funkcja sprawdzajaca odebrany znak
 */
uint8_t usart_Get_Char(void)
{
	uint8_t tmp;								// zmienna pomocnicza
	if(rxEmpty != rxBusy)						// jezeli cos przyszlo
	{
		tmp = received_buffer[rxEmpty];			// zmienna = odebrany znak
		rxEmpty++;
		if(rxEmpty >= BUFFER_SIZE) rxEmpty = 0;	// uwaga na przekroczenie granicy
		return tmp;								// zwroc wartosc znaku
	}
	else return 0;
}

//-----------------------------------------------usart{}

//-----------------------------------------------commands{}
/*
 * HELP											1
 * RGB_COLOUR_OFF      (RGB_t) {0x00,0x00,0x00}	2
 * RGB_COLOUR_BLUE     (RGB_t) {0x00,0x00,0xFF}	3
 * RGB_COLOUR_GREEN    (RGB_t) {0x00,0xFF,0x00}	4
 * RGB_COLOUR_RED      (RGB_t) {0xFF,0x00,0x00}	5
 * RGB_COLOUR_WHITE    (RGB_t) {0xFF,0xFF,0xFF}	6
 * RGB_COLOUR_CYAN     (RGB_t) {0x00,0xFF,0xFF}	7
 * RGB_COLOUR_MAGENTA  (RGB_t) {0xFF,0x00,0xFF}	8
 * RGB_COLOUR_YELLOW   (RGB_t) {0xFF,0xFF,0x00}	9
 */
char c_Command_Help(void)	// 1
{
	if(frame[0] == 'h' && frame[1] == 'e' && frame[2] == 'l' && frame[3] == 'p' && frame_position == 4)
	{
		return 1;
	}
	return 0;
}

char c_Command_Off(void)	// 2
{
	if(frame[0] == 'o' && frame[1] == 'f' && frame[2] == 'f' && frame_position == 3)
	{
		return 1;
	}
	return 0;
}

char c_Command_Blue(void)	// 3
{
	if(frame[0] == 'b' && frame[1] == 'l' && frame[2] == 'u' && frame[3] == 'e' && frame_position == 4)
	{
		return 1;
	}
	return 0;
}

char c_Command_Green(void)	// 4
{
	if(frame[0] == 'g' && frame[1] == 'r' && frame[2] == 'e' && frame[3] == 'e' && frame[4] == 'n' && frame_position == 5)
	{
		return 1;
	}
	return 0;
}

char c_Command_Red(void)	// 5
{
	if(frame[0] == 'r' && frame[1] == 'e' && frame[2] == 'd' && frame_position == 3)
	{
		return 1;
	}
	return 0;
}

char c_Command_White(void)	// 6
{
	if(frame[0] == 'w' && frame[1] == 'h' && frame[2] == 'i' && frame[3] == 't' && frame[4] == 'e' && frame_position == 5)
	{
		return 1;
	}
	return 0;
}

char c_Command_Cyan(void)	// 7
{
	if(frame[0] == 'c' && frame[1] == 'y' && frame[2] == 'a' && frame[3] == 'n' && frame_position == 4)
	{
		return 1;
	}
	return 0;
}

char c_Command_Magenta(void)	// 8
{
	if(frame[0] == 'm' && frame[1] == 'a' && frame[2] == 'g' && frame[3] == 'e' && frame[4] == 'n' && frame[5] == 't' && frame[6] == 'a' && frame_position == 7)
	{
		return 1;
	}
	return 0;
}

char c_Command_Yellow(void)	// 9
{
	if(frame[0] == 'y' && frame[1] == 'e' && frame[2] == 'l' && frame[3] == 'l' && frame[4] == 'o' && frame[5] == 'w' && frame_position == 6)
	{
		return 1;
	}
	return 0;
}
//-----------------------------------------------commands{}


/*
 * Init_led
 * funkcja inicjujace diody led znajdujace sie na plytce
 */
void Init_led(void)
{
	GPIO_InitTypeDef gpio;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	gpio.GPIO_Pin = GPIO_Pin_15 | GPIO_Pin_14 | GPIO_Pin_13 | GPIO_Pin_12;
	gpio.GPIO_Mode = GPIO_Mode_OUT;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &gpio);
}

/*
 * Init_button
 * funkcja inicjujaca przycisk uzytkownika
 */
void Init_button(void)
{
	GPIO_InitTypeDef gpio;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	gpio.GPIO_Pin = GPIO_Pin_0;
	gpio.GPIO_Mode = GPIO_Mode_IN;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOA, &gpio);
}


/*
 * Init_all
 * inicjalizacja wszystkich peryferiow niezbednych do dzialania projektu
 */
void Init_all(void)
{
	Init_button();				// przycisk
	Init_led();					// ledy uC

	GPIO_SetBits(BLUE_LED4);	// zapalenie diody kontrolnej

	ws2812_Init();				// panel ws2812


	usart_Init_gpio();			// usart
	usart_Init_usart();
	GPIO_ResetBits(BLUE_LED4);	// zgaszenie diody kontrolnej
}

/*
 * funkcja nie jest uzywana!
 *
 * timeout protokolu (?)
 */
void p_Init_Timer(void)
{
	TIM_TimeBaseInitTypeDef timer;
	NVIC_InitTypeDef nvic;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	timer.TIM_CounterMode = TIM_CounterMode_Up;
	timer.TIM_Period = 1;
	timer.TIM_Prescaler = SystemCoreClock/1000;
	TIM_TimeBaseInit(TIM2, &timer);
	TIM_Cmd(TIM2, ENABLE);

	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

	nvic.NVIC_IRQChannel = TIM2_IRQn;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	nvic.NVIC_IRQChannelPreemptionPriority = 0;
	nvic.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&nvic);
}

/*
 *
 */
void TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

		GPIO_SetBits(RED_LED2);

		if(protocol_timer)
		{
			GPIO_ToggleBits(ORANGE_LED3);
			protocol_timer--;
		}
	}
}

/*
 * p_Clear_Buff_And_Frame
 * wyzerowanie wszystkich buforow
 */
void p_Clear_Buff_And_Frame2(void)
{
	int i;

	for(frame_position = 0; frame_position <= FRAME_SIZE; frame_position++)	// wyczyszczenie ramki
	{
		frame[frame_position] = 0;
	}
	frame_position = 0;

	for(rxBusy = 0; rxBusy <= BUFFER_SIZE; rxBusy++) 						// wyczyszczenie danych odebranych
	{
		received_buffer[rxBusy] = 0;
	}

	/*
	 * ver 2
	 */
	for(i = 0; i < TIMER_BUF_LENGTH; i ++)									// reset wartosci dla diod
	{
		TIMER_BUF[i] = 0;
	}

	/*
	 * ver 2
	 */

	protocol_status2 = IDLE;												// protokol oczekujacy
}

void p_Clear_Buff_And_Frame(void)
{
	//protocol_status2 = IDLE;

	int i;

	for(i = 0; i <= FRAME_SIZE; i++)	// wyczyszczenie ramki
	{
		frame[i] = 0;
	}
	frame_position = 0;

	for(i = 0; i <= BUFFER_SIZE; i++) 						// wyczyszczenie danych odebranych
	{
		received_buffer[i] = 0;
	}
	rxBusy = 0;	// <----------------------------------
	rxEmpty = 0;

	for(i = 0; i < TIMER_BUF_LENGTH; i ++)									// reset wartosci dla diod
	{
	//	TIMER_BUF[i] = 0;
	}

	for(i = 0; i < 8; i++)
	{
		led_array[i] = 0;
	}
	led_array_pointer = 0;

	protocol_status2 = IDLE;												// protokol oczekujacy
}


/*
 *
 */
int p_Is_Value_In_Array(int value, int *array, int size)
{
	int i;

	for(i = 0; i < size; i++)
	{
		if(atoi(array[i]) == value) return 1;
	}
	return 0;
}

/*
 * p_Check_Frame_Content
 * analiza zawartosci ramki, czyli odebranej komendy
 */
void p_Check_Frame_Content(void)
{
	int i = 0;

	RGB_t led1,led2,led3,led4,led5,led6,led7,led8;

	if(c_Command_Help() == 1)
	{
		usart_uSend(" --  POMOC  -- \n\r");
		usart_uSend(" format: (command) \n\r");
		usart_uSend(" lista komend: \n\r");
		usart_uSend("  -help \n\r");
		usart_uSend("  -off \n\r");
		usart_uSend("  -blue \n\r");
		usart_uSend("  -red \n\r");
		usart_uSend("  -green \n\r");
		usart_uSend("................ \n\r");

		p_Clear_Buff_And_Frame();
		return 0;
	}

	if(c_Command_Blue() == 1)
	{

		//p_Clear_Buff_And_Frame();

		usart_uSend(" ..BLUE.. \n\r");
		led1 = RGB_COLOUR_BLUE;
		ws2812_One_LED_RGB(0, &led1, 0);
		ws2812_One_LED_RGB(1, &led1, 0);
		ws2812_One_LED_RGB(2, &led1, 0);
		ws2812_One_LED_RGB(3, &led1, 0);
		ws2812_One_LED_RGB(4, &led1, 0);
		ws2812_One_LED_RGB(5, &led1, 0);
		ws2812_One_LED_RGB(6, &led1, 0);
		ws2812_One_LED_RGB(7, &led1, 1);

		p_Clear_Buff_And_Frame();
		return 0;
	}

	if(c_Command_Red() == 1)
	{
		//p_Clear_Buff_And_Frame();

		usart_uSend(" ..RED.. \n\r");
		led1 = RGB_COLOUR_RED;
		ws2812_One_LED_RGB(0, &led1, 1);

		p_Clear_Buff_And_Frame();
		return 0;
	}

	if(c_Command_Green() == 1)
	{
		//p_Clear_Buff_And_Frame();

		usart_uSend(" ..GREEN.. \n\r");
		led1 = RGB_COLOUR_GREEN;
		ws2812_One_LED_RGB(0, &led1, 1);

		p_Clear_Buff_And_Frame();
		for(int i=0;i<8;i++){
			usart_uSend("%03d,%03d,%03d\r\n",str[i]->green,str[i]->red,str[i]->blue);
		}
		return 0;
	}

	if(c_Command_Off() == 1)
	{
		//p_Clear_Buff_And_Frame();

		usart_uSend(" ..OFF.. \n\r");
		led1 = RGB_COLOUR_OFF;
		ws2812_One_LED_RGB(0, &led1, 1);

		p_Clear_Buff_And_Frame();
		return 0;
	}

	usart_uSend(" ..KOMENDA NIEZNANA, WPISZ (help) PO POMOC.. \n\r");
	p_Clear_Buff_And_Frame();
}

/*
 * p_Protocol
 * protokol komunikacyjny
 */
void p_Protocol(void)
{
	if(protocol_status2 == IDLE) 													// czy protokol oczekujacy
	{
		if(usart_Get_Char() == 40)													// czy odebrano '('
		{
			protocol_status2 = START;												// protokol zaczyna dzialanie

		}
	}

	if(protocol_status2 == START)													// czy protokol dziala
	{
		char read = usart_Get_Char();												// zmienna pomocnicza = odebrany znak
		if(read != 0 && read != 41)													// znak nie jest 'null' oraz ')'
		{
			frame[frame_position] = read;											// zapisz do ramki
			frame_position == FRAME_SIZE ? frame_position = 0 : frame_position++;	// uwaga na przekroczenie wielkosci ramki
			if(read == 40) frame_position = 0;
		}

		if(read == 41)																// odebrano ')'
		{
			protocol_status2 = STOP;												// protokol konczy swoje dzialanie
			p_Check_Frame_Content();												// analiza odebranych danych
		}
	}
	/*
	if(protocol_status2 == BLUE)
	{
		//usart_uSend("diody: 0 - 7, e = exit\r\n");

		char read = usart_Get_Char();
		if(read != 0 && read > 47 && read < 58)
		{
			//usart_uSend("#\r\n");
			led_array[led_array_pointer] = read;
			led_array_pointer++;
		}
		if(read == 101 || led_array_pointer == 8)
		{
			usart_uSend("#\r\n");
			protocol_status2 = ;
			p_Check_Frame_Content();
		}
		//protocol_status2 = ;
	}
*/
}

int main(void)
{
	Init_all();
	GPIO_SetBits(GREEN_LED1);

	GPIO_ToggleBits(GREEN_LED1);


	usart_uSend(" ------- ws2812 ------- \r\n");				// Komunikat powitalny
	usart_uSend("    Panel programu \r\n");
	usart_uSend(" (help) wyswietli pomoc \r\n");
	usart_uSend(" ...................... \r\n");

	for(int i = 0; i < 10000; i++)
	{
		i=i;
	}

	RGB_t led1 = RGB_COLOUR_BLUE;
	GPIO_ToggleBits(GREEN_LED1);
	ws2812_One_LED_RGB(0, &led1, 0);
	ws2812_One_LED_RGB(1, &led1, 0);
	ws2812_One_LED_RGB(2, &led1, 0);
	ws2812_One_LED_RGB(3, &led1, 0);
	ws2812_One_LED_RGB(4, &led1, 0);
	ws2812_One_LED_RGB(5, &led1, 0);
	ws2812_One_LED_RGB(6, &led1, 0);
	ws2812_One_LED_RGB(7, &led1, 1);
    while(1)
    {
    	p_Protocol();
    }
}
