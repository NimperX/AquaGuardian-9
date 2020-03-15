/*
 * AG_Remote_Final.c
 *
 * 
 */ 

#define F_CPU 8000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include "i2cmaster.h"
#include "i2c_lcd.h"

#define USART_BAUDRATE 9600
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

char RecievedByte,data[16];
float ph;

void system_init(void);
void UART_init(void);
unsigned char UART_RxChar(void);
void UART_TxChar(uint8_t data);
void UART_SendString(char *str);
void ADC_Init();
int ADC_Read(char channel);

void UART_init(void)
{
	UBRRH = (BAUD_PRESCALE >> 8);	/* Load upper 8-bits*/
	UBRRL = BAUD_PRESCALE;		/* Load lower 8-bits of the baud rate value */
	UCSRB |= (1 << RXEN) | (1 << TXEN) | (1<<RXCIE);/* Turn on transmission and reception */
	UCSRC |= (1 << URSEL) | (1 << UCSZ0) | (1 << UCSZ1);/* Use 8-bit character sizes */
}

unsigned char UART_RxChar(void)
{
	while ((UCSRA & (1 << RXC)) == 0);/* Wait till data is received */
	return UDR;			/* Return the byte*/
}

void UART_TxChar(uint8_t data)
{
	while (! (UCSRA & (1<<UDRE)));	/* Wait for empty transmit buffer*/
	UDR = data ;
}

void UART_SendString(char *str)
{
	for(int i=0;i<strlen(str);i++){
		UART_TxChar(str[i]);
	}
}

ISR(USART_RXC_vect){
	int ph_int;
	float ph_float;
	
	lcd_puts_at("Checking pH...  ",0,0);
	lcd_puts_at("                ",0,1);
	_delay_ms(3000);

	RecievedByte = UDR;
	
	ph_int = (RecievedByte/127)*1023;
	ph_float = -0.0343*ph_int + 34.31;

	sprintf(data,"pH = %2.2f",ph_float);
	lcd_puts_at(data,0,1);
	_delay_ms(3000);
}

int main(void)
{
	char Output;
	int engi,engi_old=512, rud,rud_old=512, dir=0,side=0;
	system_init();
	ADC_Init();
	UART_init();
	sei();
	
	DDRB |= 1<<PB0;
	
    while (1)
    {
			Output = '@';  //carrier char
			
			ADMUX &= 0b11111000;
			
			engi=ADC_Read(1);
			if(abs(engi-engi_old)>10){
				engi_old=engi;
			}
			
			ADMUX &= 0b11111000;
			
			rud=ADC_Read(0);
			if(abs(rud-rud_old)>10){
				rud_old=rud;
			}
			
			if(engi_old>800){
				Output = Output | (1<<0) | (0<<1);
				if(dir!=1){
					lcd_puts_at("Moving Backward  ",0,0);
					dir=1;
				}
			}else if(engi_old<250){
				Output = Output | (1<<0) | (1<<1);
				lcd_goto_xy(0,1);
				if(dir!=2){
					lcd_puts_at("Moving Forward  ",0,0);
					dir=2;
				}
			}else{
				Output = Output | (0<<0) | (0<<0);
				lcd_goto_xy(0,1);
				if(dir!=0){
					lcd_puts_at("Engine Off      ",0,0);
					dir=0;
				}
				
			}
			
			if(rud>800){
				Output = Output | (1<<2) | (0<<3);
				if(side!=1){
					lcd_puts_at("Turning left    ",0,1);
					side=1;
				}
			}else if(rud<250){
				Output = Output | (0<<2) | (1<<3);
				if(side!=2){
					lcd_puts_at("Turning right    ",0,1);
					side=2;
				}
			}else{
				Output = Output | (0<<2) | (0<<3);
				if(side!=0){
					lcd_puts_at("                ",0,1);
					side=0;
				}
			}
			
			if(PINB & (1<<PB0)){
				Output = Output | (1<<4);
			}else{
				Output = Output | (0<<4);
			}
			
			if(PINB & (1<<PB1)){
				lcd_puts_at("Engine Off      ",0,0);
				lcd_puts_at("                ",0,1);
			}
					
				
			UART_TxChar(Output);
			_delay_ms(100);
    }
}



void system_init(void){
	DDRA &= ~(1<<PA0) & ~(1<<PA1) & ~(1<<PA2) & ~(1<<PA3);
	DDRB &= ~(1<<PB0) & ~(1<<PB1);
	
	lcd_init(LCD_BACKLIGHT_ON);
	lcd_goto_xy(0,0);
	lcd_puts("     Aqua Guardian #9");
	lcd_puts_at("Team #28",4,1);
	_delay_ms(2000);
	lcd_puts_at("Engine Off      ",0,0);
	lcd_puts_at("                ",0,1);
}

void ADC_Init()
{
	DDRA=0x0;			/* Make ADC port as input */
	ADCSRA = 0x87;			/* Enable ADC, fr/128  */
	ADMUX = 0x40;			/* Vref: Avcc, ADC channel: 0 */
	
}

int ADC_Read(char channel)
{
	int Ain,AinLow;
	
	ADMUX=ADMUX|(channel & 0x0f);	/* Set input channel to read */

	ADCSRA |= (1<<ADSC);		/* Start conversion */
	while((ADCSRA&(1<<ADIF))==0);	/* Monitor end of conversion interrupt */
	
	_delay_us(10);
	AinLow = (int)ADCL;		/* Read lower byte*/
	Ain = (int)ADCH*256;		/* Read higher 2 bits and 
					Multiply with weight */
	Ain = Ain + AinLow;				
	return(Ain);			/* Return digital value*/
}
