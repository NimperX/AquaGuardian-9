/*
 * AG_Final.c
 *
 * 
 */ 

#define F_CPU 8000000UL
			
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#define USART_BAUDRATE 9600
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)


void servo_init(void);
void system_init(void);
void UART_init(void);
unsigned char UART_RxChar(void);
void UART_TxChar(uint8_t data);
void UART_SendString(char *str);
void Wait();
void ADC_Init();
int ADC_Read(char channel);

int lastservopos=0;
char RecievedByte;
volatile int eng,dir,rudl,rudr,ph_check,rudlrot=0, rudrrot=0, laststate=0, led;
volatile int ph_buff=0;
uint8_t temp;

int main(void)
{
	UART_init();
	servo_init();	//turn rudder to 90 degree
	system_init();	//initialize system
	ADC_Init();		//Initialize ADC
	sei();
	
	
    while (1) 
    {
			//if(RecievedByte=UART_RxChar()){
						
			//}
		
			if(PINB & (1<<PB2)){			//Light on flash bulb in the dark
				PORTB |= (1<<PB3);
			}
			else{
				PORTB &= ~(1<<PB3);
			}
			
			if(ph_buff){
				temp = (int)((ADC_Read(0)/1023)*127);
				UART_TxChar(ADC_Read(0));
				ph_buff=0;
			}
			
    }
}

void servo_init(void){
	DDRD |= (1<<PD5);	/* Make OC1A pin as output */
	TCNT1 = 0;		/* Set timer1 count zero */
	ICR1 = 2499;		/* Set TOP count for timer1 in ICR1 register */

	/* Set Fast PWM, TOP in ICR1, Clear OC1A on compare match, clk/64 */
	TCCR1A = (1<<WGM11)|(1<<COM1A1);
	TCCR1B = (1<<WGM12)|(1<<WGM13)|(1<<CS10)|(1<<CS11);
	
	OCR1A = 175;
	lastservopos=175;

}

void system_init(void){
	DDRB |= (1<<PB0) | (1<<PB1) | (1<<PB3);
	DDRB &= ~(1<<PB2);
	
	PORTB |= (1<<PB0);
	PORTB |= (1<<PB1);	//Turn off dc motor
	
	    
}

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

// void Wait(){
// 	uint8_t i;
// 	for(i=0;i<50;i++)
// 	{
// 		_delay_loop_2(0);
// 		_delay_loop_2(0);
// 		_delay_loop_2(0);
// 	}
// 
// }

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

ISR(USART_RXC_vect){
	int t=20;
	RecievedByte=UDR;
	
	eng = (RecievedByte & (1<<0));		//1-ON			0-OFF
	dir = (RecievedByte & (1<<1));		//1-Backward		0-Forward
	rudl = (RecievedByte & (1<<2));		//1-RotateRight
	rudr = (RecievedByte & (1<<3));		//1-RotateLeft
	ph_check = (RecievedByte & (1<<4));	//Ph check
	led = (RecievedByte & (1<<5));		//LED
	
	if(eng){
		if(!dir){
			if(laststate==1){
				PORTB |= (1<<PB0);
				PORTB |= (1<<PB1);
				_delay_ms(2000); //Turn off dc motor for 2 sec
				laststate=0;
			}
			
			PORTB |= (1<<PB0); //engine on and forward
			PORTB &= ~(1<<PB1);
		}
		else{
			if(laststate==0){
				PORTB |= (1<<PB0);
				PORTB |= (1<<PB1);
				_delay_ms(2000); //Turn off dc motor for 2 sec
				laststate=1;
			}
			
			PORTB |= (1<<PB1); //engine on and backward
			PORTB &= ~(1<<PB0);
		}
		
	}
	else{
		PORTB |= (1<<PB0);
		PORTB |= (1<<PB1); // engine off
	}
	
	if(rudl && rudlrot==0){
		//turn left rudder;
						
		if(lastservopos<100){
			for(int	i=lastservopos;i<100;i++){
				OCR1A = i;
				_delay_ms(t);
			}
		}else{
			for(int	i=lastservopos;i>100;i--){
				OCR1A = i;
				_delay_ms(t);
			}
		}
						
		lastservopos=100;
		rudlrot=1;
	}
	else if(rudr && rudrrot==0){
		//turn right rudder
						
		if(lastservopos<250){
			for(int	i=lastservopos;i<250;i++){
				OCR1A = i;
				_delay_ms(t);
			}
			}else{
			for(int	i=lastservopos;i>250;i--){
				OCR1A = i;
				_delay_ms(t);
			}
		}
						
		lastservopos=250;
		rudrrot=1;
	}
	else{
		if((rudlrot==1 || rudrrot==1) && (!rudr && !rudl)){
			//turn rudder to initial position
							
			if(lastservopos<175){
				for(int	i=lastservopos;i<175;i++){
					OCR1A = i;
					_delay_ms(t);
				}
				}else{
				for(int	i=lastservopos;i>175;i--){
					OCR1A = i;
					_delay_ms(t);
				}
			}
							
			lastservopos=175;
			rudlrot=0;
			rudrrot=0;
		}
						
	}
	
	if(ph_check){
		ph_buff = 1;
	}
}