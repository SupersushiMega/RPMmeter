/*
 ATmega328p 
 * Timer 1
 * UART
 * */
#define F_CPU 8000000UL// set the CPU clock
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <avr/io.h>
#include <stdio.h>  

#define USART_BAUDRATE 9600
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 8UL)))-1)

//TIMER
ISR (TIMER1_COMPA_vect);

//UART
void uart_send_char(char c);
void uart_send_string(volatile char *s);

/* 9600 baud / Geschwindikeit Uebertragung RS232 Schnittstelle*/
#define UART_BAUD_RATE      9600      
ISR(USART_RX_vect)
{
	char received_byte;
	received_byte = UDR0;
	UDR0 = received_byte;//Echo Byte

}//end of USART_rx 

int main(void)
{
	
	DDRD |= (1<<PD1)| (1<<PD0);//set TX0 and RX as output
	
    //Timer 1 Configuration
	OCR1A = 1249;	//OCR1A = 0x3D08;==1sec
	
    TCCR1B |= (1 << WGM12);
    // Mode 4, CTC on OCR1A

    TIMSK1 |= (1 << OCIE1A);
    //Set interrupt on compare match

    TCCR1B |= (1 << CS11) | (1 << CS10);
    // set prescaler to 64 and start the timer
    
    //UART0
    UCSR0B = (1<<RXEN0) | (1<<TXEN0) | (1<<RXCIE0);	//Turn on RX and TX circuits RXCIE0 enables Interrupt when byte received
	UCSR0C = (1<<UCSZ01) | (1<<UCSZ00);	//8-Bit Char size
	UBRR0H = (BAUD_PRESCALE >> 8);	//load upper 8-Bits of baud rate value into high byte of UBRR0H
	UBRR0L = BAUD_PRESCALE;			//load lower 8-Bits of Baud rate into low byte of UBRR0L

    sei();
    // enable interrupts
    
    
  
	while(1)
	{ 	
		//uart_send_char('A');
		//uart_send_char('\n');
		//uart_send_string("This is typo dummy text. You can see from it whether all the letters are there and what they look like. Sometimes words like hamburg fonts, rafgenduks or handgloves are used to test fonts. Sometimes sentences that contain all the letters of the alphabet - these sentences are called This one is very well known: The quick brown fox jumps over the lazy old dog. Often, foreign language parts of sentences are built into typo blind texts (AVAIL and Wefox are testing also kerning) in order to test the effect in other languages. For example, almost any font looks good in Latin. Quod erat demonstrandum. Since 1975 the numbers have been missing in most test texts, which is why according to TypoGb. 204 § from the year 2034 numbers in 86 of the texts become mandatory. Failure to comply will result in a penalty of up to 245 or 368. In the meantime, are just as important, but they are almost always included in newer scripts. OpenType functionalities are an important but difficult to integrate field. Depending on the software and preferences, built-in small caps, kerning or ligatures (very smart) may not be displayed correctly. You can see from it whether all the letters are there and what they look like. Sometimes you use words like Hamburgefonts, Rafgenduks\n\r");
		uart_send_numb(4000);
		_delay_ms(500);
	} //end while
}//end of main


ISR (TIMER1_COMPA_vect)
{
	
}

void uart_send_numb (uint16_t number)
{
	char buffer[20];
	sprintf(buffer, "%d", number);
	uart_send_string(buffer);
}


void uart_send_char(char c)
{
	while((UCSR0A & (1<<UDRE0)) == 0){};
    UDR0 = c;
}
void uart_send_string(volatile char *s)
{
	while(*s != 0x00)
	{
		uart_send_char(*s);
		s++;
	}
}//end of send_string
