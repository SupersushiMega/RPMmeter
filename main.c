/*
 ATmega328p 
 * Timer 1
 * UART
 * */
 
 //Header
 //==============================================
 //Name: Umdrehungszaehler
 //uP: ATmega328
 //Made by Sascha Rutz
 //==============================================
 
#define F_CPU 8000000UL// set the CPU clock
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <avr/io.h>
#include <stdio.h>  

#define USART_BAUDRATE 9600
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 8UL)))-1)

#define SpeedMinusPin (PINC & (1<<PC0))	//define Speed- Pin
#define SpeedPlusPin (PINC & (1<<PC1))	//define Speed+ Pin

#define SpeedMinusMask (1<<PC0)	//define Speed- Mask
#define SpeedPlusMask (1<<PC1)	//define Speed+ Mask

#define FullRotationMask = (1<<PD2)

#define PWM_ON (PORTC |= (1<<PC3))
#define PWM_OFF (PORTC &= ~(1<<PC3))

#define PWM_MIN 71

//TIMER
ISR (TIMER1_COMPA_vect);


volatile uint8_t ISR_counter = 0;
volatile uint16_t ms = 0;
volatile uint16_t msUntilInput = 0;
ISR (TIMER0_OVF_vect)
{
	ISR_counter++;
	if(ISR_counter >= 32)
	{
		ms++;
		if (msUntilInput)
		{
			msUntilInput--;
		}
		ISR_counter = 0;
	}
}

//UART
void uart_send_numb (uint16_t number);
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
	
	//Konfiguration Timer Overflow
	//==================================================================
	TCCR0A	= 0x00;
	TCCR0B	= 0x01;
	TIMSK0	|= (1 << TOIE0);
	TIFR0 |= (1 << TOV0);
	//==================================================================
	
    sei();
    // enable interrupts
    
    DDRD &= ~(1<<PD2);	//Set PD2 as Input
    
    DDRC &= ~(1<<PC0) | ~(1<<PC1);	//Set PC0 and PC1 as Inputs
    
    DDRC |= (1<<PC3);	//Set PC3 as Output
    
    uint8_t cycleCounter = 0;
    uint8_t PWMcounter = 0;
    uint8_t PWMedge = PWM_MIN;
    
    uint8_t Input = 0;
    uint8_t LastInput = 0;
    
    uint8_t FullRotationLast = 0;
    uint8_t FullRotation = 0;
    
    uint8_t RPScounter = 0;
    uint16_t RPM = 0;
    
	while(1)
	{ 
		Input = PINC;
		FullRotation = (PIND & (1<<PD2));
		cycleCounter++;
		if ((cycleCounter % 8) == 0)
		{
			PWMcounter++;
		}
		if(!msUntilInput)
		{
			if((Input & SpeedMinusMask) & ~(LastInput & SpeedMinusMask))
			{
				if(PWMedge > PWM_MIN)
				{
					PWMedge -= 10;
				}
				else
				{
					PWMedge = 0;
				}
				msUntilInput = 10;
			}
			else if((Input & SpeedPlusMask) & ~(LastInput & SpeedPlusMask))
			{
				if((PWMedge + 10) < 256)
				{
					if(PWMedge < PWM_MIN)
					{
						PWMedge = PWM_MIN;
					}
					else
					{
						PWMedge += 10;
					}
				}
				msUntilInput = 10;
			}
			LastInput = Input;
		}
		
		if(!FullRotation && FullRotationLast)
		{
			RPScounter++;
		}
		
		FullRotationLast = FullRotation;
		
		if(ms >= 1000)
		{
			RPM = RPScounter * 60;
			uart_send_numb(RPM);
			uart_send_char('RPM \n');
			RPScounter = 0;
			ms = 0;
		}
		
		if(PWMcounter <= PWMedge)
		{
			PWM_ON;
		}
		else
		{
			PWM_OFF;
		}
	} //end while
	return 0;
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
