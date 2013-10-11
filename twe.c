/*****************************************************
This program was produced by the
CodeWizardAVR V1.25.9 Standard
Automatic Program Generator
© Copyright 1998-2008 Pavel Haiduc, HP InfoTech s.r.l.
http://www.hpinfotech.com

Project : 
Version : 
Date    : 14.01.2013
Author  : F4CG                            
Company : F4CG                            
Comments: 


Chip type           : ATmega8L
Program type        : Application
Clock frequency     : 8,000000 MHz
Memory model        : Small
External SRAM size  : 0
Data Stack size     : 256
*****************************************************/

#include <mega8.h>    
#include <m8_128.h>               

#define MINCOUNT 1
#define MAXCOUNT 65530
                        
//начальное значение количетвациклов отсчета
unsigned int count;

//временная переменная для отсчетов
unsigned int capasiter;                     

//Переменная указывающая текущий зуб
unsigned char currentProtrusion;   
//Переменная указывающая выступ либо впадина
unsigned char protrusion;
//Переменная для предотвращения удердния зажатой клавиши
unsigned char scipCurrenIteration;

// External Interrupt 0 service routine
interrupt [EXT_INT0] void ext_int0_isr(void)
{
        GICR&=~(1<<7);   //запрещем прерывание по INT0
        GICR&=~(1<<6);   //запрещем прерывание по INT1   
        if (scipCurrenIteration==0) {
                // Увеличение количества отсчетов для генерации нового зуба
                if (count<MAXCOUNT) {
                        count++;
                }
                scipCurrenIteration=1;
        }                                              
}

// External Interrupt 1 service routine
interrupt [EXT_INT1] void ext_int1_isr(void)
{
        GICR&=~(1<<6);   //запрещем прерывание по INT1   
        GICR&=~(1<<7);   //запрещем прерывание по INT0    
        
        if (scipCurrenIteration==0) {
        
                // Уменьшение количества отсчетов для генерации нового зуба
                if (count>MINCOUNT) {
                        count--;
                }         
                scipCurrenIteration=1;
        }                               
}

// Standard Input/Output functions
#include <stdio.h>

// Timer 0 overflow interrupt service routine
interrupt [TIM0_OVF] void timer0_ovf_isr(void)
{              
        //#asm("cli")         
        //#asm("sei")     
        GICR&=~(1<<7);   //запрещем прерывание по INT0    
        GICR&=~(1<<6);   //запрещем прерывание по INT1    
        
        if (capasiter==0){          
        
                if (protrusion==0) {
                        protrusion=1;
                }
                else  {    
                        protrusion=0;
                };
        
                if (currentProtrusion<=3 && currentProtrusion>=0) {
                        PORTB.4=0;
                        PORTB.5=0;
                }                          
                else {
                        PORTB.4=protrusion;
                        PORTB.5=protrusion;        
                }                           
                currentProtrusion++;
                if (currentProtrusion>=120)
                        currentProtrusion=0;
                capasiter=count;   
                scipCurrenIteration=0;
        }                       
        else {
                capasiter--;
        }
                    
        GICR|=(1<<7);   //разрешаем прерывание по INT0
        GICR|=(1<<6);   //разрешаем прерывание по INT1
}

// Declare your global variables here

void main(void)
{
// Declare your local variables here

// Input/Output Ports initialization
// Port B initialization
// Func7=In Func6=In Func5=Out Func4=Out Func3=In Func2=In Func1=In Func0=In 
// State7=T State6=T State5=0 State4=0 State3=T State2=T State1=T State0=T 
PORTB=0x00;
DDRB=0x30;

// Port C initialization
// Func6=In Func5=In Func4=In Func3=In Func2=In Func1=In Func0=In 
// State6=T State5=T State4=T State3=T State2=T State1=T State0=T 
PORTC=0x00;
DDRC=0x00;

// Port D initialization
// Func7=In Func6=In Func5=In Func4=In Func3=In Func2=In Func1=In Func0=In 
// State7=T State6=T State5=T State4=T State3=T State2=T State1=T State0=T 
PORTD=0x00;
DDRD=0x00;

// Timer/Counter 0 initialization
// Clock source: System Clock
// Clock value: 8000,000 kHz
TCCR0=0x01;
TCNT0=0x00;

// Timer/Counter 1 initialization
// Clock source: System Clock
// Clock value: Timer 1 Stopped
// Mode: Normal top=FFFFh
// OC1A output: Discon.
// OC1B output: Discon.
// Noise Canceler: Off
// Input Capture on Falling Edge
// Timer 1 Overflow Interrupt: Off
// Input Capture Interrupt: Off
// Compare A Match Interrupt: Off
// Compare B Match Interrupt: Off
TCCR1A=0x00;
TCCR1B=0x00;
TCNT1H=0x00;
TCNT1L=0x00;
ICR1H=0x00;
ICR1L=0x00;
OCR1AH=0x00;
OCR1AL=0x00;
OCR1BH=0x00;
OCR1BL=0x00;

// Timer/Counter 2 initialization
// Clock source: System Clock
// Clock value: Timer 2 Stopped
// Mode: Normal top=FFh
// OC2 output: Disconnected
ASSR=0x00;
TCCR2=0x00;
TCNT2=0x00;
OCR2=0x00;

// External Interrupt(s) initialization
// INT0: On
// INT0 Mode: Low level
// INT1: On
// INT1 Mode: Low level
GICR|=0xC0;
MCUCR=0x00;
GIFR=0xC0;

// Timer(s)/Counter(s) Interrupt(s) initialization
TIMSK=0x01;

// USART initialization
// Communication Parameters: 8 Data, 1 Stop, No Parity
// USART Receiver: Off
// USART Transmitter: On
// USART Mode: Asynchronous
// USART Baud Rate: 9600
UCSRA=0x00;
UCSRB=0x08;
UCSRC=0x86;
UBRRH=0x00;
UBRRL=0x33;

// Analog Comparator initialization
// Analog Comparator: Off
// Analog Comparator Input Capture by Timer/Counter 1: Off
ACSR=0x80;
SFIOR=0x00;

// Global enable interrupts
#asm("sei")
         
  
//первоначально считаем раз до переключения  
count=60;
//первоначальная установка количества редстоящих отсчетов
capasiter=count;           
//устанавливаем первоначально в 0 позицию
protrusion=0;         

scipCurrenIteration=0;
  
while (1)
      {
      //TODO Add USART send current speed.

      };
}
