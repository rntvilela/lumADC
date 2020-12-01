/*#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "nokia5110.h"

#define tam_vetor 4

unsigned char leitura_ADC_string[tam_vetor];
uint16_t leitura_ADC = 0;

void int_to_str(uint16_t s, unsigned char *d)
{
  uint8_t n = tam_vetor - 2;

  for (int8_t i = n; i >= 0; i--)
  {
    d[i]=s%10+48;
    s/=10;
  }  
}

ISR(ADC_vect)
{
  leitura_ADC = ADC;
}

int main()
{
  PORTD = (1<<PD2);
  DDRD = (1<<PD6);
  ADMUX = (3<<REFS0);
  ADCSRA = (7<<ADPS0)|(1<<ADIE)|(1<<ADATE)|(1<<ADSC)|(1<<ADEN);

  DIDR0 = 0b00111110;

  TCCR0A = (3<<WGM00)|(1<<COM0A1);
  TCCR0B = (4<<CS00);

  sei();

  nokia_lcd_init();

  while(1)
  { 
    OCR0A = (leitura_ADC>>2)-1;
    leitura_ADC = (leitura_ADC/1023.0)*100;
    int_to_str(leitura_ADC, leitura_ADC_string);
    nokia_lcd_clear();
    nokia_lcd_write_string("Luminosidade",1);
    nokia_lcd_set_cursor(0, 15);
    nokia_lcd_write_string((const char*)leitura_ADC_string,3);
    nokia_lcd_write_char('%',3);
    nokia_lcd_render();
  }
}*/

#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "nokia5110.h"

#define tam_vetor 4					
unsigned char leitura_ADC_string[tam_vetor];
uint16_t leitura_ADC = 0;
void int2string(unsigned int valor, unsigned char *disp);
ISR(ADC_vect)
{
	leitura_ADC = ADC;
	OCR0A = leitura_ADC/4-1;
	leitura_ADC = (leitura_ADC/1023.0)*100;
}

int main(void)
{
    
	DDRB = 0xFF; // saida
	DDRC = 0x00; // entrada
	PORTC = 0xFE; // so C0 em baixo nivel 
	DDRD = 0b01000000; // todos os pinos da porta D como entrada, somente o 6 como saida
	
	//configurando o ADC
	ADMUX = 0b11000000; // tensao interna de ref(1.1v)
	ADCSRA = 0b11101111;//habilita o AD, interrupção, conversao, prescaler 128
	ADCSRB = 0x00;// conversão continua 
	DIDR0 = 0b00111110;// habilita o PC0 como entrada do ADC0
	
	//modo PWM
	TCCR0A = 0b10100011; // habilita o Modo PWM rapido e modo nao invertido  nos pinos OC0A e OC0B
	TCCR0B = 0b00000101;	//liga TC0, prescaler = 1024, fpwm =  16MHZ/(256*1024) = 61Hz
	OCR0A = 0; // controle do ciclo ativo do PWM OC0A(PD6)Duty = 200/256 = 78%
	
	sei();
	nokia_lcd_init();
	
    while(1)
    {
	    nokia_lcd_clear();
	    int2string(leitura_ADC, leitura_ADC_string);
	    nokia_lcd_write_string(leitura_ADC_string, 3);
	    nokia_lcd_write_char('%', 3);
	    nokia_lcd_render();
	    _delay_ms(1000);
    }
    
}

void int2string(unsigned int valor, unsigned char *disp)
{
	for (uint8_t n = 0; n<tam_vetor-1; n++)
	disp[n] = 0 +48;
	
	disp += tam_vetor -2;
	do 
	{
		*disp = (valor%10)+48;
		valor/=10;
		disp--;
	} while (valor!=0);
}