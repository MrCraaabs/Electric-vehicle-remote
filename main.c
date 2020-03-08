#include "main.h"				//Einbindung der Bibliotheken und Unterprogramme
#include "uart.h"

#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <avr/wdt.h>
#include <string.h>

#include "display.h"
#include "ringbufferAveraging.h"


#define CTC_MATCH_OVERFLOW ((F_CPU / 1000)/8)	//Timer wird auf 1ms Takt eingestellt

#define SSD1306_ADDR  0x3C			//I2C Adresse des Displays

int motorSpeed = 0;				//Gewschindigkeit der Motoren wird auf 0 deklariert
volatile unsigned long timer1_millis;		//Der Timer kann nur positive Werte annehmen und außerdem zu jeder Zeit
						//auch ohne expliziten Zugriff im Quelltext geändert werden

u8g_t u8g;					//Display wird initialisiert
ringbufferAveraging_t HallSensorApproximation;	//Der Ringbuffer für den Magnetfeldsensor wird initialisiert
ringbufferAveraging_t Stick;			//Der Ringbuffer für die Potis des Joysticks wird initialisiert

void ADC_Init()
{
	 ADMUX = (1<<REFS0);			//Auswahl der internen Referenzspannung		
	 ADCSRA = (1<<ADPS1) | (1<<ADPS0);   	//Der Takt des AD-Wandlers wird gewählt (ein achtel des Prozessortaktes)
	 ADCSRA |= (1<<ADEN);			//Der AD-Wandler wird aktiviert
	 ADCSRA |= (1<<ADSC);                  	//Eine ADC-Wandlung wird durchgeführt (zum Einschwingen)
	 while (ADCSRA & (1<<ADSC) ) {         	//Es wird auf Abschluss der Konvertierung warten
	 }
	 (void) ADCW;				//Der ausgelesene Wert wird in ADCW geschrieben
						//Muss einmal aufgerufen werden, da sonst die nächsten Ergebnisse des ADC
						//nicht gespeichert werden können
}

/* ADC Einzelmessung */
uint16_t ADC_Read( uint8_t channel )
{
							// Kanal waehlen, ohne andere Bits zu beeinflußen
	ADMUX = (ADMUX & ~(0x1F)) | (channel & 0x1F); 	//Die Mux Channel werden auf 0 gesetzt anschließend auf den ensprechenden
							//Channel gesetzt (Multiplexer)
	ADCSRA |= (1<<ADSC);            		//Eine Wandlung der ausgewählten Channel
	while (ADCSRA & (1<<ADSC) ) {   //Es wird gewartet bis lesen Bit wieder auf 0 (erfolgreiches Lesen) ist (siehe Datenblatt)
	}
	return ADCW;                    // Die umgewandelten Werte werden in das Register ADCW geschrieben
}

unsigned long millis ()			
{
	unsigned long millis_return;	//Lokale Variable 

	ATOMIC_BLOCK(ATOMIC_FORCEON) {		//Es wird sichergestellt, dass beim beim auslesen des wertes,
		millis_return = timer1_millis;	//kein Interrrupt stattfinden kann, damit alle Bits überschrieben werden können
	}
	
	return millis_return;		//Der Wert wird wieder zurückgegeben
}

ISR (TIMER1_COMPA_vect)			//Sobald ein Interrupt bei Timer1 auftritt wird timer1_millis um 1 erhöht
{
	timer1_millis++;
}

void sendData(){
	
	static unsigned long lasttime=0;	
	if(lasttime+49>millis()){	//Es werden nur alle 50ms Daten an das Bluetoothmodul gesendet
	return ;
	}
	addValue(&Stick,percentage(ADC_Read(ADC1D))); 	//Der Ringbuffer wird gefüllt mit einem neuen Wert
	
	motorSpeed = getRingbufferAverage(&Stick);	//Der Durchschnitt des Ringbuffers wird in motorspeed geschrieben
	char text[20] = "0\t";		//Arry wird deklariert und mit dem text "0\t" aufgefüllt 
					//Diese Nachricht hat eine bestimmte Reihenfolge und beinhaltet am Ende Informationen
					//über Fahrtrichtung, Geschwindigkeit und Lenkung
	char n[4];			//Platzhalter für Motorgeschwindigkeit
	
	integerToChar(n,motorSpeed);	//Aus den Zahlenwerten werden Buchstaben gemacht (auch 0,1,2 ist ein Buchstabe)
					//Nachricht bleibt vom Format her immer gleich
	strcat(text,n);			//Die Motorwerte werden zur Nachricht addiert
	strcat(text,"\t1\n");		//Lenkung wird nicht betätigt (Testzwecke)
		
	uart_puts(text);		//Text wird in den Buffer von Uart gesetzt
	lasttime = millis();		//Die Aktuelle Zeit wird gespeichert 
}
void setup()
{
   uart_init(UART_BAUD_SELECT_DOUBLE_SPEED(UART_BAUD_RATE,F_CPU)); //Uart wird initialisiert mit den entsprechenden Werten

	// Timer    	
	// CTC mode, Clock/8
	 TCCR1B |= (1 << WGM12) | (1 << CS11);	//Timer wird auf CTC Mode gesetzt, Clockspeed auf ein Achtel der Taktrate
	  // Load the high byte, then the low byte
	  // into the output compare
	  OCR1AH = (CTC_MATCH_OVERFLOW >> 8)& 0xFF; //Der Match Overflow wird ins obere und untere Register geschoben
	  OCR1AL = CTC_MATCH_OVERFLOW& 0xFF;		
	  
	  // Enable the compare match interrupt
	  TIMSK1 |= (1 << OCIE1A);	//Interrupt wird eingeschaltet
	  sei();			//Globaler Interrupt wird aktiviert
	  //Timer ende
	  
	  DDRD |= (0<< PIND5);		//Pin D5 als Input	
	  //LED-Test-pin
	  DDRC |= (1 << PINC0);		//Pin C0 als Output
	  
	  //CLKPR = 0x80; prozessorgeschwindigkeit			
	  //CLKPR = 0x00;

	  u8g_InitI2C(&u8g, &u8g_dev_ssd1306_128x32_i2c, U8G_I2C_OPT_NONE); 	//display wird initialisiert
	  u8g_SetFont(&u8g,u8g_font_6x10);					//Schriftart wird ausgewählt
		  
	  InitRingbufferAveraging(&HallSensorApproximation);			//Ringbuffer werden initialisiert
	  InitRingbufferAveraging(&Stick);
}

void draw(void)
{
	static unsigned long lasttime=0;	//es wird zu beginn einmal der timer auf 0 gesetzt	
	static unsigned int count = 0;		//es wird zu beginn einmal der counter auf 0 gesetzt
	
	if(lasttime+50>millis()){		//Es wird wieder alle 50 ms auf das Display geschrieben	
		return ;
	}
	char a[20];				//Nachricht für den Bildschirm wird vorbereitet
	char m[4];
	int magnet = getRingbufferAverage(&HallSensorApproximation);
	integerToChar(m,magnet);
	sprintf(a,"%d",motorSpeed);		//Motorspeed wird in a als text geschrieben
	if(count==0)				//Es wird immer nur eine Bildschirmhälfte neu beschrieben
	{
		u8g_FirstPage(&u8g);
		renderDisplay(&u8g,a,m);
		u8g_NextPage(&u8g);
		count = 1;
	}
	else
	{
		if (count>2)
		{			
			count=0;
		}
		else{
			count++;
		}
		renderDisplay(&u8g,a,m);
		u8g_NextPage(&u8g);
		
	}
	lasttime = millis();
	
}

void sample()					//Es wird alle 20ms ein Wert vom Hallsensor gespeichert
{	
	static unsigned long lasttime=0;
	
	if(lasttime+20>millis()){
		return ;
	}
	addValue(&HallSensorApproximation,ADC_Read(ADC3D));
	lasttime = millis();
}


int main(void)
{
	wdt_enable(WDTO_8S);			//Watchdog wird eingeschaltet
	wdt_reset();				//auf 0 gesetzt
	wdt_disable();				//Ausgeschaltet
	ADC_Init();				//Funktion wird aufgerufen
	enum statemachine state = empty;   	//Anfangszustand der Statemachine
    while (1) 					//Die verschiedenen States werden abgearbeitet
    {
		switch (state)			
		{
		case empty:			
			state = boot;
			break;
		case boot:			//Der Controller wird "hochgefahren" und geht anschließend in den "send" state
			setup();
			state = send;
			break;
		case receive:			//Platzhalter da in Zukunft auch Daten vom Bollerwagen empfangen werden sollen
			state = send		//Nach dem Emfpangen wird in den "send" state gewechselt
			wdt_enable(WDTO_1S);
			//Empfangs funktion 
			wdt_disable();
			wdt_reset();
			break;
		case send:			//Daten werden an den Bollerwagen gesendet
			state = display;	//der State wird auf Display gewechselt
			wdt_enable(WDTO_1S);	//Watchdog Timer wird auf 1 Sekunde gesetzt, wird länger als 1 Sekunde
						//nicht mit der Watchdog Funktion interagiert, wird der Mikrocontroller resetet
			sendData();		//Datensende Funktion wird aufgerufen
			wdt_disable();
			wdt_reset();
			break;
		case display:			//das Display wird beschrieben
			state = receive;	//der state wird auf "receive" geändert, und die 3 States laufen vortan im loop
			wdt_enable(WDTO_1S);
			draw();
			wdt_disable();
			wdt_reset();
			
			break;
		}
		sample();			//ADC sampling funktion	um neue Daten aus dem Magnetfeldsensor zu lesen
		
		
    }
}




void integerToChar(char* chararray, int number)		//Zahlenwerte werden in ASCII umgewandelt und an die entsprechende
{							//stelle im Text gesetzt
	chararray[0] = (number/100)%10+48;
	chararray[1] = (number/10)%10+48;
	chararray[2] = (number/1)%10+48;
	chararray[3] = '\0';
}

int percentage(long x)					//Funktion um % wert zu ermitteln
{
	return (100*x)/1023;
} 
