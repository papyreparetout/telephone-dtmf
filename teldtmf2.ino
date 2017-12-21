/* Décodage pour cadran à impulsion type S63 et generation d'un signal DTMF
le décodage  des impulsions est du à Ph Martorell
la generation des signaux DTMF et l'assemblage est du à J ROGUIN
sur la base d'un generateur PWM de signaux code par Gary Hill 
*/
#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include "sinewavedata3.h"

const int PinLecture = 2 ; // Entrée lecture impulsions
const boolean Bavard = 0 ; // Génére les commentaires sur le port série
const boolean Debug = 1 ; // Génére les mesures de temps
//
boolean PlusDuneFois = 1 ; // pour éviter la répétition de commentaire
unsigned long TempsImpulsionMin = 45 ;
unsigned long TempsImpulsionMax = 95 ;
unsigned long TempsInterImpulsionMin = 25 ;
unsigned long TempsInterImpulsionMax = 60 ;
unsigned long TempsInterNumero = 200 ;
unsigned long rebonds = 2 ;
unsigned long duree = 0;
unsigned long temps = 0 ;
// int Numero = 0 ;

int speakerPin = 11; // Can be either 3 or 11, two PWM outputs connected to Timer 2
volatile uint16_t sample, sample_deb, Numero;
volatile uint16_t sinewave_length = 1070;

void stopPlayback()
{
    // Disable playback per-sample interrupt.
    TIMSK1 &= ~_BV(OCIE1A);

    // Disable the per-sample timer completely.
    TCCR1B &= ~_BV(CS10);

    // Disable the PWM timer.
    TCCR2B &= ~_BV(CS20);

    digitalWrite(speakerPin, LOW);
}

// This is called at 16000 Hz to load the next sample.
ISR(TIMER1_COMPA_vect) {
    if (sample >= (sample_deb + 1)*sinewave_length ) {
        stopPlayback();
         }
    else {
            OCR2A = pgm_read_byte(&sinewave_data[sample]);
          }
    ++sample;
}

void startPlayback()
{
    sample = sample_deb*sinewave_length;
    // Set up Timer 2 to do pulse width modulation on the speaker
    // pin.
    // Use internal clock (datasheet p.160)
    ASSR &= ~(_BV(EXCLK) | _BV(AS2));

    // Set fast PWM mode  (p.157)
    TCCR2A |= _BV(WGM21) | _BV(WGM20);
    TCCR2B &= ~_BV(WGM22);

        // Do non-inverting PWM on pin OC2A (p.155)
        // On the Arduino this is pin 11.
        TCCR2A = (TCCR2A | _BV(COM2A1)) & ~_BV(COM2A0);
        TCCR2A &= ~(_BV(COM2B1) | _BV(COM2B0));
        // No prescaler (p.158)
        TCCR2B = (TCCR2B & ~(_BV(CS12) | _BV(CS11))) | _BV(CS10);

        // Set initial pulse width to the first sample.
        OCR2A = pgm_read_byte(&sinewave_data[sample_deb*sinewave_length]);
    
    // Set up Timer 1 to send a sample every interrupt.

    cli();

    // Set CTC mode (Clear Timer on Compare Match) (p.133)
    // Have to set OCR1A *after*, otherwise it gets reset to 0!
    TCCR1B = (TCCR1B & ~_BV(WGM13)) | _BV(WGM12);
    TCCR1A = TCCR1A & ~(_BV(WGM11) | _BV(WGM10));

    // No prescaler (p.134)
    TCCR1B = (TCCR1B & ~(_BV(CS12) | _BV(CS11))) | _BV(CS10);

    // Set the compare register (OCR1A).
    // OCR1A is a 16-bit register, so we have to do this with
    // interrupts disabled to be safe.
    OCR1A = 1000;    // 16kHz d'échantillonnage

    // Enable interrupt when TCNT1 == OCR1A (p.136)
    TIMSK1 |= _BV(OCIE1A);

    sei();
}

void setup()
{
	//if (Debug || Bavard) // Si les deux à 0, on initialise pas
	//{
	Serial.begin(115200);
	Serial.println ("Initialisation Serie") ;
	//}
	pinMode(PinLecture , INPUT_PULLUP);
	pinMode(speakerPin, OUTPUT); 
	//digitalWrite(PinLecture , HIGH);
}

void loop()
{
Numero = 0 ;
delay ( rebonds );
while ( !digitalRead(PinLecture) )
{
if ( PlusDuneFois == 1 && Bavard == 1 )
	{
	Serial.println ("Attente") ;
	PlusDuneFois = 0 ;
	}
}
temps = millis() ;
if (Debug == 1 ) {debug();}
PlusDuneFois = 1 ;
delay ( rebonds );
// int Numero = 0 ;
numerotation () ;
delay ( rebonds );
}

void numerotation()
{
	while (1) //Boucle infinie, on ne sort qu'avec les "return"
	{
	if ( Bavard == 1 )	{	Serial.println ("Entree impulsion") ;	}
	
	while ( digitalRead(PinLecture) )
	{
		duree = millis() - temps ;
		if ( duree > TempsImpulsionMax )
		{
			if ( Bavard == 1 )	{ 	Serial.println ("Impulsion trop longue") ;	}
			debug();
			return ;
		}
	}
	if (millis() - temps < TempsImpulsionMin )
	{
		if ( Bavard == 1 ) 	{ Serial.println ("Impulsion trop courte") ; }
		debug();
		return ;
	}
	delay ( rebonds );
	if ( Bavard == 1 ) 	{	Serial.println ("Sortie impulsion") ;	}
	debug();
	Numero ++ ;
	if ( Bavard == 1 )
	{
	Serial.print ("Increment numero ; ") ;
	Serial.println (Numero) ;
	}
	temps = millis() ;
	delay ( rebonds );
	if ( Bavard == 1 ) 	{	Serial.println ("Entree inter-impulsion") ;	}
	debug();
	while( !digitalRead(PinLecture) )
	{
	duree = millis() - temps ;
	if ( duree > TempsInterImpulsionMax )
	{
	if ( Bavard == 1 ) 	{	Serial.println ("Inter-impulsions trop longue; action") ;	}
	debug();
	Action () ;
	return ;
	}
	}
	if ( Bavard == 1 ) 	{	Serial.println ("Sortie inter-impulsion") ;	}
	if ( duree < TempsInterImpulsionMin )
	{
	if ( Bavard == 1 )	{	Serial.println ("Inter-impulsion trop courte") ;	}
	debug();
	return ;
	}
	debug();
	temps = millis() ;
	delay ( rebonds );
	}
}

void debug ()
{
	if (Debug == 1 )
	{
	Serial.print ("Lecture ; ") ;
	Serial.print (digitalRead(PinLecture)) ;
	Serial.print (" duree = ") ;
	Serial.println (millis() - temps) ;
	}
	delay(2) ; // petite tempo par principe
}

void Action () // action a engager lorsque le numero est compose
{
	Serial.print ("Action avec numero ; ") ;
	Serial.println (Numero % 10) ;
	// Placez ici l'action à faire avec la variable "Numéro"
  Numero = Numero % 10;
	sample_deb = Numero;
	startPlayback();
//	Numero = 0 ;
}
