int sensorPin = A1;    // select the input pin for the potentiometer
int sensorValue = 0;  // variable to store the value coming from the sensor

double keymap[] = { 1.0/0.0 // inf
                  , 0.992
                  , 2.196
                  , 0.683322459222
                  , 3.87
                  , 0.789600987248
                  , 1.40100890208
                  , 0.58077545372
                  , 8.19
                  , 0.884826835112
                  , 1.73168110919
                  , 0.630700728701
                  , 2.62813432836
                  , 0.720169202924
                  , 1.19635619414
                  , 0.542318178258
                  };

int tones[] = {
880
,932
,987
,1046
,1108
,1174
,1244
,1318
,1396
,1479
,1567
,1661
,1760
,1864
,1975
,2093
};

//void setup() {
//	// declare the ledPin as an OUTPUT:
//	//Serial.begin(9600); 
//	
//	tone(tonePin, 1000);
//	delay(1000);
//	noTone(tonePin);
//}
//
//void loop() {
//	// read the value from the sensor:
//	sensorValue = analogRead(sensorPin);
//	double resistance = 0.998/((double)sensorValue / 1024.0) - 0.998;
//	double closest_delta;
//	int    closest = -1;
//	for (int i = 0; i< 1<<4; i++) {
//		double delta = abs(keymap[i] - resistance);
//		if (closest == -1 || closest_delta > delta) {
//			closest = i;
//			closest_delta = delta;
//		}
//	}
//	//Serial.println(closest);
//	if (closest == 0)
//		noTone(tonePin);
//	else
//		tone(tonePin, (tones[closest]));
//}
//

#include <avr/io.h>
#include <avr/interrupt.h>

void setup(void)
{
    //Serial.begin(9600);
    initToneGenerator();
    setToneFrequency(1000L);
    busyDelay(100000);
}

void busyDelay(long cycles) {
	for (volatile long i = 0L; i < cycles; i++)
		i++;
}

void loop(void)
{
	// read the value from the sensor:
	sensorValue = analogRead(sensorPin);
	double resistance = 0.998/((double)sensorValue / 1024.0) - 0.998;
	double closest_delta;
	int    closest = -1;
	for (int i = 0; i< 1<<4; i++) {
		double delta = abs(keymap[i] - resistance);
		if (closest == -1 || closest_delta > delta) {
			closest = i;
			closest_delta = delta;
		}
	}
	//Serial.println(closest);
	if (closest == 0)
		setToneFrequency(0L);
	else
		setToneFrequency((long)tones[closest]);
}


/******************************************************************************
 * Tone generator implementation (as the Arduino provided "tone/noTone" do
 * nothing on ATTiny85s. Because I was too lazy to read enough of the manual to
 * do it properly, an ISR is used to toggle the piezo rather than a PWM
 * generator...
 ******************************************************************************/

void initToneGenerator(void)
{
	// Set the appropriate pin mode
	pinMode(0, OUTPUT);
	digitalWrite(0, LOW);
	
	// Stop the timer
	TCCR1 = 0;
	// Zero the timer counter
	TCNT1 = 0;
	// Reset the prescaler
	GTCCR = _BV(PSR1);
	
	// Initially don't make a sound
	setToneFrequency(0L);
}


/**
 * Work out how many bits there are in a given number.
 */
int
blen(int num)
{
	int l = 0;
	while (num >= 1<<l)
		l++;
	return l;
}


void
setToneFrequency(int freq)
{
	int clock_div_code = 0;
	int compare = 255;
	
	// Calculate the clock divider/compare register values for the given frequency
	if (freq > 0 && freq <= 20000) {
		clock_div_code = blen(F_CPU/(freq*2)/256) + 1;
		int clock_div = 1 << (clock_div_code-1);
		compare = F_CPU/(freq*2L)/clock_div;
	}
	
	cli();
	// Cause the interrupt to happen at this time
	OCR1A = (char) compare;
	// ...and also the timer to get reset
	OCR1C = (char) compare;
	// Set the clock divider (keeping the "reset on compare" bit)
	TCCR1 = _BV(CTC1) | (char) clock_div_code;
	sei();
}


ISR(TIMER1_COMPA_vect)
{
	// Toggle pin 0 (the piezo)
	PINB |= _BV(PINB0);
}
