
#define NUM_BANKS 3

const int bank_pins[NUM_BANKS] = {A1, A2, A3};

#include "bank0_keyboard.h"
#include "bank1_keyboard.h"
#include "bank2_keyboard.h"

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

void setup(void)
{
	initToneGenerator();
	setToneFrequency(1000);
	busyDelay(100000L);
}


void busyDelay(long cycles) {
	for (volatile long i = 0L; i < cycles; i++)
		i++;
}


/**
 * Given an ADC value and a number of switches and ADC value lookup table,
 * return a bit mask stating which buttons are pressed.
 */
int
get_bank_key_bits(int adc_value, int num_switches, const int *adc_values)
{
	int closest_delta;
	int closest_bits = -1;
	
	for (int i = 0; i < 1<<num_switches; i++) {
		int delta = abs(adc_values[i] - adc_value);
		if (closest_bits == -1 || closest_delta > delta) {
			closest_delta = delta;
			closest_bits = i;
		}
	}
	
	return closest_bits;
}


void loop(void)
{
	// read the value from the sensor:
	int bank0_value = analogRead(bank_pins[0]);
	int bank0_key_bits = get_bank_key_bits(bank0_value, BANK0_NUM_SWITCHES, BANK0_ADC_VALUES);
	if (bank0_key_bits == 0)
		setToneFrequency(0);
	else
		setToneFrequency(tones[bank0_key_bits]);
}


/******************************************************************************
 * Tone generator implementation (as the Arduino provided "tone/noTone" do
 * nothing on ATTiny85s. Because I was too lazy to read enough of the manual to
 * do it properly, an ISR is used to toggle the piezo rather than a PWM
 * generator...
 ******************************************************************************/

#include <avr/io.h>
#include <avr/interrupt.h>


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
	setToneFrequency(0);
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
		compare = F_CPU/(freq*2)/clock_div;
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
