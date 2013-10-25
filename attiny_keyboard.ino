#define MIN(a,b) (((a) < (b)) ? (a) : (b))
#define MAX(a,b) (((a) > (b)) ? (a) : (b))

/******************************************************************************
 * ADC-based button Input
 ******************************************************************************/

// The number of button banks
#define NUM_BANKS 3

// ADC Values for each button bank
#include "bank0_keyboard.h"
#include "bank1_keyboard.h"
#include "bank2_keyboard.h"

// The pin for each button bank
const int bank_pins[NUM_BANKS] = {A1, A2, A3};
const int bank_num_switches[NUM_BANKS] = { BANK0_NUM_SWITCHES
                                         , BANK1_NUM_SWITCHES
                                         , BANK2_NUM_SWITCHES
                                         };
const int *bank_adc_values[NUM_BANKS] = { BANK0_ADC_VALUES
                                        , BANK1_ADC_VALUES
                                        , BANK2_ADC_VALUES
                                        };


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


/**
 * Read the keys for each bank and return them as one bitfield in big-endian
 * order.
 */
int
get_pressed_keys(void)
{
	
	int pressed_keys = 0;
	
	for (int i = 0; i < NUM_BANKS; i++) {
		int bank_value = analogRead(bank_pins[i]);
		int bank_key_bits = get_bank_key_bits( bank_value
		                                     , bank_num_switches[i]
		                                     , bank_adc_values[i]
		                                     );
		pressed_keys <<= bank_num_switches[i];
		pressed_keys |= bank_key_bits;
	}
	
	return pressed_keys;
}

/******************************************************************************
 * Piano Keyboard Constants
 ******************************************************************************/

#include "piano_key_frequencies.h"

/******************************************************************************
 * Tunes
 ******************************************************************************/

#define K_C  (1<<11)
#define K_CS (1<<10)
#define K_D  (1<< 9)
#define K_DS (1<< 8)
#define K_E  (1<< 7)
#define K_F  (1<< 6)
#define K_FS (1<< 5)
#define K_G  (1<< 4)
#define K_GS (1<< 3)
#define K_A  (1<< 2)
#define K_AS (1<< 1)
#define K_B  (1<< 0)

// Set which keys are being pressed.
#define SET_KEYS(keys)     ((keys)<<2)  | 0x0

// Delay for a given amount
#define DELAY(delay)       ((delay)<<2) | 0x1

// Set the current offset +- Middle C
#define SET_OFFSET(offset) ((offset)<<2) | 0x2

const int grieg_morning_mood[] = {
	SET_OFFSET(0),
	SET_KEYS(K_G),  DELAY(10),
	SET_KEYS(K_E),  DELAY(10),
	SET_KEYS(K_D),  DELAY(10),
	SET_KEYS(K_C),  DELAY(10),
	SET_KEYS(K_D),  DELAY(5),
	SET_KEYS(K_E),  DELAY(5),
	SET_KEYS(K_D),  DELAY(5),
	SET_KEYS(K_E),  DELAY(5),
	
	SET_KEYS(K_G),  DELAY(10),
	SET_KEYS(K_E),  DELAY(10),
	SET_KEYS(K_D),  DELAY(10),
	SET_KEYS(K_C),  DELAY(10),
	SET_KEYS(K_D),  DELAY(5),
	SET_KEYS(K_E),  DELAY(5),
	SET_KEYS(K_D),  DELAY(5),
	SET_KEYS(K_E),  DELAY(5),
	
	SET_KEYS(K_G),  DELAY(10),
	SET_KEYS(K_E),  DELAY(10),
	SET_KEYS(K_G),  DELAY(10),
	SET_KEYS(K_A),  DELAY(10),
	SET_KEYS(K_E),  DELAY(10),
	SET_KEYS(K_A),  DELAY(10),
	SET_KEYS(K_B),  DELAY(10),
	SET_KEYS(K_GS), DELAY(10),
	SET_KEYS(K_FS), DELAY(10),
	SET_KEYS(K_E),  DELAY(20),
	
	SET_KEYS(0), DELAY(100),
};
const int grieg_morning_mood_length = sizeof(grieg_morning_mood)/sizeof(int);



/******************************************************************************
 * Main Program
 ******************************************************************************/

// The pin the mode key is attached to
const int mode_key_pin = 1;

// The current offset into the keys
int cur_offset = PIANO_MIDDLE_C_KEY;


// What order are keys played when in a chord
enum arpeggiator_mode {
	AM_UP,
	AM_DOWN,
	AM_UPDOWN,
} arpeggiator_mode = AM_UP;


void
busyDelay(long cycles) {
	for (volatile long i = 0L; i < cycles; i++)
		i++;
}


void
setup(void)
{
	// Set up the tone generator
	initToneGenerator();
	
	// Set the mode key input mode with pullup
	pinMode(mode_key_pin, INPUT);
	digitalWrite(mode_key_pin, HIGH);
}

#define ADVANCE_BIT() do { \
	switch (arpeggiator_mode) { \
		default: \
		case AM_DOWN:   cur_direction = +1; break; \
		case AM_UP:     cur_direction = -1; break; \
		case AM_UPDOWN: \
			if (cur_direction == 1 && cur_bit == 0xF) \
				cur_direction = -1; \
			else if (cur_direction == -1 && cur_bit == 0x0) \
				cur_direction = +1; \
			break; \
	} \
	cur_bit += cur_direction; \
} while (false)

/**
 * Drive the keyboard as if an arpeggiator.
 */
void
arpeggiator(int pressed_keys)
{
	static const int period = 30;
	static int ticks = 0;
	static int cur_bit = 0;
	static int cur_direction = 1;
	
	// Move through the pressed keys
	ticks += 1;
	if (ticks >= period) {
		ADVANCE_BIT();
		cur_bit &= (sizeof(int)*8)-1;
		ticks = 0;
	}
	
	// Play it!
	if (pressed_keys) {
		// Must loop round enough times that when going up/down we actually get back
		// to the bit
		for (int i = 0; i < sizeof(int)*8*2; i++) {
			if (pressed_keys & (1<<cur_bit)) {
				break;
			} else {
				ADVANCE_BIT();
				cur_bit &= (sizeof(int)*8)-1;
			}
		}
		
		setToneFrequency(PIANO_KEYBOARD_FREQUENCIES[cur_offset + (12 - cur_bit) - 1]);
	} else {
		setToneFrequency(0);
	}
}


void
loop(void)
{
	// Get input
	int pressed_keys = get_pressed_keys() ;
	int mode_pressed = digitalRead(mode_key_pin) == 0;
	
	// The previous state of mode_pressed/pressed_keys. Used to insert a
	// debounce-wait when deasserted.
	static int last_pressed_keys = 0;
	
	// Play a tune?
	static int const *tune = NULL;
	static int        tune_length;
	
	// Demo the arpeggiator?
	static bool da = false;
	// Demo the current offset?
	static bool ds = false;
	
	if (mode_pressed) {
		// Mode button pressed
		
		// Handle a button release when mode button is held
		if (last_pressed_keys != 0 && pressed_keys == 0) {
			
			switch (last_pressed_keys) {
				// Change the octave when the white keys are pressed
				case K_B: cur_offset = PIANO_MIDDLE_C_KEY + 12*3; da=false; ds=true; break;
				case K_A: cur_offset = PIANO_MIDDLE_C_KEY + 12*2; da=false; ds=true; break;
				case K_G: cur_offset = PIANO_MIDDLE_C_KEY + 12*1; da=false; ds=true; break;
				case K_F: cur_offset = PIANO_MIDDLE_C_KEY;        da=false; ds=true; break;
				case K_E: cur_offset = PIANO_MIDDLE_C_KEY - 12*1; da=false; ds=true; break;
				case K_D: cur_offset = PIANO_MIDDLE_C_KEY - 12*2; da=false; ds=true; break;
				case K_C: cur_offset = PIANO_MIDDLE_C_KEY - 12*3; da=false; ds=true; break;
				
				// Move the offset down/up using the first two black keys respectively
				case K_CS: cur_offset = MAX(cur_offset-1, 0);              da=false; ds=true; break;
				case K_DS: cur_offset = MIN(cur_offset+1, PIANO_NUM_KEYS); da=false; ds=true; break;
				
				// Change the arpeggiator direction using the last three black keys
				case K_AS: arpeggiator_mode = AM_UP;     da=true; ds=false; break;
				case K_GS: arpeggiator_mode = AM_UPDOWN; da=true; ds=false; break;
				case K_FS: arpeggiator_mode = AM_DOWN;   da=true; ds=false; break;
				
				// Play a tune
				case K_CS | K_DS: tune = grieg_morning_mood; tune_length = grieg_morning_mood_length; da=false; ds=false; break;
				
				// Do nothing if not recognised
				default: da=false; ds=false; break;
			}
		}
		
		// Do the demos
		     if (da) arpeggiator(0xFFF);
		else if (ds) setToneFrequency(PIANO_KEYBOARD_FREQUENCIES[cur_offset]);
		else         play_tune(tune, tune_length, true);
		
	} else {
		// Play that tune (until a key is pressed)!
		if (tune != NULL && pressed_keys == 0) {
			play_tune(tune, tune_length, false);
		} else {
			tune = NULL;
			arpeggiator(pressed_keys);
		}
		
		// Clear the flags used by the mode demos
		da = false;
		ds = false;
	}
	
	// Record the key states accumulatively (for debouncing)
	if (pressed_keys != 0)
		last_pressed_keys |= pressed_keys;
	else
		last_pressed_keys = 0;
}

/******************************************************************************
 * Tune Playback
 ******************************************************************************/

#define TUNE_DELAY_MULTIPLIER 20

/**
 * Handle playback of a tune.
 *
 * tune is an array of ints where the first two bits indicate if a key is to be
 * pressed, released or a delay is to be inserted.
 *
 * tune_length is the length of the array.
 *
 * reset should be false while playing (in a loop)
 */
void
play_tune(const int *tune, int tune_length, bool reset)
{
	static int position = 0;
	static int delay_remaining = 0;
	static int cur_keys = 0;
	
	if (reset) {
		position = 0;
		delay_remaining = 0;
		cur_keys = 0;
	}
	
	arpeggiator(cur_keys);
	
	// A delay is being executed
	if (delay_remaining) {
		delay_remaining -= 1;
	} else {
		// Read instructions until a delay is inserted or the track loops (to avoid
		// infinate loops on traks with no delays).
		while (!delay_remaining) {
			// Continue reading instructions!
			int instruction = tune[position];
			switch (instruction & 0x3) {
				// Set keys
				case 0x0:
					cur_keys = ((unsigned int)instruction)>>2u;
					break;
				
				// Delay
				case 0x1:
					delay_remaining = (((unsigned int)instruction)>>2u) * TUNE_DELAY_MULTIPLIER;
					break;
				
				// Set offset
				case 0x2:
					cur_offset = PIANO_MIDDLE_C_KEY + (instruction>>2u);
					break;
				
				// Unknown command: NOP.
				default: break;
			}
			
			if (++position >= tune_length) {
				position = 0;
				break;
			}
		}
	}
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
	// Reset the Counters
	OCR1A = 255;
	OCR1B = 255;
	OCR1C = 255;
	// Disable the interrupt
	TIMSK = _BV(OCIE1A);
	
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
