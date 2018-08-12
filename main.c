/*
 * Title:    Line following and collision avoidance robot car
 * Hardware: ATmega8 @ 16 MHz, HC-SR04 Ultrasonic Ranging Module,
 *           L293D Motor Driver, SG90 Servo, TCRT5000 Reflective Optical Sensor
 * Created:  26-7-2018 18:51:54
 * Author :  Tim Dorssers
 *
 * In line follow mode the robot car uses five TCRT5000 sensors connected to
 * ADC0-ADC4 inputs to detect a dark line on a light surface. ADC conversion is
 * interrupt driven. Weighted average calculation is used to determine the
 * "exact" location of the line. The sensors can be calibrated via UART for
 * better accuracy. Measured values of the light and dark surface are stored in
 * EEPROM. The motors are driven by a PID control. The K values are user
 * configurable via UART as well.
 * In collision avoidance mode the robot car uses the HC-SR04 module and the
 * SG90 micro servo to "look around". The servo is hardware PWM driven via OC1A
 * output using 16-bit Timer1. The HC-SR04 module is connected to INT0 and uses
 * 8-bit Timer0 to measure RTT using two ISRs. Because of the limited "viewing"
 * angle of the sonar, the servo sweeps between 54 and 126 degrees. When an
 * object is detected within the turning distance, the robot performs a sweep
 * from 0 to 180 degrees and chooses the angle with the maximum average clear
 * distance to turn to. When an object is detected ahead within the collision
 * distance, the robot reverses until no object is within collision distance
 * anymore. The turning and collision distance are user configurable from the
 * UART. 
 * The DC motors are driven by software PWM implementation using PD4-PD7
 * outputs connected to the L293D motor driver and 8-bit Timer2 interrupt.
 * A mode button, connected to INT1, is used to switch between the different
 * modes; line follow, collision avoid and idle. In idle mode, the robot can be
 * driven "remotely" from the UART as well.
 */ 

#ifndef F_CPU
#define F_CPU 16000000UL  // 16 MHz
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sfr_defs.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <avr/version.h>
#include <util/delay.h>
#include <stdlib.h>
#include <string.h>
#include "uart.h"

// Globals
#define FALSE 0
#define TRUE  1
char buffer[10];

// Ultrasonic ranging sensor
#define MAX_DISTANCE  300      // sets maximum usable sensor measuring distance
#define CM_ECHO_TIME  (F_CPU / 17013)    // Speed of sound in cm divided by two
#define MAX_ECHO_TIME (CM_ECHO_TIME * MAX_DISTANCE) // Maximum sensor distance
uint32_t countTimer0;
volatile uint8_t echoDone;

// Collision avoid
uint8_t EEMEM nvCollDist = 20; // sets distance at which robot stops and reverses
uint8_t EEMEM nvTurnDist = 40; // sets distance at which robot veers away from object
uint8_t collDist, turnDist;
#define DRIVING        0
#define SWEEP_PENDING  1
#define SWEEP_PROGRESS 2
#define SWEEP_FINISHED 3
#define TURNING        4
#define REVERSING      5

// IR sensor
#define ADC_CHANNEL_COUNT 5 // Number of ADC channels we use
volatile uint8_t adc_values[ADC_CHANNEL_COUNT];
volatile uint8_t adc_read;
uint8_t EEMEM nv_adc_min[ADC_CHANNEL_COUNT] = {26, 26, 26, 28, 17};
uint8_t EEMEM nv_adc_max[ADC_CHANNEL_COUNT] = {141, 136, 142, 160, 101};
uint8_t adc_min[ADC_CHANNEL_COUNT], adc_max[ADC_CHANNEL_COUNT];

// PID control K values multiplied by 100
uint16_t EEMEM nvKp = 500, nvKd = 300, nvKi = 0;
uint16_t Kp, Kd, Ki;

// Servo PWM, assuming prescaler of 8
#define TIMER1_TOP   ((F_CPU / 50 / 8) - 1) // 50 Hz (20 ms) PWM period
#define SERVO_0DEG   ((F_CPU / (1000 / 0.75) / 8) - 1)
#define SERVO_90DEG  ((F_CPU / (1000 / 1.65) / 8) - 1) // 1.65 ms duty cycle
#define SERVO_180DEG ((F_CPU / (1000 / 2.55) / 8) - 1)
#define STEP_18DEG   ((SERVO_180DEG - SERVO_0DEG) / 10)

// Soft PWM, no prescaler, 8-bit
#define TIMER2_TOP 64  // 16MHz / 256 / 64 ~ 1 KHz PWM period
int8_t speedMotor1, speedMotor2;

// Robot running mode
#define IDLE            0
#define LINE_FOLLOW     1
#define COLLISION_AVOID 2
uint8_t robot_mode = IDLE;

// Function prototypes
void adc_once(void);
void collision_avoid(void);
void constrain_speeds(void);
void dump_adc_values(void);
void dump_array_p(const char *progmem_s, uint8_t *p, size_t t);
void dump_newline(void);
void dump_nv_values(void);
void dump_separator(void);
void dump_status(void);
void dump_value_p(const char *progmem_s, int16_t val);
void handle_uart(void);
inline void init_adc(void);
inline void init_eeprom(void);
inline void init_etc(void);
inline void init_motor(void);
inline void init_servo(void);
inline void init_sonar(void);
int16_t input_val_p(const char *progmem_s, int16_t val);
void line_follow(void);
uint16_t ping_cm(void);
void toggle_mode(void);

//Function macros
#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define adc_stop() ADCSRA &= ~(_BV(ADEN) | _BV(ADSC)) // Disable ADC, stop conversion
#define adc_start() ADCSRA |= (_BV(ADEN) | _BV(ADSC)) // Enable ADC, start conversion
#define dump_array_P(__s, __p, __t) dump_array_p(PSTR(__s), __p, __t)
#define dump_value_P(__s, __v) dump_value_p(PSTR(__s), __v)
#define input_val_P(__s, __v) input_val_p(PSTR(__s), __v)

// Initialize ADC
inline void init_adc(void) {
	// AVCC with external capacitor at AREF pin and ADC Left Adjust Result
	ADMUX = _BV(ADLAR) | _BV(REFS0);
	// free running select, interrupt flag, ADC prescaler of 128
	ADCSRA = _BV(ADFR) | _BV(ADIE) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);
}

// ADC interrupt
ISR(ADC_vect) {
	static uint8_t adc_selector, adc_reading;
	
	// Update adc_values if requested. Note that we are running "one channel behind"
	// in free running mode.
	if (adc_read && adc_reading) {
		if(adc_selector == 0) {
			adc_values[ADC_CHANNEL_COUNT - 1] = ADCH;
		} else {
			adc_values[adc_selector - 1] = ADCH;
		}
	}
	// Next channel
	adc_selector++;
	// If we hit the max channel, go back to channel 0 and check status vars
	if(adc_selector >= ADC_CHANNEL_COUNT) {
		adc_selector = 0;
		// Start updating adc_value array if requested; Otherwise request is completed
		if (adc_read && !adc_reading) { // Start updating
			adc_reading = TRUE;
		} else { // Request is completed
			adc_read = FALSE;
			adc_reading = FALSE;
		}
	}
	// Update ADC channel selection
	ADMUX = (_BV(ADLAR) | _BV(REFS0)) + adc_selector;
}

// Initialize SG-90 servo PWM
inline void init_servo(void) {
	// Set PB1 as output
	DDRB |= _BV(PB1);
	// Set Fast PWM mode 14, prescaler of 8, non-inverting mode
	TCCR1A |= _BV(WGM11) | _BV(COM1A1);
	TCCR1B |= _BV(WGM12) | _BV(WGM13) | _BV(CS11);
	// Set PWM period
	ICR1 = TIMER1_TOP;
	// Set neutral position
	OCR1A = SERVO_90DEG;
}

// Get distance in cm from HC-SR04
uint16_t ping_cm(void) {
	static uint16_t lastPing;
	
	MCUCR |= _BV(ISC00); // Any logical change on INT0 generates an interrupt request
	GICR |= _BV(INT0);   // External Interrupt Request 0 Enable
	echoDone = FALSE;    // set echo flag
	countTimer0 = 0;     // reset counter
	// send 10us trigger pulse
	PORTC &= ~_BV(PC5);
	_delay_us(4);
	PORTC |= _BV(PC5);
	_delay_us(10);
	PORTC &= ~_BV(PC5);
	// Previous ping hasn't finished, abort.
	if(PIND & _BV(PIND2))
		return lastPing;
	// loop till echo pin goes low
	while(!echoDone);
	// disable interrupt
	MCUCR &= ~_BV(ISC00);
	GICR &= ~_BV(INT0);
	// calculate distance
	lastPing = countTimer0 / CM_ECHO_TIME;
	return lastPing;
}

// Timer0 interrupt
ISR(TIMER0_OVF_vect) {
	countTimer0 += 255;
	if (countTimer0 > MAX_ECHO_TIME) {
		TCCR0 = 0;            // Timer0 Stopped
		countTimer0 += TCNT0; // calculate time passed
		TCNT0 = 0;            // Reset Timer0
		echoDone = TRUE;      // set flag
	}
}

// Echo pin interrupt
ISR(INT0_vect){
	if(PIND & _BV(PIND2)) {
		// rising edge
		TCCR0 = _BV(CS00);    // Timer0 Clock Select No Prescaling
	} else {
		// falling edge
		TCCR0 = 0;            // Timer0 Stopped
		countTimer0 += TCNT0; // calculate time passed
		TCNT0 = 0;            // Reset Timer0
		echoDone = TRUE;      // set flag
	}
}

// Initialize HC-SR04
inline void init_sonar(void) {
	DDRC |= _BV(PC5);    // Trigger pin as output
	PORTD |= _BV(PD2);   // Pull up echo pin
	TIMSK |= _BV(TOIE0); // Timer0 Overflow Interrupt Enable
}

// Initialize soft PWM and L293D pins
inline void init_motor(void) {
	// Timer2 enable
	TCCR2 |= _BV(CS20);
	TIMSK |= _BV(TOIE2); // enable timer2 overflow interrupt
	// Motor activation pins
	DDRD |= (_BV(4) | _BV(5) | _BV(6) | _BV(7)); // PD4, PD5, PD6, PD7 output
	PORTD &= ~(_BV(4) | _BV(5) | _BV(6) | _BV(7)); // PD4, PD5, PD6, PD7 clear
}

// Timer2 interrupt
ISR(TIMER2_OVF_vect) {
	static uint8_t countTimer2;
	static int8_t curSpeed1, curSpeed2;
	
	// Start of duty cycle
	if (countTimer2 == TIMER2_TOP) {
		countTimer2 = 0;
		// Increment or decrement by one until current speed matches set speed
		// to save battery life
		if (curSpeed1 < speedMotor1) 
			curSpeed1++; 
		else if (curSpeed1 > speedMotor1) 
			curSpeed1--;
		if (curSpeed2 < speedMotor2) 
			curSpeed2++; 
		else if (curSpeed2 > speedMotor2) 
			curSpeed2--;
		// Turn on appropriate pins
		if (curSpeed1 < 0) { // Backward
			PORTD &= ~(1 << PD4);
			PORTD |= (1 << PD5);
		} else if (curSpeed1 > 0) { // Forward
			PORTD &= ~(1 << PD5);
			PORTD |= (1 << PD4);
		}
		if (curSpeed2 < 0) { // Backward
			PORTD &= ~(1 << PD7);
			PORTD |= (1 << PD6);
		} else if (curSpeed2 > 0) { // Forward
			PORTD &= ~(1 << PD6);
			PORTD |= (1 << PD7);
		}
	}
	// End of duty cycle; turn off pins
	if (countTimer2 == abs(curSpeed1))
		PORTD &= ~((1 << PD4) | (1 << PD5));
	if (countTimer2 == abs(curSpeed2))
		PORTD &= ~((1 << PD6) | (1 << PD7));
	// Increment counter
	countTimer2++;
}

// Dump array of uint8_t to UART with label string stored in progmem
void dump_array_p(const char *progmem_s, uint8_t *p, size_t t) {
	uart_puts_p(progmem_s);
	while (t--) {
		utoa(*p++, buffer, 10);
		uart_puts(buffer);
		if (t)
			dump_separator();
	}
}

// Dump int16_t to UART with label string stored in progmem
void dump_value_p(const char *progmem_s, int16_t val) {
	uart_puts_p(progmem_s);
	itoa(val, buffer, 10);
	uart_puts(buffer);
}

// Dump ADC values array to UART
void dump_adc_values(void) {
	dump_array_P("adc=", (uint8_t *)adc_values, sizeof(adc_values));
}

// Put newline chars to UART
void dump_newline(void) {
	uart_puts_P("\r\n");
}

void dump_separator(void) {
	uart_puts_P(", ");
}

// Dump motor status to UART
void dump_status(void) {
	if (speedMotor1 < speedMotor2)
		uart_puts_P("right, ");
	else if (speedMotor1 > speedMotor2)
		uart_puts_P("left, ");
	else if (speedMotor1 < 0 && speedMotor2 < 0)
		uart_puts_P("backward, ");
	else if (speedMotor1 == 0 && speedMotor2 == 0)
		uart_puts_P("stop, ");
	else
		uart_puts_P("forward, ");
	
	dump_value_P("motor=", speedMotor1);
	dump_value_P(", ", speedMotor2);
	dump_newline();
}

// Advanced line follow robot routine
void line_follow(void) {
	uint8_t i;
	int16_t w, v, pos, lpos = 0;
	int16_t der, pid, it = 0;
	int32_t sum, div;
	
	PORTB |= _BV(PB0);  // Turn IR sensor array on
	adc_start();
	while (!uart_available() && robot_mode == LINE_FOLLOW) {
		adc_read = TRUE; // Request ADC value update
		while(adc_read); // Wait until we get fresh values
		// Calculate weighted average of each sensor channel
		sum = 0;
		div = 0;
		for (i = 0; i < ADC_CHANNEL_COUNT; i++) {
			w = adc_values[i];            // value goes from 0 to 255
			w = min(w, adc_max[i]);       // if w is higher than B, then w is B
			w = max(w, adc_min[i]);       // if w is lower than A, then w is A
			w -= adc_min[i];              // let w logically go from 0 to B-A
			w *= 256;                     // w will go from 0 to (B-A)*256
			w /= adc_max[i] - adc_min[i]; // make w go from 0 to 255
			v = 64 * (i - 2);
			sum += (int32_t)w * v;
			div += w;
		}
		pos = sum / div;         // positional error and the proportional term
		der = pos - lpos;        // derivative term
		lpos = pos;              // save error for next pass
		it += pos;               // integral term
		pid = (pos * (int16_t)Kp + it * (int16_t)Ki + der * (int16_t)Kd) / 100;
		pid = constrain(pid, -128, 128);
		// dump values
		dump_adc_values();
		dump_value_P(", pos=", pos);
		dump_value_P(", it=", it);
		dump_value_P(", der=", der);
		dump_value_P(", pid=", pid);
		dump_separator();
		// calculate motor speeds	
		if (pid < 0) {
			speedMotor1 = pid + 64;
			speedMotor2 = 64;
		} else {
			speedMotor1 = 64;
			speedMotor2 = 64 - pid;
		}
		dump_status();
		_delay_ms(100);
	}
	adc_stop();
	PORTB &= ~_BV(PB0); // Turn IR sensor array off
}

// Collision avoidance robot routine
void collision_avoid(void) {
	uint16_t distance[11], minDist, avgDistL, avgDistR;
	uint8_t reverse = FALSE, state = 0, minDistI, divL, divR;
	uint8_t i = 5, j, minI = 3, maxI = 7, retention = 0;
	
	memset(distance, 0, sizeof(distance));
	OCR1A = SERVO_90DEG;
	_delay_ms(100);
	while (!uart_available() && robot_mode == COLLISION_AVOID) {
		// Measure distance for current angle
		distance[i] = ping_cm();
		// Go back and forth in steps of 18 degrees
		if (reverse) {
			OCR1A -= STEP_18DEG;
			i--;
		} else {
			OCR1A += STEP_18DEG;
			i++;
		}
		// Center servo
		if (i == 5)
			OCR1A = SERVO_90DEG;
		// Reverse direction
		if (i >= maxI)
			reverse = TRUE;
		else if (i <= minI)
			reverse = FALSE;
		// Loop through distance array
		avgDistR = 0;
		avgDistL = 0;
		divR = 0;
		divL = 0;
		minDistI = 0;
		minDist = MAX_DISTANCE;
		uart_puts_P("ping=");
		for (j = 0; j < 11; j++) {
			if (distance[j] > 0 && distance[j] < MAX_DISTANCE) {
				// Count distances on the right and left to make averages
				if (j < 5) {
					avgDistR += distance[j];
					divR++;
				}
				if (j > 5) {
					avgDistL += distance[j];
					divL++;
				}
				// Find minimum distance and store angle index
				if (distance[j] < minDist) {
					minDist = distance[j];
					minDistI = j;
				}
			}
			// Dump ping values
			utoa(distance[j], buffer, 10);
			uart_puts(buffer);
			dump_separator();
		}
		// Make average
		if (divR)
			avgDistR /= divR;
		if (divL)
			avgDistL /= divL;
		// Keep data until retention reaches zero
		if (retention) {
			retention--;
			if (!retention)
				memset(distance, 0, sizeof(distance));
		}
		// Dump values
		dump_value_P("min=", minDist);
		dump_value_P("@", minDistI);
		dump_value_P(", avg=", avgDistR);
		dump_value_P(", ", avgDistL);
		dump_value_P(", state=", state);
		dump_separator();
		dump_status();
		// State machine
		if (state != SWEEP_PENDING && state != SWEEP_PROGRESS) {
			// No sweep pending or in progress
			if (minDist < collDist && minDistI >= 3 && minDistI <= 7) {
				if (state != REVERSING) {
					// Object ahead within collision distance; go reverse
					state = REVERSING;
					speedMotor1 = -48;
					speedMotor2 = -48;
				}
			} else if (minDist < turnDist) {
				// Object within turning distance or within collision distance sideways
				if (state == SWEEP_FINISHED) {
					// Full range sweep has finished; start turning
					state = TURNING;
					if (avgDistL < avgDistR) {
						// Turn right
						speedMotor1 = 0;
						speedMotor2 = 64;
					} else {
						// Turn left
						speedMotor1 = 64;
						speedMotor2 = 0;
					}
				} else if (state != TURNING) {
					// Request full range sweep if not already turning
					state = SWEEP_PENDING;
					// Range between 0 and 180 degrees
					minI = 0;
					maxI = 10;
					speedMotor1 = 0;
					speedMotor2 = 0;
				}
			} else {
				// No object within turn and collision distance
				if (state == REVERSING) {
					// Stop reversing and request full range sweep
					state = SWEEP_PENDING;
					minI = 0;
					maxI = 10;
					speedMotor1 = 0;
					speedMotor2 = 0;
				} else {
					// Drive forward
					state = DRIVING;
					speedMotor1 = 64;
					speedMotor2 = 64;
				}
			}
		} else if (state == SWEEP_PROGRESS && (i == minI || i == maxI)) {
			// Full range sweep has finished
			state = SWEEP_FINISHED;
			// Range between 54 and 126 degrees
			minI = 3;
			maxI = 7;
			// Set data retention
			retention = 10;
		} else if (state == SWEEP_PENDING) {
			// Full range sweep is pending
			if (i == minI || i == maxI) {
				// Start sweep
				state = SWEEP_PROGRESS;
			}
		}
		_delay_ms(100);
	}
}

// Toggle robot mode
void toggle_mode(void) {
	switch (robot_mode) {
		case LINE_FOLLOW:
			robot_mode = COLLISION_AVOID;
			PORTB &= ~_BV(PB0);
			PORTB |= _BV(PB2);
			uart_puts_P("Collision avoid\r\n");
			break;
		case COLLISION_AVOID:
			robot_mode = IDLE;
			PORTB &= ~(_BV(PB0) | _BV(PB2));
			uart_puts_P("Idle\r\n");
			break;
		case IDLE:
			robot_mode = LINE_FOLLOW;
			PORTB &= ~_BV(PB2);
			PORTB |= _BV(PB0);
			uart_puts_P("Line follow\r\n");
	}
	// Stop motors
	speedMotor1 = 0;
	speedMotor2 = 0;
	// Servo to neutral
	OCR1A = SERVO_90DEG;
}

// Mode button interrupt
ISR(INT1_vect, ISR_NOBLOCK) {
	_delay_ms(100);
	toggle_mode();
	_delay_ms(1000);
}

// Initialize LEDs and button
inline void init_etc(void) {
	MCUCR |= (_BV(ISC10) | _BV(ISC11)); // Raising edge on INT1 generates an interrupt request
	GICR |= _BV(INT1);                  // External Interrupt Request 1 Enable	
	DDRB |= (_BV(PB0) | _BV(PB2));      // IR sensor array driver and LED pins as output
}

// Read vars from EEPROM
inline void init_eeprom(void) {
	collDist = eeprom_read_byte(&nvCollDist);
	turnDist = eeprom_read_byte(&nvTurnDist);
	eeprom_read_block(&adc_max, &nv_adc_max, sizeof(nv_adc_max));
	eeprom_read_block(&adc_min, &nv_adc_min, sizeof(nv_adc_min));
	Kp = eeprom_read_word(&nvKp);
	Kd = eeprom_read_word(&nvKd);
	Ki = eeprom_read_word(&nvKi);
}

// Get a new value for an int16_t from UART with label string stored in progmem
int16_t input_val_p(const char *progmem_s, int16_t val) {
	uint8_t i, c;
	
	dump_value_p(progmem_s, val);
	uart_puts_P("\r\nNew value: ");
	i = 0;
	do {
		while (!uart_available()); // Wait for character
		c = uart_getc();
		if (c == '-' || (c >= '0' && c <= '9')) { // Numeric character
			uart_putc(c);
			buffer[i++] = c;
		}
		if ((c == 8 || c == 127) && i > 0) { // Backspace
			uart_putc(c);
			i--;
		}
	} while (c != 13 && i <= sizeof(buffer) - 1); // Enter
	buffer[i] = 0;
	dump_newline();
	return (int16_t)atoi(buffer);
}

// Start single ADC conversion to refresh ADC values array
void adc_once(void) {
	PORTB |= _BV(PB0);  // Turn IR sensor array on
	adc_start();
	adc_read = TRUE;    // Request ADC value update
	while(adc_read);    // Wait until we get fresh values
	adc_stop();
	PORTB &= ~_BV(PB0); // Turn IR sensor array off
	dump_adc_values();  // Dump values to UART
	dump_newline();
}

// Constrain motor speeds within -64 and 64
void constrain_speeds(void) {
	speedMotor1 = constrain(speedMotor1, -64, 64);
	speedMotor2 = constrain(speedMotor2, -64, 64);
}

// Dump EEPROM stored values to UART
void dump_nv_values(void) {
	dump_value_P("collDist=", collDist);
	dump_value_P(", turnDist=", turnDist);
	dump_value_P("\r\nKp=", Kp);
	dump_value_P(", Ki=", Ki);
	dump_value_P(", Kd=", Kd);
	dump_array_P("\r\nadc_min=", adc_min, sizeof(adc_min));
	dump_array_P("\r\nadc_max=", adc_max, sizeof(adc_max));
	dump_newline();
}

// Handle UART input
void handle_uart(void) {
	switch (uart_getc()) {
		case 'a': // left
			speedMotor1 += 22;
			speedMotor2 -= 22;
			constrain_speeds();
			dump_status();
			break;
		case 'd': // right
			speedMotor1 -= 22;
			speedMotor2 += 22;
			constrain_speeds();
			dump_status();
			break;
		case 'w': // forward
			speedMotor1 += 22;
			speedMotor2 += 22;
			constrain_speeds();
			dump_status();
			break;
		case 's': // backward
			speedMotor1 -= 22;
			speedMotor2 -= 22;
			constrain_speeds();
			dump_status();
			break;
		case ' ': // stop
			speedMotor1 = 0;
			speedMotor2 = 0;
			dump_status();
			break;
		case '4': // servo to right
			OCR1A = SERVO_0DEG;
			break;
		case '5': // servo to neutral
			OCR1A = SERVO_90DEG;
			break;
		case '6': // servo to left
			OCR1A = SERVO_180DEG;
			break;
		case 'g': // single sonar ping
			dump_value_P("ping=", ping_cm());
			dump_newline();
			break;
		case 'o': // single adc conversion
			adc_once();
			break;
		case 't': // toggle robot mode
			toggle_mode();
			break;
		case 'c': // enter new collision distance
			collDist = input_val_P("collDist=", collDist);
			eeprom_write_byte(&nvCollDist, collDist);
			break;
		case 'u': // enter new turning distance
			turnDist = input_val_P("turnDist=", turnDist);
			eeprom_write_byte(&nvTurnDist, turnDist);
			break;
		case 'm': // calibrate light surface
			adc_once();
			memcpy((void*)&adc_max, (void*)&adc_values, sizeof(adc_max));
			eeprom_write_block(&adc_max, &nv_adc_max, sizeof(adc_max));
			break;
		case 'n': // calibrate dark line
			adc_once();
			memcpy((void*)&adc_min, (void*)&adc_values, sizeof(adc_min));
			eeprom_write_block(&adc_min, &nv_adc_min, sizeof(adc_min));
			break;
		case 'p': // enter new Kp value
			Kp = input_val_P("Kp=", Kp);
			eeprom_write_word(&nvKp, Kp);
			break;
		case 'k': // enter new Kd value
			Kd = input_val_P("Kd=", Kd);
			eeprom_write_word(&nvKd, Kd);
			break;
		case 'i': // enter new Ki value
			Ki = input_val_P("Ki=", Ki);
			eeprom_write_word(&nvKi, Ki);
			break;
		case 'v': // show stored values
			dump_nv_values();
			break;
		default: // show valid input characters
			uart_puts_P("ASDW,space,Toggle_mode,pinG,adc_Once,Colldist,tUrndist,Max,miN,kP,Kd,kI,dump_nV\r\n");
	}
}

// Main loop
int main(void) {
	uart_init(UART_BAUD_SELECT(9600, F_CPU));
	init_adc();
	init_motor();
	init_servo();
	init_sonar();
	init_etc();	
	init_eeprom();
	sei();         // enable all interrupts
	uart_puts_P("\ecCompiled "__DATE__" "__TIME__" with AVR-GCC "__VERSION__" and AVR Libc "__AVR_LIBC_VERSION_STRING__"\r\n");
	dump_nv_values();
	uart_puts_P("Idle\r\n");
	while (1) {
		switch (robot_mode) {
			case LINE_FOLLOW: 
				line_follow();
				break;
			case COLLISION_AVOID:
				collision_avoid();
		}
		if (uart_available()) 
			handle_uart();
	}
}
