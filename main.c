/*
 * Title:    Line following and collision avoidance robot car
 * Hardware: ATmega8 @ 16 MHz, HC-SR04 Ultrasonic Ranging Module,
 *           L293D Motor Driver, SG90 Servo, TCRT5000 Reflective Optical Sensor
 * Created:  26-7-2018 18:51:54
 * Author :  Tim Dorssers
 *
 * In line-follow mode the robot car uses five TCRT5000 sensors connected to
 * ADC0-ADC4 inputs to detect a dark line on a light surface or a light line on
 * a dark surface, selected by the set button. ADC conversion is interrupt
 * driven. The sensors can be calibrated for better accuracy by holding the set
 * button when the robot car is in idle mode to enter auto calibration mode and
 * moving the robot car over the dark and light surface. Measured values of the
 * light and dark surface are stored in EEPROM. Weighted average calculation is
 * used to determine the "exact" location of the line. The motors are driven by
 * a PID control. The K values are scaled integers to avoid floating point
 * numbers and are user configurable via UART.
 * In collision avoidance mode the robot car uses the HC-SR04 module and the
 * SG90 micro servo to "look around". The servo is hardware PWM driven via OC1A
 * output using 16-bit Timer1. The HC-SR04 module is connected to external
 * interrupt INT0 and uses 8-bit Timer0 interrupt to measure RTT. Because of
 * the limited "viewing" angle of the sonar, the servo sweeps between 54 and
 * 126 degrees. When an object is detected within the turning distance, the
 * robot performs a sweep from 0 to 180 degrees and chooses the angle with the
 * maximum average clear distance to turn to. When an object is detected ahead
 * within the collision distance, the robot reverses until no object is within
 * collision distance anymore. The turning and collision distance as well as
 * motor speeds are user configurable from the UART.
 * The DC motors are driven by a software PWM implementation using 8-bit Timer2
 * CTC interrupt and PD4-PD7 outputs connected to the L293D motor driver.
 * The mode button is used to switch between the different modes; line follow,
 * collision avoid and idle. If you press and hold the mode button, the auto
 * toggle mode is activated, switching the mode according to the presence of
 * the line. In idle mode, the robot can be driven "remotely" from the UART as
 * well.
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
#define MAX_ECHO_TIME (CM_ECHO_TIME * MAX_DISTANCE)  // Maximum sensor distance
uint32_t countTimer0;
volatile uint8_t echoDone;

// Collision avoid
#define DRIVING        0
#define SWEEP_PENDING  1
#define SWEEP_PROGRESS 2
#define SWEEP_FINISHED 3
#define TURNING        4
#define REVERSING      5
#define STOPPED        6
uint8_t EEMEM nvCollDist = 20;   // sets distance at which robot stops and reverses
uint8_t EEMEM nvTurnDist = 40;   // sets distance at which robot veers away from object
uint8_t EEMEM nvFwdSpeed = 64, nvRevSpeed = -64;
uint8_t EEMEM nvTimeOut = 40;    // 4000 ms
uint8_t EEMEM nvEqThres = 10;    // 1000 ms
uint8_t collDist, turnDist, timeOut, eqThres; //, retention
int8_t fwdSpeed, revSpeed;

// IR sensor
#define ADC_CHANNEL_COUNT    5   // Number of ADC channels we use
#define ADC_CALIBRATE_CYCLES 50  // 5000 ms
volatile uint8_t adc_values[ADC_CHANNEL_COUNT];
volatile uint8_t adc_read = FALSE;
uint8_t EEMEM nv_adc_min[ADC_CHANNEL_COUNT] = {45, 45, 45, 45, 45};
uint8_t EEMEM nv_adc_max[ADC_CHANNEL_COUNT] = {135, 135, 135, 135, 135};
uint8_t adc_min[ADC_CHANNEL_COUNT], adc_max[ADC_CHANNEL_COUNT];
uint8_t EEMEM nv_inverse = FALSE;
uint8_t inverse, requestCalibrate = FALSE;

// PID control K values multiplied by 100
uint16_t EEMEM nvKp = 500, nvKd = 50, nvKi = 0;
int16_t Kp, Kd, Ki;

// Servo PWM, using prescaler of 8
#define TIMER1_TOP   ((F_CPU / 50 / 8) - 1)      // 50 Hz (20 ms) PWM period
#define SERVO_0DEG   ((uint16_t)(F_CPU / (1000 / 0.75) / 8) - 1)
#define SERVO_90DEG  ((uint16_t)(F_CPU / (1000 / 1.65) / 8) - 1) // 1.65 ms duty cycle
#define SERVO_180DEG ((uint16_t)(F_CPU / (1000 / 2.55) / 8) - 1)
#define SERVO_18DEG  ((SERVO_180DEG - SERVO_0DEG) / 10)
#define SERVO_1DEG   (SERVO_18DEG / 18)

// Software PWM, 64 steps, prescaler of 8
#define TIMER2_TOP ((F_CPU / 64 / 8 / 975) - 1)  // 975 Hz PWM period
int8_t speedMotor1, speedMotor2;
uint8_t saveBatt = FALSE;

// Robot running mode
#define IDLE            0
#define LINE_FOLLOW     1
#define COLLISION_AVOID 2
uint8_t robot_mode = IDLE;
uint8_t auto_toggle = FALSE;

// Program space strings
const char KpStr[] PROGMEM = "Kp=";
const char KdStr[] PROGMEM = "Kd=";
const char KiStr[] PROGMEM = "Ki=";
const char ping_str[] PROGMEM = "ping=";
const char separator_str[] PROGMEM = ", ";
const char timeOutStr[] PROGMEM = "timeOut=";
const char eqThresStr[] PROGMEM = "eqThres=";
const char collDistStr[] PROGMEM = "collDist=";
const char turnDistStr[] PROGMEM = "turnDist=";
const char fwdSpeedStr[] PROGMEM = "fwdSpeed=";
const char revSpeedStr[] PROGMEM = "revSpeed=";
const char auto_toggle_str[] PROGMEM = "Auto toggle\r\n";

// Push buttons
#define DEBOUNCE  5     // 100 ms
#define LONG_PUSH 50    // 1000 ms
uint8_t modeButton = 0, setButton = 0;

// Function prototypes
void adc_calibrate(void);
inline void adc_manual(void);
void adc_once(void);
void adc_write_values(void);
void clear_all(void);
void collision_avoid(void);
void constrain_speeds(void);
void dump_adc_min_max(void);
void dump_adc_values(void);
void dump_angle(void);
void dump_array_p(const char *progmem_s, uint8_t *p, size_t n);
void dump_inverse(void);
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
void status_led(void);
void toggle_inverse(void);
void toggle_mode(void);
void wait_while(void);

//Function macros
#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define adc_stop() ADCSRA &= ~(_BV(ADEN) | _BV(ADSC)) // Disable ADC, stop conversion
#define adc_start() ADCSRA |= (_BV(ADEN) | _BV(ADSC)) // Enable ADC, start conversion
#define ir_stop() PORTD &= ~_BV(PD3)  // Turn IR sensor array off
#define ir_start() PORTD |= _BV(PD3)  // Turn IR sensor array on
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
	if (bit_is_set(PIND, PIND2))
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

// Timer0 interrupt fires F_CPU / 256 times per second for echo pulse time measurement
ISR(TIMER0_OVF_vect) {
	countTimer0 += 255;
	if (countTimer0 > MAX_ECHO_TIME) {
		TCCR0 = 0;            // Timer0 Stopped
		countTimer0 += TCNT0; // calculate time passed
		TCNT0 = 0;            // Reset Timer0
		echoDone = TRUE;      // set flag
	}
}

// HC-SR04 echo pin interrupt
ISR(INT0_vect){
	if (bit_is_set(PIND, PD2)) {
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
	// Timer2 enable in CTC mode using prescaler 8
	TCCR2 |= _BV(WGM21) | _BV(CS21);
	TIMSK |= _BV(OCIE2); // Timer2 Output Compare Match Interrupt Enable
	OCR2 = TIMER2_TOP;
	// Motor activation pins as output
	DDRD |= _BV(PD4) | _BV(PD5) | _BV(PD6) | _BV(PD7);
}

// Timer2 interrupt implements software PWM for DC motor control
ISR(TIMER2_COMP_vect) {
	static uint8_t countTimer2;
	static int8_t curSpeed1, curSpeed2;
	
	// Start of duty cycle
	if (countTimer2 == 64) {
		countTimer2 = 0;
		if (saveBatt) {
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
		} else {
			curSpeed1 = speedMotor1;
			curSpeed2 = speedMotor2;
		}
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
void dump_array_p(const char *progmem_s, uint8_t *p, size_t n) {
	uart_puts_p(progmem_s);
	while (n--) {
		utoa(*p++, buffer, 10);
		uart_puts(buffer);
		if (n)
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

// Put comma space chars to UART
void dump_separator(void) {
	uart_puts_p(separator_str);
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
	dump_value_p(separator_str, speedMotor2);
	dump_newline();
}

// Wait a second as long as robot_mode doesn't change
void wait_while(void) {
	uint8_t i = 100, rm;
	
	rm = robot_mode;
	while(i-- && !uart_available() && robot_mode == rm)
		_delay_ms(10);
}

// Advanced line follow robot routine
void line_follow(void) {
	uint8_t i;
	int16_t w, v, div, pos, lpos = 0;
	int16_t der, pid, it = 0;
	int32_t sum;
	
	ir_start();
	adc_start();
	PORTB |= _BV(PB0);  // Turn LED on
	wait_while();
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
			if (inverse)
				w = 256 - w;              // white line on black background
			v = 64 * (i - 2);             // 64 per channel, v goes from -128 to 128
			sum += (int32_t)w * v;
			div += w;
		}
		// positional error and the proportional term
		pos = (div) ? sum / div : 0;
		der = pos - lpos;        // derivative term
		lpos = pos;              // save error for next pass
		it += pos;               // integral term
		pid = (pos * Kp + it * Ki + der * Kd) / 100;
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
		// no line when sum of all weights is zero or max
		if (div == 0 || div == 1280) {
			if (auto_toggle)
				toggle_mode();
			else {
				speedMotor1 = 0;
				speedMotor2 = 0;
				PORTB ^= _BV(PB0);  // Toggle LED
			}
		} else
			PORTB |= _BV(PB0);  // Turn LED on
		dump_status();
		_delay_ms(100);
	}
	clear_all();
	adc_stop();
	ir_stop();
}

// Collision avoidance robot routine
void collision_avoid(void) {
	uint16_t distance[11], minDist, avgDistL, avgDistR, lastAvgDistL = 0, lastAvgDistR = 0, minDistAhead;
	uint8_t reverse = FALSE, state = 0, minDistI, divL, divR, eqCnt = 0;
	uint8_t i = 5, j, minI = 3, maxI = 7, manCnt = 0, temp;
	
	memset(distance, 0, sizeof(distance));
	saveBatt = TRUE;
	PORTB |= _BV(PB2);  // Turn LED on
	if (auto_toggle) {
		ir_start();
		adc_start();
	}
	wait_while();
	while (!uart_available() && robot_mode == COLLISION_AVOID) {
		// Measure distance for current angle
		distance[i] = ping_cm();
		// Go back and forth in steps of 18 degrees
		if (state != STOPPED)
			i += (reverse) ? -1 : 1;
		OCR1A = SERVO_0DEG + (SERVO_18DEG * i);
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
		minDistI = 5;
		minDist = MAX_DISTANCE;
		minDistAhead = MAX_DISTANCE;
		uart_puts_p(ping_str);
		for (j = 0; j < 11; j++) {
			// Zero elements that are out of range
			if (j < minI || j > maxI)
				distance[j] = 0;
			if (distance[j] > 0 && distance[j] < MAX_DISTANCE) {
				// Count distances on the right and left to make averages
				if (j < 5) {
					avgDistR += distance[j];
					divR++;
				} else if (j > 5) {
					avgDistL += distance[j];
					divL++;
				}
				// Find minimum distance and store angle index
				if (distance[j] < minDist) {
					minDist = distance[j];
					minDistI = j;
				}
				// Find minimum distance ahead
				if (distance[j] < minDistAhead && j >= 3 && j <= 7)
					minDistAhead = distance[j];
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
		// Count consecutive equal averages
		if (lastAvgDistR == avgDistR && lastAvgDistL == avgDistL && state != SWEEP_PROGRESS)
			eqCnt++;
		else
			eqCnt = 0;
		lastAvgDistR = avgDistR;
		lastAvgDistL = avgDistL;
		// Auto toggle mode
		if (auto_toggle) {
			adc_read = TRUE;    // Request ADC value update
			while(adc_read);    // Wait until we get fresh values
			dump_adc_values();
			dump_separator();
			// Check for line
			for (j = 0; j < ADC_CHANNEL_COUNT; j++) {
				if (adc_values[j] >= adc_min[j] && adc_values[j] < adc_max[j]) {
					toggle_mode();
					break;
				}
			}
		}
		// Dump values
		dump_value_P("min=", minDist);
		dump_value_P("@", minDistI);
		dump_value_p(separator_str, minDistAhead);
		dump_value_P(", avg=", avgDistR);
		dump_value_p(separator_str, avgDistL);
		dump_value_P(", cnt=", eqCnt);
		dump_value_p(separator_str, manCnt);
		dump_value_P(", state=", state);
		dump_separator();
		dump_status();
		// State machine
		if (state != SWEEP_PENDING && state != SWEEP_PROGRESS && state != STOPPED) {
			// No sweep pending or in progress
			if (minDistAhead < collDist) {
				// Object ahead within collision distance
				manCnt++;
				if (state != REVERSING) {
					// Go reverse and turn to prevent going straight forward to same object
					state = REVERSING;
					if (avgDistL < avgDistR) {
						// Turn right
						speedMotor1 = revSpeed;
						speedMotor2 = 0;
					} else {
						// Turn left
						speedMotor1 = 0;
						speedMotor2 = revSpeed;
					}
				} 
			} else if (minDistAhead < turnDist || minDist < collDist) {
				// Object ahead within turning distance or within collision distance sideways
				manCnt++;
				if (state == SWEEP_FINISHED) {
					// Full range sweep has finished; start turning
					state = TURNING;
					if (avgDistL < avgDistR) {
						// Turn right
						speedMotor1 = revSpeed;
						speedMotor2 = fwdSpeed;
					} else {
						// Turn left
						speedMotor1 = fwdSpeed;
						speedMotor2 = revSpeed;
					}
					// Middle of angle of 90 degrees towards minimum distance
					temp = constrain(minDistI, 2, 8);
					minI = temp - 2;
					maxI = temp + 2;
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
				manCnt = 0;
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
					// Range between 54 and 126 degrees
					minI = 3;
					maxI = 7;
					speedMotor1 = fwdSpeed;
					speedMotor2 = fwdSpeed;
				}
			}
		} else if (state == SWEEP_PROGRESS && (i == minI || i == maxI))
			// Full range sweep has finished
			state = SWEEP_FINISHED;
		else if (state == SWEEP_PENDING) {
			// Full range sweep is pending
			if (i == minI || i == maxI) {
				// Start sweep
				state = SWEEP_PROGRESS;
			}
		} else if (state == STOPPED)
			PORTB ^= _BV(PB2);  // Toggle LED
		// When the maneuver timeout has expired or when consecutive equal
		// averages exceed the threshold, it is likely that the robot car is stuck
		if (manCnt >= timeOut || eqCnt >= eqThres) {
			state = STOPPED;
			speedMotor1 = 0;
			speedMotor2 = 0;
			i = 5;
			minI = 5;
			maxI = 5;
		}
		_delay_ms(100);
	}
	clear_all();
	saveBatt = FALSE;
	if (auto_toggle) {
		ir_stop();
		adc_stop();
	}
}

// Toggle robot mode
void toggle_mode(void) {
	switch (robot_mode) {
		case LINE_FOLLOW:
			robot_mode = COLLISION_AVOID;
			uart_puts_P("Collision avoid\r\n");
			break;
		case COLLISION_AVOID:
			robot_mode = IDLE;
			uart_puts_P("Idle\r\n");
			// Don't break if auto toggle is true
			if (!auto_toggle) 
				break;
		case IDLE:
			robot_mode = LINE_FOLLOW;
			uart_puts_P("Line follow\r\n");
	}
}

// Reset motors, LEDs and servo
void clear_all(void) {
	// Stop motors
	speedMotor1 = 0;
	speedMotor2 = 0;
	// Servo to neutral
	OCR1A = SERVO_90DEG;
	// Turn off LEDs
	PORTB &= ~(_BV(PB0) | _BV(PB2));
}

// Initialize LEDs and buttons
inline void init_etc(void) {
	DDRD |= _BV(PD3);                       // IR sensor array driver as output
	DDRB |= _BV(PB0) | _BV(PB2) | _BV(PB4); // LED pins as output
	PORTB |= _BV(PB3) | _BV(PB5);           // Pull up button pins
	TIMSK |= _BV(TOIE1);                    // Timer1 overflow interrupt enable
}

// LED shows inverse status
void status_led(void) {
	if (inverse)
		PORTB |= _BV(PB4);  // Turn on LED
	else
		PORTB &= ~_BV(PB4); // Turn off LED
}

// Toggle inverse mode
void toggle_inverse(void) {
	inverse ^= TRUE;
	inverse &= TRUE;
	eeprom_write_byte(&nv_inverse, inverse);
	status_led();
	dump_inverse();
	dump_newline();
}

// Timer1 interrupt implements button scanning and debouncing
ISR(TIMER1_OVF_vect) {
	// Check if mode button is pushed
	if (bit_is_clear(PINB, PB3))
		modeButton++;
	else {
		if (modeButton >= DEBOUNCE) {
			if (modeButton >= LONG_PUSH) {
				// Set auto toggle mode
				auto_toggle = TRUE;
				uart_puts_p(auto_toggle_str);
			} else
				auto_toggle = FALSE;
			// Toggle robot mode
			toggle_mode();
		}
		modeButton = 0;
	}
	// Check if set button is pushed
	if (bit_is_clear(PINB, PB5))
		setButton++;
	else {
		if (setButton >= DEBOUNCE) {
			if (setButton >= LONG_PUSH) {
				// Request auto calibrate
				if (robot_mode == IDLE)
					requestCalibrate = TRUE;
			} else {
				// Toggle inverse mode
				toggle_inverse();
			}
		}
		setButton = 0;
	}
}

// Read vars from EEPROM
inline void init_eeprom(void) {
	collDist = eeprom_read_byte(&nvCollDist);
	turnDist = eeprom_read_byte(&nvTurnDist);
	fwdSpeed = eeprom_read_byte(&nvFwdSpeed);
	revSpeed = eeprom_read_byte(&nvRevSpeed);
	timeOut = eeprom_read_byte(&nvTimeOut);
	eqThres = eeprom_read_byte(&nvEqThres);
	eeprom_read_block(&adc_max, &nv_adc_max, sizeof(nv_adc_max));
	eeprom_read_block(&adc_min, &nv_adc_min, sizeof(nv_adc_min));
	inverse = eeprom_read_byte(&nv_inverse);
	eeprom_read_block(&Kp, &nvKp, sizeof(Kp));
	eeprom_read_block(&Kd, &nvKd, sizeof(Kd));
	eeprom_read_block(&Ki, &nvKi, sizeof(Ki));
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
	ir_start();
	adc_start();
	adc_read = TRUE;    // Request ADC value update
	while(adc_read);    // Wait until we get fresh values
	adc_stop();
	ir_stop();
}

// Dump adc_min and adc_max arrays to UART
void dump_adc_min_max(void) {
	dump_array_P("adc_min=", adc_min, sizeof(adc_min));
	dump_array_P("\r\nadc_max=", adc_max, sizeof(adc_max));
	dump_newline();
}

// Store new values in EEPROM
void adc_write_values(void) {
	eeprom_write_block(&adc_max, &nv_adc_max, sizeof(adc_max));
	eeprom_write_block(&adc_min, &nv_adc_min, sizeof(adc_min));
}

// Perform auto calibration
void adc_calibrate(void) {
	uint8_t i, j, quarter;
	
	requestCalibrate = FALSE;
	// Set max to lowest possible value and set min to highest possible value
	memset(adc_max, 0, sizeof(adc_max));
	memset(adc_min, 180, sizeof(adc_min));
	// Repeatedly get ADC values
	for (i = 0; i < ADC_CALIBRATE_CYCLES; i++) {
		PORTB ^= _BV(PB4);   // Toggle LED
		adc_once();          // Refresh ADC values
		for (j = 0; j < ADC_CHANNEL_COUNT; j++) {
			// Find min and max values for each channel
			adc_min[j] = min(adc_min[j], adc_values[j]);
			adc_max[j] = max(adc_max[j], adc_values[j]);
		}
		_delay_ms(100);
	}
	// Use center half of range
	for (i = 0; i < ADC_CHANNEL_COUNT; i++) {
		quarter = (adc_max[i] - adc_min[i]) / 4;
		adc_min[i] = quarter;
		adc_max[i] = quarter * 3;
	}
	adc_write_values();
	dump_adc_min_max();
	// Restore LED state
	status_led();
}

// Manual input of adc_min and adc_max
inline void adc_manual(void) {
	uint8_t i;
	
	for (i = 0; i < ADC_CHANNEL_COUNT; i++)
		adc_min[i] = input_val_P("adc_min #", i);
	for (i = 0; i < ADC_CHANNEL_COUNT; i++)
		adc_max[i] = input_val_P("adc_max #", i);
	adc_write_values();
}

// Constrain motor speeds within -64 and 64
void constrain_speeds(void) {
	speedMotor1 = constrain(speedMotor1, -64, 64);
	speedMotor2 = constrain(speedMotor2, -64, 64);
}

// Dump inverse variable to UART
void dump_inverse(void) {
	dump_value_P("inverse=", inverse);
}

// Dump EEPROM stored values to UART
void dump_nv_values(void) {
	dump_value_p(collDistStr, collDist);
	dump_separator();
	dump_value_p(turnDistStr, turnDist);
	dump_separator();
	dump_value_p(timeOutStr, timeOut);
	dump_newline();
	dump_value_p(fwdSpeedStr, fwdSpeed);
	dump_separator();
	dump_value_p(revSpeedStr, revSpeed);
	dump_separator();
	dump_value_p(eqThresStr, eqThres);
	dump_newline();
	dump_value_p(KpStr, Kp);
	dump_separator();
	dump_value_p(KiStr, Ki);
	dump_separator();
	dump_value_p(KdStr, Kd);
	dump_separator();
	dump_inverse();
	dump_newline();
	dump_adc_min_max();
}

// Dump servo angle to UART
void dump_angle(void) {
	dump_value_P("angle=", (OCR1A - SERVO_0DEG) / SERVO_1DEG);
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
			if (OCR1A < SERVO_180DEG)
				OCR1A += SERVO_18DEG;
			dump_angle();
			break;
		case '5': // servo to neutral
			OCR1A = SERVO_90DEG;
			dump_angle();
			break;
		case '6': // servo to left
			if (OCR1A > SERVO_0DEG)
				OCR1A -= SERVO_18DEG;
			dump_angle();
			break;
		case 'g': // single sonar ping
			dump_value_p(ping_str, ping_cm());
			dump_newline();
			break;
		case 'o': // single adc conversion
			adc_once();
			dump_adc_values();
			dump_newline();
			break;
		case 't': // toggle robot mode
			auto_toggle = FALSE;
			toggle_mode();
			break;
		case 'c': // enter new collision distance
			collDist = input_val_p(collDistStr, collDist);
			eeprom_write_byte(&nvCollDist, collDist);
			break;
		case 'u': // enter new turning distance
			turnDist = input_val_p(turnDistStr, turnDist);
			eeprom_write_byte(&nvTurnDist, turnDist);
			break;
		case 'p': // enter new Kp value
			Kp = input_val_p(KpStr, Kp);
			eeprom_write_word(&nvKp, Kp);
			break;
		case 'k': // enter new Kd value
			Kd = input_val_p(KdStr, Kd);
			eeprom_write_word(&nvKd, Kd);
			break;
		case 'i': // enter new Ki value
			Ki = input_val_p(KiStr, Ki);
			eeprom_write_word(&nvKi, Ki);
			break;
		case 'v': // show stored values
			dump_nv_values();
			break;
		case 'f': // enter new fwdSpeed value
			fwdSpeed = input_val_p(fwdSpeedStr, fwdSpeed);
			eeprom_write_byte(&nvFwdSpeed, fwdSpeed);
			break;
		case 'r': // enter new revSpeed value
			revSpeed = input_val_p(revSpeedStr, revSpeed);
			eeprom_write_byte(&nvRevSpeed, revSpeed);
			break;
		case 'n': // toggle inverse mode
			toggle_inverse();
			break;
		case 'b': // run auto calibration
			adc_calibrate();
			break;
		case 'm': // enter new timeOut value
			timeOut = input_val_p(timeOutStr, timeOut);
			eeprom_write_byte(&nvTimeOut, timeOut);
			break;
		case 'q': // enter new eqThres value
			eqThres = input_val_p(eqThresStr, eqThres);
			eeprom_write_byte(&nvEqThres, eqThres);
			break;
		case 'l': // enter new values for adc_min and adc_max manually
			adc_manual();
			break;
		case 'e': // set auto toggle mode
			auto_toggle = TRUE;
			uart_puts_p(auto_toggle_str);
			toggle_mode();
			break;
		default: // show valid input characters
			uart_puts_P("WASD,spacebar,Toggle_mode,adc_caliBrate,adc_manuaL,adc_Once,kP,Kd,kI,iNverse,\r\npinG,Colldist,tUrndist,Fwdspeed,Revspeed,tiMeout,eQthres,dump_nV,auto_togglE,456\r\n");
	}
}

int main(void) {
	// Setup
	uart_init(UART_BAUD_SELECT(9600, F_CPU));
	init_adc();
	init_motor();
	init_servo();
	init_sonar();
	init_etc();	
	init_eeprom();
	status_led();  // Inverse mode status LED
	sei();         // enable all interrupts
	uart_puts_P("\ecCompiled "__DATE__" "__TIME__" with AVR-GCC "__VERSION__" and AVR Libc "__AVR_LIBC_VERSION_STRING__"\r\n");
	dump_nv_values();
	uart_puts_P("Idle\r\n");
	// Main loop
	while (1) {
		// Run routines
		switch (robot_mode) {
			case LINE_FOLLOW: 
				line_follow();
				break;
			case COLLISION_AVOID:
				collision_avoid();
		}
		// Handle characters in UART ring buffer if any
		if (uart_available()) 
			handle_uart();
		// Run auto calibration if requested
		if (requestCalibrate)
			adc_calibrate();
	}
}
