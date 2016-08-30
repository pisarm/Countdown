/*
 * Copyright (c) 2016 Flemming Pedersen
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <avr/interrupt.h>
#include <avr/io.h>

// #define LED PB3
//
// #define output_low(port, pin) port &= ~(1 << pin)
// #define output_high(port, pin) port |= (1 << pin)
// #define set_input(portdir, pin) portdir &= ~(1 << pin)
// #define set_output(portdir, pin) portdir |= (1 << pin)

// Functions
static void setup(void);

// Constants/variables
static const uint8_t COUNT_ONE_SECOND = 30;
volatile uint16_t timerCount;
volatile uint8_t buttonCount;

int main(void) {
	setup();

	for (;;) { }
}

static void setup(void) {
    //Outputs
	DDRB |= (1 << PB3);

    //Timer intrrupt
    TCCR1 |= (1 << CS10) | (1 << CS11) | (1 << CS13); //Set prescaler to 1024
    TIMSK |= (1 << TOIE1); //Enable timer1 overflow interrupt

    //Pin change interrupt
    PCMSK |= (1 << PCINT4); //PB4 generates intrrupt
    GIMSK |= (1 << PCIE); //Enable pin change interrupts

    sei(); //Enable interrupts
}

ISR(PCINT0_vect){
	//TODO: Make sure that we only react once per button push (interrupt triggers on all changes - low is when button is pushed)
    ++buttonCount;
}

ISR(TIM1_OVF_vect) {
    if (buttonCount > 0) {
		//If led is not turned on - do so
        if (!(PORTB & (1 << PB3))) {
            PORTB |= (1 << PB3);
        }

        if (++timerCount == COUNT_ONE_SECOND) {
            // --buttonCount;
			PORTB ^= (1 << PB3);
            timerCount = 0;
        }
    } else {
		//If led is turned on - turn it off
		if ((PORTB & (1 << PB3))) {
			PORTB &= ~(1 << PB3);
		}
    }
}
