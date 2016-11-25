// ----------------------------------------------
// OpóŸnienia

#include <avr/io.h>

#include "delay.h"

// ---------------- teoretycznie dok³adniejsze funkcje :))

void delay1us(uint16_t t) {
	while (t>0) {
		// ~250ns (271)
		asm volatile("nop"::);
		asm volatile("nop"::);
		// ~250ns (271)
		asm volatile("nop"::);
		asm volatile("nop"::);
		// ~250ns (271)
		asm volatile("nop"::);
		asm volatile("nop"::);
		// ~250ns (271)
		asm volatile("nop"::);
		asm volatile("nop"::); //*/
		--t;
	}
}

void delay1ms(uint16_t t) {
	while (t>0) {
		delay1us(995);
		--t;
	}
}

