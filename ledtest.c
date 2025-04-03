/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * Bare minimum empty user application template
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# "Insert system clock initialization code here" comment
 * -# Minimal main function that starts with a call to board_init()
 * -# "Insert application code here" comment
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */
#include <asf.h>

#define LED_PIN IOPORT_CREATE_PIN(PIOD, 5)





typedef struct {    // struct representing a color
	uint8_t red;
	uint8_t green;
	uint8_t blue;
} Color24;

#define LENGTH 9

static volatile Color24 colors[LENGTH] = {
	{255, 0, 0},      // Red
	{0, 255, 0},      // Green
	{0, 0, 255},      // Blue
	{255, 255, 0},    // Yellow
	{0, 255, 255},    // Cyan
	{255, 0, 255},    // Magenta
	{192, 192, 192},  // Silver
	{128, 0, 0},      // Maroon
	{0, 128, 0},      // Dark Green
	//{0, 0, 128}       // Navy Blue
};

// color bit array, each byte holds a bit of each color
#define COLORARRAYLENGTH LENGTH * 3
static uint8_t colorArray[COLORARRAYLENGTH];

void convertColorsToColorArray() {
	for (int i = 0; i < LENGTH; i++) {
		colorArray[i * 3] = colors[i].green;
		colorArray[i * 3 + 1] = colors[i].red;
		colorArray[i * 3 + 2] = colors[i].blue;
	}
}





__attribute__((always_inline)) static inline void send_high_bit() {
	REG_PIOD_SODR = PIO_PD5;
	asm volatile (
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	);
	REG_PIOD_CODR = PIO_PD5;
	asm volatile (
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	);
}


__attribute__((always_inline)) static inline void send_low_bit() {
	REG_PIOD_SODR = PIO_PD5;
	asm volatile (
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	);
	REG_PIOD_CODR = PIO_PD5;
	asm volatile (
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	);
}


void send_colors() {
	// disable interrupts, stop tc0 channel 0
	convertColorsToColorArray();

	for (int i = 0; i < COLORARRAYLENGTH; i++) {
		uint8_t temp = colorArray[i];
		uint8_t mask = 0x80;  // Reset mask for each byte

		for (int j = 0; j < 8; j++) {  // Process all 8 bits
			if (temp & mask) {
				send_high_bit();
			} else {
				send_low_bit();
			}
			mask >>= 1;  // Shift mask
		}
	}

	// reset ie pin out = low
	REG_PIOD_CODR = PIO_PD5;
	volatile int i = 0;
	while (i < 4200) { i += 1; }
}


int main (void)
{
	/* Insert system clock initialization code here (sysclk_init()). */
	sysclk_init();
	board_init();
	
	ioport_init();
	ioport_enable_pin(LED_PIN);
	ioport_set_pin_dir(LED_PIN, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(LED_PIN, false);
	
	while (true) {
		send_colors();
		colors[0].green += 1;
	}
}
