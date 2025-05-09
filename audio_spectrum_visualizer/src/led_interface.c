/*
 * led_interface.c
 *
 * Created: 4/22/2025 5:44:36 PM
 *  Author: latent
 */ 
/**
 * \file
 *
 * \brief Empty user application template
 *
 */

#include "led_interface.h"
#include <asf.h>

typedef struct {
	uint8_t red;
	uint8_t green;
	uint8_t blue;
} Color24;

#define COLS 10
#define ROWS 10
#define COLORARRAYLENGTH (COLS * ROWS * 3)

static volatile Color24 colors[COLS][ROWS] = {
	{ {1, 0, 0}, {1, 1, 0}, {1, 1, 1}, {2, 1, 1}, {2, 2, 1}, {2, 2, 2}, {3, 2, 2}, {3, 3, 2}, {3, 3, 3}, {4, 3, 3} },
	{ {4, 4, 3}, {4, 4, 4}, {5, 4, 4}, {5, 5, 4}, {5, 5, 5}, {6, 5, 5}, {6, 6, 5}, {6, 6, 6}, {7, 6, 6}, {7, 7, 6} },
	{ {7, 7, 7}, {8, 7, 7}, {8, 8, 7}, {8, 8, 8}, {9, 8, 8}, {9, 9, 8}, {9, 9, 9}, {10, 9, 9}, {10, 10, 9}, {10, 10, 10} },
	{ {11, 10, 10}, {11, 11, 10}, {11, 11, 11}, {12, 11, 11}, {12, 12, 11}, {12, 12, 12}, {13, 12, 12}, {13, 13, 12}, {13, 13, 13}, {14, 13, 13} },
	{ {14, 14, 13}, {14, 14, 14}, {15, 14, 14}, {15, 15, 14}, {15, 15, 15}, {16, 15, 15}, {16, 16, 15}, {16, 16, 16}, {17, 16, 16}, {17, 17, 16} },
	{ {17, 17, 17}, {18, 17, 17}, {18, 18, 17}, {18, 18, 18}, {19, 18, 18}, {19, 19, 18}, {19, 19, 19}, {20, 19, 19}, {20, 20, 19}, {20, 20, 20} },
	{ {21, 20, 20}, {21, 21, 20}, {21, 21, 21}, {22, 21, 21}, {22, 22, 21}, {22, 22, 22}, {23, 22, 22}, {23, 23, 22}, {23, 23, 23}, {24, 23, 23} },
	{ {24, 24, 23}, {24, 24, 24}, {25, 24, 24}, {25, 25, 24}, {25, 25, 25}, {26, 25, 25}, {26, 26, 25}, {26, 26, 26}, {27, 26, 26}, {27, 27, 26} },
	{ {27, 27, 27}, {28, 27, 27}, {28, 28, 27}, {28, 28, 28}, {29, 28, 28}, {29, 29, 28}, {29, 29, 29}, {30, 29, 29}, {30, 30, 29}, {30, 30, 30} },
	{ {31, 30, 30}, {31, 31, 30}, {31, 31, 31}, {32, 31, 31}, {32, 32, 31}, {32, 32, 32}, {33, 32, 32}, {33, 33, 32}, {33, 33, 33}, {34, 33, 33} }
};

static uint8_t colorArray[COLORARRAYLENGTH];

void convertColorsToColorArray() {
	// outputs each subpixel into colorArray
	int index = 0;
	for (int col = 0; col < COLS; col++) {
		for (int row = 0; row < ROWS; row++) {
			colorArray[index++] = colors[col][row].green;
			colorArray[index++] = colors[col][row].red;
			colorArray[index++] = colors[col][row].blue;
		}
	}
}




void set_volume_column(int col, int volume_level) {
	// leftmost column is 0, rightmost is 9. no leds lit is 0 volume, all lit is 10
	//if (col < 0 || col >= COLS) return; // Safety check
	//if (volume_level < 0 || volume_level > ROWS) return;
	if (col < 0 || col >= COLS) col = 0; // Safety check
	if (volume_level < 0 || volume_level > ROWS) volume_level = 3;
	
	// Clear the column first
	for (int row = 0; row < ROWS; row++) {
		colors[col][row].red = 0;
		colors[col][row].green = 0;
		colors[col][row].blue = 0;
	}
	
	if (volume_level != 0) {
		volume_level -= 1;
		
		for (int row = 0; row <= volume_level; row++) {
			if (row < 4) {
				colors[col][row].green = 75;        // Green zone
				} else if (row < 8) {
				colors[col][row].red = 75;          // Orange-ish   // 255
				colors[col][row].green = 48;    // 165
				} else {
				colors[col][row].red = 75;          // Red zone
			}
		}
	}	
}



__attribute__((always_inline)) static inline void send_high_bit() {
	REG_PIOA_SODR = PIO_PA22;
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
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	);
	REG_PIOA_CODR = PIO_PA22;
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
	REG_PIOA_SODR = PIO_PA22;
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
	);
	REG_PIOA_CODR = PIO_PA22;
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
	REG_PIOA_CODR = PIO_PA22;
	volatile int i = 0;
	while (i < 4200) { i += 1; }
}



