/*
 * led_interface.h
 *
 * Created: 4/22/2025 5:47:32 PM
 *  Author: latent
 */ 

#ifndef LED_INTERFACE_H_
#define LED_INTERFACE_H_

#include <asf.h>

#define LED_PIN IOPORT_CREATE_PIN(PIOA, 22)

void convertColorsToColorArray(void);
void set_volume_column(int col, int volume_level);
void send_colors(void);

#endif /* LED_INTERFACE_H_ */