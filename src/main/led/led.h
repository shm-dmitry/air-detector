#ifndef MAIN_LED_LED_H_
#define MAIN_LED_LED_H_

#include "stdint.h"

void led_init();
void led_set_color(uint32_t rgbw);

void led_set_override_color(uint32_t rgbw);
void led_reset_override_color();

#endif /* MAIN_LED_LED_H_ */
