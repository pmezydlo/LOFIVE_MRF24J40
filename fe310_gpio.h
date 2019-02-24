#ifndef _FE310_GPIO_H_
#define _FE310_GPIO_H_

#include <stdint.h>
#include "gpio.h"

struct fe310_gpio_protocol {
	int (*init)(struct gpio_protocol *gpio, uint8_t gpio_number);
	int (*set_value)(struct gpio_protocol *gpio, uint8_t value);
	uint8_t (*get_value)(struct gpio_protocol *gpio);
	int (*set_direction)(struct gpio_protocol *gpio, uint8_t direction);
	void *private_data;
};


#endif
