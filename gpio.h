#ifndef _GPIO_H_
#define _GPIO_H_

#include <stdint.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>

struct gpio_protocol {
	int (*init)(struct gpio_protocol *gpio, uint8_t gpio_number);
	int (*set_value)(struct gpio_protocol *gpio, uint8_t value);
	uint8_t (*get_value)(struct gpio_protocol *gpio);
	int (*set_direction)(struct gpio_protocol *gpio, uint8_t direction);
};

enum Direction {
	OUTPUT = 0,
	INPUT  = 1
};

struct gpio_protocol *install_gpio_protocol(void);
void uninstall_gpio_protocol (struct gpio_protocol* gpio);

#endif
