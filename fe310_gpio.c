#include "bits.h"
#include <errno.h>
#include "platform.h"
#include "fe310_gpio.h"
#include "gpio.h"

#define MAX_GPIO_NUMBER 23
#define MIN_GPIO_NUMBER 0

struct fe310_gpio {
	uint8_t gpio_number;
};

static int fe310_gpio_init(struct gpio_protocol *gpio, uint8_t gpio_number)
{
	struct fe310_gpio *fe310_private_data = ((struct fe310_gpio_protocol*)gpio)->private_data;
	if (gpio_number > MAX_GPIO_NUMBER || gpio_number < MIN_GPIO_NUMBER) {
		return -EINVAL;
	}
//	printf("init gpio: %d\n", fe310_private_data->gpio_number);

	fe310_private_data->gpio_number = gpio_number;
}

static int fe310_gpio_set_direction(struct gpio_protocol *gpio, uint8_t direction)
{
	struct fe310_gpio *fe310_private_data = ((struct fe310_gpio_protocol*)gpio)->private_data;

	//printf("gpio: %d\n", fe310_private_data->gpio_number);
//	printf("set direction %d\n", direction);


	if (direction == OUTPUT) {
		GPIO_REG(GPIO_INPUT_EN)  &= ~(0x1 << fe310_private_data->gpio_number);
		GPIO_REG(GPIO_OUTPUT_EN) |=  (0x1 << fe310_private_data->gpio_number);
	} else if (direction == INPUT) {
		GPIO_REG(GPIO_INPUT_EN)  |=  (0x1 << fe310_private_data->gpio_number);
		GPIO_REG(GPIO_OUTPUT_EN) &= ~(0x1 << fe310_private_data->gpio_number);
	} else {
		return -EINVAL;
	}
	return 0;
}

static int fe310_gpio_set_value(struct gpio_protocol *gpio, uint8_t value)
{
	struct fe310_gpio *fe310_private_data = ((struct fe310_gpio_protocol*)gpio)->private_data;
//	printf("gpio: %d\n", fe310_private_data->gpio_number);

//	printf("set value %d\n", value);

	if (value == 0) {
		GPIO_REG(GPIO_OUTPUT_VAL) &= ~(0x1 << fe310_private_data->gpio_number);
	} else if (value == 1) {
		GPIO_REG(GPIO_OUTPUT_VAL) |=  (0x1 << fe310_private_data->gpio_number);
	} else {
		return -EINVAL;
	}
	return 0;
}

struct gpio_protocol* install_gpio_protocol(void)
{
	struct fe310_gpio_protocol *fe310_gpio = malloc (sizeof (struct fe310_gpio_protocol));
	struct fe310_gpio *fe310_private_data;
	if (fe310_gpio != NULL) {
		fe310_gpio->init          = &fe310_gpio_init;
		fe310_gpio->set_value     = &fe310_gpio_set_value;
		fe310_gpio->set_direction = &fe310_gpio_set_direction;
	} else {
		return NULL;
	}

	fe310_private_data      = malloc (sizeof (fe310_gpio));
	if (fe310_private_data != NULL) {
		fe310_gpio->private_data = (void *)fe310_private_data;
	} else {
		free(fe310_gpio);
		return NULL;
	}

//	printf("install protocol\n");

	return (struct gpio_protocol*) fe310_gpio;
}

void uninstall_gpio_protocol(struct gpio_protocol* gpio)
{
	struct fe310_gpio_protocol *fe310_gpio = ((struct fe310_gpio_protocol*)gpio)->private_data;
	free(fe310_gpio->private_data);
	free(gpio);
}
