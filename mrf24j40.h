#ifndef _MRF24J40_H_
#define _MRF24J40_H_

void mrf24j40_init(struct spi_protocol *spi, struct gpio_protocol *reset_pin, struct gpio_protocol *wake_pin);

#endif
