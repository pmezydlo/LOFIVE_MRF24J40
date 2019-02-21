#ifndef _FE310_SPI_H_
#define _FE310_SPI_H_

#include "spi.h"

// private fe310 protocol
struct fe310_spi_protocol {
	int (*init)(struct spi_protocol *spi);
	int (*transfer)(struct spi_protocol *spi, void *msg, uint32_t size);
	void *private_data;
};

#endif
