#ifndef _SPI_H_
#define _SPI_H_

#include <stdint.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>

struct spi_protocol {
	int (*init)(struct spi_protocol *spi);
	int (*transfer)(struct spi_protocol *spi, void *msg, uint32_t size);
};

// private fe310 protocol
struct fe310_spi_protocol {
	int (*init)(struct spi_protocol *spi);
	int (*transfer)(struct spi_protocol *spi, void *msg, uint32_t size);
	void *private_data;
};

struct spi_protocol *install_spi_protocol(void);
void uninstall_spi_protocol (struct spi_protocol* spi);

#endif
