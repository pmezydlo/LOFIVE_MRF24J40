#include "fe310_spi.h"
#include "spi.h"
#include "bits.h"
#include <errno.h>
#include "platform.h"

#define R_FE310_SPI0_BASE_ADDR   0x10014000
#define R_FE310_SPI1_BASE_ADDR   0x10024000

#define R_FE310_SCKDIV           0x00
#define B_FE310_SCKDIV_MASK      0xFFF
#define R_FE310_SCKMODE          0x04
#define B_FE310_SCKMODE_PHA      BIT0
#define B_FE310_SCKOMDE_POL      BIT1
#define R_FE310_CSID             0x10
#define R_FE310_CSDEF            0x14
#define R_FE310_CSMODE           0x18
#define V_FE310_CSMODE_AUTO      0x00
#define V_FE310_CSMODE_HOLD      0x02
#define V_FE310_CSMODE_OFF       0x03
#define R_FE310_DELAY0           0x28
#define B_FE310_DELAY0_CSSCK     0xFF
#define N_FE310_DELAY0_CSSCK     0
#define B_FE310_DELAY0_SCKCS     0xFF
#define N_FE310_DELAY0_SCKCS     16
#define R_FE310_DELAY1           0x2C
#define B_FE310_DELAY1_INTERCS   0xFF
#define N_FE310_DELAY1_INTERCS   0
#define B_FE310_DELAY1_INTERXFR  0xFF
#define N_FE310_DELAY1_INTERXFR  16
#define R_FE310_FMT              0x40
#define B_FE310_FMT_PROTO        (BIT1 | BIT0)
#define V_FE310_FMT_PROTO_SINGLE 0x0
#define V_FE310_FMT_PROTO_DUAL   0x1
#define V_FE310_FMT_PROTO_QUAD   0x2
#define B_FE310_FMT_ENDIAN       BIT2
#define B_FE310_FMT_DIR          BIT3
#define B_FE310_FMT_LEN          0xF
#define N_FE310_FMT_LEN          16
#define R_FE310_TXDATA           0x48
#define B_FE310_TXDATA_DATA      0xFF
#define B_FE310_TXDATA_FULL      BIT31
#define R_FE310_RXDATA           0x4C
#define B_FE310_RXDATA_DATA      0xFF
#define B_FE310_RXDATA_EMPTY     BIT31
#define R_FE310_TXMARK           0x50
#define R_FE310_RXMARD           0x54
#define R_FE310_FCTRL            0x60
#define R_FE310_FFMT             0x64
#define R_FE310_IE               0x70
#define R_FE310_IP               0x74

struct fe310_spi {
	int spi_number;
};

#define IDLE asm volatile ("")
#define BIT(x) (1<<(x))

#define MAX_SPI_FREQ (10000000)

#define IOF_SPI1_SS0          (2u)
#define IOF_SPI1_MOSI         (3u)
#define IOF_SPI1_MISO         (4u)
#define IOF_SPI1_SCK          (5u)

static int fe310_spi_init(struct spi_protocol *spi) {
	struct fe310_spi *fe310_private_data = ((struct fe310_spi_protocol*)spi)->private_data;

	printf("SpiInit()\n");

GPIO_REG(GPIO_OUTPUT_EN)    &= ~(BIT(IOF_SPI1_MISO));
GPIO_REG(GPIO_PULLUP_EN)    &= ~(BIT(IOF_SPI1_MISO));
  GPIO_REG(GPIO_INPUT_EN)    &= ~(BIT(IOF_SPI1_SS0)|BIT(IOF_SPI1_MOSI)|BIT(IOF_SPI1_SCK));
  GPIO_REG(GPIO_INPUT_EN)    |= BIT(IOF_SPI1_MISO);
  GPIO_REG(GPIO_OUTPUT_EN)   |=  BIT(IOF_SPI1_SS0)|BIT(IOF_SPI1_MOSI)|BIT(IOF_SPI1_SCK);
    // Select IOF SPI1.MOSI [SDIN] and SPI1.SCK [SLCK] and SPI1.SS0 [DC]
    GPIO_REG(GPIO_IOF_SEL)    &= ~(BIT(IOF_SPI1_MOSI) | BIT(IOF_SPI1_SCK) | BIT(IOF_SPI1_SS0) | BIT(IOF_SPI1_MISO));
    GPIO_REG(GPIO_IOF_EN )    |=  (BIT(IOF_SPI1_MOSI) | BIT(IOF_SPI1_SCK) | BIT(IOF_SPI1_SS0) | BIT(IOF_SPI1_MISO));
//GPIO_REG(GPIO_OUTPUT_VAL) |=  (BIT(IOF_SPI1_MOSI) | BIT(IOF_) | BIT(OLED_DC)); /* is this necessary? */
        // Select chip
    //GPIO_REG(GPIO_OUTPUT_VAL)  &= ~BIT(OLED_CS);

    // Set up SPI controller
    /** SPI clock divider: determines the speed of SPI
     * transfers. This cannot exceed 10Mhz for the SSD1306.
     * CPUfreq is set to 16Mhz in this demo.
     * The formula is CPU_FREQ/(1+SPI_SCKDIV)
     */


    SPI1_REG(SPI_REG_SCKDIV)    = 4;//(get_cpu_freq() / MAX_SPI_FREQ) - 1;



    SPI1_REG(SPI_REG_SCKMODE)   = 0; /* pol and pha both 0 - SCLK is active-high, */
    SPI1_REG(SPI_REG_CSID)      = 1; /* CS 0 */
    SPI1_REG(SPI_REG_CSDEF)     = 0xffff; /* CS is active-low */
    SPI1_REG(SPI_REG_CSMODE)    = SPI_CSMODE_HOLD; /* hold CS where possible */
     SPI1_REG(SPI_REG_DCSSCK)    = 0;
     SPI1_REG(SPI_REG_DSCKCS)    = 0;
    /* SPI1_REG(SPI_REG_DINTERCS)  = */
    /* SPI1_REG(SPI_REG_DINTERXFR) = */
    SPI1_REG(SPI_REG_FMT)       =   SPI_FMT_LEN(8);
    SPI1_REG(SPI_REG_TXCTRL)    = 1; /* interrupt when <1 in tx fifo (completion) */
     SPI1_REG(SPI_REG_RXCTRL)    = 1;//*/
    /* SPI1_REG(SPI_REG_IE)        = */

	return 0;
}
static int fe310_spi_transfer(struct spi_protocol *spi, void *buf, uint32_t size)
{
	struct fe310_spi *fe310_private_data = ((struct fe310_spi_protocol*)spi)->private_data;
	uint8_t *buffer = (uint8_t *)buf;
	volatile int32_t x;
	uint8_t r,d;
	uint32_t count = size;
	uint32_t index;

	printf("SpiTransfer() - start\n");
	if (size == 0)
		return -EMSGSIZE;

	if (buf == NULL)
		return -ENOMEM;

	SPI1_REG(SPI_REG_CSID)      = 0; /* CS 0 */

	for (index = 0;index < size;index++) {
		while (SPI1_REG(SPI_REG_TXFIFO) & SPI_TXFIFO_FULL);
		printf ("tx:%x\n", *buffer);
		SPI1_REG(SPI_REG_TXFIFO) = *buffer;

		// Prepare next byte
		//d = *(buffer+1);

		while (!((SPI1_REG(SPI_REG_RXFIFO)) & SPI_RXFIFO_EMPTY)) {
		//printf ("rx:%x\n", SPI1_REG(SPI_REG_RXFIFO));
		}

		*buffer = SPI1_REG(SPI_REG_RXFIFO) & 0xFF;
		buffer++;
	}

	SPI1_REG(SPI_REG_CSID)      = 1; /* CS 1 */
	printf("SpiTransfer() - stop\n");
	return 0;
}

struct spi_protocol* install_spi_protocol(void)
{
	struct fe310_spi_protocol *fe310_spi = malloc (sizeof (struct fe310_spi_protocol));
	struct fe310_spi *fe310_private_data;
	if (fe310_spi != NULL) {
		fe310_spi->init         = &fe310_spi_init;
		fe310_spi->transfer     = &fe310_spi_transfer;
	} else {
		return NULL;
	}

	fe310_private_data      = malloc (sizeof (fe310_spi));
	if (fe310_private_data != NULL) {
		fe310_spi->private_data = (void *)fe310_private_data;
	} else {
		free(fe310_spi);

		return NULL;
	}

	return (struct spi_protocol*) fe310_spi;
}

void uninstall_spi_protocol (struct spi_protocol* spi)
{
	struct fe310_spi_protocol *fe310_spi = ((struct fe310_spi_protocol*)spi)->private_data;
	free(fe310_spi->private_data);
	free(spi);
}
