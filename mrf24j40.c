#include <errno.h>
#include "debug.h"
#include "spi.h"
#include "gpio.h"
#include "bits.h"
#include "mrf24j40.h"
#include "mrf24j40_reg.h"
#include "platform.h"


static uint8_t mrf24j40_read_long_mem(struct spi_protocol* spi, uint16_t addr)
{
	uint8_t msg[3] = {0, 0, 0};

	msg[0] = (((addr >> N_MRF24J40_SPI_FRAME_LONG_ADDR_MS_BYTE) & B_MRF24J40_SPI_FRAME_LONG_ADDR_MS_BYTE) | B_MRF24J40_SPI_FRAME_LONG_ADDR_SPACE);
	msg[1] = ((addr & B_MRF24J40_SPI_FRAME_LONG_ADDR_LS_BYTE) << N_MRF24J40_SPI_FRAME_LONG_ADDR_LS_BYTE);
	msg[2] = 0; // container for data

	spi->transfer(spi, &msg, sizeof(msg));
	return msg[2];
}

static void mrf24j40_write_long_mem(struct spi_protocol* spi, uint16_t addr, uint8_t data)
{
	uint8_t msg[3] = {0, 0, 0};

	msg[0] = (((addr >> N_MRF24J40_SPI_FRAME_LONG_ADDR_MS_BYTE) & B_MRF24J40_SPI_FRAME_LONG_ADDR_MS_BYTE) | B_MRF24J40_SPI_FRAME_LONG_ADDR_SPACE);
	msg[1] = ((addr & B_MRF24J40_SPI_FRAME_LONG_ADDR_LS_BYTE) << N_MRF24J40_SPI_FRAME_LONG_ADDR_LS_BYTE) | B_MRF24J40_SPI_FRAME_LONG_ADDR_WRITE;
	msg[2] = data; // container for data

	spi->transfer(spi, &msg, sizeof(msg));
}

static uint8_t mrf24j40_read_short_mem(struct spi_protocol* spi, uint16_t addr)
{
	uint8_t msg[2] = {0, 0};

	msg[0] = ((addr & B_MRF24J40_SPI_FRAME_SHORT_ADDR) << N_MRF24J40_SPI_FRAME_SHORT_ADDR);
	msg[1] = 0; // container for data

	spi->transfer(spi, &msg, sizeof(msg));
	return msg[1];
}

static void mrf24j40_write_short_mem(struct spi_protocol* spi, uint8_t addr, uint32_t data)
{
	uint8_t msg[2] = {0, 0};

	msg[0] = ((addr & B_MRF24J40_SPI_FRAME_SHORT_ADDR) << N_MRF24J40_SPI_FRAME_SHORT_ADDR) | B_MRF24J40_SPI_FRAME_SHORT_ADDR_WRITE;
	msg[1] = data; // container for data

	spi->transfer(spi, &msg, sizeof(msg));
}

static void mrf24j40_read_fifo(struct spi_protocol* spi, uint16_t addr, uint8_t *buf, uint16_t size)
{
	uint16_t index;

	for (index = 0; index < size; index++) {
		buf[index] = mrf24j40_read_long_mem (spi, addr++);
	}
}

static void mrf24j40_write_fifo(struct spi_protocol* spi, uint16_t addr, uint8_t *buf, uint16_t size)
{
	uint16_t index;

	for (index = 0; index < size; index++) {
		mrf24j40_write_long_mem (spi, addr++, buf[index]);
	}
}

static void mrf24j40_hard_reset(struct gpio_protocol *reset_pin)
{
	reset_pin->set_value(reset_pin, 1);
	// delay_ms(2)
	reset_pin->set_value(reset_pin, 0);
	//delay_ms(2);
}

static void mrf24j40_rxfifo_flush()
{

}

static uint8_t mrf24j40_measure_signal_strength(struct spi_protocol *spi)
{
        uint8_t rssi;
        mrf24j40_write_short_mem(spi, R_MRF24J40_SM_BBREG6, (B_MRF24J40_SM_BBREG6_CALC_RSSI_INIT | B_MRF24J40_SM_BBREG6_CALC_RSSI_EN));

        while (!(mrf24j40_read_short_mem(spi, R_MRF24J40_SM_BBREG6) & B_MRF24J40_SM_BBREG6_CALC_RSSI_READY));
        rssi = mrf24j40_read_long_mem(spi, R_MRF24J40_LM_RSSI);

        return rssi;
}

static void mrf24j40_soft_reset(struct spi_protocol *spi)
{
        mrf24j40_write_short_mem(spi, R_MRF24J40_SM_SOFTRST, B_MRF24J40_SM_SOFTRST_COMPLETE);
	while (mrf24j40_read_short_mem(spi, R_MRF24J40_SM_SOFTRST) & B_MRF24J40_SM_SOFTRST_COMPLETE != 0); // wait for soft reset to finish
}

static void mrf24j40_set_pan_id(struct spi_protocol *spi, uint16_t pan_id)
{
        mrf24j40_write_short_mem(spi, R_MRF24J40_SM_PANIDH, pan_id >> 8);
        mrf24j40_write_short_mem(spi, R_MRF24J40_SM_PANIDL, pan_id);
}

static void mrf24j40_rf_reset(struct spi_protocol *spi)
{
        uint8_t reg8 = mrf24j40_read_short_mem(spi, R_MRF24J40_SM_RFCTL);

        mrf24j40_write_short_mem(spi, R_MRF24J40_SM_RFCTL, (reg8 | B_MRF24J40_SM_RFCTL_RFRST));
        mrf24j40_write_short_mem(spi, R_MRF24J40_SM_RFCTL, (reg8 & ~B_MRF24J40_SM_RFCTL_RFRST));
        //delay
}

static uint8_t mrf24j40_set_channel(struct spi_protocol *spi, uint8_t channel)
{
        uint8_t reg8;

        if (channel < 11 || channel > 26)
            return -EINVAL;

        reg8 = mrf24j40_read_long_mem(spi, R_MRF24J40_LM_RFCON0);

        mrf24j40_write_long_mem(spi, R_MRF24J40_LM_RFCON0, (reg8 | (channel - 11) << N_MRF24J40_LM_RFCON0_CHANNEL));

        mrf24j40_rf_reset(spi);
        return 0; // return channel value from mrf24j40
}

void mrf24j40_init(struct spi_protocol *spi, struct gpio_protocol *reset_pin, struct gpio_protocol *wake_pin)
{
	DEBUG("MRF24J40 initializing.\n");

	reset_pin->set_direction(reset_pin, OUTPUT);
	wake_pin->set_direction(wake_pin, OUTPUT);

        mrf24j40_hard_reset(reset_pin);

	DEBUG("MRF24J40 hard reset performed.\n");
        // SOFTRST (0x2A) = 0x07 – Perform a software Reset. The bits will be automatically cleared to ‘0’ by hardware.
        mrf24j40_soft_reset(spi);

	DEBUG("MRF24J40 soft reset performed.\n");

        // delay(192 us)

        // PACON2 (0x18) = 0x98 – Initialize FIFOEN = 1 and TXONTS = 0x6.
        mrf24j40_write_short_mem(spi, R_MRF24J40_SM_PACON2, (B_MRF24J40_SM_PACON2_FIFOEN | (V_MRF24j40_SM_PACON2_TXONTS << N_MRF24J40_SM_PACON2_TXONTS)));

        // TXSTBL (0x2E) = 0x95 – Initialize RFSTBL = 0x9.
        mrf24j40_write_short_mem(spi, R_MRF24J40_SM_TXSTBL, ((V_MRF24J40_SM_TXSTBL_RFSTBL << N_MRF24J40_SM_TXSTBL_RFSTBL) | V_MRF24J40_SM_TXSTBL_MSIFS));

        // RFCON0 (0x200) = 0x03 – Initialize RFOPT = 0x03
        mrf24j40_write_long_mem(spi, R_MRF24J40_LM_RFCON0, V_MRF24J40_LM_RFOCN0_RFOPT);

        // RFCON1 (0x201) = 0x01 – Initialize VCOOPT = 0x02.
        mrf24j40_write_long_mem(spi, R_MRF24J40_LM_RFCON1, V_MRF24J40_LM_RFCON1_VCOOPT);

        // RFCON2 (0x202) = 0x80 – Enable PLL (PLLEN = 1)
        mrf24j40_write_long_mem(spi, R_MRF24J40_LM_RFCON2, B_MRF24J40_LM_RFCON2_PLLEN);

        // RFCON6 (0x206) = 0x90 – Initialize TXFIL = 1 and 20MRECVR = 1
        mrf24j40_write_long_mem(spi, R_MRF24J40_LM_RFCON6, (B_MRF24J40_LM_RFCON6_TXFIL | B_MRF24J40_LM_RFCON6_20MRECVR));

        // RFCON7 (0x207) = 0x80 – Initialize SLPCLKSEL = 0x2 (100 kHz Internal oscillator).
        mrf24j40_write_long_mem(spi, R_MRF24J40_LM_RFCON7, V_MRF24J40_LM_RFCON7_SLPCLKSEL_100KHZ);

        // RFCON8 (0x208) = 0x10 – Initialize RFVCO = 1
        mrf24j40_write_long_mem(spi, R_MRF24J40_LM_RFCON8, B_MRF24J40_LM_RFCON8_RFVCO);

        //
        mrf24j40_write_long_mem(spi, R_MRF24J40_LM_SLPCON0, B_MRF24J40_LM_SLPCON0_SLPCLKEN);  // is it required

        // SLPCON1 (0x220) = 0x21 – Initialize ~CLKOUTEN = 1 and SLPCLKDIV = 0x01.
        mrf24j40_write_long_mem(spi, R_MRF24J40_LM_SLPCON1, (B_MRF24J40_LM_SLPCON1_CLKOUTEN | V_MRF24J40_LM_SLPCON1_SLPCLKDIV));

        // BBREG2 (0x3A) = 0x80 – Set CCA mode to ED.
        mrf24j40_write_short_mem(spi, R_MRF24J40_SM_BBREG2, ((V_MRF24J40_SM_BBREG2_CCACSTH << N_MRF24J40_SM_BBREG2_CCACSTH) | (V_MRF24J40_SM_BBREG2_CCAMODE << N_MRF24J40_SM_BBREG2_CCAMODE)));

        // CCAEDTH = 0x60 – Set CCA ED threshold.
        mrf24j40_write_short_mem(spi, R_MRF24J40_SM_CCAEDTH, V_MRF24J40_SM_BBREG2_CCAMODE);

        // BBREG6 (0x3E) = 0x40 – Set appended RSSI value to RXFIFO.
        mrf24j40_write_short_mem(spi, R_MRF24J40_SM_BBREG6, B_MRF24J40_SM_BBREG6_CALC_RSSI_EN);

        //
        mrf24j40_write_short_mem(spi, R_MRF24J40_SM_INTCON, (B_MRF24J40_SM_INTCON_SLPIE | B_MRF24J40_SM_INTCON_WAKEIE | B_MRF24J40_SM_INTCON_HSYMTMRIE |
          B_MRF24J40_SM_INTCON_SECIE | B_MRF24J40_SM_INTCON_TXG2IE | B_MRF24J40_SM_INTCON_TXG1IE | B_MRF24J40_SM_INTCON_TXNIE));

	DEBUG("MRF24J40 Primary configuration applied\n");

        mrf24j40_set_pan_id(spi, 0x9923);

        mrf24j40_set_channel(spi, 11);

        //mrf24j40_rxfifo_flush(spi);
        mrf24j40_rf_reset(spi);
}
