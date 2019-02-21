#include <stdint.h>
#include "platform.h"
#include "encoding.h"
#include <unistd.h>
#include "spi.h"
#include "debug.h"
#include "gpio.h"
#include "mrf24j40.h"


#define MRF_RESET_PIN 23
#define MRF_WAKE_PIN  18

void ClockInit()
{
    // Make sure the HFROSC is on before the next line:
    PRCI_REG(PRCI_HFROSCCFG) |= ROSC_EN(1);
    // Run off 16 MHz Crystal for accuracy. Note that the
    // first line is
    PRCI_REG(PRCI_PLLCFG) = (PLL_REFSEL(1) | PLL_BYPASS(1));
    PRCI_REG(PRCI_PLLCFG) |= (PLL_SEL(1));
    // Turn off HFROSC to save power
    PRCI_REG(PRCI_HFROSCCFG) &= ~(ROSC_EN(1));
}

void UartInit()
{
    // Configure UART to print
    GPIO_REG(GPIO_OUTPUT_VAL) |= IOF0_UART0_MASK;
    GPIO_REG(GPIO_OUTPUT_EN)  |= IOF0_UART0_MASK;
    GPIO_REG(GPIO_IOF_SEL)    &= ~IOF0_UART0_MASK;
    GPIO_REG(GPIO_IOF_EN)     |= IOF0_UART0_MASK;

    // 115200 Baud Rate
    UART0_REG(UART_REG_DIV) = 138;
    UART0_REG(UART_REG_TXCTRL) = UART_TXEN;
    UART0_REG(UART_REG_RXCTRL) = UART_RXEN;
}

int main (void)
{
	ClockInit();
	UartInit();

	struct spi_protocol *mrf_spi = install_spi_protocol();
	mrf_spi->init(mrf_spi);

	struct gpio_protocol *mrf_reset_pin = install_gpio_protocol();
	mrf_reset_pin->init(mrf_reset_pin, MRF_RESET_PIN);

	struct gpio_protocol *mrf_wake_pin = install_gpio_protocol();
	mrf_wake_pin->init(mrf_wake_pin, MRF_WAKE_PIN);

	mrf24j40_init(mrf_spi, mrf_reset_pin, mrf_wake_pin);

	mrf_reset_pin->set_direction(mrf_reset_pin, OUTPUT);

  while(1) {

  }
}
