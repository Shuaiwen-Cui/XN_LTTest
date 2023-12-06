#include "trx_access.h"
#include "delay.h"

// === Pins ================================================================
// Pin config: port, pin number, pin mode, function mode (description)
#define SPI_SCLK_CONF		0xF, 0, SSP_IO, FUNC0	// SSP0_SCLK PF_0
#define SPI_MISO_CONF		0xF, 2, SSP_IO, FUNC2	// SSP0_MISO PF_2
#define SPI_MOSI_CONF		0xF, 3, SSP_IO, FUNC2	// SSP0_MOSI PF_3

#define RF233_CS_CONF		0xF,  1, MD_PLN, FUNC4 // SSP0_SSEL PF_1 => GPIO7[16]
#define RF233_CS_DIR		7, 1 << 16, 1
#define RF233_RSTN_CONF		0x6,  2, MD_PLN, FUNC0	// P6_2 => GPIO3[1]
#define RF233_RSTN_DIR		3, 1 << 1, 1
#define RF233_SLP_CONF		0x6,  7, MD_BUK, FUNC4 // P6_7 => GPIO5[15]
#define RF233_SLP_DIR		5, 1 << 15, 1
#define RF233_INT_CONF		0x6,  1, GPIO_NOPULL, FUNC0 // P6_1 => GPIO3[0]
#define RF233_INT_DIR		3, 1 << 0, 0

#define RAMP_CSD_CONF		0xB,  5, MD_PUP, FUNC4 // LCD_VD14 PB_5 => GPIO5[25]
#define RAMP_CSD_DIR		5, 1 << 25, 1
#define RAMP_CPS_CONF		0xB,  2, MD_PUP, FUNC4 // LCD_VD21 PB_2 => GPIO5[22]
#define RAMP_CPS_DIR		5, 1 << 22, 1
#define RAMP_ASLECT_CONF	0x7,  1, MD_PUP, FUNC0 // LCD_VD19 P7_1 => GPIO3[9]
#define RAMP_ASLECT_DIR		3, 1 << 9, 1

#define ADS131_CS_CONF		0x3,  8, MD_PDN, FUNC4	// SPIFI_CS P3_8 => GPIO5[11]
#define ADS131_CS_DIR		5, 1 << 11, 1

void rf233_arch_init(void)
{
	// Configure and disable ADC CS
	scu_pinmux(ADS131_CS_CONF);
	GPIO_SetDir(ADS131_CS_DIR);
	GPIO_SetValue(5, 1 << 11);

	// Configure pins for SPI
	scu_pinmux(SPI_SCLK_CONF);
	scu_pinmux(SPI_MISO_CONF);
	scu_pinmux(SPI_MOSI_CONF);
	scu_pinmux(RF233_CS_CONF);
	GPIO_SetDir(RF233_CS_DIR);

	// Configure other radio pins
	scu_pinmux(RF233_INT_CONF);
	scu_pinmux(RF233_RSTN_CONF);
	scu_pinmux(RF233_SLP_CONF);
	GPIO_SetDir(RF233_INT_DIR);
	GPIO_SetDir(RF233_RSTN_DIR);
	GPIO_SetDir(RF233_SLP_DIR);

	// Configure front end
	scu_pinmux(RAMP_CSD_CONF);
	scu_pinmux(RAMP_CPS_CONF);
	scu_pinmux(RAMP_ASLECT_CONF);
	GPIO_SetDir(RAMP_CSD_DIR);
	GPIO_SetDir(RAMP_CPS_DIR);
	GPIO_SetDir(RAMP_ASLECT_DIR);

	// Init pins
	CS_HIGH();
	CSD_HIGH();
	CPS_HIGH();
	
	ASLECT_LOW();
}

void rf233_arch_reset(void)
{
	/* Ensure control lines have correct levels. */
	RST_HIGH();
	SLP_TR_LOW();

	/* Wait typical time of timer TR1. */
	delay_us(330);

	RST_LOW();
	delay_us(10);
	RST_HIGH();
	CSD_HIGH();
	CPS_HIGH();
}

void rf233_arch_sleep(void)
{
	CSD_LOW();
	CPS_LOW();
	SLP_TR_HIGH();
}

void rf233_arch_wake(void)
{
  /*
   * Triggers a radio state transition - assumes that the radio already is in
   * state SLEEP or DEEP_SLEEP and SLP_TR pin is low. Refer to datasheet 6.6.
   *
   * Note: this is the only thing that can get the radio from state SLEEP or
   * state DEEP_SLEEP!
   */
	SLP_TR_LOW();
  CSD_HIGH();
	CPS_HIGH();
}
