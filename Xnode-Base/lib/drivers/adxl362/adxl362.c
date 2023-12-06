/*************************************************
*@File        adxl362.c
*@Brief       Mainfile for driver of adxl362 wake-up sensor
*@Version     1.1
*@Date        01/02/17
*@Author      Yuguang Fu
**************************************************/

#include <xnode.h>
#undef SUCCESS
#include "adxl362.h"
#include "spi.h"
#include <lpc43xx.h>
#include <lpc43xx_scu.h>
#include <lpc43xx_cgu.h>
#include <lpc43xx_timer.h>
#include <lpc43xx_gpio.h>
#include <led.h>
#include <string.h>

#define _DEBUG_               0
#if _DEBUG_
#define PRINTF(...)           lpc_printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

/* set activity detection flag */
volatile uint8_t ADXL362_IACTD=0;	   // 0 idle, 1 activity, 2 inactivity

/********************************************************************
	                 Private function
*********************************************************************/

const char *byte_to_binary(int x)
{
    static char b[9];
    int z;
	  b[0] = '\0';
    for (z = 128; z > 0; z >>= 1)
    {
        strcat(b, ((x & z) == z) ? "1" : "0");
    }

    return b;
}

/*------ ADXL362 Write Function ------*/
void adxl362_write8(uint8_t addr, uint8_t val)
{
	ADXL362_CS_LOW();
	SPI_SendByte(ADEXL362_SPI, ADXL362_SPI_WRITE);
	SPI_SendByte(ADEXL362_SPI, addr);
	SPI_SendByte(ADEXL362_SPI, val);
	ADXL362_CS_HIGH();
}

uint8_t adxl362_read8(uint8_t addr)
{
	uint8_t val=0;
	ADXL362_CS_LOW();
	SPI_SendByte(ADEXL362_SPI, ADXL362_SPI_READ);
	SPI_SendByte(ADEXL362_SPI, addr);
	SPI_TransferByte(ADEXL362_SPI, 0x00, &val);
	ADXL362_CS_HIGH();
	return val;
}

void adxl362_write12(uint8_t addr, uint16_t val)
{
	uint8_t val_H=val>>8;
	uint8_t val_L=val;

	ADXL362_CS_LOW();
	SPI_SendByte(ADEXL362_SPI, ADXL362_SPI_WRITE);
	SPI_SendByte(ADEXL362_SPI, addr);
	SPI_SendByte(ADEXL362_SPI, val_L);
	SPI_SendByte(ADEXL362_SPI, val_H);
	ADXL362_CS_HIGH();
}

uint16_t adxl362_read12(uint8_t addr)
{
	uint8_t val_L=0, val_H=0;
	uint16_t val=0;
	ADXL362_CS_LOW();
	SPI_SendByte(ADEXL362_SPI, ADXL362_SPI_READ);
	SPI_SendByte(ADEXL362_SPI, addr);
	SPI_TransferByte(ADEXL362_SPI, 0x00, &val_L);
	SPI_TransferByte(ADEXL362_SPI, 0x00, &val_H);
	val=val_L+(val_H<<8);
	ADXL362_CS_HIGH();
	return val;
}

/*---------- configure SPI and pins -----------*/

void ADXL362_pin_init(void)
{
	scu_pinmux(SPI_SCLK_CONF);
	scu_pinmux(SPI_MISO_CONF);
	scu_pinmux(SPI_MOSI_CONF);
	scu_pinmux(ADXL362_CS_CONF);
	GPIO_SetDir(ADXL362_CS_DIR);
	scu_pinmux(ADXL362_IACTD_CONF);
	GPIO_SetDir(ADXL362_IACTD_DIR);
}

/*---------- set ADXL362 interrupt pin ----------*/

void adxl362_int_init(void)
{
	#define SCU_PINTSEL1_INTPIN6_Pos                    16                 // SCU PINTSEL1: INTPIN6 Position
	#define SCU_PINTSEL1_PORTSEL6_Pos                   21                 // SCU PINTSEL1: PORTSEL6 Position


	//LPC_SCU->PINTSEL1 &= (0x00 << SCU_PINTSEL1_INTPIN6_Pos);
	LPC_SCU->PINTSEL1 |= (6 << SCU_PINTSEL1_PORTSEL6_Pos) | (23 << SCU_PINTSEL1_INTPIN6_Pos);

	// Set interrupt mode
	LPC_GPIO_PIN_INT->ISEL = 0;              // sets pin interrupt mode to edge sensitive
	//LPC_GPIO_PIN_INT->SIENF = (1 << 6);      // set pin interrupt enable falling edge
	LPC_GPIO_PIN_INT->SIENR = (1 << 6);      // set pin interrupt enable rising edge  , consider it to be active high

	NVIC_SetPriority(PIN_INT6_IRQn, ADXL362_INT_PRIORITY);
}


/*---------- enable ADXL362 interrupt pin ----------*/
void adxl362_int_enable(void)
{
	NVIC_EnableIRQ(PIN_INT6_IRQn);
}

/*---------- disable ADXL362 interrupt pin ----------*/
void adxl362_int_disable(void)
{
	NVIC_DisableIRQ(PIN_INT6_IRQn);
}


/*---------- ADXL362 IRQ function ----------*/

void GPIO6_IRQHandler(void)
{
	if (LPC_GPIO_PIN_INT->RISE & (1 << 6)) {
		// Clear interrupt
		//LPC_GPIO_PIN_INT->IST = (1 << 7);
		LPC_GPIO_PIN_INT->RISE = (1 << 6);
		// Inactivity detection
		ADXL362_IACTD = 1;
		LED_RGB(1,0,0);
		//NodeReset();
	}
}

void adxl362_clearirq(void)
{
	adxl362_read8(ADXL362_REG_STATUS);
}

int adxl362_motiondetection(void)
{
	uint8_t inact_ctl_reg;
	inact_ctl_reg = adxl362_read8(ADXL362_REG_STATUS);
	inact_ctl_reg &= (0x10);
	return inact_ctl_reg;
}

void adxl362_disableint(Bool int_pin)
{
	uint8_t int_pin_addr;
	uint8_t int_mode_val;
	if (int_pin) {
		int_pin_addr = ADXL362_REG_INTMAP1;  // INT1
	} else {
		int_pin_addr = ADXL362_REG_INTMAP2;  // INT2
	}
	int_mode_val = ADXL362_CLR_INT_MODE;  // no interrupt mapped, default (reset)
	adxl362_write8(int_pin_addr, int_mode_val);
}

// This function is for read GPIO, as alternative way of Interrupt function.
/*
uint32_t adxl362_get_drdy(void)
{
	uint32_t ttp=0;
	ttp = GPIO_ReadValue(5);
	ttp = ttp & (1<<10);
	return ttp;
}
*/

/********************************************************************
	                 Public function
*********************************************************************/

/*---------- Initialize ADXL362 ----------*/
Status adxl362_init(Bool reset)
{
	ADXL362_IACTD=0;
	adxl362_int_disable();
	ADXL362_pin_init();

	// spi init  SSP_CPHA_SECOND, SSP_CPOL_LO
	if (SPI_Init(ADEXL362_SPI, SSP_CPHA_FIRST, SSP_CPOL_HI, ADEXL362_FSCLK, SSP_DATABIT_8,
	SSP_MASTER_MODE) != SUCCESS) {
		return ERROR;
	}

	ADXL362_CS_HIGH();
	LED_RGB(1,0,0);

	if(reset) {
		TIM_Waitms(10);
		// soft reset
		adxl362_write8(ADXL362_REG_SOFT_RESET, ADXL362_SOFTRESET_EN);
		TIM_Waitms(10);
	}

	return SUCCESS;
}

void adxl362_init_noINT(void)
{
	ADXL362_pin_init();
	if (SPI_Init(ADEXL362_SPI, SSP_CPHA_FIRST, SSP_CPOL_HI, ADEXL362_FSCLK, SSP_DATABIT_8,
	SSP_MASTER_MODE) != SUCCESS) {
		PRINTF("init error\r\n");
	}
	adxl362_write8(ADXL362_REG_SOFT_RESET, ADXL362_SOFTRESET_EN);
	TIM_Waitms(10);
	adxl362_int_setup(TRUE, FALSE, 1);
}

/*---------- Read ID ----------*/  //by default, the expected value is AD
void adxl362_readid(void)
{
	uint8_t id=0;
	id=adxl362_read8(0x00);
	PRINTF("id is %u\r\n", id);

	if(id==173)
	{
		LED_RGB(0,0,1);
		TIM_Waitms(1000);
	}

	PRINTF("%s\r\n", byte_to_binary(id));
}

/*---------- Start ADXL362 sensing ----------*/
void adxl362_startSensing(void)
{
	uint8_t mode;
	mode = adxl362_read8(ADXL362_REG_POWER_CTL);

	// measurement mode in normal noise: 1.8uA
	/*
	mode = mode | (0x02);
	mode = mode & (0x7E);
	*/
	// measurement mode in ultra low noise: 13uA
	mode = mode | (0x22);
	mode = mode & (0x6E);

	adxl362_write8(ADXL362_REG_POWER_CTL, mode);
	TIM_Waitms(10);

	mode = adxl362_read8(ADXL362_REG_POWER_CTL);
	PRINTF("start sensing, Power_Ctl is %u\r\n", mode);
}

/*---------- Stop ADXL362 sensing ----------*/
void adxl362_stopSensing(void)
{
	uint8_t mode;
	mode = adxl362_read8(ADXL362_REG_POWER_CTL);
	mode = mode & (0x7C);
	adxl362_write8(ADXL362_REG_POWER_CTL, mode);
	TIM_Waitms(10);

	mode = adxl362_read8(ADXL362_REG_POWER_CTL);
	PRINTF("stop sensing, Power_Ctl is %u\r\n", mode);
}

/*---------- ADXL362 selt test ----------*/
/*
Status adxl362_selfTest(uint8_t range)
{
	uint8_t i;
	int16_t Data1[4]={0};
	int16_t Data2[4]={0};
	int16_t Diff[3]={0};

	adxl362_readDataXYZT(Data1);

	adxl362_write8(ADXL362_REG_SELF_TST, 0x01);
	TIM_Waitms(10);

	adxl362_readDataXYZT(Data2);
	for (i=0; i<3; i++)
	{
	   Diff[i]=(Data2[i]-Data1[i])*(range/2); //uint mg
	   lpc_printf("Difference of %u axis: %umg\r\n", i+1, Diff[i]);
	}

	if ( Diff[0]<200 || Diff[0]>2800 || Diff[1] <-2800 || Diff[1]>-200 || Diff[2]<200 || Diff[2]>2800)
	   return ERROR;
	else
     return SUCCESS;

}
*/

/*---------- Read XYZ and temperature data ----------*/
Status adxl362_readDataXYZT(int16_t *pData)
{
	uint8_t i, j;
	uint8_t temp[8];

	ADXL362_CS_LOW();
	// get data, start from XDATA_L
	SPI_SendByte(ADEXL362_SPI, ADXL362_SPI_READ);
	SPI_SendByte(ADEXL362_SPI, ADXL362_REG_XDATA_L);

	for (i=0; i<8; i++)
		SPI_TransferByte(ADEXL362_SPI, 0x00, &temp[i]);

	// parse data
	for (i=0, j=0; i<8; i+=2, j++)
		pData[j]=((int16_t)(temp[i])+((int16_t)(temp[i+1])<<8));

	ADXL362_CS_HIGH();
	PRINTF("XData: %d, YData: %d, ZData: %d\r\n", pData[0], pData[1], pData[2]);
	return SUCCESS;
}

/*---------- Set up activity detection ----------*/
void adxl362_act_int(uint16_t thres, uint16_t time, Bool abs)
{
	uint8_t act_ctl_reg;

	PRINTF("thres is %u, time is %u, abs is %u\r\n", thres, time, abs);
	PRINTF("thres_act reset value is %u\r\n", adxl362_read12(ADXL362_REG_THRESH_ACT_L));
	PRINTF("time_act reset value is %u\r\n", adxl362_read8(ADXL362_REG_TIME_ACT));

	// set up motion and time for activity waking up

	adxl362_write12(ADXL362_REG_THRESH_ACT_L, thres);
	adxl362_write8(ADXL362_REG_TIME_ACT, time);

	PRINTF("thres_act now value is %u\r\n", adxl362_read12(ADXL362_REG_THRESH_ACT_L));
	PRINTF("time_act now value is %u\r\n", adxl362_read8(ADXL362_REG_TIME_ACT));

	// turn on activity interrupt
	act_ctl_reg = adxl362_read8(ADXL362_REG_ACT_INACT_CTL);

	if (abs)
	{
		act_ctl_reg = act_ctl_reg | (0x01); // activity detection on absolution mode
		act_ctl_reg = act_ctl_reg & (0x3D);
	}
	else
		act_ctl_reg = act_ctl_reg | (0x03); // activity detection on reference mode

	adxl362_write8(ADXL362_REG_ACT_INACT_CTL, act_ctl_reg);

	PRINTF("activity detection set up: %u, %u\r\n", act_ctl_reg, adxl362_read8(ADXL362_REG_ACT_INACT_CTL));
}

/*---------- Set up inactiviey detection ----------*/
void adxl362_inact_int(uint16_t thres, uint16_t time, Bool abs)
{
	uint8_t inact_ctl_reg;

	// set up motion and time for activity waking up
	adxl362_write12(ADXL362_REG_THRESH_INACT_L, thres);
	adxl362_write12(ADXL362_REG_TIME_INACT_L, time);

	// turn on activity interrupt
	inact_ctl_reg = adxl362_read8(ADXL362_REG_ACT_INACT_CTL);

	if (abs)
	{
		inact_ctl_reg = inact_ctl_reg | (0x04); // inactivity detection on absolution mode
		inact_ctl_reg = inact_ctl_reg & (0x37);
	}
	else
		inact_ctl_reg = inact_ctl_reg | (0x0C); // inactivity detection on reference mode

	adxl362_write8(ADXL362_REG_ACT_INACT_CTL, inact_ctl_reg);

	PRINTF("inactivity detection set up: %u, %u\r\n", inact_ctl_reg, adxl362_read8(ADXL362_REG_ACT_INACT_CTL));
	PRINTF("threshold is %u, time is %u\r\n", adxl362_read12(ADXL362_REG_THRESH_INACT_L), adxl362_read12(ADXL362_REG_TIME_INACT_L));
}

/*---------- set up FIFO configuration ----------*/
void adxl362_FIFO_config(Bool FIFO_ah, Bool FIFO_temp, uint8_t FIFO_mode, uint16_t FIFO_num)
{
	uint8_t FIFO_reg;
	FIFO_reg = adxl362_read8(ADXL362_REG_FIFO_CTL);
	if (FIFO_ah)
		FIFO_reg = FIFO_reg | 0x08; // use above half of FIFO samples
	else
		FIFO_reg = FIFO_reg & 0x07;

	if (FIFO_temp)
		FIFO_reg = FIFO_reg | 0x04; // include temperature
	else
		FIFO_reg = FIFO_reg & 0x0B;

	switch(FIFO_mode)
	{
		case 0: FIFO_reg = FIFO_reg & 0x0C; break; // FIFO disabled
		case 1: FIFO_reg = FIFO_reg & 0x0D; FIFO_reg = FIFO_reg | 0x01; break; // Oldest saved mode
		case 2: FIFO_reg = FIFO_reg & 0x0E; FIFO_reg = FIFO_reg | 0x02; break; // Stream mode
		case 3: FIFO_reg = FIFO_reg | 0x03; break; // Triggered mode
		default: FIFO_reg = FIFO_reg & 0x0C; // FIFO is disabled
	}

	adxl362_write8(ADXL362_REG_FIFO_CTL, FIFO_reg);
	adxl362_write8(ADXL362_REG_FIFO_SPL, FIFO_num);

	PRINTF("%u, %u\r\n", FIFO_reg, adxl362_read8(ADXL362_REG_FIFO_CTL));
	PRINTF("%u, %u\r\n", FIFO_num, adxl362_read8(ADXL362_REG_FIFO_SPL));
}

/*---------- set up interrupt pin function ----------*/
void adxl362_int_setup(Bool int_pin, Bool act_mode, int int_mode)
{
	uint8_t int_pin_addr;
	uint8_t int_mode_val;
	uint8_t int_mode_act;

	if (int_pin)
		int_pin_addr = ADXL362_REG_INTMAP1;  // INT1
	else
		int_pin_addr = ADXL362_REG_INTMAP2;  // INT2

	if(act_mode)
		int_mode_act = ADXL362_ACT_INT_MODE; // active high
	else
		int_mode_act = ADXL362_ACT_INT_LOW; // active low

	if (int_mode==1)
		int_mode_val = int_mode_act;  // Activity Detection
	else if (int_mode==2)
		int_mode_val = ADXL362_IACT_INT_MODE;  // Inactivity Detection (active high by default)
	else
		int_mode_val = ADXL362_DRDY_INT_MODE; // Data ready

	adxl362_write8(int_pin_addr, int_mode_val);
}

/*---------- set up filter control ----------*/
void adxl362_filter_ctl(uint8_t range, Bool half_bw, uint16_t datarate)
{
	uint8_t filter_reg=0;
	filter_reg = adxl362_read8(ADXL362_REG_FILTER_CTL);
	PRINTF("filter control reset value is %u\r\n", filter_reg);

	switch(range)
	{
		case 4: filter_reg = filter_reg & 0x7F; filter_reg = filter_reg | 0x40; break; // 4g
		case 8: filter_reg = filter_reg | 0x80; break; // 8g
		default: filter_reg = filter_reg & 0x3F;  // 2g
	}

	if (half_bw)
		filter_reg = filter_reg | 0x10; // 1/4 ODR
	else
		filter_reg = filter_reg & 0xEF; // 1/2 ODR

	switch(datarate)
	{
		case 25: filter_reg = filter_reg & 0xF9; filter_reg = filter_reg | 0x01; break; // 25 Hz
		case 50: filter_reg = filter_reg & 0xFA; filter_reg = filter_reg | 0x02; break; // 50 Hz
		case 200: filter_reg = filter_reg & 0xFC; filter_reg = filter_reg | 0x04; break; // 200 Hz
		case 400: filter_reg = filter_reg | 0x05; break; // 400Hz
		default: filter_reg = filter_reg & 0xFB; filter_reg = filter_reg | 0x03; break; // 100 Hz
	}

	adxl362_write8(ADXL362_REG_FILTER_CTL, filter_reg);
	PRINTF("%u, %u\r\n", filter_reg, adxl362_read8(ADXL362_REG_FILTER_CTL));
}

/*---------- read FIFO buffer ----------*/////////////////////////////////should update!!
int16_t convertFIFOdata(uint8_t hiByte, uint8_t loByte)
{
    int16_t x = ((hiByte & 0x3F) << 8) + loByte;
    // get sign bits, copy into B15, B14, combine
    x = x + ((x & 0x3000) << 2);
    return(x);
}

void adxl362_FIFOread(uint16_t Sample_num, int16_t *Data)
{
	uint16_t i,j;
	uint8_t x[1020]={0};

	ADXL362_CS_LOW();
	SPI_SendByte(ADEXL362_SPI, ADXL362_FIFO_READ);
	for (i=0; i<Sample_num*6; i++)
	{
		SPI_TransferByte(ADEXL362_SPI, 0x00, &x[i]);
	}
	ADXL362_CS_HIGH();

	j=0;
  for(i=0; i<Sample_num*6; i+=6, j+=3)
  {
    Data[j]=convertFIFOdata(x[i+1], x[i]);
    Data[j+1]=convertFIFOdata(x[i+3], x[i+2]);
    Data[j+2]=convertFIFOdata(x[i+5], x[i+4]);
  }
}

/*---------- test: SPI read, Device ID and Status ----------*/
void adxl362_get_idst(void)
{
	uint8_t wake_reg;
	PRINTF("device id is %u\r\n", adxl362_read8(ADXL362_REG_DEVID_AD));  // by default, the return value should be AD after soft reset
	PRINTF("silicon revision id is %u\r\n", adxl362_read8(ADXL362_REG_REVID));
	wake_reg = adxl362_read8(ADXL362_REG_STATUS);
	if ((wake_reg & 0x40) == 0x40)
		PRINTF("adxl362 is awake\r\n");
	else
		PRINTF("adxl362 is sleep\r\n");
}
