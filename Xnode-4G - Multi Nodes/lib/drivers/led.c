#include <xnode.h>
#include <lpc43xx_gpio.h>
#include <lpc43xx_scu.h>

bool r = 0, g = 0, b = 0;

void LED_Init(void)
{
	scu_pinmux(0xE, 6, MD_PLN, FUNC4); // PE_6: GPIO7[6]
	scu_pinmux(0xE, 2, MD_PLN, FUNC4); // PE_2: GPIO7[2]
	scu_pinmux(0xE, 5, MD_PLN, FUNC4); // PE_5: BPIO7[5]
	
	GPIO_SetDir(7, 1 << 6, 1);
	GPIO_SetDir(7, 1 << 2, 1);
	GPIO_SetDir(7, 1 << 5, 1);
	
	LED_Off();
}

void LED_RGB(bool ron, bool gon, bool bon)
{
	LED_R(ron);
	LED_G(gon);
	LED_B(bon);
}

void LED_Off(void)
{
	LED_RGB(0, 0, 0);
}

void LED_R(bool on)
{
	if (on) {
		GPIO_ClearValue(7, 1 << 6);
	} else {
		GPIO_SetValue(7, 1 << 6);
	}
	r = on;
}

void LED_Rtoggle(void)
{
	LED_R(!r);
}
	
void LED_G(bool on)
{
	if (on) {
		GPIO_ClearValue(7, 1 << 2);
	} else {
		GPIO_SetValue(7, 1 << 2);
	}
	g = on;
}

void LED_Gtoggle(void)
{
	LED_G(!g);
}

void LED_B(bool on)
{
	if (on) {
		GPIO_ClearValue(7, 1 << 5);
	} else {
		GPIO_SetValue(7, 1 << 5);
	}
	b = on;
}

void LED_Btoggle(void)
{
	LED_B(!b);
}
