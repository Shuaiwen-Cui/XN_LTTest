#include <xnode.h>
#include <lpc43xx_gpio.h>
#include <lpc43xx_scu.h>
#include <switch.h>
bool a = 0, bb = 0;
void SWITCH_Init(void)
{
//configure pins
	  /*scu_pinmux(0xD,   2, MD_PDN, FUNC4);
	  scu_pinmux(0xD,   8, MD_PDN, FUNC4);
		scu_pinmux(0x1,   4, MD_PDN, FUNC0);*/
	  scu_pinmux(0x4,   4, MD_PDN, FUNC0);
		GPIO_SetDir(2, 1 << 4, 1);
	  //GPIO_SetDir(6, 1 << 22, 1);
	  //GPIO_SetDir(0, 1 << 11, 1);
}

