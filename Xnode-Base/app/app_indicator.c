#ifdef INDICATOR
#include <xnode.h>
#include "Radio.h"
#include "rf233.h"
#include "rf233-config.h"
#include "rf233-const.h"

int app_indicator(void)
{

	for (;;) {
		if (!rf233_channel_clear()) {
			LED_RGB(1,0,0);
		} else {
			LED_RGB(0,1,0);
		}
		vTaskDelay(10);
	}
	return 0;
}
#endif
