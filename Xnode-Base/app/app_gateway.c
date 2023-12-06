#ifdef GATEWAY
#include <xnode.h>
#include <AutoMonitor.h>
#include <RemoteSensing.h>
#include <RetrieveData.h>
#include <Settings.h>
#include <TriggerSensing.h>
#include <Utils.h>

static const char app_choice_menu[] =
	"\r\n- Choose application to run:\r\n"
	"- '1'  Remote sensing\r\n"
	"- '2'  Autonomous monitoring\r\n"
	"- '3'  Manually-triggered sensing\r\n"
	"- '4'  Event-triggered sensing\r\n"
	"- '7'  Retrieve sensor data\r\n"
	"- '6'  Check sensor status\r\n"
	"- '7'  Reset sensors\r\n"
	"- '8'  Change configuration\r\n"
	"- Xnode> ";

int app_gateway(void)
{
	char ch;
	bool app = false;
	uint8_t year, month, date, hour, minute, second;

	LED_RGB(0,1,0);

	if (RemoteSensing_init() != SUCCESS) {
		lpc_printf("ERROR: RemoteSensing init failed; resetting...\r\n");
		NodeReset();
	}
	if (RetrieveData_init() != SUCCESS) {
		lpc_printf("ERROR: DataRetrieving init failed; resetting...\r\n");
		NodeReset();
	}
	if (AutoMonitor_readcfg() != SUCCESS) {
		lpc_printf("ERROR: failed to read AutoMonitor config; resetting...\r\n");
		NodeReset();
	}

	gettime(&year, &month, &date, &hour, &minute, &second);
	lpc_printf("- gateway node initialized at 20%02u-%02u-%02u %02u:%02u:%02u\r\n", year, month, date, hour, minute, second);

	if (!AutoMonitor_running()) {
		do {
			lpc_printf(app_choice_menu);
			do {
				ch = GetChar();
			} while (!ch);
			lpc_printf("%c\r\n", ch);
			app = true;

			switch (ch) {
			case '1': // RemoteSensing
				RemoteSensing_menu(false);
				break;
			case '2': // AutoMonitor
				AutoMonitor_menu();
				break;
			case '3': // ManualSensing
				RemoteSensing_menu(true);
				break;
			case '4': // TriggerSensing
				if (TrigSenCfgSend(rsnodes, rsncnt) != SUCCESS) {
					lpc_printf("ERROR: failed to set trigger configuration remotely\r\n");
				}
				break;
			case '5': // RetrieveData
				RetrieveData_menu();
				break;
			case '6': // Check status
				if (Util_ReadVoltage(rsnodes, rsncnt) != SUCCESS) {
					lpc_printf("ERROR: failed to read voltage\r\n");
				}
				break;
			case '7': // Reset nodes
				if (Util_ResetNodes(rsnodes, rsncnt) != SUCCESS) {
					lpc_printf("ERROR: failed to reset nodes\r\n");
				}
				break;
			case '8': // Settings
				Settings_menu();
				break;
			default:
				app = false;
				lpc_printf("ERROR: bad input!\r\n");
				continue;
			}
		} while(!app);
	}

	vTaskDelay(200);
	NodeReset();
	return 0;
}
#endif
