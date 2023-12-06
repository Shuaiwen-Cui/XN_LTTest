#ifdef GATEWAY
#include <xnode.h>
#include <AutoMonitor.h>
#include <RemoteSensing.h>
#include <RetrieveData.h>
#include <Settings.h>
#include <TriggerSensing.h>
#include <Utils.h>

#ifdef FRA	// So tasks/options are selected/hard-coded
char ch;
#endif

static const char app_choice_menu[] =
	"\r\n- Choose application to run:\r\n"
	"- '1'  Remote sensing\r\n"
	"- '2'  Autonomous monitoring\r\n"
	"- '3'  Event-triggered sensing\r\n"
	"- '4'  Retrieve sensor data\r\n"
	"- '5'  Check sensor status\r\n"
	"- '6'  Reset sensors\r\n"
	"- '7'  Change configuration\r\n"
#ifdef FRA
	"- '8'  FRA tasks\r\n"
#endif
	"- Xnode> ";

int app_gateway(void)
{
	bool app = false;
#ifndef FRA	
	char ch;
#endif
	if (RemoteSensing_init() != SUCCESS) {
		lpc_printf("ERROR: RemoteSensing init failed; resetting...\r\n");
		NodeReset();
	}

	if (RetrieveData_init() != SUCCESS) {
		lpc_printf("ERROR: DataRetrieving init failed; resetting...\r\n");
		NodeReset();
	}
#ifndef FRA	
	if (AutoMonitor_readcfg() != SUCCESS) {
		lpc_printf("ERROR: failed to read AutoMonitor config; resetting...\r\n");
		NodeReset();
	}
#endif

	lpc_printf("- gateway node initialized.\r\n");

	if (!AutoMonitor_running()) {
		do {
			lpc_printf(app_choice_menu);
#ifndef FRA			
			do {
				ch = GetChar();
			} while (!ch);
#endif
			lpc_printf("%c\r\n", ch);
			app = true;

			switch (ch) {
			case '1': // RemoteSensing
				RemoteSensing_menu();
				break;
			case '2': // AutoMonitor
				AutoMonitor_menu();
				break;
			case '3': // TriggerSensing
				if (TrigSenCfgSend(rsnodes, rsncnt) != SUCCESS) {
					lpc_printf("ERROR: failed to set trigger configuration remotely\r\n");
				}
				break;
			case '4': // RetrieveData
				RetrieveData_menu();
				break;
			case '5': // Check status
				if (Util_ReadVoltage(rsnodes, rsncnt) != SUCCESS) {
					lpc_printf("ERROR: failed to read voltage\r\n");
				}
				break;
			case '6': // Reset nodes
				if (Util_ResetNodes(rsnodes, rsncnt) != SUCCESS) {
					lpc_printf("ERROR: failed to reset nodes\r\n");
				}
				break;
			case '7': // Settings
				Settings_menu();
				break;
#ifdef FRA
			case '8': // FRA tasks
				FRA_menu();
				break;
#endif
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
