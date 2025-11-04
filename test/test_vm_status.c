#include <iostream>
#include <assert.h>
#include <signal.h>
#include "../inc/hypstar.h"
#include <sys/stat.h>
#include "common_functions.c"

using namespace std;
Hypstar *hs = NULL;

//signal handler
void sigHandler(int sigNum)
{
    // switch VM off if test is interrupted
	hs->enableVM(false);
	sleep(1);
	delete(hs);
	exit(sigNum);
}

int main() {
	std::string port = HYPSTAR_PORTNAME;
	VM_Status_t vm_status;
	e_loglevel l = TRACE;
//	l = DEBUG;
	l = TRACE;

	hs = Hypstar::getInstance(port, &l);

	//connect handler to SIGTERM (15) and SIGINT (2, Ctrl-C) signal
	signal(SIGTERM, sigHandler);
	signal(SIGINT, sigHandler);

	if (!hs) exit(-1);

	hs->enableVM(true);
	sleep(1);
	hs->getVmStatus(&vm_status);
	hs->enableVM(false);
	printf("VM setpoint: %2.2f, VM temperature: %2.2f, VM sink temp: %2.2f, current setting: %2.2f, voltage: %2.4f V; TEC: %2.2fV %2.2fA\n", vm_status.temp_setpoint, vm_status.temp_current, vm_status.temp_sink, vm_status.led_current, vm_status.led_voltage, vm_status.tec_voltage, vm_status.tec_current);
	delete(hs);

	printf("--------------\nC++ test pass\n");

	hypstar_t *pHs;
	pHs = hypstar_init(port.c_str());
	hypstar_set_loglevel(pHs, TRACE);
	hypstar_VM_enable(pHs, 1);
	sleep(1);
	hs->getVmStatus(&vm_status);
	hypstar_VM_enable(pHs, 0);
	printf("VM setpoint: %2.2f, VM temperature: %2.2f, VM sink temp: %2.2f, current setting: %2.2f, voltage: %2.4f V; TEC: %2.2fV %2.2fA\n", vm_status.temp_setpoint, vm_status.temp_current, vm_status.temp_sink, vm_status.led_current, vm_status.led_voltage, vm_status.tec_voltage, vm_status.tec_current);

	hypstar_close(pHs);
	printf("--------------\nC test pass\n");

	return 0;
}
