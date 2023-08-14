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
	e_loglevel l = DEBUG;
//	e_loglevel l = TRACE;

	hs = Hypstar::getInstance(port, &l);

	//connect handler to SIGTERM (15) and SIGINT (2, Ctrl-C) signal
	signal(SIGTERM, sigHandler);
	signal(SIGINT, sigHandler);

	if (!hs) exit(-1);

	s_spectrum_dataset ds[200];

    hs->enableVM(true);
    sleep(1);

//	hs->setTECSetpoint(-10);
	hs->enableVM(true);
	sleep(1);
	hs->enableVM(false);
	sleep(1);
	hs->enableVM(true);
	sleep(1);
//	hs->measureVM((e_entrance)0x01, (e_vm_light_source)0x01, 100, 0.5f, ds);
//	hs->measureVM((e_entrance)0x01, (e_vm_light_source)0x01, 100, 1.0f, ds);
	hs->measureVM((e_entrance)0x01, (e_vm_light_source)0x01, 100, 1.5f, ds);
	hs->measureVM((e_entrance)0x01, (e_vm_light_source)0x01, 100, 1.5f, ds, 20);
//	hs->measureVM((e_entrance)0x01, (e_vm_light_source)0x01, 100, 2.0f, ds);
//	hs->measureVM((e_entrance)0x01, (e_vm_light_source)0x01, 100, 2.5f, ds);
//	hs->measureVM((e_entrance)0x01, (e_vm_light_source)0x01, 100, 3.5f, ds);

<<<<<<< Updated upstream
	printf("IT: %d, pix: %d, temp: %2.2f\n", ds[0].spectrum_header.integration_time_ms, ds[0].spectrum_header.pixel_count, ds[0].spectrum_header.sensor_temperature);

    hs->enableVM(false);
	delete(hs);

	printf("--------------\nC++ test pass\n");
=======
	mkdir("data", S_IRWXU | S_IRWXG | S_IRWXO);
//	while (1) {
//		save_spec(hs, BOTH, DARK, 256, 256, 1);
//		save_spec(hs, BOTH, RADIANCE, 256, 256, 1);
//	}
//	printf("IT: %d, pix: %d, temp: %2.2f\n", ds[0].spectrum_header.integration_time_ms, ds[0].spectrum_header.pixel_count, ds[0].spectrum_header.sensor_temperature);
//	delete(hs);
//	printf("--------------\nC++ test pass\n");
>>>>>>> Stashed changes


//	hypstar_t *pHs;
//	pHs = hypstar_init(port.c_str());
//	hypstar_set_loglevel(pHs, TRACE);
//	hypstar_VM_enable(pHs, 1);
//	sleep(3);
//	hypstar_VM_enable(pHs, 0);
//	hypstar_close(pHs);
//	printf("--------------\nC test pass\n");
//
//	return 0;
}
