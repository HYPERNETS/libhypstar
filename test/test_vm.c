#include <iostream>
#include <assert.h>
#include "../inc/hypstar.h"

using namespace std;

int main() {
	std::string port = HYPSTAR_PORTNAME;
//	e_loglevel l = DEBUG;
	e_loglevel l = TRACE;
	Hypstar *hs = Hypstar::getInstance(port, &l);

	s_spectrum_dataset ds[2];

	hs->measureVM((e_entrance)0x01, (e_vm_light_source)0x01, 100, 0.0f, ds);

	printf("IT: %d, pix: %d, temp: %2.2f\n", ds[0].spectrum_header.integration_time_ms, ds[0].spectrum_header.pixel_count, ds[0].spectrum_header.sensor_temperature);
	delete(hs);
	printf("--------------\nC++ test pass\n");

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
