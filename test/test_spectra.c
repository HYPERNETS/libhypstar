#include <iostream>
#include <assert.h>
#include "../inc/hypstar.h"
#include <ctime>
#include <iomanip>
#include "common_functions.c"


int main() {
	std::string port = HYPSTAR_PORTNAME;
//	std::string port = "/dev/ttyUSB1";
	Hypstar *hs = Hypstar::getInstance(port);
	hs->setLoglevel(TRACE);
	hs->setBaudRate(B_3000000);
//	hs->setLoglevel(DEBUG);

//	while (1) {
//		sleep(1);
//		test_spec(hs, BOTH, IRRADIANCE, 1000, 1000, 1);
//		sleep(2);
//		test_spec(hs, BOTH, DARK, 1000, 1000, 1);
//		sleep(2);
//		test_spec(hs, BOTH, RADIANCE, 1000, 1000, 1);
//		sleep(2);
//		test_spec(hs, BOTH, DARK, 100, 100, 3);
//	}

//	while (1)
	{
//		sleep(1);
		test_spec(hs, VNIR, IRRADIANCE, 1000, 1000, 1);
//		sleep(2);
		test_spec(hs, VNIR, DARK, 1000, 1000, 1);
//		sleep(2);
		test_spec(hs, VNIR, RADIANCE, 105, 150, 2);
//		sleep(2);
		test_spec(hs, VNIR, DARK, 100, 100, 3);
		test_spec(hs, VNIR, RADIANCE, 1, 150, 2);
//		sleep(1);
		test_spec(hs, SWIR, IRRADIANCE, 1000, 1000, 1);
//		sleep(2);
		test_spec(hs, SWIR, DARK, 1000, 1000, 1);
//		sleep(2);
		test_spec(hs, SWIR, RADIANCE, 105, 150, 2);
//		sleep(2);
		test_spec(hs, SWIR, DARK, 100, 100, 3);
		test_spec(hs, SWIR, RADIANCE, 1, 150, 2);
	}

//	std::cout << "Testing VIS DARK 10ms" << std::endl;
//	test_spec(hs, VNIR, DARK, 50, 0, 10);

	if (hs->available_hardware.swir_module)
	{
		std::cout << "Testing BOTH IRRAD 100ms" << std::endl;
		test_spec(hs, BOTH, IRRADIANCE, 1000, 1000, 1);

		std::cout << "Testing SWIR RAD 300ms" << std::endl;
		test_spec(hs, SWIR, RADIANCE, 0, 100, 3);

//
		// count = 0 on 32/256 2x dark
//		std::cout << "Testing BOTH IRRAD 100ms" << std::endl;
//		test_spec(hs, BOTH, DARK, 32, 256, 2);
	}
	else
	{
		// should not segfault
//		int r = hs->captureSpectra(SWIR, RADIANCE, 0, 1000, 1, 0, false);
//		assert (r == 0);
	}
	// @TODO: acceleration == 0 for ITs lower than accelerometer refresh rate?
	// @TODO: test incorrect input params, that should get blocked by the driver

	delete hs;
	printf("--------------\nC++ Test pass\n");

//	hypstar_t *pHs;
//	pHs = hypstar_init(port.c_str());
////	hypstar_set_loglevel(pHs, DEBUG);
//	int count = hypstar_capture_spectra(pHs, VNIR, RADIANCE, 100, 0, 2, 1, false);
//	assert(count == 2);
//	unsigned short slots[10];
//	count = hypstar_get_last_capture_memory_slots(pHs, slots, count);
//	assert(count == 2);
//	assert(slots[0] > 1);
//	s_spectrum_dataset spec_data[count];
//	count = hypstar_download_spectra(pHs, slots, count, spec_data);
//	assert(count == 2);
//	assert(spec_data[0].spectrum_header.integration_time_ms == 100);
//	assert(spec_data[0].spectrum_header.spectrum_config.radiance);
//	assert(!spec_data[0].spectrum_header.spectrum_config.irradiance);
//	assert(!spec_data[0].spectrum_header.spectrum_config.swir);
//	assert(spec_data[0].spectrum_header.pixel_count == 2048);
//
//	hypstar_close(pHs);
	printf("--------------\nC Test pass\n");
}
