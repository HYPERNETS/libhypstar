#include <iostream>
#include <iomanip>
#include <assert.h>
#include "../inc/hypstar.h"
#include <ctime>
#include <iomanip>
#include "common_functions.c"

using namespace std;

int main() {
	std::string port = "/dev/ttyUSB1";
	port = "/dev/ttyUSB0";
	Hypstar *hs = Hypstar::getInstance(port);
	hs->setLoglevel(TRACE);
//	hs->setLoglevel(DEBUG);
	hs->setBaudRate(B_8000000);
	int count = 0;

//	std::cout << "Testing VIS RAD auto" << std::endl;
//	int last_it = test_spec(hs, VNIR, RADIANCE, 0, 0, 1);
//
//	std::cout << "Testing VIS IRRAD auto" << std::endl;
//	last_it = test_spec(hs, VNIR, IRRADIANCE, 0, 0, 1);
//
//	std::cout << "Testing VIS DARK auto" << std::endl;
//	int latest_it = test_spec(hs, VNIR, DARK, 0, 0, 1);
//	assert (last_it == last_it);

//	std::cout << "Testing BOTH IRRADIANCE auto with reusing" << std::endl;
//	int count = hs->captureSpectra(BOTH, IRRADIANCE, 0, 0, 2, 0, 1);

	while (1) {
		if (hs->available_hardware.swir_module)
		{
	//		std::cout << "Testing SWIR RAD auto" << std::endl;
	//		test_spec(hs, SWIR, RADIANCE, 0, 0, 3);
	//
			std::cout << "Testing BOTH IRRAD auto" << std::endl;
			count = test_spec(hs, BOTH, IRRADIANCE, 0, 0, 3);
			if ((count == 0) || (count < 6)) {
				return -1;
			}

			std::cout << "Testing BOTH RAD auto" << std::endl;
			count = test_spec(hs, BOTH, RADIANCE, 0, 0, 3);
			if ((count == 0) || (count < 6)) {
				return -1;
			}
	//
			std::cout << "Testing BOTH DARK auto" << std::endl;
			count = test_spec(hs, BOTH, DARK, 0, 0, 3);
			if ((count == 0) || (count < 6)) {
				return -1;
			}

	//		std::cout << "Testing BOTH DARK auto with reusing" << std::endl;
	//		int count = hs->captureSpectra(VNIR, IRRADIANCE, 0, 0, 2, 0, 1);
	//		test_spec(hs, BOTH, DARK, 0, 0, 1);
		}
		else
		{
			// should not segfault
			int r = hs->captureSpectra(SWIR, RADIANCE, 0, 0, 1, 0);
			assert (r == 0);
			printf("Got 0 spectra as expected\n");
		}
	}

	// @TODO: test incorrect input params, that should get blocked by the driver
	// @TODO: acceleration == 0 for ITs lower than accelerometer refresh rate?
	printf("--------------\nC++ Test pass\n");

	hypstar_t *pHs;
	pHs = hypstar_init(port.c_str());
	hypstar_set_loglevel(pHs, TRACE);
//	hypstar_set_loglevel(pHs, DEBUG);
	// 1s is unrealistic, so this should return 0
//	count = hypstar_capture_spectra(pHs, VNIR, RADIANCE, 0, 0, 200, 1, true);
//	assert(count == 0);

//	count = hypstar_capture_spectra(pHs, VNIR, RADIANCE, 0, 0, 1, 60, true);

	unsigned short slots[10];
//	count = hypstar_get_last_capture_memory_slots(pHs, slots, count);

//	s_spectrum_dataset spec_data[count];
//	count = hypstar_download_spectra(pHs, slots, count, spec_data);

//	assert(spec_data[0].spectrum_header.spectrum_config.radiance);
//	assert(!spec_data[0].spectrum_header.spectrum_config.irradiance);
//	assert(!spec_data[0].spectrum_header.spectrum_config.swir);
//	assert(spec_data[0].spectrum_header.pixel_count == 2048);
//
	hypstar_close(pHs);
	printf("--------------\nC Test pass\n");
}
