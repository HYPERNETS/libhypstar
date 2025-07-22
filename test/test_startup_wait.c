#include <iostream>

#include "../inc/hypstar.h"
#include <assert.h>
#include <chrono>

using namespace std;

int main() {
	std::string port = HYPSTAR_PORTNAME;
	auto t1 = std::chrono::high_resolution_clock::now();
	bool booted = hypstar_wait_for_instrument(port.c_str(), 50, TRACE);

	if (booted)
	{
		auto t2 = std::chrono::high_resolution_clock::now();
		auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
		printf("Got boot in %ldms \n", duration);

		Hypstar *hs;
		hs = Hypstar::getInstance(port);
		uint32_t sn = hs->hw_info.instrument_serial_number;
		printf("Instrument SN: %d\n", hs->hw_info.instrument_serial_number);
		printf("FW version: %d.%d.%d\n", hs->hw_info.firmware_version_major, hs->hw_info.firmware_version_minor, hs->hw_info.firmware_version_revision);
		printf("HW versions - PSU: %d, MCU: %d\n", hs->hw_info.psu_hardware_version, hs->hw_info.mcu_hardware_version);
		printf("VIS SN: %d\n", hs->hw_info.vis_serial_number);
		printf("SWIR SN: %d\n", hs->hw_info.swir_serial_number);
		printf("Memory slot count: %d\n", hs->hw_info.memory_slot_count);

		if (hs->hw_info.vm_available) {
			printf("VM FW version: %d.%d.%d\n", hs->hw_info.vm_firmware_version_major, hs->hw_info.vm_firmware_version_minor, hs->hw_info.vm_firmware_version_revision);
		}
	}
	else
	{
		printf("Did not get instrument\n");
	}
}
