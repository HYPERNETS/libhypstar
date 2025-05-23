#include <iostream>
#include <assert.h>
#include "../inc/hypstar.h"

using namespace std;

int main() {
	std::string port = HYPSTAR_PORTNAME;
//	std::string port = "/dev/ttyUSB1";
//	e_loglevel l = TRACE;
	e_loglevel l = DEBUG;
	Hypstar *hs;
//	while(1) {
		hs = Hypstar::getInstance(port, &l);
//	}
//	hs->setBaudRate(B_3000000);
//	// test double inits
////	sleep(10);
//	Hypstar *hs_copy = Hypstar::getInstance(port);
//	assert (hs == hs_copy);
//
//
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

//	hs->setBaudRate(B_6000000);
//
//	// test destructor resetting BR
	delete hs;
//
//	printf("Trying 2rd invocation, should succeed\n");
//	Hypstar *hs2 = Hypstar::getInstance(port);
//	printf("Success\n");
//	delete hs2;

//	printf("--------------\nC++ Test pass\n");
//
//	hypstar_t *pHs;
//	pHs = hypstar_init(port.c_str());
//	if (!pHs) {
//		printf("Could not obtain instance!\n");
//		return 0;
//	}
//	hypstar_set_loglevel(pHs, TRACE);
//	s_booted boot_info_struct;
//	hypstar_get_hw_info(pHs, &boot_info_struct);
////	assert (sn == boot_info_struct.instrument_serial_number);
//	hypstar_close(pHs);
//	printf("Instrument SN: %d\n", boot_info_struct.instrument_serial_number);
//	printf("FW version: %d.%d.%d\n", boot_info_struct.firmware_version_major, boot_info_struct.firmware_version_minor, boot_info_struct.firmware_version_revision);
//	printf("HW versions - PSU: %d, MCU: %d\n", boot_info_struct.psu_hardware_version, boot_info_struct.mcu_hardware_version);
//	printf("VIS SN: %d\n", boot_info_struct.vis_serial_number);
//	printf("SWIR SN: %d\n", boot_info_struct.swir_serial_number);
//	printf("Memory slot count: %d\n", boot_info_struct.memory_slot_count);
//	printf("VNIR pixel count: %d, SWIR pixel count: %d\n", boot_info_struct.vnir_pixel_count, boot_info_struct.swir_pixel_count);

	// change baudrate
//	pHs = hypstar_init(port.c_str());
//	hypstar_set_baudrate(pHs, B_3000000);
//	hypstar_get_hw_info(pHs, &boot_info_struct);
//	hypstar_close(pHs);
//	// check that baud rate has been reset to default 115200 after reinit
//	pHs = hypstar_init(port.c_str());
//	hypstar_get_hw_info(pHs, &boot_info_struct);
//	hypstar_close(pHs);
//
//	// test double inits
//	pHs = hypstar_init(port.c_str());
//	pHs = hypstar_init(port.c_str());
//	hypstar_close(pHs);
//	printf("--------------\nC test pass\n");

	return 0;
}
