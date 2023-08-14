#include <iostream>

#include "../inc/hypstar.h"
#include <assert.h>
#include <chrono>

using namespace std;

int main() {
	std::string port = HYPSTAR_PORTNAME;
	e_loglevel l = TRACE;
	Hypstar *hs = Hypstar::getInstance(port, &l);
	auto t1 = std::chrono::high_resolution_clock::now();
	hs->reboot();
	auto t2 = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
	hs->getFirmwareInfo();

	int previousFwSlot = hs->firmware_info.current_flash_slot;
	printf("Current FW version: %d.%d.%d in slot %d\n", hs->firmware_info.firmware_version_major,
			hs->firmware_info.firmware_version_minor, hs->firmware_info.firmware_version_revision, hs->firmware_info.current_flash_slot);
	printf("--------------\nC++ test pass, reboot took %.2f s\n", (float)duration/1000);

//	hypstar_t *pHs;
//	pHs = hypstar_init(port.c_str());
//	hypstar_set_loglevel(pHs, TRACE);
//	t1 = std::chrono::high_resolution_clock::now();
//	hypstar_reboot(pHs);
//	t2 = std::chrono::high_resolution_clock::now();
//	duration = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
//	hypstar_close(pHs);
	printf("--------------\nC test pass, reboot took %.2f s\n", (float)duration/1000);
}
