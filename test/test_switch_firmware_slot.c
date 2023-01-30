#include <iostream>

#include "../inc/hypstar.h"
#include <assert.h>
#include <chrono>
#include <dirent.h>
#include <string>
#include <sstream>

using namespace std;

int main(int argc, char **argv) {

	std::string port = HYPSTAR_PORTNAME;
	Hypstar *hs = Hypstar::getInstance(port);
//	hs->setLoglevel(DEBUG);
	hs->setLoglevel(TRACE);

	hs->getFirmwareInfo();

	int previousFwSlot = hs->firmware_info.current_flash_slot;
	printf("Current FW version: %d.%d.%d in slot %d\n", hs->firmware_info.firmware_version_major,
			hs->firmware_info.firmware_version_minor, hs->firmware_info.firmware_version_revision, hs->firmware_info.current_flash_slot);

	hs->switchFirmwareSlot();

	hs->getFirmwareInfo();
	printf("New FW version: %d.%d.%d in slot %d\n", hs->firmware_info.firmware_version_major,
				hs->firmware_info.firmware_version_minor, hs->firmware_info.firmware_version_revision, hs->firmware_info.current_flash_slot);

	printf("--------------\nC++ test pass\n");
}
