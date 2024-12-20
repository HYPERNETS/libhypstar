#include <iostream>

#include "../inc/hypstar.h"
#include <assert.h>
#include <chrono>
#include <dirent.h>
#include <string>
#include <sstream>

using namespace std;


// $ make test_firmware_update -- -f mm_fw_0_15_23.bin

int main(int argc, char **argv) {
	DIR *pDir;
	struct dirent *ep;

	int c;
	char *file_name = NULL;

	while ((c = getopt (argc, argv, "f:")) != -1)
		switch (c)
		{
		case 'f':
			file_name = optarg;
			break;
		case '?':
			if (optopt == 'f')
				fprintf (stderr, "Option -%c requires file name.\n", optopt);
			else if (isprint (optopt))
				fprintf (stderr, "Unknown option `-%c'.\n", optopt);
			else
				fprintf (stderr, "Unknown option character `\\x%x'.\n", optopt);
			return 1;
		default:
			abort ();
		}

	int latest_file_version[3] = {0, 0, 0};
	std::string latest, fw_path;

	if (file_name) {
		printf("Forcing firmware file %s\n", file_name);
		fw_path = file_name;
	}
	else
	{
		pDir = opendir("./firmware");
		if (pDir == NULL)
		{
			printf("No firmware directory!\n");
			return 1;
		}
		else
		{
			std::string prefix("mm_fw_");
			std::string extension(".bin");
	
			int latest_version_joined = 0;
			while ((ep = readdir(pDir)) != NULL) {
				if (file_name) {
					if (!strcmp(file_name, ep->d_name)) {
						latest = string(ep->d_name);
						break;
					} else {
						continue;
					}
				}
				std::string s = ep->d_name;
				if ((s.rfind(prefix, 0) != -1) && (s.rfind(extension, s.size() - extension.size()) != -1)) {
					int v[3];
					sscanf(s.substr(prefix.size(), s.size() - extension.size()).c_str(), "%d_%d_%d", &v[0], &v[1], &v[2]);
					int joined_version = (v[0] << 16) | (v[1] << 8) | v[2];
					if (joined_version >= latest_version_joined) {
						printf("Newer version info: %d %d %d (%06x), current v: %d %d %d (%06x)\n", v[0], v[1], v[2], joined_version, latest_file_version[0], latest_file_version[1], latest_file_version[2], latest_version_joined);
						memcpy(latest_file_version, v, sizeof(int) * 3);
						latest_version_joined = joined_version;
						latest = s;
					}
				}
			}
			closedir(pDir);
		}

		fw_path = "firmware/" + latest;
	}

	printf("Attempting to upload new firmware %s\n", fw_path.c_str());
	std::string port = HYPSTAR_PORTNAME;
//	port = "/dev/ttyUSB0";
	Hypstar *hs = Hypstar::getInstance(port);
	if (!hs)
		return -1;
	hs->setLoglevel(DEBUG);
	hs->setLoglevel(TRACE);
//	hs->setBaudRate(B_921600);
	hs->getFirmwareInfo();

	int previousFwSlot = hs->firmware_info.current_flash_slot;
	printf("Current FW version: %d.%d.%d in slot %d\n", hs->firmware_info.firmware_version_major,
			hs->firmware_info.firmware_version_minor, hs->firmware_info.firmware_version_revision, hs->firmware_info.current_flash_slot);

	s_firwmare_info old_fw = hs->firmware_info;
//	memcpy(old_fw, hs->firmware_info, sizeof(struct Hypstar::s_firmware_info));

	hs->sendNewFirmwareData(fw_path.c_str());
	hs->saveNewFirmwareData();
	hs->switchFirmwareSlot();

	hs->getFirmwareInfo();
	printf("New FW version: %d.%d.%d in slot %d (was %d.%d.%d in slot %d)\n", hs->firmware_info.firmware_version_major,
				hs->firmware_info.firmware_version_minor, hs->firmware_info.firmware_version_revision, hs->firmware_info.current_flash_slot,
				old_fw.firmware_version_major, old_fw.firmware_version_minor, old_fw.firmware_version_revision, old_fw.current_flash_slot);

	printf("--------------\nC++ test pass\n");
}
