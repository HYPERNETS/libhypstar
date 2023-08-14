#include <iostream>
#include <iomanip>
#include <assert.h>
#include <unistd.h>

#include "../inc/hypstar.h"
#include "common_functions.c"

using namespace std;

int main() {
	std::string port = HYPSTAR_PORTNAME;
//	port = "/dev/ttyUSB0";
	Hypstar *hs = Hypstar::getInstance(port);
	s_environment_log_entry log, log2;
	hs->setLoglevel(DEBUG);

	hs->setBaudRate(B_8000000);
//	hs->setBaudRate(B_115200);
	int i = 0;
	while(1) {
//		hypstar_get_env_log(pHs, i, &log);
		hs->getEnvironmentLogEntry(&log, 0xff);
//		hs->getEnvironmentLogEntry(&log2, 1);
		// log2 has to be older
//		assert(log2.timestamp < log.timestamp);
		printEnv(&log, hs);
		usleep(0.01);

//		printf("--------------\nC++ Test pass\n");

//		hypstar_t *pHs;
//		pHs = hypstar_init(port.c_str());
//		s_environment_log_entry env_log;

//		hypstar_get_env_log(pHs, 0, &env_log);
//		assert (log.timestamp == env_log.timestamp);
//		sleep(0.3);
	}
	printf("--------------\nC Test pass\n");
}
