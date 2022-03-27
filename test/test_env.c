#include <iostream>
#include <iomanip>
#include <assert.h>

#include "../inc/hypstar.h"
#include "common_functions.c"

using namespace std;

int main() {
	std::string port = HYPSTAR_PORTNAME;
	e_loglevel l = DEBUG;
	Hypstar *hs = Hypstar::getInstance(port, &l);
	s_environment_log_entry log, log2;

	hypstar_t *pHs;
	pHs = hypstar_init(port.c_str());
	int i = 0;
	for (i; i < 1; i++) {
//	while(1) {
//		hypstar_get_env_log(pHs, i, &log);
		hs->getEnvironmentLogEntry(&log, i);
//		hs->getEnvironmentLogEntry(&log2, 1);
		// log2 has to be older
//		assert(log2.timestamp < log.timestamp);
		printEnv(&log, hs);

		printf("--------------\nC++ Test pass\n");

		hypstar_t *pHs;
		pHs = hypstar_init(port.c_str());
		s_environment_log_entry env_log;

		hypstar_get_env_log(pHs, 0, &env_log);
//		assert (log.timestamp == env_log.timestamp);
//		sleep(5);
	}
	printf("--------------\nC Test pass\n");
}
