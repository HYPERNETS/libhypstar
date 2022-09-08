#include <iostream>
#include <assert.h>
#include "../inc/hypstar.h"

using namespace std;

int main() {
	std::string port = HYPSTAR_PORTNAME;
//	e_loglevel l = DEBUG;
	e_loglevel l = TRACE;
	Hypstar *hs = Hypstar::getInstance(port, &l);
	hs->enableVM(true);
	sleep(3);
	hs->enableVM(false);

	delete(hs);
	printf("--------------\nC++ test pass\n");

	hypstar_t *pHs;
	pHs = hypstar_init(port.c_str());
	hypstar_set_loglevel(pHs, TRACE);
	hypstar_VM_enable(pHs, 1);
	sleep(3);
	hypstar_VM_enable(pHs, 0);
	hypstar_close(pHs);
	printf("--------------\nC test pass\n");

	return 0;
}
