#include <iostream>
#include <assert.h>
#include "../inc/hypstar.h"

using namespace std;

int main() {
	std::string port = HYPSTAR_PORTNAME;
	e_loglevel l = TRACE;
	Hypstar *hs = Hypstar::getInstance(port, &l);
	hs->setTECSetpoint(0);

	printf("shutting down TEC and letting it warm up for 5s\n");

	return 0;
}
