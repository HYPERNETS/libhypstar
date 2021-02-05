#include <iostream>

#include "../inc/hypstar.h"
#include <assert.h>
#include <chrono>

using namespace std;

int main() {
	std::string port = "/dev/ttyUSB0";
	bool booted = hypstar_wait_for_instrument(port.c_str(), 15);
	if (booted)
	{
		printf("Got boot\n");
	}
	else
	{
		printf("Did not get instrument\n");
	}

}
