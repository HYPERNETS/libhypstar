# C/C++ driver for Hypstar instrument

## Dependencies:
build-essentials

## Usage:
> $ make lib	# compiles source code into binary

> $ make \<test_file_name> # runs predefined tests

> $ sudo make install	# copies current binary version to /usr/lib and (re)creates a symlink to current version

/test/ directory shows how to use C/C++ for controlling the instrument. These tests can be run using "make <test_file_name>", e.g.:
> $ make test_basic_comms

bin/ directory has precompiled binary file that should work on most x64 linux systems. Haven't tested outside Debian(Mint) though.

See inc/hypstar.h header for documentation and inc/hypstar_typedefs.hpp for data structure definitions.

debug/info/trace output uses stdout, error logging goes to stderr. Log levels can be set by user:
 * ERROR - logs only error messages
 * INFO - default, minimalistic information;
 * DEBUG - prints also function call information;
 * TRACE - prints also binary data sent to and received from the instrument (VERY VERBOSE!);


## Python:
/python/ directory has python3 wrapper for instrument together with usage examples
Python wrapper uses CTypes, so it shouldn't depend on extra packages. It does expect finding "libhypstar.so" in system path, so please have it there or modify reference in hypstar_wrapper.py. Init function expects serial device url string, so if you use Joel's udev rule, it should be instantiated as Hypstar('/dev/radiometer0'), otherwise you can use '/dev/ttyUSBx' or whatever you have.
Some comments are in wrapper itself, most of the info is in the C++ header file inc/hypstar.h.
Python unit tests can be run using unittest library:
> $ python3 -m unittest ctypes_test.CtypeTests.test_hw_info

## TODO:
 * Currently C wrapper does not handle exceptions, eventually those will get passed back to caller using callback functions
 * Callback functions for some functionality are not implemented (e.g. automatic integration time adjustment)
 * add mutexes for multi-threaded operation


# Project references
https://www.hypernets.eu/

The HYPERNETS project is funded by the European Union’s Horizon 2020 research and innovation program under grant agreement No 775983.

The HYPERNETS project is carried out by a consortium consisting of: 
 * Royal Belgian Institute of Natural Sciences (coordinator) (RBINS, Belgium)
 * University of Tartu Tartu Observatory (TO, Estonia)
 * Sorbonne University (SU, France)
 * National Research Council (CNR, Italy)
 * National Physical Laboratory(NPL, United Kingdom)
 * German Research Centre for Geosciences(GFZ, Germany)
 * National Scientific and Technical Research Council(CONICET, Argentina)
