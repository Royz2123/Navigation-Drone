#pragma once

#include <string>
#include <vector>
#ifdef _WIN32
#include <windows.h>
#endif

//
// Class for controlling drones through a computer.
// A special Arduino remote has to be connected for this to work.
//

class QuadSerial
{
public:
	QuadSerial() = default;
	QuadSerial(const QuadSerial&) = delete; // Copying serials will complicate things
	~QuadSerial();

	bool openDefault();
	bool open(const std::string &device);
	bool open(const std::vector<std::string> &devicesToTry);

	bool isOpened();
	void printErrorIfNotConnected();
	void close();

	// Use this with the new remotes (an Arduino + nrf24l01 antenna)
	//
	// When using floats all channels are in range [0, 1].
	// When using ints all channels are in range [-127, 127].
	bool send(float ch1, float ch2, float ch3, float ch4);
	bool send(int ch1, int ch2, int ch3, int ch4);

	// Use this with the old setup: an Arduino connected to a Walkera Devo remote
	// through trainer mode.
	//
	// When using floats all channels are in range [0, 1].
	// When using ints all channels are in range [0, 127].
	bool sendDevo(float ch1, float ch2, float ch3, float ch4);
	bool sendDevo(int ch1, int ch2, int ch3, int ch4);

	// This sends data directly to the Arduino, without any formating.
	// Use this only when you know what format the Arduino expects.
	bool sendRaw(const void *data, size_t length);

#ifdef _WIN32
	HANDLE _hSerial = INVALID_HANDLE_VALUE;
#else
	int _serialfd = -1;
#endif
};
