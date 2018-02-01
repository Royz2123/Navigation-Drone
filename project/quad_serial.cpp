//
// The cross-platform QuadSerial.
// Based on previous RBD Lab solutions.
// Andrey Leshenko, July 2017.
//

#include "quad_serial.hpp"

#ifdef _WIN32
static void windows_arduino_serial_close(HANDLE hSerial);
static int windows_arduino_serial_write(HANDLE hSerial, const void *buffer, size_t length);
static HANDLE windows_arduino_serial_open(const char *serialport);
#else
static int linux_arduino_serial_close(int fd);
static int linux_arduino_serial_write(int serialfd, const void *buffer, size_t length);
static int linux_arduino_serial_open(const char *serialport, int baud);
#endif

bool QuadSerial::open(const std::string &device)
{
	this->close();
#ifdef _WIN32
	_hSerial = windows_arduino_serial_open(device.c_str());
#else
	_serialfd = linux_arduino_serial_open(device.c_str(), 115200);
#endif

	return isOpened();
}

bool QuadSerial::open(const std::vector<std::string> &devicesToTry)
{
	for (const auto &dev : devicesToTry)
	{
		bool success = open(dev);
		if (success)
			return true;
	}

	return false;
}

bool QuadSerial::openDefault()
{
#ifdef _WIN32
	char device[] = "COM***";

	for (int i = 20; i >= 2; i--)
	{
		snprintf(device, sizeof(device), "COM%d", i);
		bool success = open(device);
		if (success)
			return true;
	}

	return false;
#else
	// ttyACM0|1 came from the old Arduino Uno boards
	// ttyUSB0 comes from the newer Arduino Nano boards
	return open({"/dev/ttyUSB0", "/dev/ttyACM0", "/dev/ttyACM1"});
#endif
}

bool QuadSerial::isOpened()
{
#ifdef _WIN32
	return _hSerial != INVALID_HANDLE_VALUE;
#else
	return _serialfd >= 0;
#endif
}

void QuadSerial::printErrorIfNotConnected()
{
	if (!isOpened())
	{
#ifdef _WIN32
		printf("\tERROR: QuadSerial couldn't connect to any of the serial devices specified.\n"
		       "\tCommands WILL NOT be sent.\n"
		       "\n"
		       "\tMake sure that:\n"
		       "\tA. Your device is connected and listed in the Windows Device Manager\n"
		       "\tB. QuadSerial tries to open the correct device number, as listed in Windows Device Manager\n"
		       "\n"
		       "press ENTER to continue");
#else
		printf("\tERROR: QuadSerial couldn't connect to any of the serial devices specified.\n"
		       "\tCommands WILL NOT be sent.\n"
		       "\n"
		       "\tMake sure that:\n"
		       "\tA. IMPORTANT: You added your user to the 'dialout' group to gain access permission to the device.\n"
		       "\tB. Your device is connected and listed in /dev.\n"
		       "\tC. QuadSerial tries to open the correct device, as listed in /dev.\n"
		       "\n"
		       "press ENTER to continue");
#endif
		fflush(stdout);
		getchar();
	}
}

void QuadSerial::close()
{
#ifdef _WIN32
	windows_arduino_serial_close(_hSerial);
	_hSerial = INVALID_HANDLE_VALUE;
#else
	if (_serialfd >= 0)
		linux_arduino_serial_close(_serialfd);
	_serialfd = -1;
#endif
}

QuadSerial::~QuadSerial()
{
	this->close();
}

static int clampInt(int n, int minVal, int maxVal)
{
	if (n > maxVal) return maxVal;
	if (n < minVal) return minVal;
	return n;
}

static unsigned char toSignBitFormat(int n)
{
	unsigned char result;

	if (n >= 0)
	{
		result = n;
	}
	else
	{
		result = -n;
		result |= 0x80;
	}

	return result;
}

bool QuadSerial::sendDevo(float ch1, float ch2, float ch3, float ch4)
{
	return sendDevo(
		(int)(ch1 * 127),
		(int)(ch2 * 127),
		(int)(ch3 * 127),
		(int)(ch4 * 127));
}

bool QuadSerial::sendDevo(int ch1, int ch2, int ch3, int ch4)
{
	unsigned char controls[] = {
		0xFF,
		(unsigned char)clampInt(ch1, 0, 127),
		(unsigned char)clampInt(ch2, 0, 127),
		(unsigned char)clampInt(ch3, 0, 127),
		(unsigned char)clampInt(ch4, 0, 127),
	};
	return sendRaw(controls, sizeof(controls));
}

bool QuadSerial::send(float ch1, float ch2, float ch3, float ch4)
{
	return send(
		(int)(-127 + ch1 * 254),
		(int)(-127 + ch2 * 254),
		(int)(-127 + ch3 * 254),
		(int)(-127 + ch4 * 254));
}


bool QuadSerial::send(int ch1, int ch2, int ch3, int ch4)
{
	unsigned char controls[] = {
		0xFF,
		toSignBitFormat(clampInt(ch1, -127, 127)),
		toSignBitFormat(clampInt(ch2, -127, 127)),
		toSignBitFormat(clampInt(ch3, -127, 127)),
		toSignBitFormat(clampInt(ch4, -127, 127)),
		0, 0, 0};

	return sendRaw(controls, sizeof(controls));
}

bool QuadSerial::sendRaw(const void *data, size_t length)
{
	if (!isOpened())
		return false;

#ifdef _WIN32
	return windows_arduino_serial_write(_hSerial, data, length) == (int)length;
#else
	return linux_arduino_serial_write(_serialfd, data, length) == (int)length;
#endif
}

//
// Platform specific functions
//

#ifdef _WIN32

//
// Taken from the old QuadSerial implementation
//

static HANDLE windows_arduino_serial_open(const char *serialport)
{
	DCB dcbSerialParams = {0};

	HANDLE hSerial = CreateFileA(serialport,
			GENERIC_READ | GENERIC_WRITE,
			0,
			NULL,
			OPEN_EXISTING,
			FILE_ATTRIBUTE_NORMAL,
			NULL);

	if (hSerial == INVALID_HANDLE_VALUE)
	{
		if(GetLastError() == ERROR_FILE_NOT_FOUND)
			printf("ERROR (QuadSerial): Handle was not attached. Reason: %s not available.\n", serialport);
		else
			printf("ERROR (QuadSerial)");

		goto error;
	}

	//Try to get the current
	if (!GetCommState(hSerial, &dcbSerialParams))
	{
		//If impossible, show an error
		printf("failed to get current serial parameters!");
		goto error;
	}

	//Define serial connection parameters for the arduino board
	dcbSerialParams.BaudRate=CBR_115200;
	dcbSerialParams.ByteSize=8;
	dcbSerialParams.StopBits=ONESTOPBIT;
	dcbSerialParams.Parity=NOPARITY;

	// NOTE(Andrey): Prevent Arduino reset on each connection.
	// https://playground.arduino.cc/Main/DisablingAutoResetOnSerialConnection
	// http://forum.arduino.cc/index.php/topic,28167.0.html
	dcbSerialParams.fDtrControl = DTR_CONTROL_DISABLE;

	//Set the parameters and check for their proper application
	if(!SetCommState(hSerial, &dcbSerialParams))
	{
		printf("ALERT: Could not set Serial Port parameters");
		goto error;
	}

	return hSerial;

error:
	if (hSerial != INVALID_HANDLE_VALUE)
	{
		CloseHandle(hSerial);
	}
	return INVALID_HANDLE_VALUE;
}

static void windows_arduino_serial_close(HANDLE hSerial)
{
	if (hSerial != INVALID_HANDLE_VALUE)
		CloseHandle(hSerial);
}

static int windows_arduino_serial_write(HANDLE hSerial, const void *buffer, size_t length)
{
	DWORD bytesSend;
	COMSTAT status;
	DWORD errors;

	//Try to write the buffer on the Serial port
	if(!WriteFile(hSerial, buffer, (DWORD)length, &bytesSend, 0))
	{
		//In case it don't work get comm error and return false
		ClearCommError(hSerial, &errors, &status);
	}

	return bytesSend;
}

#else

//
// Part of arduino-serial-lib -- simple library for reading/writing serial ports
//
// 2006-2013, Tod E. Kurt, http://todbot.com/blog/
//

// Modifed by Andrey Leshenko, 2016

#include <stdio.h>    // Standard input/output definitions
#include <unistd.h>   // UNIX standard function definitions
#include <fcntl.h>    // File control definitions
#include <errno.h>    // Error number definitions
#include <termios.h>  // POSIX terminal control definitions
#include <sys/ioctl.h>

// takes the string name of the serial port (e.g. "/dev/tty.usbserial","COM1")
// and a baud rate (bps) and connects to that port at that speed and 8N1.
// opens the port in fully raw mode so you can send binary data.
// returns valid fd, or -1 on error
static int linux_arduino_serial_open(const char* serialport, int baud)
{
	struct termios toptions;
	int fd;

	//fd = open(serialport, O_RDWR | O_NOCTTY | O_NDELAY);
	fd = open(serialport, O_RDWR | O_NONBLOCK);

	if (fd == -1)  {
		//perror("serialport_init: Unable to open port ");
		return -1;
	}

	//int iflags = TIOCM_DTR;
	//ioctl(fd, TIOCMBIS, &iflags);     // turn on DTR
	//ioctl(fd, TIOCMBIC, &iflags);    // turn off DTR

	if (tcgetattr(fd, &toptions) < 0) {
		perror("serialport_init: Couldn't get term attributes");
		return -1;
	}
	speed_t brate = baud; // let you override switch below if needed
	switch(baud) {
	case 4800:   brate=B4800;   break;
	case 9600:   brate=B9600;   break;
#ifdef B14400
	case 14400:  brate=B14400;  break;
#endif
	case 19200:  brate=B19200;  break;
#ifdef B28800
	case 28800:  brate=B28800;  break;
#endif
	case 38400:  brate=B38400;  break;
	case 57600:  brate=B57600;  break;
	case 115200: brate=B115200; break;
	}
	cfsetispeed(&toptions, brate);
	cfsetospeed(&toptions, brate);

	// 8N1
	toptions.c_cflag &= ~PARENB;
	toptions.c_cflag &= ~CSTOPB;
	toptions.c_cflag &= ~CSIZE;
	toptions.c_cflag |= CS8;
	// no flow control
	toptions.c_cflag &= ~CRTSCTS;

	// NOTE(Andrey): Prevent Arduino reset on each connection.
	// https://playground.arduino.cc/Main/DisablingAutoResetOnSerialConnection
	// http://forum.arduino.cc/index.php/topic,28167.0.html
	toptions.c_cflag &= ~HUPCL; // disable hang-up-on-close to avoid reset

	toptions.c_cflag |= CREAD | CLOCAL;  // turn on READ & ignore ctrl lines
	toptions.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl

	toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
	toptions.c_oflag &= ~OPOST; // make raw

	// see: http://unixwiz.net/techtips/termios-vmin-vtime.html
	toptions.c_cc[VMIN]  = 0;
	toptions.c_cc[VTIME] = 0;
	//toptions.c_cc[VTIME] = 20;

	tcsetattr(fd, TCSANOW, &toptions);
	if( tcsetattr(fd, TCSAFLUSH, &toptions) < 0) {
		perror("init_serialport: Couldn't set term attributes");
		return -1;
	}

	return fd;
}

static int linux_arduino_serial_close(int fd)
{
	return close(fd);
}

static int linux_arduino_serial_write(int serialfd, const void *buffer, size_t length)
{
	// NOTE(Andrey): The exact type is ssize_t, but here it
	// isn't very important
	return (int)write(serialfd, buffer, length);
}

#endif
