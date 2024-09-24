#include <iostream>
#include "serialib.h" // https://github.com/imabot2/serialib

int main() 
{
	std::cout << "Program will test each serial port and tell which ones it managed to connect to.\n" << std::endl;

	// Serial object
	serialib serial;

	char device_name[50] = { 0 };
	// Test each port between COM1 and COM99 on Windows and between /dev/ttyS0 and /dev/ttyS99 on Linux
	for (int i=1;i<99;i++)
	{
		// Prepare the port name (Windows)
		#if defined (_WIN32) || defined( _WIN64)
			sprintf (device_name,"\\\\.\\COM%d",i);
		#endif

		// Prepare the port name (Linux)
		#ifdef __linux__
			sprintf (device_name,"/dev/ttyACM%d",i-1);
		#endif

		// try to connect to the device
		if (serial.openDevice(device_name,115200)==1)
		{
			printf ("Device detected on %s\n", device_name);
			// Close the device before testing the next port
			serial.closeDevice();
		}
	}

	std::cout << "Choose port number" << std::endl;

	int num_device = -1;
	std::cin >> num_device;

#if defined (_WIN32) || defined( _WIN64)
	if(num_device > 99 || num_device <= 1)
#endif
#ifdef __linux__
	if(num_device > 100 || num_device <= 0)
#endif
	{
		std::cout << "Wrong port specified" << std::endl;
		return 0;
	}

	std::cout << "Specify baud rate" << std::endl;

	int baud = -1;
	std::cin >> baud;

#if defined (_WIN32) || defined( _WIN64)
	if(baud < 110 || baud > 256000)
#endif
#ifdef __linux__
	if(baud < 110 || baud > 4000000)
#endif
	{
		std::cout << "Wrong baud rate specified" << std::endl;
		return 0;
	}

	#if defined (_WIN32) || defined( _WIN64)
		sprintf (device_name,"\\\\.\\COM%d",num_device);
	#endif
	#ifdef __linux__
		sprintf (device_name,"/dev/ttyACM%d",num_device);
	#endif

	// try to connect to the device
	if (serial.openDevice(device_name,baud)==1)
	{
	}
	else
	{
		std::cout << "Couldn't connect to device" << std::endl;
		serial.closeDevice();
		return 0;
	}

	unsigned char* buffer = new unsigned char[1024 * 1024]; //1 MiB

	serial.writeBytes(buffer, 1024 * 1024);

	delete[] buffer;
	serial.closeDevice();
	std::cout << "All done" << std::endl;
	return 0;
}