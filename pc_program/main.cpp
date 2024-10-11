#include <iostream>
#include "serialib.h" // https://github.com/imabot2/serialib
#include "string.h"
#include <stdio.h>

#define FILE_VERSION 1

#define FLASH_BLOCK_SIZE 2048
#define RAM_BLOCK_SIZE 256
#define EXT_FLASH_BLOCK_SIZE 256

#define SYNTH_SYNC_BYTE 0x69
#define SYNTH_CMD_LOAD_FLASH 0
#define SYNTH_CMD_LOAD_RAM 1
#define SYNTH_CMD_LOAD_EXT_FLASH 2

#define MCU_FIRMWARE_SIZE (50 * 1024) /*TODO: replace with final size*/
#define BASE_ADDR_FLASH 0x08000000 + MCU_FIRMWARE_SIZE
#define BASE_ADDR_RAM 0x20000000 /*TODO: replace with final array addr*/

int load_file(serialib serial)
{
	unsigned char txbuf[4096] = {0};
	unsigned char rxbuf[4096] = {0};

	char buffer_cout[666] = {0};

	txbuf[0] = SYNTH_SYNC_BYTE;

	std::cout << "Enter STM32CrapSynth file name:\n" << std::endl;

	std::string filename;
	std::cin >> filename;

	FILE* f = fopen(filename.c_str(), "rb");

	if(f == NULL)
	{
		std::cout << "Couldn't open file!" << std::endl;
		return -1;
	}
	if(fseek(f, 59, SEEK_SET) != 0)
	{
		std::cout << "Error, file too short!" << std::endl;
		return -1;
	}

	fseek(f, 0, SEEK_SET);

	char magic[5] = {0};
	fread(magic, 4, 1, f);
	magic[4] = '\0';

	if(strncmp(magic, "CRAP", 5) != 0)
	{
		std::cout << "Bad file signature: \"" << magic << "\"!" << std::endl;
		return -1;
	}
	uint32_t version = 0;
	fread(&version, sizeof(uint32_t), 1, f);

	if(version > FILE_VERSION)
	{
		std::cout << "File version too high: " << version << "!" << std::endl;
		return -1;
	}
	uint32_t size = 0;
	fread(&size, sizeof(uint32_t), 1, f);

	char flashsamples[20] = {0};
	fread(flashsamples, 13, 1, f);
	flashsamples[13] = '\0';

	if(strncmp(flashsamples, "FLASH SAMPLES", 14) != 0)
	{
		std::cout << "Bad flash samples block signature: \"" << flashsamples << "\"!" << std::endl;
		return -1;
	}

	std::cout << "\nWriting data...\n" << std::endl;

	uint32_t flash_samples_size = 0;
	fread(&flash_samples_size, sizeof(uint32_t), 1, f);

	uint32_t flash_samples_pos = 0;

	std::cout << "\nWriting flash samples " << "(" << flash_samples_size << " bytes)...\n" << std::endl;

	while(flash_samples_pos < flash_samples_size - 1)
	{
		uint32_t block_size = ((flash_samples_size - flash_samples_pos) > FLASH_BLOCK_SIZE ? FLASH_BLOCK_SIZE : (flash_samples_size - flash_samples_pos));
		//txbuf[0] = SYNTH_SYNC_BYTE;
		txbuf[1] = SYNTH_CMD_LOAD_FLASH;
		txbuf[2] = block_size >> 8;
		txbuf[3] = block_size & 0xff;

		for(int i = 0; i < block_size; i++) //fill buffer
		{
			fread(&txbuf[4 + i], 1, 1, f);
		}
		
		snprintf(buffer_cout, 666, "Writing flash samples, block size %d bytes: 0x%08X-0x%08X (%.2f%)", block_size, BASE_ADDR_FLASH + flash_samples_pos, BASE_ADDR_FLASH + flash_samples_pos + block_size, (double)(flash_samples_pos) * 100.0 / (double)flash_samples_size);
		std::cout << buffer_cout << std::endl;

		serial.writeBytes(txbuf, 4 + block_size);

		flash_samples_pos += block_size;

		//todo: wait for response, checksum
	}

	char ramsamples[20] = {0};
	fread(ramsamples, 11, 1, f);
	ramsamples[12] = '\0';

	if(strncmp(ramsamples, "RAM SAMPLES", 12) != 0)
	{
		std::cout << "Bad RAM samples block signature: \"" << ramsamples << "\"!" << std::endl;
		return -1;
	}

	uint16_t ram_samples_size = 0;
	fread(&ram_samples_size, sizeof(uint16_t), 1, f);

	std::cout << "\nWriting RAM samples " << "(" << ram_samples_size << " bytes)...\n" << std::endl;

	uint16_t ram_samples_pos = 0;

	while(ram_samples_pos < ram_samples_size - 1)
	{
		uint32_t block_size = ((ram_samples_size - ram_samples_pos) > RAM_BLOCK_SIZE ? RAM_BLOCK_SIZE : (ram_samples_size - ram_samples_pos));
		//txbuf[0] = SYNTH_SYNC_BYTE;
		txbuf[1] = SYNTH_CMD_LOAD_RAM;
		txbuf[2] = block_size >> 8;
		txbuf[3] = block_size & 0xff;

		for(int i = 0; i < block_size; i++) //fill buffer
		{
			fread(&txbuf[4 + i], 1, 1, f);
		}

		snprintf(buffer_cout, 666, "Writing RAM samples, block size %d bytes: 0x%08X-0x%08X (%.2f%)", block_size, BASE_ADDR_RAM + ram_samples_pos, BASE_ADDR_RAM + ram_samples_pos + block_size, (double)(ram_samples_pos) * 100.0 / (double)ram_samples_size);
		std::cout << buffer_cout << std::endl;

		serial.writeBytes(txbuf, 4 + block_size);

		ram_samples_pos += block_size;

		//todo: wait for response, checksum
	}

	char regdump[20] = {0};
	fread(regdump, 14, 1, f);
	regdump[15] = '\0';

	if(strncmp(regdump, "REGISTERS DUMP", 15) != 0)
	{
		std::cout << "Bad registers dump block signature: \"" << regdump << "\"!" << std::endl;
		return -1;
	}

	uint32_t regdump_size = 0;
	fread(&regdump_size, sizeof(uint32_t), 1, f);

	std::cout << "\nWriting data (registers dump and wavetable data) to external flash " << "(" << regdump_size << " bytes)...\n" << std::endl;

	uint32_t regdump_pos = 0;

	while(regdump_pos < regdump_size - 1)
	{
		uint32_t block_size = ((regdump_size - regdump_pos) > EXT_FLASH_BLOCK_SIZE ? EXT_FLASH_BLOCK_SIZE : (regdump_size - regdump_pos));
		//txbuf[0] = SYNTH_SYNC_BYTE;
		txbuf[1] = SYNTH_CMD_LOAD_EXT_FLASH;
		txbuf[2] = block_size >> 8;
		txbuf[3] = block_size & 0xff;

		for(int i = 0; i < block_size; i++) //fill buffer
		{
			fread(&txbuf[4 + i], 1, 1, f);
		}

		snprintf(buffer_cout, 666, "Writing external flash, block size %d bytes: 0x%08X-0x%08X (%.2f%)", block_size, regdump_pos, regdump_pos + block_size, (double)(regdump_pos) * 100.0 / (double)regdump_size);
		std::cout << buffer_cout << std::endl;

		serial.writeBytes(txbuf, 4 + block_size);

		regdump_pos += block_size;

		//todo: wait for response, checksum
	}

	std::cout << "\nDone writing data!\n" << std::endl;

	return 0;
}

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
		std::cout << "Connection successful" << std::endl;
	}
	else
	{
		std::cout << "Couldn't connect to device" << std::endl;
		serial.closeDevice();
		return 0;
	}

	bool quit = false;

	while(!quit)
	{
		//todo: wait for init packet from device, receive actual flash and RAM base addresses
		load_file(serial);
	}

	//unsigned char* buffer = new unsigned char[1024 * 1024]; //1 MiB
	//serial.writeBytes(buffer, 1024 * 1024);
	//delete[] buffer;

	serial.closeDevice();
	std::cout << "All done" << std::endl;
	return 0;
}