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

#define SYNTH_RESPONSE_SIZE 48

#define SYNTH_RESPONSE_READY 0
#define SYNTH_RESPONSE_FLASH_BLOCK 1
#define SYNTH_RESPONSE_RAM_BLOCK 2
#define SYNTH_RESPONSE_EXTERNAL_FLASH_BLOCK 3
#define SYNTH_RESPONSE_BAD_XOR 4
#define SYNTH_RESPONSE_UNKNOWN_COMMAND 5

//#define MCU_FIRMWARE_SIZE (50 * 1024) /*TODO: replace with final size*/
//#define BASE_ADDR_FLASH 0x08000000 + MCU_FIRMWARE_SIZE
//#define BASE_ADDR_RAM 0x20000000 /*TODO: replace with final array addr*/

int load_file(serialib serial)
{
	uint32_t MCU_FIRMWARE_SIZE = 0;
	uint32_t BASE_ADDR_FLASH = 0;
	uint32_t BASE_ADDR_RAM = 0;
	uint32_t SUPPORTED_FILE_VERSION = 0;
	uint32_t FIRMWARE_VERSION = 0;
	uint32_t EXTERNAL_FLASH_SIZE = 0;
	uint64_t EXTERNAL_FLASH_UID = 0;
	uint8_t EXTERNAL_FLASH_MANUFACTURER_ID = 0;

	uint8_t our_xor = 0;

	unsigned char txbuf[4096] = {0};
	unsigned char rxbuf[4096] = {0};

	serial.readBytes(rxbuf, SYNTH_RESPONSE_SIZE);

	for(int i = 0; i < SYNTH_RESPONSE_SIZE - 1; i++)
	{
		our_xor ^= rxbuf[i];
	}

	if(our_xor != rxbuf[SYNTH_RESPONSE_SIZE - 1])
	{
		std::cout << "Wrong checksum in comms establishing packet!" << std::endl;
		return -1;
	}

	if(rxbuf[1] != SYNTH_RESPONSE_READY)
	{
		std::cout << "Wrong packet ID in comms establishing packet!" << std::endl;
		return -1;
	}

	//TODO: uncomment

	MCU_FIRMWARE_SIZE = (uint32_t)rxbuf[5] + ((uint32_t)rxbuf[4] << 8) + ((uint32_t)rxbuf[3] << 16) + ((uint32_t)rxbuf[2] << 24);
	BASE_ADDR_FLASH = (uint32_t)rxbuf[9] + ((uint32_t)rxbuf[8] << 8) + ((uint32_t)rxbuf[7] << 16) + ((uint32_t)rxbuf[6] << 24);
	BASE_ADDR_RAM = (uint32_t)rxbuf[13] + ((uint32_t)rxbuf[12] << 8) + ((uint32_t)rxbuf[11] << 16) + ((uint32_t)rxbuf[10] << 24);

	SUPPORTED_FILE_VERSION = (uint32_t)rxbuf[17] + ((uint32_t)rxbuf[16] << 8) + ((uint32_t)rxbuf[15] << 16) + ((uint32_t)rxbuf[14] << 24);
	FIRMWARE_VERSION = (uint32_t)rxbuf[21] + ((uint32_t)rxbuf[20] << 8) + ((uint32_t)rxbuf[19] << 16) + ((uint32_t)rxbuf[18] << 24);
	EXTERNAL_FLASH_SIZE = (uint32_t)rxbuf[25] + ((uint32_t)rxbuf[24] << 8) + ((uint32_t)rxbuf[23] << 16) + ((uint32_t)rxbuf[22] << 24);
	EXTERNAL_FLASH_UID = (uint64_t)rxbuf[33] + ((uint64_t)rxbuf[32] << 8) + ((uint64_t)rxbuf[31] << 16) + ((uint64_t)rxbuf[30] << 24)
		 + ((uint64_t)rxbuf[29] << 32) + ((uint64_t)rxbuf[28] << 40) + ((uint64_t)rxbuf[27] << 48) + ((uint64_t)rxbuf[26] << 56);
	EXTERNAL_FLASH_MANUFACTURER_ID = rxbuf[34];

	EXTERNAL_FLASH_SIZE = 1024 * 1024 * 16; //16 MiB TODO: remove when actual comms are established

	char buffer_cout[1666] = {0};

	snprintf(buffer_cout, 1666, 
		"\nSTM32CrapSynth responded!\n"
		"Firmware version: %d.%d.%d.%d\n"
		"MCU firmware size: 0x%08X\n"
		"Samples range in Flash starting address: 0x%08X\n"
		"Samples array in RAM starting address: 0x%08X\n"
		"Highest supported .scs file version: %d.%d.%d.%d\n"
		"External Flash memory size: %d bytes (%d%s)\n"
		"External Flash memory UID: 0x%08X\n"
		"External Flash manufacturer ID: 0x%02X\n",
			rxbuf[21], rxbuf[20], rxbuf[19], rxbuf[18],
			MCU_FIRMWARE_SIZE,
			BASE_ADDR_FLASH,
			BASE_ADDR_RAM,
			rxbuf[17], rxbuf[16], rxbuf[15], rxbuf[14],
			EXTERNAL_FLASH_SIZE, //%d bytes
			EXTERNAL_FLASH_SIZE > 1024 * 1024 ? EXTERNAL_FLASH_SIZE / (1024 * 1024) : EXTERNAL_FLASH_SIZE / 1024, //%d
			EXTERNAL_FLASH_SIZE > 1024 * 1024 ? "MiB" : "KiB", //%s
			EXTERNAL_FLASH_UID,
			EXTERNAL_FLASH_MANUFACTURER_ID);
	
	std::cout << buffer_cout << std::endl;

	txbuf[0] = SYNTH_SYNC_BYTE;

	std::cout << "\nEnter STM32CrapSynth file name:\n" << std::endl;

	std::string filename;
	std::cin >> filename;

	FILE* f = fopen(filename.c_str(), "rb");

	if(f == NULL)
	{
		std::cout << "Couldn't open file!" << std::endl;
		fclose(f);
		return -1;
	}
	if(fseek(f, 59, SEEK_SET) != 0)
	{
		std::cout << "Error, file too short!" << std::endl;
		fclose(f);
		return -1;
	}

	fseek(f, 0, SEEK_SET);

	char magic[5] = {0};
	fread(magic, 4, 1, f);
	magic[4] = '\0';

	if(strncmp(magic, "CRAP", 5) != 0)
	{
		std::cout << "Bad file signature: \"" << magic << "\"!" << std::endl;
		fclose(f);
		return -1;
	}
	uint32_t version = 0;
	fread(&version, sizeof(uint32_t), 1, f);

	if(version > FILE_VERSION)
	{
		std::cout << "File version too high: " << version << "!" << std::endl;
		fclose(f);
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
		fclose(f);
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
		uint8_t data_xor = 0;

		txbuf[1] = (block_size + 8) >> 8;
		txbuf[2] = (block_size + 8) & 0xff;
		txbuf[3] = SYNTH_CMD_LOAD_FLASH;

		txbuf[4] = (flash_samples_pos) & 0xff;
		txbuf[5] = ((flash_samples_pos) >> 8) & 0xff;
		txbuf[6] = ((flash_samples_pos) >> 16) & 0xff;
		txbuf[7] = ((flash_samples_pos) >> 24) & 0xff;

		for(int i = 0; i < block_size; i++) //fill buffer
		{
			fread(&txbuf[8 + i], 1, 1, f);
		}
		for(int i = 0; i < block_size + 8; i++)
		{
			data_xor ^= txbuf[i];
		}

		for(int i = 0; i < 8; i++)
		{
			//snprintf(buffer_cout, 666, "%d %02X", txbuf[i], txbuf[i]);
			//std::cout << buffer_cout << std::endl;
		}

		txbuf[8 + block_size] = data_xor;
		
		snprintf(buffer_cout, 666, "Writing flash samples, block size %d bytes: 0x%08X-0x%08X (%.2f%)", block_size, BASE_ADDR_FLASH + flash_samples_pos, BASE_ADDR_FLASH + flash_samples_pos + block_size, (double)(flash_samples_pos) * 100.0 / (double)flash_samples_size);
		std::cout << buffer_cout << std::endl;

		serial.writeBytes(txbuf, 8 + block_size + 1);

		flash_samples_pos += block_size;

		serial.readBytes(rxbuf, SYNTH_RESPONSE_SIZE);
		our_xor = 0;

		for(int i = 0; i < SYNTH_RESPONSE_SIZE - 1; i++)
		{
			our_xor ^= rxbuf[i];
		}

		if(our_xor != rxbuf[SYNTH_RESPONSE_SIZE - 1])
		{
			std::cout << "Wrong checksum in Flash block receive confirmation packet!" << std::endl;
			fclose(f);
			return -1;
		}

		if(rxbuf[1] != SYNTH_RESPONSE_FLASH_BLOCK)
		{
			std::cout << "Wrong packet ID in Flash block receive confirmation packet: " << rxbuf[1] << "!" << std::endl;
			fclose(f);
			return -1;
		}

		if(rxbuf[2] != data_xor)
		{
			std::cout << "Wrong data checksum in Flash block receive confirmation packet!" << std::endl;
			fclose(f);
			return -1;
		}

		uint32_t mcu_response_start_addr = (uint32_t)rxbuf[3] + ((uint32_t)rxbuf[4] << 8) + ((uint32_t)rxbuf[5] << 16) + ((uint32_t)rxbuf[6] << 24);

		if(mcu_response_start_addr != flash_samples_pos - block_size)
		{
			std::cout << "Wrong data start position in Flash block receive confirmation packet!" << std::endl;
			fclose(f);
			return -1;
		}

		uint32_t mcu_response_data_len = (uint32_t)rxbuf[7] + ((uint32_t)rxbuf[8] << 8);

		if(mcu_response_data_len != block_size)
		{
			std::cout << "Wrong block length telemetry in Flash block receive confirmation packet: " << (int)mcu_response_data_len << "!" << std::endl;
			fclose(f);
			return -1;
		}

		//TODO: uncomment
	}

	char ramsamples[20] = {0};
	fread(ramsamples, 11, 1, f);
	ramsamples[12] = '\0';

	if(strncmp(ramsamples, "RAM SAMPLES", 12) != 0)
	{
		std::cout << "Bad RAM samples block signature: \"" << ramsamples << "\"!" << std::endl;
		fclose(f);
		return -1;
	}

	uint16_t ram_samples_size = 0;
	fread(&ram_samples_size, sizeof(uint16_t), 1, f);

	std::cout << "\nWriting RAM samples " << "(" << ram_samples_size << " bytes)...\n" << std::endl;

	uint16_t ram_samples_pos = 0;

	while(ram_samples_pos < ram_samples_size - 1)
	{
		uint32_t block_size = ((ram_samples_size - ram_samples_pos) > RAM_BLOCK_SIZE ? RAM_BLOCK_SIZE : (ram_samples_size - ram_samples_pos));
		uint8_t data_xor = 0;

		txbuf[1] = (block_size + 8) >> 8;
		txbuf[2] = (block_size + 8) & 0xff;
		txbuf[3] = SYNTH_CMD_LOAD_RAM;

		txbuf[4] = (ram_samples_pos) & 0xff;
		txbuf[5] = ((ram_samples_pos) >> 8) & 0xff;
		txbuf[6] = ((ram_samples_pos) >> 16) & 0xff;
		txbuf[7] = ((ram_samples_pos) >> 24) & 0xff;

		for(int i = 0; i < block_size; i++) //fill buffer
		{
			fread(&txbuf[8 + i], 1, 1, f);
		}
		for(int i = 0; i < block_size + 8; i++)
		{
			data_xor ^= txbuf[i];
		}

		txbuf[8 + block_size] = data_xor;

		snprintf(buffer_cout, 666, "Writing RAM samples, block size %d bytes: 0x%08X-0x%08X (%.2f%)", block_size, BASE_ADDR_RAM + ram_samples_pos, BASE_ADDR_RAM + ram_samples_pos + block_size, (double)(ram_samples_pos) * 100.0 / (double)ram_samples_size);
		std::cout << buffer_cout << std::endl;

		serial.writeBytes(txbuf, 8 + block_size + 1);

		ram_samples_pos += block_size;

		serial.readBytes(rxbuf, SYNTH_RESPONSE_SIZE);
		our_xor = 0;

		for(int i = 0; i < SYNTH_RESPONSE_SIZE - 1; i++)
		{
			our_xor ^= rxbuf[i];
		}

		if(our_xor != rxbuf[SYNTH_RESPONSE_SIZE - 1])
		{
			std::cout << "Wrong checksum in RAM block receive confirmation packet!" << std::endl;
			fclose(f);
			return -1;
		}

		if(rxbuf[1] != SYNTH_RESPONSE_RAM_BLOCK)
		{
			std::cout << "Wrong packet ID in RAM block receive confirmation packet: " << rxbuf[1] << "!" << std::endl;
			fclose(f);
			return -1;
		}

		if(rxbuf[2] != data_xor)
		{
			std::cout << "Wrong data checksum in RAM block receive confirmation packet!" << std::endl;
			fclose(f);
			return -1;
		}

		uint32_t mcu_response_start_addr = (uint32_t)rxbuf[3] + ((uint32_t)rxbuf[4] << 8) + ((uint32_t)rxbuf[5] << 16) + ((uint32_t)rxbuf[6] << 24);

		if(mcu_response_start_addr != ram_samples_pos - block_size)
		{
			std::cout << "Wrong data start position in RAM block receive confirmation packet!" << std::endl;
			fclose(f);
			return -1;
		}

		uint32_t mcu_response_data_len = (uint32_t)rxbuf[7] + ((uint32_t)rxbuf[8] << 8);

		if(mcu_response_data_len != block_size)
		{
			std::cout << "Wrong block length telemetry in RAM block receive confirmation packet!" << std::endl;
			fclose(f);
			return -1;
		}

		//TODO: uncomment
	}

	char regdump[20] = {0};
	fread(regdump, 14, 1, f);
	regdump[15] = '\0';

	if(strncmp(regdump, "REGISTERS DUMP", 15) != 0)
	{
		std::cout << "Bad registers dump block signature: \"" << regdump << "\"!" << std::endl;
		fclose(f);
		return -1;
	}

	uint32_t regdump_size = 0;
	fread(&regdump_size, sizeof(uint32_t), 1, f);

	if(regdump_size > EXTERNAL_FLASH_SIZE)
	{
		std::cout << "Registers dump block is bigger than external Flash memory!" << std::endl;
		std::cout << "Exactly by " << regdump_size - EXTERNAL_FLASH_SIZE << " bytes, if you're curious." << std::endl;
		std::cout << "External Flash size is " << EXTERNAL_FLASH_SIZE << " bytes when regdump is " << regdump_size << " bytes." << std::endl;
		fclose(f);
		return -1;
	}

	std::cout << "\nWriting data (registers dump and wavetable data) to external flash " << "(" << regdump_size << " bytes)...\n" << std::endl;

	uint32_t regdump_pos = 0;

	while(regdump_pos < regdump_size - 1)
	{
		uint32_t block_size = ((regdump_size - regdump_pos) > EXT_FLASH_BLOCK_SIZE ? EXT_FLASH_BLOCK_SIZE : (regdump_size - regdump_pos));
		uint8_t data_xor = 0;

		txbuf[1] = (block_size + 8) >> 8;
		txbuf[2] = (block_size + 8) & 0xff;
		txbuf[3] = SYNTH_CMD_LOAD_EXT_FLASH;

		txbuf[4] = (regdump_pos) & 0xff;
		txbuf[5] = ((regdump_pos) >> 8) & 0xff;
		txbuf[6] = ((regdump_pos) >> 16) & 0xff;
		txbuf[7] = ((regdump_pos) >> 24) & 0xff;

		for(int i = 0; i < block_size; i++) //fill buffer
		{
			fread(&txbuf[8 + i], 1, 1, f);
		}
		for(int i = 0; i < block_size + 8; i++)
		{
			data_xor ^= txbuf[i];
		}

		txbuf[8 + block_size] = data_xor;

		snprintf(buffer_cout, 666, "Writing external flash, block size %d bytes: 0x%08X-0x%08X (%.2f%)", block_size, regdump_pos, regdump_pos + block_size, (double)(regdump_pos) * 100.0 / (double)regdump_size);
		std::cout << buffer_cout << std::endl;

		serial.writeBytes(txbuf, 8 + block_size + 1);

		regdump_pos += block_size;

		serial.readBytes(rxbuf, SYNTH_RESPONSE_SIZE);
		our_xor = 0;

		for(int i = 0; i < SYNTH_RESPONSE_SIZE - 1; i++)
		{
			our_xor ^= rxbuf[i];
		}

		if(our_xor != rxbuf[SYNTH_RESPONSE_SIZE - 1])
		{
			std::cout << "Wrong checksum in external Flash block receive confirmation packet!" << std::endl;
			fclose(f);
			return -1;
		}

		if(rxbuf[1] != SYNTH_RESPONSE_EXTERNAL_FLASH_BLOCK)
		{
			std::cout << "Wrong packet ID in external Flash block receive confirmation packet: " << rxbuf[1] << "!" << std::endl;
			fclose(f);
			return -1;
		}

		if(rxbuf[2] != data_xor)
		{
			std::cout << "Wrong data checksum in external Flash block receive confirmation packet!" << std::endl;
			fclose(f);
			return -1;
		}

		uint32_t mcu_response_start_addr = (uint32_t)rxbuf[3] + ((uint32_t)rxbuf[4] << 8) + ((uint32_t)rxbuf[5] << 16) + ((uint32_t)rxbuf[6] << 24);

		if(mcu_response_start_addr != regdump_pos - block_size)
		{
			std::cout << "Wrong data start position in external Flash block receive confirmation packet!" << std::endl;
			fclose(f);
			return -1;
		}

		uint32_t mcu_response_data_len = (uint32_t)rxbuf[7] + ((uint32_t)rxbuf[8] << 8);

		if(mcu_response_data_len != block_size)
		{
			std::cout << "Wrong block length telemetry in external Flash block receive confirmation packet!" << std::endl;
			fclose(f);
			return -1;
		}

		//TODO: uncomment
	}

	std::cout << "\nDone writing data!\n" << std::endl;

	fclose(f);
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

	std::cout << "Power cycle your device to establish communication..." << std::endl;

	while(!quit)
	{
		load_file(serial);
	}

	serial.closeDevice();
	std::cout << "All done" << std::endl;
	return 0;
}