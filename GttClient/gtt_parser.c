// Print debugging messages
#define DEBUG_PSOC 1

#include <gtt_parser.h>
#include <gtt_device.h>
#include <stdio.h>
#include "gtt_protocol.h"
#include "gtt_events.h"

#if DEBUG_PSOC
#include <project.h>
#include <stdio.h>
char buff[128];
#endif


uint8_t gtt_process_packet(gtt_device *device, size_t packetstart)
{
    (void)packetstart;
    
	size_t originalstart = device->Parser.PacketStart;
	uint8_t WaitingPacket = 0;
	uint8_t Result = 0;
	for (size_t i = 0; i < device->wait_idx; i++)
	{
		if (!device->waitlist[i].Done)
		{
			if (device->waitlist[i].Command == device->Parser.Command)
			{
				if (device->Parser.Command == 250)
				{
					size_t index = device->Parser.PacketStart;
					uint16_t SubCommand = gtt_parser_getU16(device, index, &index);
					if (SubCommand == device->waitlist[i].SubCommand)
					{
						device->waitlist[i].Done = 1;
						WaitingPacket = 1;
					}
				}
				else
				{
					device->waitlist[i].Done = 1;
					WaitingPacket = 1;
				}
			}
			if (WaitingPacket)
				device->waitlist[i].Length = device->Parser.Length;
			break;
		}
	}
	if (device->Parser.Command == 135)
	{
		size_t index = device->Parser.PacketStart;
		eTouchReportingType type = (eTouchReportingType)gtt_parser_getU8(device, index, &index);
		if (device->Parser.Length == 2)
		{
			uint8_t Region = gtt_parser_getU8(device, index, &index);
			if (device->events.regiontouch)
			{
				device->events.regiontouch(device,type, Region);
			}
		} 
		else 
			if (device->Parser.Length == 4)
			{
				uint8_t Region = gtt_parser_getU8(device, index, &index);
				int16_t Value = gtt_parser_getS16(device, index, &index);
				if (device->events.sliderchange)
				{
					device->events.sliderchange(device, type, Region, Value);
				}
			} 
			else
			if (device->Parser.Length == 5)
			{
				uint16_t X = gtt_parser_getU16(device, index, &index);
				uint16_t Y = gtt_parser_getU16(device, index, &index);
				if (device->events.touch)
				{
					device->events.touch(device, type, X, Y);
				}
			}
	}
	if (device->Parser.Command == 165)
	{ 
		if (device->events.key)
		{
			device->events.key(device, device->rx_buffer[1], device->rx_buffer[0]);
		}
		Result = 1;
	}
	else if (device->Parser.Command == 235) //gtt 25 event
	{
		process_gtt25events(device);
		Result = 1;
	}
	if (Result || !WaitingPacket)
	{
		device->Parser.Index = originalstart;
	}
	return Result;
}

void gtt_out_of_bounds(gtt_device* device, uint8_t data)
{
    (void)device;
    (void)data;
    CYASSERT(0);
}

#ifdef GTT_ORIG_PARSER
// The original Matrix Orbital Byte Based Parser
    
uint8_t gtt_parser_process(gtt_device *device)
{
	int Res = device->Read(device);
	if (Res != -1)
	{
		switch (device->Parser.state)
		{
		case GTT_PARSER_IDLE:
			if (Res == 252)
				device->Parser.state = GTT_PARSER_COMMAND;
			else if (Res == 0) // Ignore 0's 
            {
                return 0;
            }
            else
				gtt_out_of_bounds(device,Res);
			break;
		case GTT_PARSER_COMMAND:
			device->Parser.Command = Res;
			device->Parser.state = GTT_PARSER_LENGTH_1;
			break;
		case GTT_PARSER_LENGTH_1:
			device->Parser.Length = Res << 8;
			device->Parser.state = GTT_PARSER_LENGTH_2;
			break;
		case GTT_PARSER_LENGTH_2:
			device->Parser.Length |= Res;
			if (device->Parser.Length)
			{
				device->Parser.PacketStart = device->Parser.Index;
				device->Parser.state = GTT_PARSER_DATA;
			}
			else
			{
				 device->Parser.state = GTT_PARSER_IDLE;
				 uint8_t Result = gtt_process_packet(device, device->Parser.PacketStart);
				 if (Result)
					 return 0;
				 else
					 return 1;
			}
			break;
		case GTT_PARSER_DATA:
			device->rx_buffer[(device->Parser.Index++)] = Res;
			if (device->Parser.Index - device->Parser.PacketStart == device->Parser.Length) // got right # of bytes
			{
				device->Parser.state = GTT_PARSER_IDLE;
				uint8_t Result = gtt_process_packet(device, device->Parser.PacketStart);
				if (Result)
					return 0;
				else
					return 1;
			}
			break;
		}
	}
	return 0;
}

#else

// This function process a whole packet at a time    
uint8_t gtt_parser_process(gtt_device *device)
{
    
    gtt_packet_error_t rval;
    
    rval = device->ReadPacket(device);
    
    if(rval == GTT_PACKET_NODATA)
        return 0;
    
    if(rval != GTT_PACKET_OK)
    {
#if DEBUG_PSOC        
        sprintf(buff,"GTT_PACKET_ERROR %d\r\n",rval);
        UART_UartPutString(buff);
#endif
        return 0; // No data
    }

    device->Parser.PacketStart = device->Parser.Index;
    device->Parser.Index += device->Parser.Length;
    
	uint8_t Result = gtt_process_packet(device, device->Parser.PacketStart);
	if (Result)
	    return 0;
	else
	    return 1;

}
#endif


size_t gtt_parser_waitpacket(gtt_device *device,int packetType)
{
	gtt_waitlist_item *item = &device->waitlist[device->wait_idx++];
	item->Command = packetType;
	item->SubCommand = 0;
	item->Done = 0;
	while (!item->Done)
		gtt_parser_process(device);
	size_t result = item->PacketStart;
	device->Parser.Index = item->PacketStart;
	device->wait_idx--;
	return result;
}

size_t gtt_parser_waitpacket_250(gtt_device *device, uint16_t commandID)
{
	gtt_waitlist_item *item;
	item = &device->waitlist[device->wait_idx++];
	item->Command = 250;
	item->SubCommand = commandID;
	item->Done = 0;
	while (!item->Done)
		gtt_parser_process(device);
	device->wait_idx--;
	size_t result = device->Parser.PacketStart;
	device->Parser.Index = item->PacketStart;
	return result+2;
}

// ARH - changed the way bytes were built up to fix alignment bug
#if 0

#define swap32(a)                    \
	((((a) >> 24) & 0x000000ff) | \
	(((a) >> 8) & 0x0000ff00) | \
	(((a) << 8) & 0x00ff0000) | \
	(((a) << 24) & 0xff000000))


#define swap16(a) ((((a) >> 8) & 0x00ff) | (((a) << 8) & 0xff00))

#endif

uint32_t gtt_parser_getU32(gtt_device* device, size_t index, size_t *outIndex)
{
    uint32_t data = device->rx_buffer[index]<<24 | device->rx_buffer[index+1]<<16 |
	        device->rx_buffer[index+2]<<8 | device->rx_buffer[index+3];
	
	*outIndex = index + 4;
    return data;
}

uint16_t gtt_parser_getU16(gtt_device* device, size_t index, size_t *outIndex)
{
    
    uint16_t data = (device->rx_buffer[index]<<8 | device->rx_buffer[index+1]);	
	*outIndex = index + 2;
    return data;
}


int16_t gtt_parser_getS16(gtt_device* device, size_t index, size_t *outIndex)
{
    int16_t data = (device->rx_buffer[index]<<8 | device->rx_buffer[index+1]);
	    
	*outIndex = index + 2;
    return data;
}


uint8_t gtt_parser_getU8(gtt_device* device, size_t index, size_t *outIndex)
{
	*outIndex = index + 1;
	return device->rx_buffer[index];
}

float gtt_parser_getFloat(gtt_device* device, size_t index, size_t *outIndex)
{
	uint32_t val;
	val = gtt_parser_getU32(device, index, outIndex);
	float res = *(float*)&val;
	return res;
}

float gtt_parser_getNumber(gtt_device* device, size_t index, size_t *outIndex)
{
    (void)device;
    (void)index;
    (void)outIndex;
	NIWARN
	return 0;
}


gtt_bytearray_l8 gtt_parser_getByteArrayL8(gtt_device* device, size_t index, size_t *outIndex)
{
	gtt_bytearray_l8 res = { .Data = 0 , .length = 0 };
	res.length = device->rx_buffer[index];
	res.Data = &device->rx_buffer[index+1];
	*outIndex = 1 + res.length;
	return res;
}


gtt_bytearray_l16 gtt_parser_getByteArrayL16(gtt_device* device, size_t index, size_t *outIndex)
{
	gtt_bytearray_l16 res = { .Data = 0 , .length = 0 };
	res.length = gtt_parser_getU16(device, index, &index);
	res.Data = &device->rx_buffer[index]; 
	*outIndex = index + res.length;
	return res;
}

gtt_wordarray_l32 gtt_parser_getWordArrayL32(gtt_device* device, size_t index, size_t *outIndex)
{
	gtt_wordarray_l32 res = { .Data = 0 , .length = 0 };
	res.length = gtt_parser_getU32(device, index, &index);
	res.Data = (int16_t*) &device->rx_buffer[index];
	for (size_t i = 0; i < res.length; i++)
	{
		res.Data[i] = gtt_parser_getS16(device, index, &index);
	}
	*outIndex = index;
	return res;

	return res;
}

char* gtt_parser_getStringASCII(gtt_device* device, size_t index, size_t *outIndex)
{
	char* Result = (char *)&device->rx_buffer[index];
	while (device->rx_buffer[index] != 0)
		index++;
	*outIndex = index+1;
	return Result;
} 

gtt_text gtt_parser_getText(gtt_device* device, size_t index, size_t *outIndex)
{
	gtt_text result = { .Data = 0 , .Length = 0, .Encoding = 0 };
	result.Encoding = gtt_parser_getU8(device, index, &index);
	result.Length = gtt_parser_getU16(device, index, &index);
	result.Data = &device->rx_buffer[index];
	*outIndex = index + result.Length;
	return result;
}
