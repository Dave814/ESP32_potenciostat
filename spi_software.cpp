#include "spi_software.h"
#include "defines.h"

void InitializeSPI()
{
    pinMode(CS1, OUTPUT); //DAC
    pinMode(CS2, OUTPUT); //2x ADC
    pinMode(CS_ONBOARD_ELECTRODES, OUTPUT);
    pinMode(CS_MODULE_1, OUTPUT);
    pinMode(CS_MODULE_2, OUTPUT);
    pinMode(CS_MODULE_3, OUTPUT);

    pinMode(SDIO, INPUT);
    pinMode(SDIO2, INPUT);
    pinMode(CLK, OUTPUT);

    digitalWrite(CS1, HIGH);
    digitalWrite(CS2, HIGH);
    digitalWrite(CS_ONBOARD_ELECTRODES, HIGH);
    digitalWrite(CS_MODULE_1, HIGH);
    digitalWrite(CS_MODULE_2, HIGH);
    digitalWrite(CS_MODULE_3, HIGH);

    digitalWrite(CLK, LOW);
}

void MCP23S09_Init()
{
    pinMode(SDIO,OUTPUT);
    digitalWrite(CS_ONBOARD_ELECTRODES,LOW);
    SPIDelay(); 
    WriteByteSPI(OPCODEW);  //device opcode
    WriteByteSPI(IOCON);    //reg addr
    WriteByteSPI(0x20);     //disable address incrementing
    digitalWrite(CS_ONBOARD_ELECTRODES,HIGH);
    SPIDelay();
    ClockPulse();
    SPIDelay();
    digitalWrite(CS_ONBOARD_ELECTRODES,LOW);
    SPIDelay();
    WriteByteSPI(OPCODEW);  //device opcode
    WriteByteSPI(IODIR);    //reg addr
    WriteByteSPI(0x00);     //set up all pins as output
    digitalWrite(CS_ONBOARD_ELECTRODES,HIGH);
    SPIDelay();
    ClockPulse();
    SPIDelay();
    digitalWrite(CS_ONBOARD_ELECTRODES,LOW);
    SPIDelay();
    WriteByteSPI(OPCODEW);  //opcode
    WriteByteSPI(GPPU);     //Pull up reg addr
    WriteByteSPI(0xFF);     //set up all pull-up resistors
    SPIDelay();
    digitalWrite(CS_ONBOARD_ELECTRODES,HIGH);
    ClockPulse();
    pinMode(SDIO,INPUT);
}

void MCP23S09_Set(uint8_t port)
{
    pinMode(SDIO,OUTPUT);
    digitalWrite(CS_ONBOARD_ELECTRODES,LOW);
    SPIDelay();
    WriteByteSPI(OPCODEW); //device opcode + write command
    WriteByteSPI(GPIO_); //GPIO Addr register
    WriteByteSPI(port);
    SPIDelay();
    digitalWrite(CS_ONBOARD_ELECTRODES,HIGH);
    ClockPulse();
    pinMode(SDIO,INPUT);
}


uint8_t MCP3550_Read(uint8_t *adc_data)
{
    uint8_t data_ready = 0;
    //Poll conversion status
    digitalWrite(CS2, LOW);
    SPIDelay();
    if (!digitalRead(SDIO)) //conversions are ready
    {
        Read2BytesSPI(adc_data, adc_data + 3);
        Read2BytesSPI(adc_data + 1, adc_data + 4);
        Read2BytesSPI(adc_data + 2, adc_data + 5);
        data_ready = 1;
        //start new conversion
        digitalWrite(CS2, HIGH);
        SPIDelay();
        digitalWrite(CS2, LOW);
        SPIDelay();
    }
    digitalWrite(CS2, HIGH);
    SPIDelay();
    return data_ready;
}

void DAC1220_Reset()
{
    digitalWrite(CS1,LOW);
    SPIDelay();
    digitalWrite(CLK,HIGH);
    delayMicroseconds(264);
    digitalWrite(CLK,LOW);
    SPIDelay();
    digitalWrite(CLK,HIGH);
    delayMicroseconds(570);
    digitalWrite(CLK,LOW);
    SPIDelay();
    digitalWrite(CLK,HIGH);
    delayMicroseconds(903);
    digitalWrite(CLK,LOW);
    SPIDelay();
    digitalWrite(CS1,HIGH);
    SPIDelay();    
}

void DAC1220_Write2Bytes(const uint8_t address, const uint8_t byte1, const uint8_t byte2)
{
    digitalWrite(CS1,LOW);
    SPIDelay();
    pinMode(SDIO, OUTPUT);
    WriteByteSPI(32+address);
    WriteByteSPI(byte1);
    WriteByteSPI(byte2);
    pinMode(SDIO,INPUT);
    digitalWrite(CS1,HIGH);
    SPIDelay();
}

void DAC1220_Write3Bytes(const uint8_t address, const uint8_t byte1, const uint8_t byte2, const uint8_t byte3)
{
	digitalWrite(CS1,LOW);
	SPIDelay();
	pinMode(SDIO,OUTPUT);
	WriteByteSPI(64+address);
	WriteByteSPI(byte1);
	WriteByteSPI(byte2);
	WriteByteSPI(byte3);
	pinMode(SDIO,INPUT);
	digitalWrite(CS1,HIGH);
	SPIDelay();
}

void DAC1220_Read2Bytes(const uint8_t address, uint8_t* byte1, uint8_t* byte2)
{
	digitalWrite(CS1,LOW);
	SPIDelay();
	pinMode(SDIO,OUTPUT);
	WriteByteSPI(160+address);
	pinMode(SDIO,INPUT);
	SPIDelay();
	*byte1 = ReadByteSPI();
	*byte2 = ReadByteSPI();
	digitalWrite(CS1,HIGH);
	SPIDelay();
}

void DAC1220_Read3Bytes(const uint8_t address, uint8_t* byte1, uint8_t* byte2, uint8_t* byte3)
{
	digitalWrite(CS1,LOW);
	SPIDelay();
	pinMode(SDIO,OUTPUT);
	WriteByteSPI(192+address);
	pinMode(SDIO,INPUT);
	SPIDelay();
	*byte1 = ReadByteSPI();
	*byte2 = ReadByteSPI();
    *byte3 = ReadByteSPI();
	digitalWrite(CS1,HIGH);
	SPIDelay();
}

void DAC1220_Init()
{
	DAC1220_Write2Bytes(4, 32, 160); // command register: 20-bit resolution; straight binary
	DAC1220_Write3Bytes(0, 64, 0, 0); // set midscale output
}

void DAC1220_SelfCal()
{
	DAC1220_Write2Bytes(4, 32, 161); // command register: 20-bit resolution; straight binary, self calibration mode
}

void Read2BytesSPI(uint8_t *data1_byte, uint8_t *data2_byte)
{
    *data1_byte = 0;
    *data2_byte = 0;
    uint8_t bit_counter = 8;
    do
    {
        ClockPulse();
        *data1_byte <<= 1; // shift composed byte by 1
        *data2_byte <<= 1;
        *data1_byte &= 0xFE; // clear bit 0
        *data2_byte &= 0xFE;
        if (digitalRead(SDIO))           // is data line high
            *data1_byte |= 0x01; // set bit 0 to logic 1
        if (digitalRead(SDIO2))           // is data line high
            *data2_byte |= 0x01; // set bit 0 to logic 1

    } while (--bit_counter);
}

uint8_t ReadByteSPI()
{
	uint8_t data_byte = 0;      // data to be read in
	uint8_t bit_counter = 8;     // set bit count for byte
	do
	{
		ClockPulse();            // generate a clock pulse
		data_byte <<= 1;         // shift composed byte by 1
		data_byte &= 0xFE;       // clear bit 0
		if(digitalRead(SDIO))            // is data line high
			data_byte |= 0x01;   // set bit 0 to logic 1
	} while (--bit_counter);     // repeat until 8 bits have been acquired
	return data_byte;
}

void WriteByteSPI(uint8_t data_byte)
{
	uint8_t bit_counter = 8;     // set bit count for byte
	do
	{
        digitalWrite(SDIO,(data_byte&0x80)?HIGH:LOW);   // output most significant bit
		ClockPulse();                           // generate a clock pulse
		data_byte <<= 1;                        // shift byte to the left
	} while (--bit_counter);                    // repeat until 8 bits have been transmitted
}

void ClockPulse()
{
    //Generate clock pulse
    digitalWrite(CLK,HIGH);
    SPIDelay();
    digitalWrite(CLK,LOW);
    SPIDelay();
}

void SPIDelay()
{
    delayMicroseconds(17);
}