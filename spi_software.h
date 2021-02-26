#ifndef spi_software_H
#define spi_software_H


#include <Arduino.h>

void InitializeSPI();
uint8_t MCP3550_Read(uint8_t * adc_data);
void DAC1220_Reset();
void DAC1220_Write2Bytes(const uint8_t address, const uint8_t byte1, const uint8_t byte2);
void DAC1220_Write3Bytes(const uint8_t address, const uint8_t byte1, const uint8_t byte2, const uint8_t byte3);
void DAC1220_Read2Bytes(const uint8_t address, uint8_t* byte1, uint8_t* byte2);
void DAC1220_Read3Bytes(const uint8_t address, uint8_t* byte1, uint8_t* byte2, uint8_t* byte3);
void DAC1220_Init();
void DAC1220_SelfCal();
uint8_t ReadByteSPI();
void WriteByteSPI(uint8_t data_byte);
void Read2BytesSPI(uint8_t *data1_byte, uint8_t *data2_byte);
void ClockPulse();
void SPIDelay();

void MCP23S09_Init();
void MCP23S09_Set(uint8_t port);


#endif