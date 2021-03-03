#ifndef STASSID //Wi-Fi Credentials
//#define STASSID "ADB-CFF9A1"
//#define STAPSK  "rce6bn743cjr"
#define STASSID "Wi-Pi"
#define STAPSK "brg18f12"
#endif

#define SERVERNAME "http://192.168.0.200:1880/espTest"
#define DATASERVER "http://192.168.0.200:1880/dataEndpoint"
//EEPROM memory addresses
#define DACcalAddr 0
#define shuntCalAddr 10
#define ADCCalAddr 20


//pinout
#define RANGE_PIN_1 21
#define RANGE_PIN_2 22
#define RANGE_PIN_3 23

//communication
#define SDIO 18
#define SDIO2 19
#define CS_MODULE_1 16
#define CS_MODULE_2 4
#define CS_MODULE_3 17
#define MODE_SW 15
#define CS1 2
#define CS2 13
#define CS_ONBOARD_ELECTRODES 12
#define CLK 5

// variables
#define POTENTIOSTATIC 1
#define GALVANOSTATIC 0
#define RANGE1 0
#define RANGE2 1
#define RANGE3 2

//MCP23S09 
    //registers
#define IODIR 0x00
#define IPOL 0x01
#define GPINTEN 0x02
#define DEFVAL 0x03
#define INTCON 0x04
#define IOCON 0x05
#define GPPU 0x06
#define INTF 0x07
#define INTCAP 0x08
#define GPIO_ 0x09
#define OLAT 0x0A

#define OPCODEW 0x40
