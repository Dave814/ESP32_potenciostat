//WIFI credentials
//#define STASSID "ADB-CFF9A1"
//#define STAPSK  "rce6bn743cjr"
#define STASSID "Wi-Pi"
#define STAPSK "brg18f12"

//Server addresses
#define SERVERNAME "http://192.168.0.200:1880/espTest"
#define DATASERVER "http://192.168.0.200:1880/dataEndpoint"

//board layout config
#define SINGLEWE false  // FW setup for different boards !!!! 
#define MULTIWE true
//board version config
#define BOARD_VER_2

#define CELL_1 1
#define CELL_2 2

//EEPROM memory addresses
#define DACGAINADDR 0
#define DACOFFSETADDR 4
#define ADCPOTOFFSETADDR 8
#define ADCCURROFFSETADDR 12 
#define SHUNTCALADDR 16

//pinout
#define RANGE_PIN_1 21
#define RANGE_PIN_2 22
#define RANGE_PIN_3 23

#define ONEWIREBUS 14

//communication
#define SDIO 18
#define SDIO2 19
#define CS_MODULE_1 16
#define CS_MODULE_2 4
#define CS_MODULE_3 17 //17 original
#define MODE_SW 15
#define CS1 2
#define CS2 13
#define CS_ONBOARD_ELECTRODES 12 //12 original
#define CLK 5

// variables
#define POTENTIOSTATIC 0
#define GALVANOSTATIC 1
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
    //instructions
#define OPCODEW 0x40

//misc
#define interval 2000 //check for instructions every 2 seconds
