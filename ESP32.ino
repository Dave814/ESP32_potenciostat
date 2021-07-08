#include <OneWire.h>

#include <ArduinoJson.h>

#include <RTC.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <HTTPClient.h>
#include <math.h>
#include "spi_software.h"
#include "defines.h"
#include <EEPROM.h>

#define DEBUG // leave uncomennted for serial debug prints

#ifdef DEBUG
#define DPRINT(...) Serial.print(__VA_ARGS__)
#define DPRINTLN(...) Serial.println(__VA_ARGS__)
#else
#define DPRINT(...)
#define DPRINTLN(...)
#endif

const char *ssid = STASSID;
const char *password = STAPSK;
const char *serverName = SERVERNAME;
const char *dataServer = DATASERVER;

HTTPClient http;

struct cv_data
{
    int ustart;
    int ustop;
    int ubound;
    int lbound;
    int scanrate;
    int n;
    unsigned long starttime;
};

uint8_t DACCalBytes[6];

float potential;
double current;
long raw_current;
long raw_potential;
long potential_offset;
long current_offset;
long dac_offset;
long dac_gain;
//double shunt_calibration[] = {0.99236, 1.0000, 1.0675};
double shunt_calibration[] = {1.00000, 0.1000, 1.0000};

char current_range = RANGE2;

unsigned long oldTime = 0;

//FIXME: testing section
cv_data cv_params = {0, 400, 400, 0, 5, 2, 0};
bool stopFlag = false;
//FIXME: testing section

StaticJsonDocument<200> doc;

OneWire ds(ONEWIREBUS);


void setup()
{
#ifdef DEBUG
    Serial.begin(115200); // debug Serial
#endif
    configPinout();
    InitializeSPI();
    set_current_range();
    set_control_mode(POTENTIOSTATIC);
    set_cell_status(CELL_1, false);
    set_cell_status(CELL_2, false);

    delay(25);
    DAC1220_Reset();
    delay(25);
    DAC1220_Init();

    MCP23S09_Init();

    EEPROM.begin(4 * sizeof(long) + 3 * sizeof(double));
    //EEPROM.get(DACOFFSETADDR,dac_offset);
    //EEPROM.get(DACGAINADDR,dac_gain);
    //EEPROM.get(ADCPOTOFFSETADDR,potential_offset);
    //EEPROM.get(ADCCURROFFSETADDR,current_offset);
    //EEPROM.get(SHUNTCALADDR,shunt_calibration);

    startConnection();


    //set_dac_calibration(dac_offset, dac_gain);
    set_offset(-290, 121);
    set_dac_calibration(-515673, 527005);
}

void stopConnection()
{
    http.end();
    WiFi.mode(WIFI_OFF);
    DPRINTLN("Connection stopped !");
}

void startConnection()
{
    WiFi.begin(ssid, password);
    DPRINTLN("");

    // Wait for connection
    uint8_t connectionCounter = 0;
    while (WiFi.status() != WL_CONNECTED) // Reboot if not connected in 5seconds... softBug ?
    {
        delay(500);
        DPRINT(".");
        connectionCounter++;
        if (connectionCounter > 10)
        {
            DPRINTLN("Rebooting ESP.......");
            ESP.restart();
        }
    }

    DPRINT("\n Connected to ");
    DPRINTLN(ssid);
    DPRINT("IP address: ");
    DPRINTLN(WiFi.localIP());

    http.begin(serverName);
    http.addHeader("Content-Type", "application/json");
}

void configPinout() //can be written in register type to perform faster
{
    pinMode(RANGE_PIN_1, OUTPUT);
    pinMode(RANGE_PIN_2, OUTPUT);
    pinMode(RANGE_PIN_3, OUTPUT);
    pinMode(MODE_SW, OUTPUT);
}

void loop(void)
{
    if (oldTime + interval <= millis()) // once per 2 seconds check if server has some commands
    {
        DPRINTLN("requesting server commands");
        int resp = http.GET();
        DPRINT("Response from server : ");
        DPRINTLN(resp);
        if (resp == 200)
        {
            DPRINTLN("Resolving server request");
            //resolveServerRequest();
            parseJsonCommand();
        }
        else
        {
            DPRINTLN("Do nothing");
        }
        oldTime = millis();
    }
}

void parseJsonCommand()
{
    String responseString = http.getString();
    char buff[100];
    byte MeasMode = 0;
    responseString.toCharArray(buff, sizeof(buff));

    DeserializationError error = deserializeJson(doc, buff);
    if (error)
    {
        DPRINT("Deserialization error: ");
        DPRINTLN(error.f_str());
    }
    else
    {
        int command = doc["command"];
        switch (command)
        {
        case 0x00:
            //statements
            break;
        case 0x01:
            if (doc["cellNum"] == 1)
            {
                set_cell_status(CELL_1, true);
            }
            else if (doc["cellNum"] == 2)
            {
                set_cell_status(CELL_2, true);
            }
            break;
        case 0x02:
            if (doc["cellNum"] == 1)
            {
                set_cell_status(CELL_1, false);
            }
            else if (doc["cellNum"] == 2)
            {
                set_cell_status(CELL_2, false);
            }
            break;

        case 0x10:
            MeasMode = doc["mode"];
            if (MeasMode == POTENTIOSTATIC)
            {
                set_control_mode(POTENTIOSTATIC);
                break;
            }
            else if (MeasMode == GALVANOSTATIC)
                ;
            {
                set_control_mode(GALVANOSTATIC);
                break;
            }
            break;

        case 0x15:
            current_range = doc["range"];
            set_current_range();
            break;

        case 0x20:
            set_output(doc["value"], doc["voltage"]);
            break;
        case 0x21:
            read_avg_pot_current();
            break;
        case 0x30:
            //cv
            break;
        case 0x40:
            set_dac_calibration(doc["offset"], doc["gain"]);
            break;
        case 0x41:
            set_offset(doc["Voffset"], doc["Coffset"]);
            break;
        case 0x42:
            shunt_calibration[0] = doc["S1"];
            shunt_calibration[1] = doc["S2"];
            shunt_calibration[2] = doc["S3"];
            ShuntCalSave();
            break;
        case 0x43:
            ShuntCalRead();
            break;
        case 0x44:
            get_offset();
            break;
        case 0x45:
            get_dac_calibration();
            break;
        case 0x46:
            dac_auto_calibrate();
            break;
        case 0x47:
            autoZero();
            break;
        case 0xA0:
            measurepH();
            break;
        case 0xA1:
            run_cv();
            break;
        case 0xA2:
            readTemp();
            break;
        case 0xEE:
        cv_params.ustart = doc["StartP"];
        cv_params.ustop = doc["StopP"];
        cv_params.ubound = doc["UB"];
        cv_params.lbound = doc["LB"];
        cv_params.scanrate = doc["SR"];
        cv_params.n = doc["N"];

        DPRINT("CV params");
        DPRINTLN(cv_params.ustart);
        DPRINTLN(cv_params.ustop);
        DPRINTLN(cv_params.ubound);
        DPRINTLN(cv_params.lbound);
        DPRINTLN(cv_params.scanrate);
        DPRINTLN(cv_params.n);
            break;
        case 0xFF:
            measCurrent();
            break;
        default:
            //do nothing
            break;
        }
    }
}

void resolveServerRequest() //TODO: change the way commands are received. Change the string to hex number. i.e. : CELL ON = 0x01;
{
    String responseString = http.getString(); // read server response
    char buff[100];
    responseString.toCharArray(buff, sizeof(buff));

    DeserializationError error = deserializeJson(doc, buff);
    if (!error)
    {
        cv_params.ustart = doc["StartP"];
        cv_params.ustop = doc["StopP"];
        cv_params.ubound = doc["UB"];
        cv_params.lbound = doc["LB"];
        cv_params.scanrate = doc["SR"];
        cv_params.n = doc["N"];

        DPRINT("CV params");
        DPRINTLN(cv_params.ustart);
        DPRINTLN(cv_params.ustop);
        DPRINTLN(cv_params.ubound);
        DPRINTLN(cv_params.lbound);
        DPRINTLN(cv_params.scanrate);
        DPRINTLN(cv_params.n);
    }
    else
    {
        DPRINT("Deserialization error: ");
        DPRINTLN(error.f_str());
    }
    DPRINTLN(responseString);
    if (responseString == "CELL 1 ON") //will be changed to switch case construction
    {
        set_cell_status(CELL_1, true);
    }
    else if (responseString == "CELL 1 OFF")
    {
        set_cell_status(CELL_1, false);
    }
    if (responseString == "CELL 2 ON") //will be changed to switch case construction
    {
        set_cell_status(CELL_2, true);
    }
    else if (responseString == "CELL 2 OFF")
    {
        set_cell_status(CELL_2, false);
    }
    else if (responseString == "POTENTIOSTATIC")
    {
        set_control_mode(POTENTIOSTATIC);
    }
    else if (responseString == "GALVANOSTATIC")
    {
        set_control_mode(GALVANOSTATIC);
    }
    else if (responseString == "RANGE1")
    {
        current_range = RANGE1;
        set_current_range();
    }
    else if (responseString == "RANGE2")
    {
        current_range = RANGE2;
        set_current_range();
    }
    else if (responseString == "RANGE3")
    {
        current_range = RANGE3;
        set_current_range();
    }
    else if (strncmp(buff, "DACSET ", 7) == 0)
    {
        long DACdata = atoi(&buff[7]);
        set_output(DACdata, true);
    }
    else if (responseString == "DACCAL")
    {
        dac_auto_calibrate();
    }
    else if (responseString == "ADCREAD") //read data from ADC converters 3x2bytes
    {
        read_potential_current();
        DPRINT(potential, 5);
        DPRINT("    ");
        DPRINT(current, 10);
        DPRINTLN(" ");
    }
    else if (responseString == "OFFSETREAD")
    {
        get_offset();
    }
    else if (responseString == "OFFSETSAVE")
    {
        set_offset(-208, -288); //FIXME: data from GUI
    }
    else if (responseString == "DACCALGET")
    {
        get_dac_calibration();
    }
    else if (responseString == "DACCALSET")
    {
        set_dac_calibration(-514936, 528136); //FIXME: data from GUI
    }
    else if (responseString == "SHUNTCALREAD")
    {
        ShuntCalRead();
    }
    else if (responseString == "SHUNTCALSAVE")
    {
        ShuntCalSave();
    }
    else if (strncmp(buff, "CV ", 3) == 0)
    {
        run_cv();
    }
    else if (responseString == "AUTOZERO")
    {
        autoZero();
    }
    else if (responseString == "test")
    {
        //stopConnection();
        //section for testing new functions
        measCurrent();
    }
    else if (responseString == "measurepH")
    {
        measurepH();
    }
    else if (strncmp(buff, "{LB:", 4) == 0)
    {
        DPRINTLN("CV data IN");
    }
    else
    {
        DPRINTLN("Do nothing");
    }
}

void set_cell_status(int cellnum, bool cell_state) //inverted logic
{
    if (cell_state)
    {
        if (cellnum == CELL_1)
        {
            MCP23S09_Set(0b11110000);
            DPRINTLN("command - CELL 1 ON");
        }
        else if (cellnum == CELL_2)
        {
            MCP23S09_Set(0b00001111);
            DPRINTLN("command - CELL 2 ON");
        }
    }
    else
    {
        if (cellnum == CELL_1)
        {
            MCP23S09_Set(0b11111111);
            DPRINTLN("command - CELL 1 OFF");
        }
        else if (cellnum == CELL_2)
        {
            MCP23S09_Set(0b11111111);
            DPRINTLN("command - CELL 2 OFF");
        }
    }
}

void set_control_mode(bool mode)
{
    if (mode)
    {
        DPRINTLN("Galvanostatic mode");
        digitalWrite(MODE_SW, GALVANOSTATIC);
    }
    else
    {
        DPRINTLN("Potentiostatic mode");
        digitalWrite(MODE_SW, POTENTIOSTATIC);
    }
}

void set_current_range() // beware of inverted logic on older boards!!!
{
    if (current_range == RANGE1) //20mA //10R
    {
        DPRINTLN("Range1");
        digitalWrite(RANGE_PIN_1, HIGH);
        digitalWrite(RANGE_PIN_2, LOW);
        digitalWrite(RANGE_PIN_3, LOW);
    }
    else if (current_range == RANGE2) //200uA //1k
    {
        DPRINTLN("Range2");
        digitalWrite(RANGE_PIN_1, LOW);
        digitalWrite(RANGE_PIN_2, HIGH);
        digitalWrite(RANGE_PIN_3, LOW);
    }
    else if (current_range == RANGE3) //2uA //100k
    {
        DPRINTLN("Range3");
        digitalWrite(RANGE_PIN_1, LOW);
        digitalWrite(RANGE_PIN_2, LOW);
        digitalWrite(RANGE_PIN_3, HIGH);
    }
}

void read_potential_current()
{
    unsigned char adcData[6]; //readout data from ADC
    if (MCP3550_Read(adcData) != 0)
    {
        raw_potential = twocomplement_to_decimal(adcData[0], adcData[1], adcData[2]);
        raw_current = twocomplement_to_decimal(adcData[3], adcData[4], adcData[5]);
        potential = (raw_potential - potential_offset) / 2097152.0 * 8.0;                                                                          //calculate potential in V compensating for offset
        current = ((double)(raw_current - current_offset)) / 2097152.0 * 25.0 / pow((shunt_calibration[current_range] * 100.0000), current_range); //calculate current in mA, taking current range into account and compensating for offset
                                                                                                                                                   // current = (((double)(raw_current - current_offset)) / 2097152.0 * 25.0 / pow((100.00), current_range) * shunt_calibration[current_range]);
    }
    else
    {
        DPRINTLN("Wait for ADC conversion");
        {
            for (int i = 0; i < 5; i++)
            {
                delay(10);
            }
            read_potential_current();
        }
    }
}

void read_avg_pot_current()
{
    read_potential_current();
    delay(100);
    read_potential_current();
    delay(100);
    double avgCurr = 0;
    double avgPot = 0;

    for (int i = 0; i < 20; i++)
    {
        read_potential_current();
        avgCurr += current;
        avgPot += potential;
        delay(100);
    }
    avgCurr = avgCurr / 20.0000;
    avgPot = avgPot / 20.0000;

    DPRINT(avgPot, 10);
    DPRINT("    ");
    DPRINT(avgCurr, 10);
    DPRINTLN(" ");
}

void get_offset() //Read 6 offset bytes from flash, bytes [0:3] = potential offset , bytes [4:6] = current offset
{
    DPRINTLN("command - OFFSETREAD");
    EEPROM.get(ADCPOTOFFSETADDR, potential_offset);
    EEPROM.get(ADCCURROFFSETADDR, current_offset);
    DPRINTLN("Offset values were read from flash memory");
    DPRINTLN("Potential offset : " + String(potential_offset));
    DPRINTLN("Current offset : " + String(current_offset));
}

void set_offset(long potOff, long currOff) //Set 6 offset bytes to flash, bytes [0:3] = potential offset , bytes [4:6] = current offset
{

    DPRINTLN("OFFSETSAVE command");
    EEPROM.put(ADCPOTOFFSETADDR, potOff);
    EEPROM.put(ADCCURROFFSETADDR, currOff);
    EEPROM.commit();
}

void get_dac_calibration() //read DAC calibration, and save it to EEPROM
{
    DPRINTLN("DACCALGET command");
    unsigned char dac_cal_data[6];

    DAC1220_Read3Bytes(8, &dac_cal_data[0], &dac_cal_data[1], &dac_cal_data[2]);
    DAC1220_Read3Bytes(12, &dac_cal_data[3], &dac_cal_data[4], &dac_cal_data[5]);
    if ((dac_cal_data[0] & dac_cal_data[1] & dac_cal_data[2] & dac_cal_data[3] & dac_cal_data[4] & dac_cal_data[5]) != 255) // if no offset values has been stored, all bits will be set
    {
        dac_offset = DAC_bytes_to_decimal(dac_cal_data[0], dac_cal_data[1], dac_cal_data[2]);
        dac_gain = DAC_bytes_to_decimal(dac_cal_data[3], dac_cal_data[4], dac_cal_data[5]) + (1 << 19);
        DPRINTLN("DAC offset value : " + String(dac_offset));
        DPRINTLN("DAC gain valuie : " + String(dac_gain));
        EEPROM.put(DACOFFSETADDR, dac_offset);
        EEPROM.put(DACOFFSETADDR, dac_offset);
        EEPROM.commit();
    }
    else
    {
        DPRINTLN("No offset values were found in flash memory");
    }
}

void set_dac_calibration(long offset, long gain) //manually set calibration data, read it back and store in EEPROM
{
    char calByte[6];
    DPRINTLN("DACCALSET command");
    decimal_to_dac_bytes(offset, &calByte[0], &calByte[1], &calByte[2]);
    decimal_to_dac_bytes((gain - (1 << 19)), &calByte[3], &calByte[4], &calByte[5]);
    DAC1220_Write3Bytes(8, calByte[0], calByte[1], calByte[2]);
    DAC1220_Write3Bytes(12, calByte[3], calByte[4], calByte[5]);
    get_dac_calibration();
}

void dac_auto_calibrate() //autocalibrate DAC, read the calibration and save it to EEPROM
{
    DPRINTLN("command - DAC auto calibrate");
    DAC1220_SelfCal();
    delay(500);
    get_dac_calibration();
}

void set_output(long value, bool volts) //set output in mV or mA
{
    char DACbyte1, DACbyte2, DACbyte3;
    //DPRINTLN("Received value : " + String(value));
    float calculatedVal = 0;
    if (volts)
    {
        calculatedVal = ((value / 1000.0) / 8.0 * pow(2, 19)) + int(round((float)potential_offset / 4.0));
    }
    else
    {
        calculatedVal = ((value / 1000.0) / 25.0 / (pow((shunt_calibration[current_range] * 100.0), current_range)) * pow(2, 19)) + int(round((float)current_offset / 4.0));
    }
    //DPRINTLN("calulated value : "+ String(calculatedVal));
    decimal_to_dac_bytes(calculatedVal, &DACbyte1, &DACbyte2, &DACbyte3);
    DAC1220_Write3Bytes(0, DACbyte1, DACbyte2, DACbyte3);
}

void ShuntCalRead()
{
    DPRINTLN("SHUNTCALREAD command");
    EEPROM.get(SHUNTCALADDR, shunt_calibration);
    for (int i = 0; i < 3; i++)
    {
        DPRINT("ShuntCal ");
        DPRINT(i);
        DPRINT(" ");
        DPRINTLN(shunt_calibration[i], 5);
    }
}

void ShuntCalSave()
{
    DPRINTLN("SHUNTCALSAVE command");
    EEPROM.put(SHUNTCALADDR, shunt_calibration);
    EEPROM.commit();
}

void decimal_to_dac_bytes(float value, char *msb, char *mid, char *lsb) // Working like a charm
{
    //Convert a floating-point number, ranging from -2**19 to 2**19-1, to three data bytes in the proper format for the DAC1220.
    long code = (1 << 19) + int(round((long)value)); // Convert the (signed) input value to an unsigned 20-bit integer with zero at midway ((1<<19) +
    if (code > ((1 << 20) - 1))                      // crop the code if it is not within 20bytes
        code = (1 << 20) - 1;
    else if (code < 0)
        code = 0;

    //DPRINT("Code : ");
    //DPRINTLN(code);
    code = code << 4;
    *msb = (code & 0x00FF0000) >> 16; //MSB
    *mid = (code & 0x0000FF00) >> 8;
    *lsb = (code & 0x000000FF); //LSB
}

long DAC_bytes_to_decimal(char byte2, char byte1, char byte0)
{
    long code = ((long)byte2 << 16);
    code = code | ((long)byte1 << 8);
    code = code | byte0;
    code = code >> 4;
    code = code & 0x00FFFFFF;
    return code - (1 << 19);
}

long twocomplement_to_decimal(int msb, int middlebyte, int lsb) // Working like a charm
{                                                               // Convert a 22-bit two-complement ADC value consisting of three bytes to a signed integer
    bool ovh = false;
    bool ovl = false;
    long answer;

    if (msb > 63 && msb < 128) // check for high overflow
    {
        ovh = true;
    }

    if (msb > 127) // check for low overflow
    {
        ovl = true;
    }
    long combined_value = ((msb % 64) << 16) + (middlebyte << 8) + lsb; // get rid of overflow bits
    if (!ovh && !ovl)
    {
        if (msb > 31) //b21 set -> negative number
        {
            answer = combined_value - (1 << 22);
        }
        else
        {
            answer = combined_value;
        }
    }
    else // overflow
    {
        if (msb > 127) //b23 set -> negative number
        {
            answer = combined_value - (1 << 22);
        }
        else
        {
            answer = combined_value;
        }
    }
    //DPRINT("ADC Answer : ");
    //DPRINTLN(answer);
    return answer;
}
/*
int autorange(int current) //return num of measurements to skip to suppress artifacts
{
    float relativecurrent=abs(current/(20/100)<<currentrange);

    if((relativecurrent > 1.05) && (currentrange !=0))
    {
        overcounter +=1;
    }
    else 
    {
        overcounter = 0;
    }
    if((relativecurrent < 0.0095)  &&(currentrange =!2))
    {
        undercounter -=1;
    }
    else 
    {
        undercounter = 0; 
    }

    if(overcounter > 3)
    {
        currentrange -=1;
        //setRange() //TODO:
        overcounter = 0;   
        return 2; //skip next two measurements to suppress artifacts
    }
    else if(undercounter > 3)
    {
        currentrange +=1;
        //setRange() //TODO:
        undercounter = 0;
        return 2; //skip next two measurements to suppress artifacts
    }
    else return 0;
}
*/
int current_range_from_current()
{
    current = abs(current);
    if (current <= 0.002)
    {
        return 2; // lowest current range (2uA)
    }
    else if (current <= 0.2)
    {
        return 1; // intermediate current range (200uA)
    }
    else
    {
        return 0; // highest current range (20mA)
    }
}

void autorange()
{
    float meas_current = abs(current);
    if (current_range == RANGE3) //2
    {
        if (meas_current > 0.00225)
        {
            current_range = RANGE2;
        }
    }
    else if (current_range == RANGE2)
    {
        if (meas_current > 0.225)
        {
            current_range = RANGE1;
        }
        else if (meas_current < 0.002)
        {
            current_range = RANGE3;
        }
    }
    else if (current_range == RANGE1)
    {
        if (meas_current < 0.2)
        {
            current_range = RANGE2;
        }
    }

    set_current_range();
}

void run_cv()
{
    if (valiadate_parameters())
    {
        cv_start_setup();
        while (!stopFlag)
        {
            cv_update();
            delay(100);
        }
    }
}

void cv_start_setup() // get ready for cv measurement
{
    http.end();
    delay(100);
    http.begin(dataServer);
    http.addHeader("Content-Type", "application/json");
    cv_params.starttime = millis();
    set_output(cv_params.ustart, true);
    set_control_mode(POTENTIOSTATIC);
    //current_range = RANGE3; // middle range - possible to start with highest range
   // set_current_range();
    //set_cell_status(true);
    read_potential_current(); //read out two samples and discard them
    read_potential_current();
}

bool valiadate_parameters()
{
    if (cv_params.ubound < cv_params.lbound)
    {
        DPRINTLN("Upper bound cannot be lower than lower bound");
        return false;
    }
    else if (cv_params.scanrate == 0)
    {
        DPRINTLN("Scanrate cannot be zero");
        return false;
    }
    else if (cv_params.scanrate > 0 && (cv_params.ubound < cv_params.ustart))
    {
        DPRINTLN("For a positive scan rate, the start potential must be lower than the upper bound");
        return false;
    }
    else if ((cv_params.ubound > 8000) || (cv_params.ubound < -8000))
    {
        DPRINTLN("Upper bound is outside interval");
        return false;
    }
    else if ((cv_params.lbound > 8000) || (cv_params.lbound < -8000))
    {
        DPRINTLN("Lower bound is outside interval");
    }
    else if ((cv_params.ustart > 8000) || (cv_params.ustart < -8000))
    {
        DPRINTLN("Start potential is outside interval");
        return false;
    }
    else if ((cv_params.ustop > 8000) || (cv_params.ustop < -8000))
    {
        DPRINTLN("Stop potential is outside interval");
        return false;
    }

    else
        return true;
}

int cv_sweep(unsigned long time_elapsed, int ustart, int ustop, int ubound, int lbound, int scanrate, int n) //Works like a charm
{
    // ustart -- start potential [mV]
    // ustop -- stop potential  [mV]
    // ubound -- upper potential bound  [mV]
    // lbound -- lower potential bound  [mV]
    // scanrate -- scanrate in mV/s
    // n -- number of scans
    int srt0 = ubound - ustart;           // Potential difference in initial stage  :  Start --> Upper boundary
    int srt1 = (ubound - lbound) * 2 * n; //Potential difference in cyclic stage :  (Lower boundary --> upper boundary) * 2 * number of scans
    int srt2 = abs(ustop - ubound);       // Potential difference in final stage : (Upper boundary -- > Stop)

    long srtime = scanrate * time_elapsed / 1000; // Actual potential in mV vs time

    if (srtime < srt0) //Initial stage
    {
        return ustart + srtime;
    }
    else if (srtime < (srt0 + srt1)) //Cyclic stage
    {
        srtime = srtime - srt0;
        return (lbound + abs(srtime % (2 * (ubound - lbound)) - (ubound - lbound)));
    }
    else if (srtime < (srt0 + srt1 + srt2)) // Final stage
    {
        srtime = srtime - srt0 - srt1;
        if (ustop > ubound)
        {
            return (ubound + srtime);
        }
        else
        {
            return (ubound - srtime);
        }
    }
    else
    {
        stopFlag = true;
        ; //CV finished
    }
}

void cv_update()
{
    long elapsed_time = millis() - cv_params.starttime;
    long cv_output = cv_sweep(elapsed_time, cv_params.ustart, cv_params.ustop, cv_params.ubound, cv_params.lbound, cv_params.scanrate, cv_params.n);

    if (stopFlag)
    {
        cv_stop();
    }
    else
    {
        set_output(cv_output, true);
        read_potential_current();
        DPRINT(potential, 5);
        DPRINT(",");
        DPRINT(current, 10);
        DPRINTLN(",");
        //current_range = current_range_from_current();
        //autorange();

        //char dataString[80];

        //sprintf(dataString, "{\"MeasuredData\":{\"Voltage\":%f,\"Current\":%f}}",potential,current);
        //String abc = dataString.toString();
        //serializeJson(abc,jsonData);
        //String dataString = "{\"MeasuredData\":{\"Voltage\":%f,\"Current\":%f}}";
        //http.POST(jsonData);
    }
}

void cv_stop() // set device to idle settings
{
    set_cell_status(CELL_1, false);
    set_cell_status(CELL_2, false);
    set_output(0, 1);
}

void autoZero() // get at l east 50 samples to average with shorted SE & RE
{
    set_cell_status(CELL_1, true);
    current_range = RANGE2;
    set_current_range();
    read_potential_current();
    delay(100);
    read_potential_current(); //discard two samples;
    potential_offset = 0;
    current_offset = 0;
    long potSum = 0;
    long currSum = 0;
    for (int i = 0; i < 50; i++)
    {
        read_potential_current();
        delay(100);
        potSum += raw_potential;
        currSum += raw_current;
    }

    potential_offset = potSum / 50;
    current_offset = currSum / 50;
    DPRINT("Potential offset : ");
    DPRINTLN(potential_offset);
    DPRINT("Current offset : ");
    DPRINTLN(current_offset);

    set_offset(potential_offset, current_offset);
}

void measCurrent()
{
    set_control_mode(POTENTIOSTATIC);
    //set_cell_status(true);
    read_potential_current();
    delay(100);
    read_potential_current();
    const int numReadings = 10;
    float currReadings[numReadings];
    float potReadings[numReadings];
    int readIndex = 0;
    float currTotal = 0;
    float potTotal = 0;
    float currAverage = 0;
    float potAverage = 0;

    for (int thisReading = 0; thisReading < numReadings; thisReading++)
    {
        currReadings[thisReading] = 0;
        potReadings[thisReading] = 0;
    }

    while (1)
    {
        read_potential_current();
        // DPRINT(potential,5);
        // DPRINT(",");
        // DPRINTLN(current,10);
        currTotal = currTotal - currReadings[readIndex];
        potTotal = potTotal - potReadings[readIndex];

        currReadings[readIndex] = current;
        potReadings[readIndex] = potential;

        currTotal = currTotal + currReadings[readIndex];
        potTotal = potTotal + potReadings[readIndex];

        readIndex = readIndex + 1;

        if (readIndex >= numReadings)
        {
            readIndex = 0;
        }
        currAverage = currTotal / numReadings;
        potAverage = potTotal / numReadings;

        DPRINT(potAverage, 7);
        DPRINT(",");
        DPRINTLN(currAverage, 10);
        //autorange();

        delay(100);
    }
}

void measurepH()
{
    DPRINTLN("MEASURE pH");
    set_control_mode(POTENTIOSTATIC);
    //set_cell_status(true);
    read_potential_current();
    delay(100);
    read_potential_current();
    delay(100);

    int numpHsamples = 100;
    float phAverage = 0.0;

    for (int i = 0; i < numpHsamples; i++)
    {
        read_potential_current();
        phAverage += potential;
        delay(100);
    }
    phAverage = phAverage / (float)numpHsamples;
    float pH = -17.397 * phAverage + 7.0309; //kalibračna krivka
    DPRINT("Measured pH :");
    DPRINTLN(pH, 2);

    //set_cell_status(false);
}

void readTemp()
{
    while(1)
    {
    byte tempData[12];
    byte addr[]={40,255,101,99,146,21,1,185};
    float celsius;
    byte present = 0;

    if (OneWire::crc8(addr, 7) != addr[7])
    {
        DPRINTLN("CRC is not valid!");
        return;
    }

    ds.reset();
    ds.select(addr);
    ds.write(0x44, 1); // start conversion, with parasite power on at the end

    delay(1000);

    present = ds.reset();
    ds.select(addr);
    ds.write(0xBE);

    for (int i = 0; i < 9; i++)
    { // we need 9 bytes
        tempData[i] = ds.read();
    }

    int16_t raw = (tempData[1] << 8) | tempData[0];

        byte cfg = (tempData[4] & 0x60);
        // at lower res, the low bits are undefined, so let's zero them
        if (cfg == 0x00)
            raw = raw & ~7; // 9 bit resolution, 93.75 ms
        else if (cfg == 0x20)
            raw = raw & ~3; // 10 bit res, 187.5 ms
        else if (cfg == 0x40)
            raw = raw & ~1; // 11 bit res, 375 ms
                            // default is 12 bit resolution, 750 ms conversion time
    
    celsius = (float)raw / 16.0;
    DPRINT("Temp:");
    DPRINTLN(celsius);
    delay(5000);
    }
}