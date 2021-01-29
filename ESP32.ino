#include <M5StickC.h>
#include <RTC.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <HTTPClient.h>
#include <math.h>

HardwareSerial potStat(2); // create 2nd serial instance for Pot/Galvanostat

#ifndef STASSID //Wi-Fi Credentials
//#define STASSID "ADB-CFF9A1"
//#define STAPSK  "rce6bn743cjr"
#define STASSID "Wi-Pi"
#define STAPSK "brg18f12"
#endif

#define POTENTIOSTATIC 1
#define GALVANOSTATIC 0
#define RANGE1 0
#define RANGE2 1
#define RANGE3 2

const char *ssid = STASSID;
const char *password = STAPSK;
const char *serverName = "http://192.168.0.200:1880/espTest";

HTTPClient http;

unsigned long oldTime = 0;
int interval = 2000;

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

long potential_offset;
long current_offset;
long dac_offset;
long dac_gain;
float shunt_calibration[] = {1.0, 1.0, 1.0};

float potential;
float current;

char current_range = RANGE3;

//FIXME: testing section
cv_data cv_params = {-3000, 2000, 1000, -1000, 50, 2, 0};
bool stopFlag = false;
//FIXME: testing section

void setup()
{
    Serial.begin(115200);                     // debug Serial
    potStat.begin(19200, SERIAL_8N1, 36, 26); // Pot/Galvanostat Serial

    WiFi.begin(ssid, password);
    Serial.println("");

    // Wait for connection
    int connectionCounter = 0;
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
        connectionCounter++;
        if (connectionCounter > 10)
        {
            Serial.println("Rebooting ESP.......");
            ESP.restart();
        }
    }

    Serial.println("");
    Serial.print("Connected to ");
    Serial.println(ssid);
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    http.begin(serverName);
    http.addHeader("Content-Type", "application/json");
}

void loop(void)
{
    if (oldTime + interval <= millis())
    { // once per second check if server has some commands
        Serial.println("requesting server commands");
        int resp = http.GET();
        Serial.print("Response from server : ");
        Serial.println(resp);
        if (resp == 200)
        {
            Serial.println("Resolving server request");
            resolveServerRequest();
        }
        else
        {
            Serial.println("Do nothing");
        }
        oldTime = millis();
    }
}
//TODO: change the way commands are received. Change the string to hex number. i.e. : CELL ON = 0x01;
void resolveServerRequest()
{
    String responseString = http.getString(); // read server response
    char buff[20];
    responseString.toCharArray(buff, sizeof(buff));
    if (responseString == "CELL ON")
    {
        set_cell_status(true);
    }
    else if (responseString == "CELL OFF")
    {
        set_cell_status(false);
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
        set_current_range;
    }
    else if (responseString == "RANGE2")
    {
        current_range = RANGE2;
        set_current_range;
    }
    else if (responseString == "RANGE3")
    {
        current_range = RANGE3;
        set_current_range;
    }
    else if (strncmp(buff, "DACSET ", 7) == 0)
    {
        long DACdata = atoi(&buff[7]);
        DACset(DACdata);
    }
    else if (responseString == "DACCAL")
    {
        dac_auto_calibrate();
    }
    else if (responseString == "ADCREAD") //read data from ADC converters
    {
        read_potential_current();
    }
    else if (responseString == "OFFSETREAD")
    {
        get_offset();
    }
    else if (responseString == "OFFSETSAVE")
    {
        set_offset(-86,-12); //FIXME: data from GUI
    }
    else if (responseString == "DACCALGET")
    {
        get_dac_calibration();
    }
    else if (responseString == "DACCALSET")
    {
        set_dac_calibration(-514936,528136); //FIXME: data from GUI
    }
    else if (responseString == "SHUNTCALREAD")
    {
        /* //TODO: under construction !! 
        //ShuntCalRead();
        char twoBytes[2];
        float floatingNum = 123456.789;
        float_to_twobytes(floatingNum,&twoBytes[0],&twoBytes[1]);
        Serial.println("Float number : "+String (twobytes_to_float(twoBytes[0],twoBytes[1])));
        */
    }
    else if (responseString == "SHUNTCALSAVE")
    {
        ShuntCalSave();
    }
    else if (strncmp(buff, "CV ", 3) == 0)
    {
        //cv_sweep(1, -1, 5, -5, 100, 2); //startpot, endpot, upbound, lowbound, scanrate, numofscans
        run_cv();
    }
    else if(responseString=="test")
    {   
        Serial.print("Converting DAC bytes to decimal :");
        Serial.println(DAC_bytes_to_decimal(8,56,28));
    }
    else
    {
        Serial.println("Do nothing");
    }
}

int expectResponse(char *expResponse, int interval)
{
    bool timeout = false;
    bool responded = false;
    unsigned long currentMillis = millis();
    unsigned long previousMillis = currentMillis;

    while (!timeout && !responded)
    {
        if (currentMillis - previousMillis >= interval)
        {
            responded = false;
            timeout = true;
        }
        else
        {
            currentMillis = millis();
        }
        if (potStat.available())
        {
            responded = true;
        }
    }

    if (responded)
    {
        char receivedResponse[10];
        uint8_t i = 0;
        uint8_t c;
        while (potStat.available())
        {
            c = potStat.read();
            while ((c != '\r') && (c != '\n') && (i < 9))
            {
                receivedResponse[i] = c;
                i++;
                c = potStat.read();
            }
        }
        receivedResponse[i] = '\0';
        Serial.print("Received response : ");
        Serial.println(receivedResponse);
        if (strncmp(receivedResponse, expResponse, 2) == 0)
        {
            Serial.println("Response OK");
            return 0;
        }
        else
        {
            Serial.println("Response !OK");
            return -1;
        }
    }
    else
    {
        Serial.println("Response timeout");
        return -2;
    }
}

int uartRead6Bytes(unsigned char *incomingData)
{
    unsigned long nowTime = millis();
    bool timeout = false;
    while (!potStat.available() && !timeout)
    {
        if ((millis() - nowTime) > 50) //wait for 50ms in the loop for data reception
        {
            timeout = true;
            return -1;
        }
    }
    if (potStat.available())
    {
        for (int i = 0; i < 6; i++)
        {
            incomingData[i] = potStat.read();
        }
    }
    else
        return -1;
}

void set_cell_status(bool cell_state)
{
    if (cell_state)
    {
        potStat.print("CELL ON\n");
        Serial.println("command - CELL ON");
    }
    else
    {
        potStat.print("CELL OFF\n");
        Serial.println("command - CELL OFF");
    }
    potStat.flush();
    expectResponse("OK", 50);
}

void set_control_mode(bool mode)
{
    if (mode)
    {
        Serial.println("Potentiostatic mode");
        potStat.print("POTENTIOSTATIC\n");
    }
    else
    {
        Serial.println("Galvanostatic mode");
        potStat.print("GALVANOSTATIC\n");
    }
    expectResponse("OK", 50);
}

void set_current_range()
{
    if (current_range == RANGE1) //20mA //10R
    {
        Serial.println("Range1");
        potStat.print("RANGE 1\n");
    }
    else if (current_range == RANGE2) //200uA //1k
    {
        Serial.println("Range2");
        potStat.print("RANGE 2\n");
    }
    else if (current_range == RANGE3) //2uA //100k
    {
        Serial.println("Range3");
        potStat.print("RANGE 3\n");
    }
    expectResponse("OK", 50);
}

void read_potential_current()
{
    potStat.print("ADCREAD\n"); //request data from ADC
    Serial.println("ADCREAD command");
    unsigned char uartData[6]; //readout data from ADC
    if (uartRead6Bytes(uartData) != -1)
    {
        if (strncmp((char *)uartData, "WAIT", 4) == 0)
        {
            Serial.println("Wait for ADC conversion");

            for (int i = 0; i < 9; i++) //loop for some time and than read again if sample is ready
            {
                delay(10); //90ms should be enough for one whole conversion
            }
            read_potential_current();
        }
        else
        {
            Serial.println("Here is ADC data :"); // here <-- 6bytes of ADC data is ready to send !
            for (int i = 0; i < 6; i++)
            {
                Serial.print(uartData[i], HEX); //print out 6 received ADC bytes
                // TODO:put data to JSON format and send to server
                Serial.print(" ");
            }
            Serial.println("");

            long raw_potential(twocomplement_to_decimal(uartData[0], uartData[1], uartData[2]));
            long raw_current(twocomplement_to_decimal(uartData[3], uartData[4], uartData[5]));

            potential = (raw_potential - potential_offset) / 2097152.0 * 8.0;                                                             //calculate potential in V compensating for offset
            current = (raw_current - current_offset) / 2097152.0 * 25.0 / pow((shunt_calibration[current_range] * 100.0), current_range); //calculate current in mA, taking current range into account and compensating for offset
        }
    }
    else
    {
        Serial.println("No or invalid data received");
    }
}

void get_offset() //Read 6 offset bytes from flash, bytes [0:3] = potential offset , bytes [4:6] = current offset
{
    potStat.print("OFFSETREAD\n");
    potStat.flush();
    Serial.println("command - OFFSETREAD");
    unsigned char uartData[6];
    if (uartRead6Bytes(uartData) != -1)
    {
        if ((uartData[0] & uartData[1] & uartData[2] & uartData[3] & uartData[4] & uartData[5]) != 255) // if no offset values has been stored, all bits will be set
        {
            potential_offset = DAC_bytes_to_decimal(uartData[0], uartData[1], uartData[2]);
            current_offset = DAC_bytes_to_decimal(uartData[3], uartData[4], uartData[5]);
            Serial.println("Offset values were read from flash memory");
            Serial.println("Potential offset : " +String(potential_offset));
            Serial.println("Current offset : " +String(current_offset));

        }
        else
        {
            Serial.println("No offset values were found in flash memory");
        }
    }
    else
    {
        Serial.println("No or invalid data received");
    }
}

void set_offset(long potOff, long currOff) //Set 6 offset bytes to flash, bytes [0:3] = potential offset , bytes [4:6] = current offset
{
    char offsetByte[6];
    decimal_to_dac_bytes(potOff,&offsetByte[0],&offsetByte[1],&offsetByte[2]); //MSB first
    decimal_to_dac_bytes(currOff,&offsetByte[3],&offsetByte[4],&offsetByte[5]); //MSB firs

    Serial.println("OFFSETSAVE command");
    potStat.print("OFFSETSAVE ");
    for (int i = 0; i < 6; i++)
    {
        potStat.write(offsetByte[i]);
    }
    potStat.print("\n");
    potStat.flush();
    expectResponse("OK", 50);
}

void get_dac_calibration()
{
    potStat.print("DACCALGET\n");
    potStat.flush();
    Serial.println("DACCALGET command");
    unsigned char uartData[6];
    uartRead6Bytes(uartData);

    if ((uartData[0] & uartData[1] & uartData[2] & uartData[3] & uartData[4] & uartData[5]) != 255) // if no offset values has been stored, all bits will be set
    {
        dac_offset = DAC_bytes_to_decimal(uartData[0],uartData[1],uartData[2]);
        dac_gain = DAC_bytes_to_decimal(uartData[3],uartData[4],uartData[5]) + (1<<19);
        Serial.println("DAC offset value : "+ String(dac_offset));
        Serial.println("DAC gain valuie : "+String(dac_gain));
    }
    else
    {
        Serial.println("No offset values were found in flash memory");
    }
    

}

void set_dac_calibration(long offset, long gain)
{
    char calByte[6];
    Serial.println("DACCALSET command");
    decimal_to_dac_bytes(offset,&calByte[0],&calByte[1],&calByte[2]);
    decimal_to_dac_bytes((gain-(1<<19)),&calByte[3],&calByte[4],&calByte[5]);
    
    potStat.print("DACCALSET ");
    for (int i = 0; i < 6; i++)
    {
        potStat.write(calByte[i]);
    }
    potStat.print("\n");
    potStat.flush();
    expectResponse("OK", 50);
}

void DACset(long rawVal) //send 3bytes of raw DAC data MSB first
{
    char DACbyte1, DACbyte2, DACbyte3;
    decimal_to_dac_bytes(rawVal, &DACbyte1, &DACbyte2, &DACbyte3);
    potStat.print("DACSET ");
    potStat.print(DACbyte1);
    potStat.print(DACbyte2);
    potStat.print(DACbyte3);
    potStat.print("\n");
    Serial.println("command - DACSET");
    expectResponse("OK", 50);
}



void dac_auto_calibrate()
{
    Serial.println("command - DAC auto calibrate");
    potStat.print("DACCAL\n");
    expectResponse("OK", 600);
    get_dac_calibration();
}



void ShuntCalRead()
{
    potStat.print("SHUNTCALREAD\n");
    potStat.flush();
    Serial.println("SHUNTCALREAD command");
    unsigned char uartData[6];
    uartRead6Bytes(uartData);
    Serial.println("SHUNTCAL Data :");
    for (int i = 0; i < 6; i++)
    {
        Serial.print(uartData[i]);
        Serial.print(" ");
    }
    Serial.println("");
}

void ShuntCalSave()
{
    int ShuntCalData[6];
    ShuntCalData[0] = 0;
    Serial.println("SHUNTCALSAVE command");
    potStat.print("SHUNTCALSAVE ");
    for (int i = 0; i < 6; i++)
    {
        potStat.write(ShuntCalData[0]);
    }
    potStat.print("\n");
    potStat.flush();
    expectResponse("OK", 50);
}

void decimal_to_dac_bytes(long value, char *msb, char *mid, char *lsb) // Working like a charm
{
    //Convert a floating-point number, ranging from -2**19 to 2**19-1, to three data bytes in the proper format for the DAC1220.
    long code = (1 << 19) + (long)value; // Convert the (signed) input value to an unsigned 20-bit integer with zero at midway ((1<<19) +
    if (code > ((1 << 20) - 1))          // crop the code if it is not within 20bytes
        code = (1 << 20) - 1;
    else if (code < 0)
        code = 0;

    Serial.print("Code : ");
    Serial.println(code);
    code = code << 4;
    *msb = (code & 0x00FF0000) >> 16;  //MSB
    *mid = (code & 0x0000FF00) >> 8;
    *lsb = (code & 0x000000FF);   //LSB
}

long DAC_bytes_to_decimal(char byte2, char byte1, char byte0)
{
    long code = ((long)byte2<<16);
    code = code | ((long)byte1<<8);
    code = code | byte0;
    code = code >> 4;
    code = code & 0x00FFFFFF;
    return code - (1<<19);
}

/* //TODO: under construction!!

void float_to_twobytes(float value, char *byte0, char *byte1) // two calibration bytes per one shunt resistor
{   
    long code = (1<<15) + int(round(value)) ;    //Convert two bytes to a number ranging from -2^15 to 2^15-1.

    if(code > ((1 << 16) - 1)) //clip the values to be within 16bit value
        code = (1<<16)-1;
    else if (code < 0)
        code = 0;

    *byte0 = (code & 0xFF00)>>8;
    *byte1 = (code & 0x00FF);    
}

float twobytes_to_float(char byte0, char byte1)
{
    float code = (byte0 << 8) + byte1;
    return (float)code - (1<<15);
}
*/
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
    Serial.print("ADC Answer : ");
    Serial.println(answer);
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
int current_range_from_current(float current)
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

void cv_start_setup()
{
    cv_params.starttime = millis();
    DACset(cv_params.ustart);
    set_control_mode(POTENTIOSTATIC);
    current_range = RANGE3;
    set_current_range;
    set_cell_status(true);
}

bool valiadate_parameters()
{
    if (cv_params.ubound < cv_params.lbound)
    {
        Serial.println("Upper bound cannot be lower than lower bound");
        return false;
    }
    else if (cv_params.scanrate == 0)
    {
        Serial.println("Scanrate cannot be zero");
        return false;
    }
    else if (cv_params.scanrate > 0 && (cv_params.ubound < cv_params.ustart))
    {
        Serial.println("For a positive scan rate, the start potential must be lower than the upper bound");
        return false;
    }
    else if ((cv_params.ubound > 8.000) || (cv_params.ubound < -8.000))
    {
        Serial.println("Upper bound is outside interval");
        return false;
    }
    else if ((cv_params.lbound > 8.000) || (cv_params.lbound < -8.000))
    {
        Serial.println("Lower bound is outside interval");
    }
    else if ((cv_params.ustart > 8.000) || (cv_params.ustart < -8.000))
    {
        Serial.println("Start potential is outside interval");
        return false;
    }
    else if ((cv_params.ustop > 8.000) || (cv_params.ustop < -8.000))
    {
        Serial.println("Stop potential is outside interval");
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

    int srtime = scanrate * time_elapsed / 1000; // Actual potential in mV vs time

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
        return NULL; //CV finished
    }
}

void cv_update()
{
    long elapsed_time = millis() - cv_params.starttime;
    int cv_output = cv_sweep(elapsed_time, cv_params.ustart, cv_params.ustop, cv_params.ubound, cv_params.lbound, cv_params.scanrate, cv_params.n);

    if (cv_output == NULL)
    {
        stopFlag = true;
        cv_stop();
    }
    else
    { /*//FIXME: only for testing purposes!!!!
        DACset(cv_output); //FIXME: passing voltage instead of DAC code
        ADCread();*/
        DACset(voltage_to_DACcode(cv_output));

        Serial.println(cv_output);
    }
}

void cv_stop()
{
    set_cell_status(false);
}

double ADCDataToVoltage(long ADCvoltage)
{
    float potOffset = 0.000;
    double voltage = ((double)ADCvoltage - potOffset) / 2097152.000 * 8.000;
    return voltage;
}
/*
long ADCDataToCurrent(long ADCcurrent)
{
    int currOffset = 0;
    long current = ADCcurrent-currOffset)/(2097152*25)/(shuntCalibration[currentRange]*100**currentrange);
    return current;
}*/

int voltage_to_DACcode(int voltage)
{
}