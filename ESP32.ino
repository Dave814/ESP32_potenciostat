#include <M5StickC.h>
#include <RTC.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <HTTPClient.h>
#include <math.h>
bool stopFlag = false;      //FIXME: testing purposes
HardwareSerial mySerial(2); // create 2nd serial instance for Pot/Galvanostat

#ifndef STASSID //Wi-Fi Credentials
//#define STASSID "ADB-CFF9A1"
//#define STAPSK  "rce6bn743cjr"
#define STASSID "Wi-Pi"
#define STAPSK "brg18f12"
#endif

#define POTENTIOSTATIC 0
#define GALVANOSTATIC 1
#define RANGE1 1
#define RANGE2 2
#define RANGE3 3

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

cv_data cv_params = {-3000, 2000, 1000, -1000, 50, 2, 0};

void setup()
{
    Serial.begin(115200);                      // debug Serial
    mySerial.begin(19200, SERIAL_8N1, 36, 26); // Pot/Galvanostat Serial

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
        cell_on();
    }
    else if (responseString == "CELL OFF")
    {
        cell_off();
    }
    else if (responseString == "POTENTIOSTATIC")
    {
        changeMode(POTENTIOSTATIC);
    }
    else if (responseString == "GALVANOSTATIC")
    {
        changeMode(GALVANOSTATIC);
    }
    else if (responseString == "RANGE1")
    {
        setRange(RANGE1);
    }
    else if (responseString == "RANGE2")
    {
        setRange(RANGE2);
    }
    else if (responseString == "RANGE3")
    {
        setRange(RANGE3);
    }
    else if (strncmp(buff, "DACSET ", 7) == 0)
    {
        long DACdata = atoi(&buff[7]);
        DACset(DACdata);
    }
    else if (responseString == "DACCAL")
    {
        DACCal();
    }
    else if (responseString == "ADCREAD") //read data from ADC converters
    {
        ADCread();
    }
    else if (responseString == "OFFSETREAD")
    {
        OffsetRead();
    }
    else if (responseString == "OFFSETSAVE")
    {
        OffsetSave();
    }
    else if (responseString == "DACCALGET")
    {
        DACCalGet();
    }
    else if (responseString == "DACCALSET")
    {
        DACCalSet();
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
        //cv_sweep(1, -1, 5, -5, 100, 2); //startpot, endpot, upbound, lowbound, scanrate, numofscans
        run_cv();
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
        if (mySerial.available())
        {
            responded = true;
        }
    }

    if (responded)
    {
        char receivedResponse[10];
        uint8_t i = 0;
        uint8_t c;
        while (mySerial.available())
        {
            c = mySerial.read();
            while ((c != '\r') && (c != '\n') && (i < 9))
            {
                receivedResponse[i] = c;
                i++;
                c = mySerial.read();
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
    while (!mySerial.available() && !timeout)
    {
        if ((millis() - nowTime) > 50) //wait for 50ms in the loop for data reception
        {
            timeout = true;
            return -1;
        }
    }
    if (mySerial.available())
    {
        for (int i = 0; i < 6; i++)
        {
            incomingData[i] = mySerial.read();
        }
    }
    else
        return -1;
}

void cell_on()
{
    mySerial.print("CELL ON\n");
    mySerial.flush();
    Serial.println("command - CELL ON");
    expectResponse("OK", 50);
}

void cell_off()
{
    mySerial.print("CELL OFF\n");
    mySerial.flush();
    Serial.println("command - CELL OFF");
    expectResponse("OK", 50);
}

void changeMode(int command)
{
    if (command == POTENTIOSTATIC)
    {
        Serial.println("Potentiostatic mode");
        mySerial.print("POTENTIOSTATIC\n");
        expectResponse("OK", 50);
    }
    else if (command == GALVANOSTATIC)
    {
        Serial.println("Galvanostatic mode");
        mySerial.print("GALVANOSTATIC\n");
        expectResponse("OK", 50);
    }
}

void setRange(int setRange)
{
    if (setRange == RANGE1) //20mA //10R
    {
        Serial.println("Range1");
        mySerial.print("RANGE 1\n");
        expectResponse("OK", 50);
    }
    else if (setRange == RANGE2) //200uA //1k
    {
        Serial.println("Range2");
        mySerial.print("RANGE 2\n");
        expectResponse("OK", 50);
    }
    else if (setRange == RANGE3) //2uA //100k
    {
        Serial.println("Range3");
        mySerial.print("RANGE 3\n");
        expectResponse("OK", 50);
    }
}

void ADCread()
{
    mySerial.print("ADCREAD\n"); //request data from ADC
    Serial.println("ADCREAD command");
    unsigned char uartData[6]; //readout data from ADC
    if (uartRead6Bytes(uartData) != -1)
    {
        //jsonFormatter(uartData, sizeof(uartData)); // format data to JSON string
        if (strncmp((char *)uartData, "WAIT", 4) == 0)
        {
            Serial.println("Wait for ADC conversion");
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
            //Serial.print("converted ADCdata : ");
            Serial.println(ADCDataToVoltage(twoComplementToDecimal(uartData[0], uartData[1], uartData[2])));
            Serial.println(ADCDataToVoltage(twoComplementToDecimal(uartData[3], uartData[4], uartData[5])));
        }
    }
    else
    {
        Serial.println("No or invalid data received");
    }
}

void OffsetRead() //Read 6 offset bytes from flash, bytes [0:3] = potential offset , bytes [4:60] = current offset
{
    mySerial.print("OFFSETREAD\n");
    mySerial.flush();
    Serial.println("OFFSETREAD command");
    unsigned char uartData[6];
    if (uartRead6Bytes(uartData) != -1)
    {
        Serial.println("OFFSET DATA :");
        for (int i = 0; i < 6; i++)
        {
            Serial.print(uartData[i]);
            Serial.print(" ");
        }
        Serial.println("");
    }
    else
    {
        Serial.println("No or invalid data received");
    }
}

void OffsetSave()
{
    uint8_t OffsetData[6];
    OffsetData[0] = 0;
    Serial.println("OFFSETSAVE command");
    mySerial.print("OFFSETSAVE ");
    for (int i = 0; i < 6; i++)
    {
        mySerial.write(OffsetData[0]);
    }
    mySerial.print("\n");
    mySerial.flush();
    expectResponse("OK", 50);
}

void DACCalGet()
{
    mySerial.print("DACCALGET\n");
    mySerial.flush();
    Serial.println("DACCALGET command");
    unsigned char uartData[6];
    uartRead6Bytes(uartData);

    for (int i = 0; i < 6; i++)
    {
        Serial.print(uartData[i]);
    TODO: //put data to JSON format and send to server
        Serial.print(" ");
    }
    Serial.println("");
}

void DACset(long rawVal) //send 3bytes of raw DAC data MSB first
{
    char DACbyte1, DACbyte2, DACbyte3;
    decimal_to_dac_bytes(rawVal, &DACbyte1, &DACbyte2, &DACbyte3);
    mySerial.print("DACSET ");
    mySerial.print(DACbyte1);
    mySerial.print(DACbyte2);
    mySerial.print(DACbyte3);
    mySerial.print("\n");
    Serial.println("command - DACSET");
    expectResponse("OK", 50);
}

void DACCal()
{
    Serial.println("command - DACCal");
    mySerial.print("DACCAL\n");
    expectResponse("OK", 600);
}

void DACCalSet()
{
    int CalData[6];
    CalData[0] = 0;
    Serial.println("DACCALSET command");
    mySerial.print("DACCALSET ");
    for (int i = 0; i < 6; i++)
    {
        mySerial.write(CalData[0]);
    }
    mySerial.print("\n");
    mySerial.flush();
    expectResponse("OK", 50);
}

void ShuntCalRead()
{
    mySerial.print("SHUNTCALREAD\n");
    mySerial.flush();
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
    mySerial.print("SHUNTCALSAVE ");
    for (int i = 0; i < 6; i++)
    {
        mySerial.write(ShuntCalData[0]);
    }
    mySerial.print("\n");
    mySerial.flush();
    expectResponse("OK", 50);
}

void decimal_to_dac_bytes(long value, char *byte1, char *byte2, char *byte3) // Working like a charm
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
    *byte1 = (code & 0x00FF0000) >> 16;
    *byte2 = (code & 0x0000FF00) >> 8;
    *byte3 = (code & 0x000000FF);
}

long twoComplementToDecimal(int msb, int middlebyte, int lsb) // Working like a charm
{                                                             // Convert a 22-bit two-complement ADC value consisting of three bytes to a signed integer
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
    changeMode(POTENTIOSTATIC);
    setRange(RANGE3);
    cell_on();
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
// ustart -- start potential [mV]
// ustop -- stop potential  [mV]
// ubound -- upper potential bound  [mV]
// lbound -- lower potential bound  [mV]
// scanrate -- scanrate in mV/s
// n -- number of scans
{
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
    cell_off();
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