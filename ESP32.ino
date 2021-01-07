#include <ArduinoJson.h>
#include <M5StickC.h>
#include <RTC.h>

#include <WiFi.h>
#include <WiFiClient.h>
#include <HTTPClient.h>

#include <math.h>

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

const char *payload = "{ \"M7I3GJBcT1qSz8XUtd51vADaaIlezNV1Yj1Za1wb9YTd\" : \"ZBkJstJ2bBIrCkXnPWOVNZYiG5KloC8ad9rW6ZpJeWfx\" }"; //temporary payload for testing purposes

unsigned long oldTime = 0;
int interval = 1000;
void setup()
{
    M5.begin();
    Serial.begin(115200);                      // debug Serial
    mySerial.begin(19200, SERIAL_8N1, 36, 26); // Pot/Galvanostat Serial

    WiFi.begin(ssid, password);
    Serial.println("");

    // Wait for connection
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }

    Serial.println("");
    Serial.print("Connected to ");
    Serial.println(ssid);
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    M5.Lcd.fillScreen(BLACK);
    delay(100);
    M5.Lcd.setCursor(0, 10);
    M5.Lcd.setTextColor(WHITE);
    M5.Lcd.setTextSize(1);
    M5.Lcd.printf("M5 Test LCD!");
    Serial.println("M5 Test UART!");

    http.begin(serverName);
    http.addHeader("Content-Type", "application/json");
}

/*
void json_test()
{
    if (server.method() != HTTP_GET)
    {
        server.send(405, "text/plain", "Method Not Allowed");
    }
    else
    {
        StaticJsonDocument<200> Buffer;
        Buffer["ADC"] = "ADC1";

        JsonArray data = Buffer.createNestedArray("data"); // vnorena struktura
        data.add(11);                                      // jednotlive bajty prosceeee
        data.add(22);
        data.add(33);

        JsonArray data_2 = Buffer.createNestedArray("data_2");
        data_2.add(44);
        data_2.add(55);
        data_2.add(66);
        //Buffer.printTo(Serial);

        String json;
        serializeJson(Buffer, json); //naondim to na json formatik

        server.send(200, "application/json", json); //proscceeee
        Serial.println("[POST] JsonSent");
    }
}
*/

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
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setCursor(0, 10);
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
    else
    {
        Serial.println("Do nothing");
        M5.Lcd.printf("Do nothing\n");
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
            M5.Lcd.printf("Response OK\n");
            return 0;
        }
        else
        {
            Serial.println("Response !OK");
            M5.Lcd.printf("Response !OK\n");
            return -1;
        }
    }
    else
    {
        Serial.println("Response timeout");
        M5.Lcd.printf("Resp. timeout\n");
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
    M5.Lcd.printf("CELL ON\n");
    expectResponse("OK", 50);
}

void cell_off()
{
    mySerial.print("CELL OFF\n");
    mySerial.flush();
    Serial.println("command - CELL OFF");
    M5.Lcd.printf("CELL OFF\n");
    expectResponse("OK", 50);
}

void changeMode(int command)
{
    if (command == POTENTIOSTATIC)
    {
        Serial.println("Potentiostatic mode");
        mySerial.print("POTENTIOSTATIC\n");
        M5.Lcd.printf("POT mode\n");
        expectResponse("OK", 50);
    }
    else if (command == GALVANOSTATIC)
    {
        Serial.println("Galvanostatic mode");
        mySerial.print("GALVANOSTATIC\n");
        M5.Lcd.printf("GALV mode\n");
        expectResponse("OK", 50);
    }
}

void setRange(int setRange)
{
    if (setRange == RANGE1)
    {
        Serial.println("Range1");
        M5.Lcd.printf("Range1\n");
        mySerial.print("RANGE 1\n");
        expectResponse("OK", 50);
    }
    else if (setRange == RANGE2)
    {
        Serial.println("Range2");
        M5.Lcd.printf("Range2\n");
        mySerial.print("RANGE 2\n");
        expectResponse("OK", 50);
    }
    else if (setRange == RANGE3)
    {
        Serial.println("Range3");
        M5.Lcd.printf("Range3\n");
        mySerial.print("RANGE 3\n");
        expectResponse("OK", 50);
    }
}

void ADCread()
{
    mySerial.print("ADCREAD\n"); //request data from ADC
    Serial.println("ADCREAD command");
    M5.lcd.printf("ADCREAD\n");
    unsigned char uartData[6]; //readout data from ADC
    if (uartRead6Bytes(uartData) != -1)
    {
        //jsonFormatter(uartData, sizeof(uartData)); // format data to JSON string
        if (strncmp((char *)uartData, "WAIT", 4) == 0)
        {
            Serial.println("Wait for ADC conversion");
            M5.Lcd.printf("Wait\n");
        }
        else
        {
            Serial.println("Here is ADC data :"); // here <-- 6bytes of ADC data is ready to send !
            M5.Lcd.printf("Data : \n");
            for (int i = 0; i < 6; i++)
            {
                Serial.print(uartData[i], HEX); //print out 6 received ADC bytes
                // TODO:put data to JSON format and send to server
                Serial.print(" ");
            }
            M5.Lcd.printf("ADC Data OK\n");
            Serial.println("");
        }
    }
    else
    {
        Serial.println("No or invalid data received");
        M5.Lcd.printf("Data err\n");
    }
}

void OffsetRead() //Read 6 offset bytes from flash, bytes [0:3] = potential offset , bytes [4:60] = current offset
{
    mySerial.print("OFFSETREAD\n");
    mySerial.flush();
    Serial.println("OFFSETREAD command");
    M5.Lcd.printf("OFFSETREAD\n");
    unsigned char uartData[6];
    if (uartRead6Bytes(uartData) != -1)
    {
        Serial.println("OFFSET DATA :");
        for (int i = 0; i < 6; i++)
        {
            Serial.print(uartData[i]);
            Serial.print(" ");
        }
        M5.Lcd.printf("OFFSET Data ok\n");
        Serial.println("");
    }
    else
    {
        Serial.println("No or invalid data received");
        M5.Lcd.print("Data err\n");
    }
}

void OffsetSave()
{
    uint8_t OffsetData[6];
    OffsetData[0] = 0;
    Serial.println("OFFSETSAVE command");
    M5.Lcd.print("OFFSETSAVE\n");
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
    M5.Lcd.printf("DACCALGET\n");
    unsigned char uartData[6];
    uartRead6Bytes(uartData);

    for (int i = 0; i < 6; i++)
    {
        Serial.print(uartData[i]);
    TODO: //put data to JSON format and send to server
        Serial.print(" ");
    }
    M5.Lcd.printf("CAL Data ok\n");
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
    M5.Lcd.printf("DACSET\n");
    expectResponse("OK", 50);
}

void DACCal()
{
    Serial.println("command - DACCal");
    M5.Lcd.printf("DACCAL\n");
    mySerial.print("DACCAL\n");
    expectResponse("OK", 600);
}

void DACCalSet()
{
    int CalData[6];
    CalData[0] = 0;
    Serial.println("DACCALSET command");
    M5.Lcd.print("DACCALSET\n");
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
    M5.Lcd.printf("SHUTCALREAD\n");
    unsigned char uartData[6];
    uartRead6Bytes(uartData);
    Serial.println("SHUNTCAL Data :");
    for (int i = 0; i < 6; i++)
    {
        Serial.print(uartData[i]);
        Serial.print(" ");
    }
    M5.Lcd.printf("SHUNTCAL Data ok\n");
    Serial.println("");
}

void ShuntCalSave()
{
    int ShuntCalData[6];
    ShuntCalData[0] = 0;
    Serial.println("SHUNTCALSAVE command");
    M5.Lcd.print("SHUNTCALSAVE\n");
    mySerial.print("SHUNTCALSAVE ");
    for (int i = 0; i < 6; i++)
    {
        mySerial.write(ShuntCalData[0]);
    }
    mySerial.print("\n");
    mySerial.flush();
    expectResponse("OK", 50);
}

void decimal_to_dac_bytes(long value, char *byte1, char *byte2, char *byte3)
{
    //Convert a floating-point number, ranging from -2**19 to 2**19-1, to three data bytes in the proper format for the DAC1220.
    long code = (1 << 19) + (long)value; // Convert the (signed) input value to an unsigned 20-bit integer with zero at midway ((1<<19) +
    if (code > ((1 << 20) - 1)) // crop the code if it is not within 20bytes
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