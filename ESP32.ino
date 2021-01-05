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
    mySerial.begin(19200, SERIAL_8N1, 36, 26); // Pot/Galvanostat serial

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
void cell_on()
{
    mySerial.print("CELL ON\n");
    mySerial.flush();

    if (expectResponse("OK") == 0)
    {
        Serial.println("command - CELL ON");
        M5.Lcd.printf("CELL ON\n");
    }
    else
    {
        Serial.println("Device not responded!");
    }
}

void cell_off()
{
    mySerial.print("CELL OFF\n");
    mySerial.flush();
    if (expectResponse("OK") == 0)
    {
        Serial.println("command - CELL OFF");
        M5.Lcd.printf("CELL OFF\n");
    }
}

char *readADCdata(char *adcData)
{
    for (int i = 0; i < 6; i++) 
    {
        adcData[i] = i;
    }
    return adcData;
}

void adcread()
{
    mySerial.println("ADCREAD"); //requset data from ADC
    

    unsigned char uartData[6]; //readout data from ADC
    if (uartRead6Bytes(uartData)!=-1)
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
        M5.Lcd.printf("ok\n");
        for (int i = 0; i < 6; i++)
        {

            Serial.print(uartData[i], HEX); //print out 6 received ADC bytes
        TODO:                               //put data to JSON format and send to server
            Serial.print(" ");
        }
        Serial.println("");
    }
    }
    else
    {
        Serial.println("No or invalid data received");
    }
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
    if(oldTime + interval <= millis()){  // once per second check if server has some commands 
    Serial.println("requesting server commands");
    int resp=http.GET();
    Serial.print("Response from server : ");
    Serial.println(resp);
    if(resp==200)
    
     {
         Serial.println("Resolving server request");
         resolveServerRequest();
     }
    else
    {
        Serial.println("Do nothing");
    }
    oldTime=millis();
    }
}

void resolveServerRequest()
{
    String responseString = http.getString(); // read server response
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setCursor(0, 10);
    char buff[20];
    responseString.toCharArray(buff,sizeof(buff));
    if(responseString == "CELL ON")
    {
        cell_on();
    }
    else if(responseString == "CELL OFF")
    {
        cell_off();
    }
    else if(responseString == "POTENTIOSTATIC")
    {
        changeMode(POTENTIOSTATIC);
    }
    else if(responseString == "GALVANOSTATIC")
    {
        changeMode(GALVANOSTATIC);
    }
    else if(responseString == "RANGE1")
    {
        setRange(RANGE1);
    }
    else if(responseString == "RANGE2")
    {
        setRange(RANGE2);
    }
    else if(responseString == "RANGE3")
    {
        setRange(RANGE3);
    }
    //else if(responseString == "DACSET")
    else if (strncmp(buff,"DACSET ",7) == 0)
    {
        DACset(-293601); //-314572 for 1V output
    }
    else if(responseString == "DACCAL")
    {
        
    }
    else if(responseString == "ADCREAD") //read data from ADC converters
    {
        adcread();
    }
    else if(responseString == "OFFSETREAD")
    {
        
    }
    else if(responseString == "OFFSETSAVE")
    {
        
    }
    else if(responseString == "DACCALGET")
    {
        DACCalGet();
    }
    else if(responseString == "DACCALSET")
    {

    }
    else if(responseString == "SHUNTCALREAD")
    {
        
    }
    else if(responseString == "SHUNTCALSAVE")
    {
        
    }
    else
    {
        Serial.println("Do nothing");
    }
}

int expectResponse(char *expResponse)
{
    bool timeout = false;
    bool responded = false;
    unsigned long currentMillis = millis();
    unsigned long previousMillis = currentMillis;
    uint interval = 50;

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
        Serial.println(receivedResponse);
        if (strncmp(receivedResponse, expResponse, 2) == 0)
        {
            Serial.println("reception correct");
            M5.Lcd.printf("Response OK\n");
            return 0;
        }
        else
        {
            Serial.println("reception incorrect");
            M5.Lcd.printf("Response !OK\n");
            return -1;
        }
    }
    else
    {
        Serial.println("response timeout");
        M5.Lcd.printf("Resp. timeout\n");
        return -2;
    }
}

int uartRead6Bytes(unsigned char *incomingData)
{
    if (mySerial.available())
    {
        for (int i = 0; i < 6; i++)
        {
            incomingData[i] = mySerial.read();
        }
    }
    else return -1;
}



void changeMode(int command)
{
    if(command==POTENTIOSTATIC)
    {
        Serial.println("Potentiostatic mode");
        mySerial.println("POTENTIOSTATIC");
        expectResponse("ok");
    }
    else if(command==GALVANOSTATIC)
    {
        Serial.println("Galvanostatic mode");
        mySerial.println("GALVANOSTATIC");
        expectResponse("ok");
    }
}

void setRange(int setRange)
{
    if(setRange==RANGE1)
    {
        Serial.println("Range1");
        mySerial.println("RANGE1");
        expectResponse("ok");
    }
    else if(setRange==RANGE2)
    {
        Serial.println("Range2");
        mySerial.println("RANGE2");
        expectResponse("ok");
    }
    else if(setRange==RANGE3)
    {
        Serial.println("Range3");
        mySerial.println("RANGE3");
        expectResponse("ok");
    }
}


void DACCalGet()
{
    mySerial.print("DACCALGET\n");
    mySerial.flush();
    unsigned char uartData[6];
    uartRead6Bytes(uartData);

            for (int i = 0; i < 6; i++)
        {

            Serial.print(uartData[i], HEX); //print out 6 received ADC bytes
        TODO:                               //put data to JSON format and send to server
            Serial.print(" ");
        }
        Serial.println("");

}

void DACset (long rawVal) //send 3bytes of raw DAC data MSB first
{
    char DACbyte1,DACbyte2,DACbyte3;
    decimal_to_dac_bytes(rawVal,&DACbyte1,&DACbyte2,&DACbyte3);
    mySerial.print("DACSET ");
    mySerial.print(DACbyte3);
    mySerial.print(DACbyte2);
    mySerial.print(DACbyte1);
    mySerial.print("\n");

        if (expectResponse("OK") == 0)
    {
        Serial.println("command - DACSET");
        M5.Lcd.printf("DACSET\n");
    }
    else
    {
        Serial.println("Device not responded!");
    }
}


void decimal_to_dac_bytes(long value,char *byte1,char *byte2,char *byte3)
{

	//Convert a floating-point number, ranging from -2**19 to 2**19-1, to three data bytes in the proper format for the DAC1220.
	long code = (1<<19)+(long)value; // Convert the (signed) input value to an unsigned 20-bit integer with zero at midway ((1<<19) +
    
	if(code > ((1<<20) - 1)) // crop the code if it is not within 20bytes 
        code = (1<<20) - 1;
    else if (code < 0 ) 
        code = 0;  
        
    Serial.print("Code : ");
    Serial.println(code);
	*byte1 = code<<4; //LSB
	*byte2 = code >> 4;
	*byte3 = code >> 12; //MSB
}