#include <ArduinoJson.h>

#include <AXP192.h> //battery lib
#include <M5Display.h>
#include <M5StickC.h>
#include <RTC.h>

#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>



HardwareSerial mySerial(2); // create 2nd serial instance for Pot/Galvanostat

#ifndef STASSID //Wi-Fi Credentials
//#define STASSID "ADB-CFF9A1"
//#define STAPSK  "rce6bn743cjr"
#define STASSID "Wi-Pi"
#define STAPSK "brg18f12"
#endif

const char *ssid = STASSID;
const char *password = STAPSK;
char adcData[20];

WebServer server(80);

const int led = 10;

void setup()
{
    M5.begin();
    Serial.begin(115200);                      // debug Serial
    mySerial.begin(19200, SERIAL_8N1, 36, 26); // Pot/Galvanostat serial

    pinMode(led, OUTPUT);
    digitalWrite(led, 0);

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
    M5.Lcd.printf("M5 Test lcd!");
    Serial.println("M5 Test uart!");

    ;

    if (MDNS.begin("ESP32"))
    {
        Serial.println("MDNS responder started");
    }

    server.on("/", handleRoot);

    server.on("/json",json_test);

    server.on("/cell_on/", cell_on);

    server.on("/cell_off/", cell_off);

    server.on("/adcread/", adcread);
    server.onNotFound(handleNotFound);

    server.begin();
    Serial.println("HTTP server started");
}

const String postForms = "<html>\
  <head>\
    <title>ESP32 test server</title>\
    <style>\
      body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }\
    </style>\
  </head>\
  <body>\
    <h1>Potenciostat test page! </h1><br>\
    <h1>LED ON! </h1><br>\
    <form method=\"post\" enctype=\"text/plain\" action=\"/cell_on/\">\
      <input type=\"submit\" value=\"LED ON\">\
    </form>\
    <h1>LED OFF! </h1><br>\
    <form method=\"post\" enctype=\"text/plain\" action=\"/cell_off/\">\
      <input type=\"submit\" value=\"LED OFF\">\
    </form>\
    <h1>ADCREAD </h1><br>\
    <form method=\"post\" enctype=\"text/plain\" action=\"/adcread/\">\
      <input type=\"submit\" value=\"adcread\">\
    </form>\
  </body>\
</html>";

void handleRoot()
{
    server.send(200, "text/html", postForms); //post main page
}

void cell_on()
{
    if (server.method() != HTTP_POST)
    {
        server.send(405, "text/plain", "Method Not Allowed");
    }
    else
    {
        server.send(200, "text/plain", "[POST] CELL ON");
        Serial.println("[POST] CELL ON");
        digitalWrite(led, 0);
        mySerial.print("CELL ON\n");
        M5.Lcd.fillScreen(BLACK);
        M5.Lcd.setCursor(0, 10);
        M5.Lcd.printf("CELL ON\n");
        expectResponse("OK");
    }
}

void adcread()
{
    if (server.method() != HTTP_POST)
    {
        server.send(405, "text/plain", "Method Not Allowed");
    }
    else
    {
        
        Serial.println("[POST] ADCread");
        digitalWrite(led, 0);
        mySerial.print("ADCREAD\n");
        M5.Lcd.fillScreen(BLACK);
        M5.Lcd.setCursor(0, 10);
        M5.Lcd.printf("ADCREAD\n"); //expect 6bytes of data or "wait"

        uartRead6Bytes();

        if (strncmp(adcData, "WAIT", 4) == 0)
        {
            server.send(200, "text/plain", "Wait for ADC conversion");
            Serial.println("Wait for ADC conversion");
            M5.Lcd.printf("Wait\n");
        }
        else
        {
            Serial.println("Here is ADC data :");  // here <-- 6bytes of ADC data is ready to send ! 
            server.send(200, "text/plain", "ADC data available");
            M5.Lcd.printf("ok\n");
            for (int i = 0; i < 6; i++)
            {
                
                Serial.print(adcData[i],HEX); //print out 6 received ADC bytes
                
                Serial.print(" ");

            }
        }
    }
}

void cell_off()
{
    if (server.method() != HTTP_POST)
    {
        server.send(405, "text/plain", "Method Not Allowed");
    }
    else
    {
        server.send(200, "text/plain", "[POST] CELL OFF");
        Serial.println("[POST] CELL OFF");
        digitalWrite(led, 1);
        mySerial.print("CELL OFF\n");
        M5.Lcd.fillScreen(BLACK);
        M5.Lcd.setCursor(0, 10);
        M5.Lcd.printf("CELL OFF\n");
        expectResponse("OK");
    }
}

void json_test()
{
 if (server.method() != HTTP_GET)
    {
        server.send(405, "text/plain", "Method Not Allowed");
    }
    else
    {
        StaticJsonDocument<200> Buffer;
        //JsonObject& root = jsonBuffer.createObject();   // struktura
        Buffer["ADC"] = "ADC1";

        JsonArray data = Buffer.createNestedArray("data"); // vnorena struktura
          data.add(11);                 // jednotlive bajty prosceeee 
          data.add(22); 
          data.add(33); 

        JsonArray data_2 = Buffer.createNestedArray("data_2");
          data_2.add(44);                 
          data_2.add(55); 
          data_2.add(66); 
        //Buffer.printTo(Serial);

        String json;
        serializeJson(Buffer, json);          //naondim to na json formatik


        

        //digitalWrite(led, 1);
        server.send(200, "application/json", json);  //proscceeee
        Serial.println("[POST] JsonSent");
    }
}

void handleNotFound()
{
    String message = "File Not Found\n\n";
    message += "URI: ";
    message += server.uri();
    message += "\nMethod: ";
    message += (server.method() == HTTP_GET) ? "GET" : "POST";
    message += "\nArguments: ";
    message += server.args();
    message += "\n";
    for (uint8_t i = 0; i < server.args(); i++)
    {
        message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
    }
    server.send(404, "text/plain", message);
}

void loop(void)
{
    server.handleClient();
}

void expectResponse(char *expResponse)
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
        }
        else
        {
            Serial.println("reception incorrect");
            M5.Lcd.printf("Response !OK\n");
        }
    }
    else
    {
        Serial.println("response timeout");
        M5.Lcd.printf("Resp. timeout\n");
    }
}

void uartRead6Bytes()
{

    if (mySerial.available())
    {
        for (int i = 0; i < 6; i++)
        {
            adcData[i] = mySerial.read();
        }
        mySerial.flush();
    }
}