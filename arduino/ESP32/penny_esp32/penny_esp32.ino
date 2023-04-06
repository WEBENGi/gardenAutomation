/****************************************************************************************************************************************************************

  Notes:
  ----------------------------------------------------------------------------------------------------------------------------------------------------------------
 
  - The ESP32 is responsible for relaying all data between the Mega and Home Assistant via MQTT, since the Mega has no WiFi capabilities. 
    - client(forpublish) for WIFI+MQTT (connect to WIFI+ MQTT / Home Assistant)
    
  - All references to Serial are for printing to console only. Serial1 is the actual wired connection to the Mega in the control box.

  - For OPs pumps, flow rates are as follows: 300ms "on" time = 1mL, 500ms = 1.5mL, 1000ms = 3.25mL, 1500ms = 5mL, 2000ms = 6.75mL, 2500ms = 8.5mL, 3000ms = 10mL.
    - Voltage at V+/V- terminals on 12V PSU was 13.045V when pump flow rates were measured.
    - Starting PWM values were: phDownSpeed = 210, calMagSpeed = 204, microSpeed = 211, bloomSpeed = 206, growSpeed = 205, phUpSpeed = 210, noctuaFanSpeed = 125.
  - Dosing pump speeds are set by Home Assistant when the ESP32 connects via MQTT.

  - The ESP32 will send serial commands to the Mega to poll the Atlas Scientific sensors at a preset interval (atlasPeriod), alternating between them. It will send
    a temperature compensation value for the AS sensors based on the measured temperature of the mixing res solution at a preset interval as well (tempCompPeriod).

  - Sensors/Modules Attached:
    - Relay Module 8 Channel 5V Relay Module for Arduino Raspberry Pi AVR PIC ARM DSP ARM MSP430 TTL Logic Level
    - Relay Module 4 Channel 5V Relay Module for Arduino Raspberry Pi AVR PIC ARM DSP ARM MSP430 TTL Logic Level
    - Water Temperature Sensor
    - BME280 Temperature, Humidity, Pressure Sensor

  - Messages to Home Assistant:
    - "feedback/general" - general info / log
    - "feedback/boxTemp" - Temperature of the control box
    - "feedback/boxHumidity" - Humidity of the control box
    - "feedback/boxPressure" - Pressure of the control box
    - "feedback/boxSeaLevel" - Altitude / Sealevel of the control box
    - "feedback/waterTemp" - Temperature of the mixing res solution
    - "feedback/ph" - pH of the mixing res solution
    - "feedback/tds" - TDS of the mixing res solution
    - "feedback/relays" - State of the relays
    - "feedback/waterLevel" - Waterlevel (ultrasonic)
    - "feedback/soilMoisture" - State of the Soil moisture (capacitive) 
    - "feedback/flood" - State of the flood sensor
    - "feedback/waterLevelFloat" - State of the water level float sensor (Drainage Bucket)
    
    Messages expected from Home Assitant / Listening (Callbacks):
    - control/dosing - Dosing pump power
    - control/relays - Relay power
    - calibrate/dosing - Dosing pump speed
    - calibrate/ph - PH calibration
    - calibrate/tds - TDS calibration

    Messages to MEGA:
    - <DosingPumpPower:???>
    - <PumpSpeed:???>
    - <CalibratePH:Cal,(mid,low,high),(7.00,4.00,10.00)>
    - <TDSCalibrate:Cal,dry> / <TDSCalibrate:Cal,low,700> / <TDSCalibrate:Cal,high,2000>

  - Messages from MEGA:
    - <WL:X:YY.YY> - Water level # (in cm)
    - <TDS:XX.XX> - TDS (in ppm)
    - <H:XX.XX> - PH
    - <M:XX:YY.YY> - Soil moisture sensor - YY.YY (in %)
    - <V:Drainage bucket:1> - Drainage bucket is full / <FV:Drainage bucket:0> Empty
    - <D:XX:YYY> - Perastaltic Pump XX on for YYY ms (Dosing)
    - <S:XX:YYY> - Perastaltic Pump XX set for YYY (Speed) PWM
    - <Flooding (Location)> - Flooding detected

    - "feedback/noctuaFanSpeed" - Speed of the noctua fan
   TODO:
  ----------------------------------------------------------------------------------------------------------------------------------------------------------------
  - AP will need to test to find the rates of the different kinds of pumps
    - "feedback/ec" - EC of the mixing res solution

   Other possible sensors to add:
    - "feedback/salinity" - Salinity of the mixing res solution
    - "feedback/flow" - Flow rate of the mixing res solution
    - "feedback/orp" - ORP of the mixing res solution
******************************************************************************************************************************************************************/

#include <OneWire.h>
#include <Wire.h>
#include <WiFi.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <PubSubClient.h>
#include <DallasTemperature.h>

#define NUM_ELEMENTS(x)  (sizeof(x) / sizeof((x)[0])) // Use to calculate how many elements are in an array
#define PIN_RELAY8_01 17 
#define PIN_RELAY8_02 25
#define PIN_RELAY8_03 32
#define PIN_RELAY8_04 12
#define PIN_RELAY8_05 4
#define PIN_RELAY8_06 0
#define PIN_RELAY8_07 2
#define PIN_RELAY8_08 23
#define PIN_RELAY4_01 19
#define PIN_RELAY4_02 18
#define PIN_RELAY4_03 17
#define PIN_RELAY4_04 16

//#define NOCTUA 6   

// Relays
int relayPins[2][8]
{
  {PIN_RELAY8_01, PIN_RELAY8_02, PIN_RELAY8_03, PIN_RELAY8_04, PIN_RELAY8_05, PIN_RELAY8_06, PIN_RELAY8_07, PIN_RELAY8_08},              
  // 8 Chan Relay Board: [0]=plug switch 3 , [1]=plug switch 2 , [2]=plug switch 1 , [3]=Solenoid 5 , [4]=Solenoid 4 , [5]=Solenoid 3 , [6]=Solenoid 2 , [7]=Solenoid 1
  {PIN_RELAY4_01, PIN_RELAY4_02, PIN_RELAY4_03, PIN_RELAY4_04}                               
  // 4 Chan Relay Board: [0]=Stirring fans , [1]=internal cooling fna , [2]=plug switch 5 , [3]=plug switch 4
};

#define PIN_BME_SDA	21
#define PIN_BME_SDL	22
#define PIN_BME_INT	35

//#define PIN_WATER_TEMP_SENSOR 32

#define PIN_ESPMINI_TX0	3
#define PIN_ESPMINI_RX0	1

const int freq = 5000;
const int resolution = 8;
#define PIN_PWM_FAN	34 

#define ONE_WIRE_BUS 32 // PIN_WATER_TEMP_SENSOR
//#define BUILTIN_LED 2
#define SEALEVELPRESSURE_HPA (1013.25) // set sea level pressure to 1013.25 hPa
Adafruit_BME280 bme;
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);  
// Pass our oneWire reference to Dallas DS18B20 Temperature sensor
char bmeBuffer[16];

float celcius = 0;
char waterTemp[16];

// Timing
unsigned long currentMillis;                         // Grab a snapshot of current millis each pass through void loop()
unsigned long tempCompMillis = millis();             // Timer for sending temperature compensation factor to EZO pH circuit
unsigned long tempCheckMillis = millis();            // Timer to check control box temp and mix reservoir solution temp
unsigned long phMillis = millis();                // Timer for polling thevpH circuits for readings
unsigned long tdsMillis = millis();                // Timer for polling thevpH circuits for readings

const unsigned long tempCheckPeriod = 30000;         // How long to wait, in milliseconds, between checking box/water temp
const unsigned long tempCompPeriod = 600000;         // How long to wait, in milliseconds, between sending temperature compensation factor to ezo pH circuit
unsigned long phPeriod = 5000;                    // How long to wait, in milliseconds, between polling ph sensor for values
unsigned long tdsPeriod = 5000;                    // How long to wait, in milliseconds, between polling tds sensor for values

// WiFi/MQTT/Serial
char ssid[]= "YOUR___SSID";
const char* password = "YOUR___PASS";
const char* mqtt_server = "YOUR___MQTTSERVERIP";
WiFiClient espClient;
PubSubClient client(espClient);
const byte numChars = 100;
char receivedChars[numChars];
boolean newData = false;                            //Is there new data coming in over the serial port from the Arduino Mega?

// Atlas
//boolean pHCalledLast = false;                       // Tracks whether pH was polled last or EC.
boolean stopPHReadings = false;                       // I set this flag to tell system to stop taking readings at certain points when calibrating sensors to avoid errors.
boolean stopTDSReadings = false;                       // I set this flag to tell system to stop taking readings at certain points when calibrating sensors to avoid errors.

const int pwmNoctuaFanPin = 34;//15;
const int pwmNOCTUA = 125;                          // This is the PWM rate for the noctua fan in the control box. It never changes.

void setup_wifi()
{
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length)
{
    char payloadStr[length + 1];              // Create a char array that's 1 byte longer than the incoming payload to copy it to and make room for the null terminator so it can be treated as string.
    memcpy(payloadStr, payload, length);
    payloadStr[length + 1] = '\0';

    /***************** CALLBACK: 8-Channel Relay Board (In Control Box) *****************/

    if (strcmp(topic, "control/relays") == 0) // Incoming message format will be <BOARD#>:<RELAY#>:<STATE>. STATE is "1" for on, "0" for off. Example payload: "1:1:0" = on board 1, turn relay 1 ON.
    {
      int boardNumber;
      int relayNumber;
      int relayPower;
      char* strtokIndx;
      char buff[20];

      strtokIndx = strtok(receivedChars, ":");                    // Skip the first segment which is the 'R' character
      strtokIndx = strtok(NULL, ":");                             // Get the board number
      boardNumber = atoi(strtokIndx);
      strtokIndx = strtok(NULL, ":");                             // Get the relay number
      relayNumber = atoi(strtokIndx);
      strtokIndx = strtok(NULL, ":");                             // Get the relay power state
      relayPower = atoi(strtokIndx);

      triggerRelay(boardNumber, relayNumber, relayPower);

      sprintf(buff, "<Relay FB:%d:%d:%d>", boardNumber, relayNumber, relayPower);
      Serial1.println(buff);
    }
    // CALLBACK: Dosing
    if (strcmp(topic, "control/dosing") == 0)   // Incoming message format will be <PUMP#>:<ONTIME>. ONTIME is in milliseconds.
    {
        Serial1.print("<DosingPumpPower:");               // Print this command to the Mega since it handles
        Serial1.print(payloadStr);
        Serial1.println('>');   
    }

    // CALLBACK: Pump Speed Adjustments 
    if (strcmp(topic, "calibrate/dosing") == 0)
    {
        Serial1.print("<PumpSpeed:");               // Print this command to the Mega since it handles
        Serial1.print(payloadStr);
        Serial1.println('>');   
    }
  
    // CALLBACK: pH Calibration 
    if (strcmp(topic, "calibrate/ph") == 0)           // pH cal values of 7.00, 4.00, and 10.00 are hard coded to match Atlas calibration solutions. Change these if you're using different solutions.
    {
        switch (payloadStr[0])                                  // Payload will be "mid", "low", or "high". Switching on first char to tell which it is.
        {
            case 'm':
            {
                stopPHReadings = true;
                delay(2000);
                Serial1.println("<CalibratePH:Cal,mid,7.00>");
                phMillis = millis();
                stopPHReadings = false;
                break;
            }
            case 'l':
            {
                stopPHReadings = true;
                delay(2000);
                Serial1.println("<CalibratePH:Cal,low,4.00>");
                phMillis = millis();
                stopPHReadings = false;
                break;
            }
            case 'h':
            {
                stopPHReadings = true;
                delay(2000);
                Serial1.println("<CalibratePH:Cal,high,10.00>");
                delay(2000);
                Serial1.println("<CalibratePH:Cal,?>");               //I'm asking the EZO pH circuit here how many points it has calibrated. To know I was successful, I'm looking for an answer of 3.
                phMillis = millis();
                stopPHReadings = false;
                break;
            }
        }
    }

    // CALLBACK: TDS Calibration 

    if (strcmp(topic, "calibrate/tds") == 0)           // EC cal values of 700 & 2000 are hard coded to match Atlas calibration solutions. Change these if you're using diff solutions.
    {
        switch (payloadStr[0])                                   // Payload will be "dry", "low", or "high". Switching on first char to tell which it is.
        {
            case 'd':
            {
                stopTDSReadings = true;
                delay(2000);
                Serial1.println("<TDSCalibrate:Cal,dry>");
                delay(1000);
                tdsMillis = millis();
                stopTDSReadings = false;
                break;
            }
            case 'l':
            {
                stopTDSReadings = true;
                delay(2000);
                Serial1.println("<TDSCalibrate:Cal,low,700>");
                delay(1000);
                tdsMillis = millis();
                stopTDSReadings = false;
                break;
            }
            case 'h':
            {
                stopTDSReadings = true;
                delay(2000);
                Serial1.println("<TDSCalibrate:Cal,high,2000>");
                delay(2000);
                Serial1.println("<TDSCalibrate:Cal,?>");              // Again, how many points of calibration?
                tdsMillis = millis();
                stopTDSReadings = false;
                break;
            }
        }
    }
}

void reconnect()
{
    byte mqttFailCount = 0;
    byte tooManyFailures = 10;
    // Loop until we're reconnected
    while (!client.connected())
    {
        if (mqttFailCount <= tooManyFailures)
        {
            Serial.print("Attempting MQTT connection...");
            if (client.connect("YOUR___ID", "YOUR___USERNAME", "YOUR___PASSWORD"))
            {
                delay(1000);        
                client.publish("feedback/general", "Garden controller connecting...");
                delay(1000);
                client.publish("feedback/general", "Garden controller connected.");
                digitalWrite(BUILTIN_LED, HIGH);

                client.subscribe("control/relays");
                client.subscribe("control/dosing");
                client.subscribe("calibrate/ph");
                client.subscribe("calibrate/tds");
                client.subscribe("calibrate/dosing");
            }
            else
            {
                digitalWrite(BUILTIN_LED, LOW);
                mqttFailCount ++;
                Serial.print("Failed. Count = ");
                Serial.println(mqttFailCount);
                Serial.println("...trying again in 5 seconds");
                // Wait 5 seconds before retrying
                delay(5000);
            }
        }
        else
        {
            Serial.print(tooManyFailures);
            Serial.println(" MQTT failures in a row. Resetting WiFi connection.");
            WiFi.disconnect();
            delay(5000);
            setup_wifi();
            mqttFailCount = 0;
        }
    }
}

void setup()
{   
    //char buff[60];
    Serial.begin(115200);
    Serial1.begin(115200);
    Serial.read();
    Serial1.read();

    pinMode(BUILTIN_LED, OUTPUT);
    setup_wifi();
    client.setServer(mqtt_server, 1883);
    client.setCallback(callback);
 
    sensors.begin();
    bme.begin(0x76);
    getWaterTemp();
    getBoxTemp();
    getBoxHumidity();

    //Configure LED PWM functionalitites
    ledcSetup(PIN_PWM_FAN, freq, resolution);
    ledcAttachPin(PIN_PWM_FAN, 1);///pwmChannel[6]);
    ledcWrite(1,125);//pwmChannel[NOCTUA], 125);
    for (unsigned int i = 0; i < 2; i++)
    {
      for (unsigned int j = 0; j < NUM_ELEMENTS(relayPins[i]); j++)
      {
        pinMode(relayPins[i][j], OUTPUT);
        digitalWrite(relayPins[i][j], HIGH);
      }
    }
}

void loop()                                                                           // I'm using millis() to try to keep my loop running as fast as possible. I tried to avoid having any "delay(x)" lines, which block the program.
{
    currentMillis = millis();
    if (!client.connected())
    {
        reconnect();
    }

    if (currentMillis - tempCheckMillis >= tempCheckPeriod)                             // Is it time to check temps?
    {
        getWaterTemp();
        getBoxTemp();
        tempCheckMillis = currentMillis;
    }

    if ((currentMillis - phMillis > phPeriod) && (stopPHReadings == false))         // Is it time to read the EZO circuits?
    {
        if (currentMillis - tempCompMillis > tempCompPeriod)                              // Is it time to compensate for temperature?
        {
            if ((celcius >= 10) && (celcius <= 30))                                         // Make sure I'm not getting a garbage reading from temp sensor prior to sending temp compensation to pH circuit
            {
                Serial1.print("<P:T,");
                Serial1.print(waterTemp);
                Serial1.println('>');
                tempCompMillis = millis();
                phMillis = millis();
            }
        }
        else                                                
        {
            Serial1.println("<P:R>");
            phMillis = millis();
        }
    }
    if ((currentMillis - tdsMillis > tdsPeriod) && (stopTDSReadings == false)) 
    { 
        Serial1.println("<T:R>");
        tdsMillis = millis();
    }

    recvWithStartEndMarkers();                                            // Gather data sent from Mega over serial
    processSerialData();
    client.loop();                                                       // MQTT client loop is required at end of void loop().
}

void recvWithStartEndMarkers()                                        // Function to receive serial data from Mega in format of "<MESSAGE>". Thanks Robin2!
{
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;

    while (Serial1.available() > 0 && newData == false)
    {
        rc = Serial1.read();

        if (recvInProgress == true)
        {
            if (rc != endMarker)
            {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars)
                {
                ndx = numChars - 1;
                }
            }
            else
            {
                receivedChars[ndx] = '\0'; //Terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker)
        {
        recvInProgress = true;
        }
    }
}

void processSerialData()
{
  if (newData != true){return;} 
  client.publish("feedback/debug", receivedChars);
  char commandChar = receivedChars[0];
  switch (commandChar)
  {
    case 'H': // If message starts with 'P' (pH)
    {
      char* strtokIndx;
      strtokIndx = strtok(receivedChars, ":");  // Skip the first segment which is the identifier
      strtokIndx = strtok(NULL, ":");
      client.publish("feedback/ph", strtokIndx);
      if (receivedChars[8] == '3')
      {
          client.publish("feedback/general", "pH Cal Successful!");   // When we ask the pH EZO circuit above how many points it has calibrated, if it responds with a 3 here, publish success message to HA.
      }
      break;
    }
    case 'T':  // If message starts with 'T' (TDS)
    {
      char* strtokIndx;
      strtokIndx = strtok(receivedChars, ":");  // Skip the first segment which is the identifier;
      strtokIndx = strtok(NULL, ":");
      client.publish("feedback/tds", strtokIndx);
      if (receivedChars[8] == '2')   // EC is considered 2 point calibration for some reason (dry, low, high)
      {
          client.publish("feedback/general", "TDS Cal Successful!");
      }
      break;
    }
    case 'F': // If message starts with F (Flood)
    {
      client.publish("feedback/flood", receivedChars);
      break;
    }
    case 'W':
    {
      char* strtokIndx;
      int waterSensorNumber;
      long waterSensorValue;
      strtokIndx = strtok(receivedChars, ":");  
      waterSensorNumber = atoi(strtokIndx);                    // Skip the first segment 
      strtokIndx = strtok(NULL, ":");
      waterSensorValue = atol(strtokIndx);  
      sprintf(strtokIndx, "%d:%d", waterSensorNumber, waterSensorValue);
      Serial.println(strtokIndx);
      client.publish("feedback/waterLevel", strtokIndx);
      break;
    }    

    case 'V':
    {
      char* strtokIndx;
      strtokIndx = strtok(receivedChars, ":");                   // Skip the first segment 
      strtokIndx = strtok(NULL, ":");
      client.publish("feedback/waterLevelFloat", strtokIndx);
      break;
    }
    case 'D':
    {
      int pumpNumber;
      long onTime;
      char* strtokIndx;
      strtokIndx = strtok(receivedChars, ":");   
      pumpNumber = atoi(strtokIndx);                // Skip the first segment 
      strtokIndx = strtok(NULL, ":");
      onTime = atol(strtokIndx);      
      
      sprintf(strtokIndx, "%d:%d", pumpNumber, onTime > 0);
      Serial.println(strtokIndx);
      client.publish("feedback/dosing", strtokIndx);       
      break;
    }
    case 'M':
    {
      char* strtokIndx;
      int soilMoistureNumber;
      int moistureReading;

      strtokIndx = strtok(receivedChars, ":");                   // Skip the first segment 
      soilMoistureNumber = atoi(strtokIndx);
      strtokIndx = strtok(NULL, ":");
      moistureReading = atol(strtokIndx);   
      sprintf(strtokIndx, "%d:%d", soilMoistureNumber, moistureReading);
      Serial.println(strtokIndx);
      client.publish("feedback/soilMoisture", strtokIndx);     
      break;
    }
    case 'P':
    {
      char* strtokIndx;
      strtokIndx = strtok(receivedChars, ":");                   // Skip the first segment 
      strtokIndx = strtok(NULL, ":");
      client.publish("feedback/waterLevel", strtokIndx);
      break;
    }
    case 'R':                                                      // Relay feedback. Message format is "Relay FB:<BOARD#>:<RELAY#>:<STATUS>". Example: "Relay FB:0:4:1"
    {
      int boardNumber;
      int relayNumber;
      int relayPower;
      char* strtokIndx;  
      char buff[18];

      strtokIndx = strtok(receivedChars, ":");                   // Skip the first segment which is the 'R'
      strtokIndx = strtok(NULL, ":");                            // Get the board number
      boardNumber = atoi(strtokIndx);
      strtokIndx = strtok(NULL, ":");                            // Get the relay number
      relayNumber = atoi(strtokIndx);  
      strtokIndx = strtok(NULL, ":");                            // Get the relay power state
      relayPower = atoi(strtokIndx);
      
      sprintf(buff, "%d:%d:%d", boardNumber, relayNumber, relayPower);
      client.publish("feedback/relays",buff);
      Serial.println(buff);
      break;
    }
  }
  newData = false;
}

void getWaterTemp()
{
    float f = celcius;
    sensors.requestTemperatures();
    celcius = sensors.getTempCByIndex(0);
    snprintf(waterTemp, sizeof(waterTemp), "%.1f", f);
    if (waterTemp[0] != '-')
    {
      client.publish("feedback/waterTemp", waterTemp);
    }
}

//Read BME280 sensor and publish to MQTT
void getBoxTemp()
{
  dtostrf(bme.readTemperature(), 3, 1, bmeBuffer);
  client.publish("feedback/boxTemp", bmeBuffer);
}

void getBoxHumidity()
{
  dtostrf(bme.readHumidity(), 3, 1, bmeBuffer);
  client.publish("feedback/boxHumidity", bmeBuffer);
}

void getBoxPressure()
{
  dtostrf(bme.readPressure() / 100.0F, 3, 1, bmeBuffer);
  client.publish("feedback/boxPressure", bmeBuffer);
}
void getWaterLevel()
{
  dtostrf(bme.readAltitude(SEALEVELPRESSURE_HPA), 3, 1, bmeBuffer);
  client.publish("feedback/boxSeaLevel", bmeBuffer);
}
void triggerRelay(int boardNumber, int relayNumber, int relayTrigger)
{
  char buff[50];
  sprintf(buff, "Triggering board#:%d, relay#:%d, state: %d",boardNumber,relayNumber,relayTrigger);
  Serial.println(buff);
  if (relayTrigger == 1)
  {
    digitalWrite(relayPins[boardNumber][relayNumber], LOW); // Turn relay ON
  }
  else if (relayTrigger == 0)
  {
    digitalWrite(relayPins[boardNumber][relayNumber], HIGH); // Turn relay OFF
  }
}
/*
 *       case 'R':                                                     // If message starts with "R", it's for relays. Message format is "Relay:<BOARD#>:<RELAY#>:<STATUS>". Example: "Relay:0:4:1"
        {
          int boardNumber;
          int relayNumber;
          int relayPower;
          char* strtokIndx;
          char buff[20];

          strtokIndx = strtok(receivedChars, ":");                    // Skip the first segment which is the 'R' character
          strtokIndx = strtok(NULL, ":");                             // Get the board number
          boardNumber = atoi(strtokIndx);
          strtokIndx = strtok(NULL, ":");                             // Get the relay number
          relayNumber = atoi(strtokIndx);
          strtokIndx = strtok(NULL, ":");                             // Get the relay power state
          relayPower = atoi(strtokIndx);

          triggerRelay(boardNumber, relayNumber, relayPower);

          sprintf(buff, "<Relay FB:%d:%d:%d>", boardNumber, relayNumber, relayPower);
          Serial3.println(buff);
          break;
        }

 */
float calculate_pH(float voltage, float calib_volt_7, float calib_volt_4, float calib_volt_10) {
  float slope = (7.0 - 4.0) / (calib_volt_7 - calib_volt_4);
  float intercept = 7.0 - slope * calib_volt_7;
  float pH = slope * voltage + intercept;
  return pH;
}