#include <DallasTemperature.h>
#include <Wire.h>

#define NUM_ELEMENTS(x)  (sizeof(x) / sizeof((x)[0])) // Use to calculate how many elements are in an array

//placeholders for the pins

#define WATER_LEAK_SENSOR_1 0

#define PIN_MC_1_ENA 31
#define PIN_MC_1_ENB 32
#define PIN_MC_1_IO1 30
#define PIN_MC_1_IO3 33
#define PIN_MC_2_ENA 51
#define PIN_MC_2_ENB 52
#define PIN_MC_2_IO1 50
#define PIN_MC_2_IO3 53
#define PIN_MC_3_ENA 36
#define PIN_MC_3_ENB 35
#define PIN_MC_3_IO1 35
#define PIN_MC_3_IO3 34

#define ANALOG_PIN_SOIL_01 4
#define ANALOG_PIN_SOIL_02 5
#define ANALOG_PIN_SOIL_03 6
#define ANALOG_PIN_SOIL_04 7
#define ANALOG_PIN_SOIL_05 8
#define ANALOG_PIN_SOIL_06 9
#define ANALOG_PIN_SOIL_07 10
#define ANALOG_PIN_SOIL_08 11
#define ANALOG_PIN_SOIL_09 12
//#define ANALOG_PIN_SOIL_10 13 Not used yet

#define PIN_SOIL_FLOOD 48
#define PIN_PH_PO_SENSOR a3
#define PIN_PH_DO_SENSOR a2
#define PIN_PH_TO_SENSOR a1
#define PIN_TDS_SENSOR a15
#define PIN_FLOAT_VALVE 49

#define PIN_US_DISTANCE_1_TRIG 16
#define PIN_US_DISTANCE_1_ECHO 17
#define PIN_US_DISTANCE_2_TRIG 18
#define PIN_US_DISTANCE_2_ECHO 19

#define PIN_MEGA_TX0 1
#define PIN_MEGA_RX0 2

unsigned long baudRate  = 115200;

const float waterLevel1Offset = 0; // offset in centimeters from the sensor to the water surface
const float waterLevel2Offset = 0; // offset in centimeters from the sensor to the water surface

// Atlas
int channel;                                    // For channel switching - 0-7 serial, 8-127 I2C addresses
char* cmd;
bool i2cCallComplete;                           // This flag allows void loop() to keep running quickly without waiting for i2c data to arrive
char sensorData[30];                            // A 30 byte character array to hold incoming data from the sensors

// Serial receiving
bool newData = false;
const byte numChars = 100;
char receivedChars[numChars];

// Flood detection
char waterSensorNames[][22]                     // For describing where flood sensor was triggered
{
  {"front of humidifier!"},
  // {"under res 2!"},
  // {"in tent!"},
  //  {"in overflow!"}
};
int waterSensorPins[] {WATER_LEAK_SENSOR_1};

// Timing
unsigned long dosingPumpMillis[6];                   // Timer to check to see if pump needs to be shut off
unsigned long dosingPumpPeriod[6];                   // Array of modifiable "pump on" times that are sent by Home Assistant. These determine how long to keep pump on before shutting off.

unsigned long currentMillis;                    // Snapshot of current time
unsigned long floodStartMillis;                 // Timer to check for flood
unsigned long waterLvlMillis;                   // Timer to check nute res water level with weigh sensors
unsigned long soilWaterLvlMillis;                   // Timer to check water soil levels
unsigned long drainBucketMillis;                 // Timer to check drain basin float sensor
unsigned long waterLvl1Millis;
unsigned long waterLvl2Millis;
//unsigned long soilWaterLvlMillis;

const unsigned long waterLvl1Period = 1000;      // Time in milliseconds between checking nute res water level
const unsigned long waterLvl2Period = 1000;      // Time in milliseconds between checking nute res water level
const unsigned long floodPeriod = 10000;        // Time between checking floor moisture sensors for flood
const unsigned long drainBucketPeriod = 3000;    // Time between checking float sensor in drain basin in tent
const unsigned long soilWaterLvlPeriod = 1000;  // Time between soil water level checking

//unsigned long scaleCalMillis;                   // Timer to read the scale when calibrating it
unsigned long i2cWaitMillis;                    // Timer to wait for i2c data after call
unsigned long i2cWaitPeriod;                    // Time to wait for sensor data after I2C_call function
//const unsigned long drainageCheckPeriod = 2000; // Time between checking drain bucket float sensor
//const unsigned long scaleCalPeriod = 500;       // Time between reading HX711 sensor when calibrating

// Atlas
boolean pHCalledLast = false;                       // Tracks whether pH was polled last or EC.F
boolean stopReadings = false;                       // I set this flag to tell system to stop taking readings at certain points when calibrating sensors to avoid errors.

const int soilWaterSensorPin[9] 
{
  ANALOG_PIN_SOIL_01, ANALOG_PIN_SOIL_02, ANALOG_PIN_SOIL_03, ANALOG_PIN_SOIL_04, ANALOG_PIN_SOIL_05, ANALOG_PIN_SOIL_06, ANALOG_PIN_SOIL_07, ANALOG_PIN_SOIL_08, ANALOG_PIN_SOIL_09
};
//,ANALOG_PIN_SOIL_10;

// GPIO Pin numbers for Enable
const int dosingPumpEnablePin[6]
{
  PIN_MC_1_IO1, PIN_MC_1_IO3, PIN_MC_1_IO1, PIN_MC_2_IO3, PIN_MC_3_IO1, PIN_MC_3_IO3
  //  19, 33, 26, 14, 13, 23
};

// GPIO Pin numbers for PWM
const int dosingPumpPWMpin[6]
{
  PIN_MC_1_ENA, PIN_MC_1_ENB, PIN_MC_2_ENA, PIN_MC_2_ENB, PIN_MC_3_ENA, PIN_MC_3_ENB
  // 18, 32, 25, 27, 12, 5
};

// PWM properties
const int freq = 5000;
const int resolution = 8;
const int pwmChannel[6]                       // This array has 7 due to 6 Dosing pumps + the noctua fan in the control box. Noctua pwm rate never changes though.
{
  0, 1, 2, 3, 4, 5//, 6
};

int pumpSpeeds[6];

void setup() {
  pinMode(PIN_US_DISTANCE_1_TRIG, OUTPUT);
  pinMode(PIN_US_DISTANCE_1_ECHO, INPUT);
  pinMode(PIN_US_DISTANCE_2_TRIG, OUTPUT);
  pinMode(PIN_US_DISTANCE_2_ECHO, INPUT);
  Serial3.begin(baudRate);
  Serial.begin(baudRate);
  Wire.begin();
  floodStartMillis = 0;
  pinMode(PIN_FLOAT_VALVE, INPUT_PULLUP);


  for (unsigned int i = 0; i < NUM_ELEMENTS(dosingPumpEnablePin); i++)
  {
    pinMode(dosingPumpEnablePin[i], OUTPUT);
  }

  /*    for (int i = 0; i < NUM_ELEMENTS(dosingPumpPWMpin); i++)   // The condition in for loop is -1 because the noctua fan pwm channel is the last one and we will update it separately below
    {
      ledcSetup(pwmChannel[i], freq, resolution);
      ledcAttachPin(dosingPumpPWMpin[i], pwmChannel[i]);
    }*/

  Serial.println("Setup complete, starting loop!");
}
void loop() {

  currentMillis = millis();
  recvWithStartEndMarkers();
  processSerialData();

  //Check water soil sensors
  if (currentMillis - soilWaterLvlMillis >= soilWaterLvlPeriod)
  {
    checkSoilWaterLvl();
  }

  //Drain bucket float sensor
  if (currentMillis - drainBucketMillis >= drainBucketPeriod)
  {
    checkDrainBucket();
  }

  if (currentMillis - waterLvl1Millis >= waterLvl1Period)
  {
    checkWaterLvl1();
  }
  if (currentMillis - waterLvl2Millis >= waterLvl2Period)
  {
    checkWaterLvl2();
  }

  //Check for flood sensors
  if (currentMillis - floodStartMillis >= floodPeriod)
  {
    checkForFlood();
  }

  /*if ((currentMillis - i2cWaitMillis >= i2cWaitPeriod) && (i2cCallComplete == true))
    {
      parseI2Cdata();
    } */
}
/*void loop2() {
    long duration, distance;

    digitalWrite(TRIGGER_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIGGER_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGGER_PIN, LOW);

    duration = pulseIn(ECHO_PIN, HIGH);
    distance = (duration / 2) / 29.1;

    Serial.print(distance);
    Serial.println(" cm");
    delay(500);

    for (int i = 0; i < NUM_ELEMENTS(dosingPumpPeriod); i++)
    {
        if ((dosingPumpPeriod[i] > 0) && (currentMillis - dosingPumpMillis[i] >= dosingPumpPeriod[i]))      // If pump is on and its timer has expired...
        {
            setPumpPower(i, 0);                                                                               // Shut it off by setting timer to 0.
        }
    }
  }
*/
void setPumpSpeeds(int pumpNumber, int pumpSpeed)
{ 
  analogWrite(pwmChannel[pumpNumber], pumpSpeed);
}

void setPumpPower(int pumpNumber, long onTime)
{
  char buff[5];
  digitalWrite(dosingPumpEnablePin[pumpNumber], onTime);           // If onTime is > 0, write the pin high. Otherwise write it low.
  if (onTime > 0)
  {
    dosingPumpMillis[pumpNumber] = millis();                      // If pump is being turned on, start the timer
    dosingPumpPeriod[pumpNumber] = onTime;
  }
  else
  {
    dosingPumpMillis[pumpNumber] = 0;                           // If pump is being turned off, zero millis/period out.
    dosingPumpPeriod[pumpNumber] = 0;
  }
  sprintf(buff, "%d:%d", pumpNumber, onTime > 0);
  Serial.println(buff);
//  client.publish("feedback/dosing", buff);                        // Send feedback (<PUMP#>:<STATE>)
}

void I2C_call()                                                     // Function to parse and call I2C commands.
{
  memset(sensorData, 0, sizeof(sensorData));                        // Clear sensorData array;

  if (cmd[0] == 'C' || cmd[0] == 'R')
  {
    i2cWaitPeriod = 1400;                                           // If a command has been sent to calibrate or take a reading we wait 1400ms so that the circuit has time to take the reading.
  }
  else
  {
    i2cWaitPeriod = 300;                                            // If any other command has been sent we wait only 300ms.
  }

  Wire.beginTransmission(channel);                                  // cCall the circuit by its ID number.
  Wire.write(cmd);                                                  // Transmit the command that was sent through the serial port.
  Wire.endTransmission();                                           // End the I2C data transmission.
  i2cWaitMillis = millis();
  i2cCallComplete = true;
}

void parseI2Cdata()
{
  byte code = 254;                                                  // Used to hold the I2C response code.
  byte inChar = 0;                                                  // Used as a 1 byte buffer to store in bound bytes from the I2C Circuit.
  byte sensorBytesReceived = 0;                                     // We need to know how many characters bytes have been received

  while (code == 254)                                               // In case the command takes longer to process, we keep looping here until we get a success or an error
  {
    Wire.requestFrom(channel, 48, 1);                               // Call the circuit and request 48 bytes (this is more then we need).
    code = Wire.read();
    while (Wire.available())                                        // Are there bytes to receive?
    {
      inChar = Wire.read();                                         // Receive a byte.

      if (inChar == 0)                                              // If we see that we have been sent a null command.
      {
        Wire.endTransmission();                                     // End the I2C data transmission.
        break;
      }
      else
      {
        sensorData[sensorBytesReceived] = inChar;                   // Load this byte into our array.
        sensorBytesReceived++;
      }
    }

    switch (code)
    {
      case 1:
        {
          if ((channel == 99) && (cmd[0] != 'T'))                   // Print the sensor data for pH to console and to ESP32 if it's a normal reading
          {
            char buff[20];
            sprintf(buff, "<PH:%s>", sensorData);
            Serial3.println(buff);
          }
          else if ((channel == 100) && (cmd[0] != 'T'))                // Print the sensor data for EC to console and to ESP32 if it's a normal reading
          {
            char buff[20];
            sprintf(buff, "<EC:%s>", sensorData);
            Serial3.println(buff);
          }
          break;
        }
    }
  }
  i2cCallComplete = false;
}
/*
  void checkWaterLvl()
  {
  float liters = (scale.get_units());
  if (liters < 0)
  {
    Serial3.print("<WL:0.00>");
  }
  else
  {
    Serial3.print("<WL:");
    Serial3.print(liters);
    Serial3.println('>');

  }

  waterLvlMillis = millis();
  }
*/
//Read Ultrasonic sensors  and publish to MQTT
void checkWaterLvl1()
{
  double duration, distance, waterLevel;
  //float f;
  char waterLevelStr[10];  // define a buffer to store the water level as a string
  
  digitalWrite(PIN_US_DISTANCE_1_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_US_DISTANCE_1_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_US_DISTANCE_1_TRIG, LOW);
  duration = pulseIn(PIN_US_DISTANCE_1_ECHO, HIGH);
  distance = duration * 0.034 / 2;
  //f = distance;
  
  // Calculate the water level based on the distance and offset
  waterLevel = distance - waterLevel1Offset;

  snprintf(waterLevelStr, sizeof(waterLevelStr), "%.1f", waterLevel);
  // print the water level measurement to the serial monitor
  /*
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.print(" cm, Water level: ");
  Serial.print(waterLevelStr);
  Serial.println(" cm");
  */
  if (waterLevelStr[0] != '-')
  {
//    client.publish("feedback/waterLevel1", waterLevel);
  }
  waterLvl1Millis = millis();
}

//Read Ultrasonic sensors  and publish to MQTT
void checkWaterLvl2()
{
  double duration, distance, waterLevel;
  //float f;
  char waterLevelStr[10];  // define a buffer to store the water level as a string

  digitalWrite(PIN_US_DISTANCE_2_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_US_DISTANCE_2_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_US_DISTANCE_2_TRIG, LOW);
  duration = pulseIn(PIN_US_DISTANCE_2_ECHO, HIGH);
  distance = duration * 0.034 / 2;
//  f = distance;
  
  // Calculate the water level based on the distance and offset
  waterLevel = distance - waterLevel2Offset;
  
  snprintf(waterLevelStr, sizeof(waterLevelStr), "%.1f", waterLevel);
  // print the water level measurement to the serial monitor
  /*
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.print(" cm, Water level: ");
  Serial.print(waterLevelStr);
  Serial.println(" cm");
  */
  
  if (waterLevelStr[0] != '-')
  {
    //client.publish("feedback/waterLevel2", waterLevel);
  }
  waterLvl2Millis = millis();
}

void checkDrainBucket()
{
  long drainBucketStatus;
  drainBucketStatus = digitalRead(PIN_FLOAT_VALVE);
  if (drainBucketStatus == HIGH)
  {
    Serial3.println("<Drainage bucket full!>");
  }
  else
  {
    Serial3.println("<Drainage Bucket OK>");
  }
  drainBucketMillis = millis();
}
void checkForFlood()                                                  // I'm doing this with analog output of sensors, but this could switch to digital since I just want high or low really.
{
  int reading;
  for (unsigned int i = 0; i < NUM_ELEMENTS(waterSensorPins); i++)
  {
    analogRead(waterSensorPins[i]);
    delay(50);
    reading = analogRead(waterSensorPins[i]);
    if (reading <= 1000)                                        // If the analog reading is less than 1000, trigger flooding message with name of flooded area.
    {
      Serial3.print("<Flooding ");
      Serial3.print(waterSensorNames[i]);
      Serial3.println('>');
    }
  }
  floodStartMillis = millis();
}
void recvWithStartEndMarkers()                                       // Check for serial data from ESP32. Thanks to Robin2 for the serial input basics thread on the Arduino forums.
{
  static bool recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

  while (Serial3.available() > 0 && newData == false)
  {
    rc = Serial3.read();

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
        receivedChars[ndx] = '\0';
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
  if (newData == true)
  {
    char commandChar = receivedChars[0];
    switch (commandChar)
    {
      case '9':                                                     // If the message from the ESP32 starts with a "9", it's related to pH.
        {
          channel = atoi(strtok(receivedChars, ":"));                 // Parse the string at each colon
          cmd = strtok(NULL, ":");
          I2C_call();                                                 // Send to I2C
          break;
        }

      case '1':                                                     // If the message from the ESP32 starts with a "1", it's related to EC.
        {
          channel = atoi(strtok(receivedChars, ":"));
          cmd = strtok(NULL, ":");
          I2C_call();
          break;
        }

      case 'D':
        {
          //Dosing / PumpPower:
          int pumpNumber;
          long onTime;
          char* strtokIndx;

          strtokIndx = strtok(receivedChars, ":");                    // Skip the first segment which is the 'D' character

          strtokIndx = strtok(strtokIndx, ":");     // Get the pump number
          pumpNumber = atoi(strtokIndx);
          strtokIndx = strtok(NULL, ":");
          onTime = atol(strtokIndx);              // Get the on time

          Serial.println(pumpNumber);
          Serial.println(onTime);

          setPumpPower(pumpNumber, onTime);
          break;
        }
      case 'P':
        {
          int pumpNumber;
          int pwmVal;
          char* strtokIndx;

          strtokIndx = strtok(receivedChars, ":");    // Get the pump number
          pumpNumber = atoi(strtokIndx);
          strtokIndx = strtok(NULL, ",");
          pwmVal = atoi(strtokIndx);              // Get the PWM val

          setPumpSpeeds(pumpNumber, pwmVal);
          break;
        }

/*      case 'H':                                                     // If message starts with the letter "H", it's for the weigh scale.
        {
          char commandChar = receivedChars[7];                        // Message format will be "HX711: Begin", "HX711: Exit", or a calibration value like "-20000"
          switch (commandChar)
          {
            case 'B':                                                 // Begin cal mode
              {
                beginCalMode();
                break;
              }
            case 'E':                                                 // Exit cal mode
              {
                int cfAddress = 0;
                int zfAddress = 4;
                scaleCalMode = false;
                EEPROM.put(cfAddress, mixingRes.calibrationFactor);
                EEPROM.put(zfAddress, mixingRes.zeroFactor);
                Serial3.print("<HX711: Calibration saved to EEPROM!>");
                Serial.println(mixingRes.calibrationFactor);
                Serial.println(mixingRes.zeroFactor);
                scale.tare();                                           // Tare the scale. Before you save and exit, the reservoir and any equipment that sit in it should be on the sensor,
                break;                                                  // however, the weights you use to calibrate it must be removed. Only the stuff that will be permanent should stay.
              }
            default:
              {
                char* strtokIndx;
                strtokIndx = strtok(receivedChars, ":");                // Skip the first segment which is the "HX711:" part of the message
                strtokIndx = strtok(NULL, ":");                         // Get the calibration factor
                mixingRes.calibrationFactor = atol(strtokIndx);
              }
          }
          break;
        }*/
    }
    newData = false;
  }
}
void checkSoilWaterLvl() {
}
