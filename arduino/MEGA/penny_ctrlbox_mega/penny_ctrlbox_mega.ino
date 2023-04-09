/***********************************************************************************************************************************************
  Notes:
  ----------------------------------------------------------------------------------------------------------------------------------------------
  - The ESP32 will poll the sensors at a preset interval, alternating between them. It will send a temperature compensation value based on the 
    temperature of the mixing res solution at a preset interval as well.
  - Data is passed betweeen the ESP32 and Mega via the Mega's Serial3 pins. All messages between them start with the '<' character and end with a '>'.
  - Anything printed to Serial(1) is just for debugging.
  - The ESP32 is responsible for relaying all data between this Mega and Home Assistant via MQTT.
  - The program uses millis() timing rather than delays to avoid blocking as much as possible. 

  - Sensors/Modules Attached:
    - 2x Ultrasonic distance sensors
    - 1x TDS sensor
    - 1x PH sensor
    - 9-10x Soil moisture sensor
    - 1x Water flood sensor
    - float valve sensor
    - Relay Module 8 Channel 5V Relay Module for Arduino Raspberry Pi AVR PIC ARM DSP ARM MSP430 TTL Logic Level
    - Relay Module 4 Channel 5V Relay Module for Arduino Raspberry Pi AVR PIC ARM DSP ARM MSP430 TTL Logic Level
 
  - Messages to ESP32:
    - <WL:X:YY.YY> - Water level # (in cm)
    - <TDS:XX.XX> - TDS (in ppm)
    - <H:XX.XX> - PH
    - <M:XX:YY.YY> - Soil moisture sensor - YY.YY (in %)
    - <V:Drainage bucket:1> - Drainage bucket is full / <FV:Drainage bucket:0> Empty
    - <Flooding (Location))> - Flooding detected
  - Messages expected from ESP32:
    - <CalibratePH:PAYLOAD>
    - <TDSCalibrate:PAYLOAD>
  - TODO:
    - How to calculate water amounts with ultrasonic?
    - Calibrate PH
    - Calibrate TDS
    - Calibrate soil moisture
    - Calibrate water level
    - Consolidate the waterlevel functions

***********************************************************************************************************************************************/

#define NUM_ELEMENTS(x) (sizeof(x) / sizeof((x)[0]))  // Use to calculate how many elements are in an array

//placeholders for the pins

#define WATER_LEAK_SENSOR_1 A14  // should this be analog?
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

#define ANALOG_PIN_SOIL_01 A4
#define ANALOG_PIN_SOIL_02 A5
#define ANALOG_PIN_SOIL_03 A6
#define ANALOG_PIN_SOIL_04 A7
#define ANALOG_PIN_SOIL_05 A8
#define ANALOG_PIN_SOIL_06 A9
#define ANALOG_PIN_SOIL_07 A10
#define ANALOG_PIN_SOIL_08 A11
#define ANALOG_PIN_SOIL_09 A12
//#define ANALOG_PIN_SOIL_10 13 Not used yet

#define PIN_SOIL_FLOOD 48
#define PIN_PH_PO_SENSOR A3
#define PIN_PH_DO_SENSOR A2
#define PIN_PH_TO_SENSOR A1
#define PIN_TDS_SENSOR A15
#define PIN_FLOAT_VALVE 49

#define PH_SENSOR_RX_PIN 2
#define PH_SENSOR_TX_PIN 3

#define PIN_US_DISTANCE_1_TRIG 16
#define PIN_US_DISTANCE_1_ECHO 17
#define PIN_US_DISTANCE_2_TRIG 18
#define PIN_US_DISTANCE_2_ECHO 19

// Relays
int relayPins[2][8]
{
  {PIN_RELAY8_01, PIN_RELAY8_02, PIN_RELAY8_03, PIN_RELAY8_04, PIN_RELAY8_05, PIN_RELAY8_06, PIN_RELAY8_07, PIN_RELAY8_08},              
  // 8 Chan Relay Board: [0]=plug switch 3 , [1]=plug switch 2 , [2]=plug switch 1 , [3]=Solenoid 5 , [4]=Solenoid 4 , [5]=Solenoid 3 , [6]=Solenoid 2 , [7]=Solenoid 1
  {PIN_RELAY4_01, PIN_RELAY4_02, PIN_RELAY4_03, PIN_RELAY4_04}                               
  // 4 Chan Relay Board: [0]=Stirring fans , [1]=internal cooling fna , [2]=plug switch 5 , [3]=plug switch 4
};

#define PIN_MEGA_TX0 1
#define PIN_MEGA_RX0 2

unsigned long baudRate = 115200;

const float waterLevel1Offset = 0;  // offset in centimeters from the sensor to the water surface
const float waterLevel2Offset = 0;  // offset in centimeters from the sensor to the water surface

#define VREF 5.0           // analog reference voltage(Volt) of the ADC
#define SCOUNT 30          // sum of sample point
int analogBuffer[SCOUNT];  // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0, copyIndex = 0;
float averageVoltage = 0, tdsValue = 0, temperature = 25;

// Serial receiving
bool newData = false;
const byte numChars = 100;
char receivedChars[numChars];
char* cmd; 

// Flood detection
char waterSensorNames[][22]  // For describing where flood sensor was triggered
  {
    // {"front of humidifier!"},
    { "back wall under box" },
    // {"in tent!"},
    //  {"in overflow!"}
  };
int waterSensorPins[]{ WATER_LEAK_SENSOR_1 };


unsigned long currentMillis;       // Snapshot of current time
unsigned long floodStartMillis;    // Timer to check for flood
unsigned long soilWaterLvlMillis;  // Timer to check water soil levels
unsigned long drainBucketMillis;   // Timer to check drain basin float sensor
unsigned long waterLvl1Millis;
unsigned long waterLvl2Millis;
unsigned long waterTDSMillis;
unsigned long waterPHMillis;

bool tdsCalibrateMode = false;
bool phCalibrateMode = false;

const unsigned long waterLvl1Period = 1000;     // Time in milliseconds between checking nute res water level
const unsigned long waterLvl2Period = 1000;     // Time in milliseconds between checking nute res water level
const unsigned long floodPeriod = 10000;        // Time between checking floor moisture sensors for flood
const unsigned long drainBucketPeriod = 3000;   // Time between checking float sensor in drain basin in tent
const unsigned long soilWaterLvlPeriod = 1000;  // Time between soil water level checking
unsigned long waterTDSPeriod = 1000;      // Time between soil water level checking
unsigned long waterPHPeriod = 1000;       // Time between soil water level checking

/*
const int pHpin = A0;    // Analog input pin for pH sensor
const float pHoffset = 0.00;    // pH offset calibration value
const float pHslope = 1.00;     // pH slope calibration value
*/

const int soilWaterSensorPin[9]{
  ANALOG_PIN_SOIL_01, ANALOG_PIN_SOIL_02, ANALOG_PIN_SOIL_03, ANALOG_PIN_SOIL_04, ANALOG_PIN_SOIL_05, ANALOG_PIN_SOIL_06, ANALOG_PIN_SOIL_07, ANALOG_PIN_SOIL_08, ANALOG_PIN_SOIL_09
};
//,ANALOG_PIN_SOIL_10;

void setup() {
  Serial3.begin(baudRate);
  Serial.begin(baudRate);

  //Why is this the only one starting 0?
  floodStartMillis = 0;

  pinMode(PIN_US_DISTANCE_1_TRIG, OUTPUT);
  pinMode(PIN_US_DISTANCE_1_ECHO, INPUT);
  pinMode(PIN_US_DISTANCE_2_TRIG, OUTPUT);
  pinMode(PIN_US_DISTANCE_2_ECHO, INPUT);
  pinMode(PIN_FLOAT_VALVE, INPUT_PULLUP);
  pinMode(PIN_TDS_SENSOR, INPUT);

  Serial.println("Setup complete, starting loop!");
}
void loop() {
/* if (Serial.available() > 0) {
    String receivedData = Serial.readString();
    // Process the received data
    Serial.println("Received data: " + receivedData);
  }*/
  currentMillis = millis();
  recvWithStartEndMarkers2();
  recvWithStartEndMarkers();
  processSerialData();
  
  for (int i = 0; i < 2; i++)
  {
    for (unsigned int j = 0; j < NUM_ELEMENTS(relayPins[i]); j++)
    {
      pinMode(relayPins[i][j], OUTPUT);
      digitalWrite(relayPins[i][j], HIGH);
    }
  }
return;
  //Check water soil sensors
  if (currentMillis - soilWaterLvlMillis >= soilWaterLvlPeriod) {
    checkSoilWaterLvl();
  }

  //Drain bucket float sensor
  if (currentMillis - drainBucketMillis >= drainBucketPeriod) {
    checkDrainBucket();
  }

  //Ultrasonic sensor 1
  if (currentMillis - waterLvl1Millis >= waterLvl1Period) {
    checkWaterLvl1();
  }

  //Ultrasonic sensor 2
  if (currentMillis - waterLvl2Millis >= waterLvl2Period) {
    checkWaterLvl2();
  }

  //Check for flood sensors
  if (currentMillis - floodStartMillis >= floodPeriod) {
    checkForFlood();
  }

  //Check TDS sensor
  if (currentMillis - waterTDSMillis >= waterTDSPeriod && (tdsCalibrateMode == false)) {
    checkTDSSensor();
  }

  //Check PH sensor
  if ((currentMillis - waterPHMillis >= waterPHPeriod) && (phCalibrateMode == false))
  //&& phCalibrateMode == )
  {
    checkPHSensor();
  }
}
void checkPHSensor() {
  char currentPHStr[10];  // define a buffer to store the ph as a string
  double currentPH = readPHSensor();
  if (currentPH > 0) {
    snprintf(currentPHStr, sizeof(currentPHStr), "%.1f", currentPH);
    Serial3.print("<PH:");
    Serial3.print(currentPHStr);
    Serial3.println('>');
  }
  waterPHMillis = millis();
}

//Read Ultrasonic sensors and send to ESP32
// Who handles the threshold of what we are watching for? HA or ESP32?
void checkWaterLvl1() {
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
  // Is it really cm?
  Serial.print("Distance1: ");
  Serial.print(distance);
  Serial.print(" cm, Water level1: ");
  Serial.print(waterLevelStr);
  Serial.println(" cm");

  if (waterLevel < 0) {
    Serial3.print("<WL:1:0.00>");
  } else {
    Serial3.print("<WL:1:");
    Serial3.print(waterLevelStr);
    Serial3.println('>');
  }
  waterLvl1Millis = millis();
}

//Read Ultrasonic sensors and send to ESP32
// Who handles the threshold of what we are watching for? HA or ESP32?
void checkWaterLvl2() {
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
  // f = distance;


  // Calculate the water level based on the distance and offset
  waterLevel = distance - waterLevel2Offset;

  snprintf(waterLevelStr, sizeof(waterLevelStr), "%.1f", waterLevel);
  // print the water level measurement to the serial monitor
  // Is it really cm?
  Serial.print("Distance2: ");
  Serial.print(distance);
  Serial.print(" cm, Water level2: ");
  Serial.print(waterLevelStr);
  Serial.println(" cm");
  if (waterLevel < 0) {
    Serial3.print("<WL:2:0.00>");
  } else {
    Serial3.print("<WL:2:");
    Serial3.print(waterLevelStr);
    Serial3.println('>');
  }
  waterLvl2Millis = millis();
}
float readPHSensor() {
  // set up the pins
  int toPin = PIN_PH_TO_SENSOR;  // analog input pin for To
  int doPin = PIN_PH_DO_SENSOR;  // analog input pin for Do
  int poPin = PIN_PH_PO_SENSOR;  // digital output pin for Po

  // set Po pin to output mode and turn on the power
  pinMode(poPin, OUTPUT);
  digitalWrite(poPin, HIGH);

  // wait for the sensor to stabilize
  delay(1000);

  // read the voltage across the To and Do pins
  float toVoltage = analogRead(toPin) * (5.0 / 1023.0);
  float doVoltage = analogRead(doPin) * (5.0 / 1023.0);

  // calculate the pH value
  float slope = 3.5;
  float intercept = 1.0;
  float ph = (slope * (doVoltage / toVoltage)) + intercept;

  // return the pH value
  return ph;
}
void checkDrainBucket() {
  long drainBucketStatus;
  drainBucketStatus = digitalRead(PIN_FLOAT_VALVE);
  if (drainBucketStatus == HIGH) {
    Serial.println("<Drainage bucket full!>");
    Serial3.println("<V:Drainage bucket:1>");
  } else {
    Serial.println("<Drainage Bucket OK>");
    Serial3.println("<V:Drainage Bucket:0>");
  }
  drainBucketMillis = millis();
}
void checkForFlood()  // I'm doing this with analog output of sensors, but this could switch to digital since I just want high or low really.
{
  int reading;
  for (unsigned int i = 0; i < NUM_ELEMENTS(waterSensorPins); i++) {
    analogRead(waterSensorPins[i]);
    delay(50);
    reading = analogRead(waterSensorPins[i]);
    if (reading <= 1000)  // If the analog reading is less than 1000, trigger flooding message with name of flooded area.
    {
      Serial3.print("<Flooding ");
      Serial3.print(waterSensorNames[i]);
      Serial3.println('>');
      Serial.print("Flooding:");
      Serial.println(waterSensorNames[i]);
    }
  }
  floodStartMillis = millis();
}

// Check for serial data from ESP32.
void recvWithStartEndMarkers() {
  static bool recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

  while (Serial3.available() > 0 && newData == false) {
    rc = Serial3.read();

    if (recvInProgress == true) {
      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) {
          ndx = numChars - 1;
        }
      } else {
        receivedChars[ndx] = '\0';
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    }

    else if (rc == startMarker) {
      recvInProgress = true;
    }
  }
}
// Check for serial data from Console.
void recvWithStartEndMarkers2() {
  static bool recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();

    if (recvInProgress == true) {
      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) {
          ndx = numChars - 1;
        }
      } else {
        receivedChars[ndx] = '\0';
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    }

    else if (rc == startMarker) {
      recvInProgress = true;
    }
  }
}
void processSerialData() {
  if (newData != true) { return; }

  Serial.print("Received Input: ");
  Serial.println(receivedChars);
  char commandChar = receivedChars[0];
  switch (commandChar) {
    case 'C':  // If the message from the ESP32 starts with a "C", it's related to Calibrate pH.
      {
        //char* strtokIndx;
        //strtokIndx = 
        strtok(receivedChars, ":");
        cmd = strtok(NULL, ":");
        if (cmd[0] == 'C' || cmd[0] == 'R') {
          waterPHPeriod = 1400;  // If a command has been sent to calibrate or take a reading we wait 1400ms so that the circuit has time to take the reading.
        } else {
          waterPHPeriod = 300;  // If any other command has been sent we wait only 300ms.
        }                       // Send to I2C
        if (cmd[0] != 'T') {
          char buff[20];
          dtostrf(readPHSensor(), 6, 3, buff); // Convert myFloat to a string with 3 decimal places and store it in buffer

          sprintf(buff, "<PH:%s>", buff);
          Serial.print("Printing to Serial: ");
          Serial.println(buff);
          Serial3.println(buff);
        }
        break;
      }
    case 'T':  // If the message from the ESP32 starts with a "1", it's related to EC.
      {
        //char* strtokIndx;
        //strtokIndx = 
        strtok(receivedChars, ":");
        cmd = strtok(NULL, ":");
        if (cmd[0] == 'C' || cmd[0] == 'R') {
          waterTDSPeriod = 1400;  // If a command has been sent to calibrate or take a reading we wait 1400ms so that the circuit has time to take the reading.
        } else {
          waterTDSPeriod = 300;  // If any other command has been sent we wait only 300ms.
        }

        if (cmd[0] != 'T') {
          char buff[20];

          // dtostrf(floatVar, minimumWidth, numDecimalPlaces, charBuffer)
          dtostrf(readTDSSensor(), 6, 3, buff); // Convert myFloat to a string with 3 decimal places and store it in buffer

          sprintf(buff, "<TDS:%s>", buff );
          Serial.print("Printing to Serial: ");
          Serial.println(buff);
          Serial3.println(buff);
        }
        break;
      }
   case 'R':                                                     // If message starts with "R", it's for relays. Message format is "Relay:<BOARD#>:<RELAY#>:<STATUS>". Example: "Relay:0:4:1"
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
  }
}
void checkSoilWaterLvl() {
  int numSensors = NUM_ELEMENTS(soilWaterSensorPin);
  readSoilCapacitanceSensors(soilWaterSensorPin, numSensors);
}
void readSoilCapacitanceSensors(const int sensorPins[], int numSensors) {
  for (int i = 0; i < numSensors; i++) {
    int sensorValue = analogRead(sensorPins[i]);
    Serial.print("Soil Sensor ");
    Serial.print(i + 1);
    Serial.print(" reading: ");
    Serial.println(sensorValue);
    delay(100);
  }
}
void checkTDSSensor() {
  readTDSSensor();
  Serial.print("TDS Value: ");
  Serial.print(tdsValue, 0);
  Serial.println("ppm");
  
}
float readTDSSensor() {
  static unsigned long analogSampleTimepoint = millis();
  if (millis() - analogSampleTimepoint > 40U)  //every 40 milliseconds,read the analog value from the ADC
  {
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(PIN_TDS_SENSOR);  //read the analog value and store into the buffer
    analogBufferIndex++;
    if (analogBufferIndex == SCOUNT) { analogBufferIndex = 0; }
  }
  static unsigned long printTimepoint = millis();
  if (millis() - printTimepoint > 800U) {
    printTimepoint = millis();
    for (copyIndex = 0; copyIndex < SCOUNT; copyIndex++) {
      analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
      averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * (float)VREF / 1024.0;                                                                                                   // read the analog value more stable by the median filtering algorithm, and convert to voltage value
      float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0);                                                                                                                //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
      float compensationVoltage = averageVoltage / compensationCoefficient;                                                                                                             //temperature compensation
      tdsValue = (133.42 * compensationVoltage * compensationVoltage * compensationVoltage - 255.86 * compensationVoltage * compensationVoltage + 857.39 * compensationVoltage) * 0.5;  //convert voltage value to tds value
      return tdsValue;
    }
  }
  return -1;
}
float getMedianNum(int* tdsValues, int numValues) {
  // Sort the array in ascending order
  qsort(tdsValues, numValues, sizeof(int), [](const void* a, const void* b) {
    int arg1 = *static_cast<const int*>(a);
    int arg2 = *static_cast<const int*>(b);
    if (arg1 < arg2) return -1;
    if (arg1 > arg2) return 1;
    return 0;
  });

  // Calculate the median value
  int middleIndex = numValues / 2;
  float medianValue;
  if (numValues % 2 == 0) {
    medianValue = (float)(tdsValues[middleIndex - 1] + tdsValues[middleIndex]) / 2.0;
  } else {
    medianValue = (float)tdsValues[middleIndex];
  }

  return medianValue;
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
