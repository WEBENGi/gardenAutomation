#include <DallasTemperature.h>
//#include <Wire.h>
#include <SoftwareSerial.h>

#define NUM_ELEMENTS(x)  (sizeof(x) / sizeof((x)[0])) // Use to calculate how many elements are in an array

//placeholders for the pins

#define WATER_LEAK_SENSOR_1 A14 // should this be analog?

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

#define PIN_MEGA_TX0 1
#define PIN_MEGA_RX0 2

unsigned long baudRate  = 115200;

const float waterLevel1Offset = 0; // offset in centimeters from the sensor to the water surface
const float waterLevel2Offset = 0; // offset in centimeters from the sensor to the water surface

#define VREF 5.0 // analog reference voltage(Volt) of the ADC
#define SCOUNT 30 // sum of sample point
int analogBuffer[SCOUNT]; // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0,copyIndex = 0;
float averageVoltage = 0,tdsValue = 0,temperature = 25;

SoftwareSerial phSerial(PH_SENSOR_RX_PIN, PH_SENSOR_TX_PIN);

/*// Atlas
int channel;                                    // For channel switching - 0-7 serial, 8-127 I2C addresses
char* cmd;
bool i2cCallComplete;                           // This flag allows void loop() to keep running quickly without waiting for i2c data to arrive
char sensorData[30];                            // A 30 byte character array to hold incoming data from the sensors
*/
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
unsigned long waterTDSMillis;
unsigned long waterPHMillis;
//unsigned long soilWaterLvlMillis;

const unsigned long waterLvl1Period = 1000;      // Time in milliseconds between checking nute res water level
const unsigned long waterLvl2Period = 1000;      // Time in milliseconds between checking nute res water level
const unsigned long floodPeriod = 10000;        // Time between checking floor moisture sensors for flood
const unsigned long drainBucketPeriod = 3000;    // Time between checking float sensor in drain basin in tent
const unsigned long soilWaterLvlPeriod = 1000;  // Time between soil water level checking
const unsigned long waterTDSPeriod = 1000;  // Time between soil water level checking
const unsigned long waterPHPeriod = 1000;  // Time between soil water level checking

//unsigned long scaleCalMillis;                   // Timer to read the scale when calibrating it
//unsigned long i2cWaitMillis;                    // Timer to wait for i2c data after call
//unsigned long i2cWaitPeriod;                    // Time to wait for sensor data after I2C_call function
//const unsigned long drainageCheckPeriod = 2000; // Time between checking drain bucket float sensor
//const unsigned long scaleCalPeriod = 500;       // Time between reading HX711 sensor when calibrating

/*// Atlas
boolean pHCalledLast = false;                       // Tracks whether pH was polled last or EC.F
boolean stopReadings = false;                       // I set this flag to tell system to stop taking readings at certain points when calibrating sensors to avoid errors.
*/
/*
const int pHpin = A0;    // Analog input pin for pH sensor
const float pHoffset = 0.00;    // pH offset calibration value
const float pHslope = 1.00;     // pH slope calibration value
*/
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
 // Wire.begin();
  floodStartMillis = 0;
  pinMode(PIN_FLOAT_VALVE, INPUT_PULLUP);
  pinMode(PIN_TDS_SENSOR, INPUT);

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

  if (currentMillis - waterTDSMillis >= waterTDSPeriod)
  {
    checkWaterTDS();
  }
  if (currentMillis - waterPHMillis >= waterPHPeriod)
  {
    readPHSensor();
  }
}
void checkWaterPH(){
   while (phSerial.available() > 0) {
    char c = phSerial.read();
    if (c == '\r') {
      String phString = phSerial.readStringUntil('\n');
      float pHValue = phString.toFloat();
      Serial.print("pH value: ");
      Serial.print(pHValue, 2);

      phSerial.write("T");
      delay(1000);

      while (phSerial.available() > 0) {
        char d = phSerial.read();
        if (d == '\r') {
          String tempString = phSerial.readStringUntil('\n');
          float tempValue = tempString.toFloat();
          Serial.print(", Temperature: ");
          Serial.println(tempValue, 2);
        }
      }
    }
  }
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
float readPHSensor() {
  // set up the pins
  int toPin = A0;  // analog input pin for To
  int doPin = A1;  // analog input pin for Do
  int poPin = 2;   // digital output pin for Po

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
  if (newData != true){return;}
  
  char commandChar = receivedChars[0];
  switch (commandChar)
  {
/*    case '9':                                                     // If the message from the ESP32 starts with a "9", it's related to pH.
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
    }*/
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
  }
  newData = false;
}
void checkSoilWaterLvl() {
}
void checkTDSSensor(){
  static unsigned long analogSampleTimepoint = millis();
  if(millis()-analogSampleTimepoint > 40U) //every 40 milliseconds,read the analog value from the ADC
  {
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin); //read the analog value and store into the buffer
    analogBufferIndex++;
    if(analogBufferIndex == SCOUNT) { analogBufferIndex = 0; }
  }
  static unsigned long printTimepoint = millis();
  if(millis()-printTimepoint > 800U)
  {
    printTimepoint = millis();
    for(copyIndex=0;copyIndex<SCOUNT;copyIndex++){
      analogBufferTemp[copyIndex]= analogBuffer[copyIndex];
      averageVoltage = getMedianNum(analogBufferTemp,SCOUNT) * (float)VREF/ 1024.0; // read the analog value more stable by the median filtering algorithm, and convert to voltage value
      float compensationCoefficient=1.0+0.02*(temperature-25.0); //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
      float compensationVolatge=averageVoltage/compensationCoefficient; //temperature compensation
      tdsValue=(133.42*compensationVolatge*compensationVolatge*compensationVolatge - 255.86*compensationVolatge*compensationVolatge + 857.39*compensationVolatge)*0.5; //convert voltage value to tds value
      Serial.print("TDS Value:");
      Serial.print(tdsValue,0);
      Serial.println("ppm");
    }
  }
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
int getMedianNum2(int bArray[], int iFilterLen)
{
  int bTab[iFilterLen];
  for (byte i = 0; i<iFilterLen; i++)
  {
    bTab[i] = bArray[i];
    int i, j, bTemp;
    for (j = 0; j < iFilterLen - 1; j++)
    {
      for (i = 0; i < iFilterLen - j - 1; i++)
      {
        if (bTab[i] > bTab[i + 1])
        {
          bTemp = bTab[i];
          bTab[i] = bTab[i + 1];
          bTab[i + 1] = bTemp;
        }
      }
    }
  }
  if ((iFilterLen & 1) > 0){
    bTemp = bTab[(iFilterLen - 1) / 2];
  }else{
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  }
  return bTemp;
}