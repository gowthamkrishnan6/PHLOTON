#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <Arduino.h>
#include <HardwareSerial.h>

#define SD_MOSI 41
#define SD_MISO 40
#define SD_SCLK 39
#define SD_CS 42
HardwareSerial SerialPort(2);

const gpio_num_t RX_PIN = GPIO_NUM_18;
const gpio_num_t TX_PIN = GPIO_NUM_17;
const gpio_num_t THERMISTOR_PIN_1 = GPIO_NUM_3;
const gpio_num_t THERMISTOR_PIN_2 = GPIO_NUM_9;
const gpio_num_t THERMISTOR_PIN_3 = GPIO_NUM_10;
const gpio_num_t THERMISTOR_PIN_4 = GPIO_NUM_11;
const gpio_num_t THERMISTOR_PIN_5 = GPIO_NUM_12;
const gpio_num_t THERMISTOR_PIN_AMB = GPIO_NUM_8;
const gpio_num_t LED_PIN = GPIO_NUM_21;
const int THERMISTOR_PIN[6] = {THERMISTOR_PIN_AMB, THERMISTOR_PIN_1, THERMISTOR_PIN_2, THERMISTOR_PIN_3, THERMISTOR_PIN_4, THERMISTOR_PIN_5};
// Constants for the NTC thermistor
const float vRef = 2.5;          // Reference voltage
const uint16_t adcMax = 4095;    // Max ADC value (12-bit ADC)
const float rFixed = 10000.0;    // Fixed resistor value in ohms (10k)
const float beta = 3950.0;       // Beta parameter of the NTC thermistor
const float r0 = 10000.0;        // Resistance at reference temperature (10k ohms)
const float t0 = 285.15;         // Reference temperature in Kelvin (4°C)
float ntcResistance[6];
float temperatureCelsius[6];

String receivedData = "";
float CSFAN_ISNS, HSFAN_ISNS, VBatt_ISNS, SOC;
float HSFAN_Dutycycle, CSFAN_Dutycycle, TEC_Dutycycle;

File myFile;

float Kp = 200.0;      // Proportional gain (tune based on your system)
float Ki = 50.0;       // Integral gain (tune based on your system)
float setpoint = 6.0;  // Desired temperature in Celsius
                       // Variables
float integral = 0.0;
float previous_error = 0.0;
float dt = 1.0; // Time step in seconds


void setup() {
  Serial.begin(115200);
  SerialPort.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);  // RX on GPIO18, TX on GPIO17
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  Serial.println("Setup start");
  SPI.begin(SD_SCLK, SD_MISO, SD_MOSI, SD_CS);
  if (!SD.begin(SD_CS)) {
    Serial.println("SD Card MOUNT FAIL");
    return;
  } else {
    Serial.println("SD Card MOUNT SUCCESS");
    Serial.println("");

    // Create or open the file to store temperature data
    myFile = SD.open("/temperature1_log.txt", FILE_WRITE);
    if (myFile) {
      myFile.close();  // Ensure the file is closed after creation
      digitalWrite(LED_PIN, HIGH);
      delay(100);
      digitalWrite(LED_PIN, LOW);
      delay(100);
    } else {
      Serial.println("Error opening file for writing");
    }
    Serial.println("Setup complete");
    digitalWrite(LED_PIN, HIGH);
    delay(1000);
    digitalWrite(LED_PIN, LOW);
    delay(1000);
  }
}


// Function to calculate NTC resistance from ADC value
float calculateNTCResistance(uint16_t adcValue, float vRef, uint16_t adcMax, float rFixed) {
    // Calculate the ADC voltage
    float vAdc = ((float)adcValue / (float)adcMax) * vRef;

    // Calculate the NTC resistance using the voltage divider equation
    float rNtc = (rFixed * vAdc) / (vRef - vAdc);

    return rNtc;
}

// Function to convert NTC resistance to temperature using the Beta parameter equation
float ntcResistanceToTemperature(float resistance, float beta, float r0, float t0) {
    // Calculate temperature in Kelvin
    float temperatureKelvin = 1.0 / ((1.0 / t0) + (1.0 / beta) * log(resistance / r0));

    // Convert temperature to Celsius
    float temperatureCelsius = temperatureKelvin - 273.15;

    return temperatureCelsius;
}

void Temperature_reading() {
  // Calculate the resistance of the NTC thermistors
    for (int i = 0; i < 6; i++) {
        ntcResistance[i] = calculateNTCResistance(analogRead(THERMISTOR_PIN[i]), vRef, adcMax, rFixed);
    }

    // Convert the NTC resistance to temperature in Celsius
    for (int i = 0; i < 6; i++) {
        temperatureCelsius[i] = ntcResistanceToTemperature(ntcResistance[i], beta, r0, t0);
    }

    // Print the temperatures to the Serial Monitor
    for (int i = 0; i < 6; i++) {
        Serial.print("Temperature[");
        Serial.print(i);
        Serial.print("]: ");
        Serial.print(temperatureCelsius[i]);
        Serial.println(" °C");
    }
}
void processReceivedData() {
  // Serial.print("Received Dutycycle Values: ");
  // Serial.println(receivedData);

  // Convert the receivedData to a mutable character array
  char receivedCharArray[receivedData.length() + 1];
  receivedData.toCharArray(receivedCharArray, receivedData.length() + 1);

  // Split the received data into individual numbers
  int index = 0;
  char *token = strtok(receivedCharArray, ",");
  while (token != NULL && index < 4) {
    float value = atof(token);
    if (index == 0) {
      SOC = value;
    } else if (index == 1) {
      VBatt_ISNS = value;
    } else if (index == 2) {
      HSFAN_ISNS = value;
    } else if (index == 3) {
      CSFAN_ISNS = value;
    }
    index++;
    token = strtok(NULL, ",");
  }
  myFile.println("Received Values:");
  myFile.print("SOC: ");
  myFile.println(SOC);
  myFile.print("VBatt_ISNS: ");
  myFile.println(VBatt_ISNS);
  myFile.print("HSFAN_ISNS: ");
  myFile.println(HSFAN_ISNS);
  myFile.print("CSFAN_ISNS: ");
  myFile.println(CSFAN_ISNS);
  //Print received ADC values
  Serial.println("Received Values:");
  Serial.print("SOC: ");
  Serial.println(SOC);
  Serial.print("VBatt_ISNS: ");
  Serial.println(VBatt_ISNS);
  Serial.print("HSFAN_ISNS: ");
  Serial.println(HSFAN_ISNS);
  Serial.print("CSFAN_ISNS: ");
  Serial.println(CSFAN_ISNS);

  // Clear the receivedData for the next reading
  receivedData = "";
}

void PI_ALGO() {

  if(HSFAN_Dutycycle<4095)
  {
  // Step 1: Measure the temperature
  float currentTemperature = temperatureCelsius[2];  // Implement this function based on your sensor
  // Step 2: Calculate the error
  float error =currentTemperature - setpoint ;

  // Step 3: Compute the Proportional term
  float P_out = Kp * error;

  // Step 4: Compute the Integral term
  integral += error * dt;
  float I_out = Ki * integral;

  // Step 5: Calculate the total output (PWM duty cycle)
  HSFAN_Dutycycle = P_out + I_out;

  // Step 6: Clamp the PWM duty cycle between 0 and 4095
  if (HSFAN_Dutycycle > 4095.0) {
    HSFAN_Dutycycle = 4095.0;
  } else if (HSFAN_Dutycycle < 0.0) {
    HSFAN_Dutycycle = 0.0;
  }
  
Serial.print("Error: ");
Serial.println(error);
Serial.print("P_out: ");
Serial.println(P_out);
Serial.print("I_out: ");
Serial.println(I_out);
// Serial.print("PWM Duty Cycle: ");
// Serial.println(pwmDutyCycle);
  }
SerialPort.println(HSFAN_Dutycycle);
}


void loop() {

  Temperature_reading();
  // Open the file for appending
  myFile = SD.open("/temperature1_log.txt", FILE_APPEND);
  if (myFile) {
    // Get timestamp (milliseconds since program started)
    unsigned long timestamp = millis();

    // Log data with timestamp
    myFile.print(timestamp);
    myFile.println(", ");
    // Print to Serial Monitor for debugging
    Serial.print("Timestamp: ");
    Serial.println(timestamp);
    // Check if data is available
    while (SerialPort.available()) {
      char incomingChar = SerialPort.read();
      if (incomingChar == '\n') {
        // End of the string, process the received data
        processReceivedData();
        digitalWrite(LED_PIN, HIGH);
      } else {
        // Append the incoming character to the receivedData
        receivedData += incomingChar;
      }
    }
    // Print the temperatures to the SD Card
    for (int i = 0; i < 6; i++) {
        myFile.print("Temperature_");
        myFile.print(i);
        myFile.print(temperatureCelsius[i]);
        myFile.println(" °C");
    }
    PI_ALGO();
    Serial.print("HSFAN_Dutycycle");
    Serial.println(HSFAN_Dutycycle);
    // delay(10000);
    myFile.close();
  } else {
    Serial.println("Error opening file for appending");
    // digitalWrite(LED_PIN,HIGH);
  }

  // Delay before the next reading (e.g., 10 seconds)
  delay(10000);  // 10 seconds
  // Close the file
}
