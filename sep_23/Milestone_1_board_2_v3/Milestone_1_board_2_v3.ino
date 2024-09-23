#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <Arduino.h>
#include <HardwareSerial.h>
#include <Adafruit_NeoPixel.h>


#define SD_MOSI 41
#define SD_MISO 40
#define SD_SCLK 39
#define SD_CS 42
#define PIN 7        // Pin connected to the data input of the WS2812B
#define NUMPIXELS 6  // Number of LEDs in your strip
HardwareSerial SerialPort(2);
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
const gpio_num_t RX_PIN = GPIO_NUM_18;
const gpio_num_t TX_PIN = GPIO_NUM_17;
const gpio_num_t THERMISTOR_PIN_1 = GPIO_NUM_3;   //flask temperature_1
const gpio_num_t THERMISTOR_PIN_2 = GPIO_NUM_9;   // Heat sink temperature
const gpio_num_t THERMISTOR_PIN_3 = GPIO_NUM_10;  //Cold sink temperature
const gpio_num_t THERMISTOR_PIN_4 = GPIO_NUM_11;
const gpio_num_t THERMISTOR_PIN_5 = GPIO_NUM_12;
const gpio_num_t THERMISTOR_PIN_AMB = GPIO_NUM_8;  //onboard ambient temperature
const gpio_num_t LED_PIN = GPIO_NUM_21;
const int THERMISTOR_PIN[6] = { THERMISTOR_PIN_AMB, THERMISTOR_PIN_1, THERMISTOR_PIN_2, THERMISTOR_PIN_3, THERMISTOR_PIN_4, THERMISTOR_PIN_5 };
// Constants for the NTC thermistor
const float vRef = 2.5;        // Reference voltage
const uint16_t adcMax = 4095;  // Max ADC value (12-bit ADC)
const float rFixed = 10000.0;  // Fixed resistor value in ohms (10k)
const float beta = 3950.0;     // Beta parameter of the NTC thermistor
const float r0 = 10000.0;      // Resistance at reference temperature (10k ohms)
const float t0 = 285.15;       // Reference temperature in Kelvin (4°C)
float ntcResistance[6];
float temperatureCelsius[6];
bool deviceactive = true;


String receivedData = "";
int SOC, fault;
float CSFAN_ISNS, HSFAN_ISNS, VBatt_ISNS, Batt_voltage;
float HSFAN_Dutycycle, CSFAN_Dutycycle, TEC_Dutycycle;
int PI_STATE = 1;
int HS_FAN_dutyCycle = 0;
int TEC_dutyCycle = 0;


File myFile;


// Define the PWM channel, frequency, and resolution
const int pwmChannel = 0;
const int pwmFreq = 5000;      // 5kHz frequency
const int pwmResolution = 12;  // 12-bit resolution


// Define the reference temperature
const float referenceTemp = 3.0;
const float MAX_TEMP = 10.0;

unsigned long timestamp;

float Flask_avg_temp;


void setup() {
  Serial.begin(115200);
  SerialPort.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);  // RX on GPIO18, TX on GPIO17
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  pixels.begin();  // Initialize the NeoPixel library.
  Serial.println("Setup start");

  // Initialize SPI for SD card
  SPI.begin(SD_SCLK, SD_MISO, SD_MOSI, SD_CS);

  if (!SD.begin(SD_CS)) {
    Serial.println("SD Card MOUNT FAIL");
    return;
  } else {
    Serial.println("SD Card MOUNT SUCCESS");
    Serial.println("");

    // Open file to check its size
    myFile = SD.open("/temperature_log.csv", FILE_APPEND);

    if (myFile) {
      // If the file is empty, print the headings
      if (myFile.size() == 0) {
        myFile.print("Timestamp: ");
        myFile.print(", ");
        myFile.print("fault: ");
        myFile.print(", ");
        myFile.print("SOC: ");
        myFile.print(", ");
        myFile.print("Batt_voltage: ");
        myFile.print(", ");
        myFile.print("VBatt_ISNS: ");
        myFile.print(", ");
        myFile.print("HSFAN_ISNS: ");
        myFile.print(", ");
        myFile.print("CSFAN_ISNS: ");
        myFile.print(", ");
        myFile.print(" PCB Temperature  ");
        myFile.print(", ");
        myFile.print("Flask top Temperature ");
        myFile.print(", ");
        myFile.print("Heat sink Temperature ");
        myFile.print(", ");
        myFile.print("Cold sink Temperature ");
        myFile.print(", ");
        myFile.print("Flask down Temperature ");
        myFile.print(", ");
        myFile.println("Flask average Temperature ");

        digitalWrite(LED_PIN, HIGH);  // Blink LED to indicate success
        delay(100);
        digitalWrite(LED_PIN, LOW);
        delay(100);
      }
      myFile.close();  // Close the file after checking
    } else {
      Serial.println("Error opening file for writing");
    }
    Serial.println("Setup complete");
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
  const int numSamples = 5;
  float adcSum, ntcResistanceSum;

  // Calculate the resistance of the NTC thermistors, taking 5 samples for each
  for (int i = 0; i < 6; i++) {
    adcSum = 0;  // Reset the sum for each thermistor

    // Take 5 samples of the analog value and average them
    for (int j = 0; j < numSamples; j++) {
      adcSum += analogRead(THERMISTOR_PIN[i]);
      delay(10);  // Short delay between each reading
    }

    // Calculate the average ADC value
    float adcAverage = adcSum / numSamples;


    // Convert average ADC value to resistance
    ntcResistance[i] = calculateNTCResistance(adcAverage, vRef, adcMax, rFixed);
  }


  // Convert the NTC resistance to temperature in Celsius for each thermistor
  for (int i = 0; i < 6; i++) {
    temperatureCelsius[i] = ntcResistanceToTemperature(ntcResistance[i], beta, r0, t0);
  }
  Flask_avg_temp=(temperatureCelsius[1] + temperatureCelsius[4])/2;
}
void processReceivedData() {


  // Convert the receivedData to a mutable character array
  char receivedCharArray[receivedData.length() + 1];
  receivedData.toCharArray(receivedCharArray, receivedData.length() + 1);


  // Split the received data into individual numbers
  int index = 0;
  char *token = strtok(receivedCharArray, ",");
  while (token != NULL && index < 6) {
    float value = atof(token);
    if (index == 0) {
      fault = value;
    } else if (index == 1) {
      SOC = value;
    } else if (index == 2) {
      Batt_voltage = value;
    } else if (index == 3) {
      VBatt_ISNS = value;
    } else if (index == 4) {
      HSFAN_ISNS = value;
    } else if (index == 5) {
      CSFAN_ISNS = value;
    }
    index++;
    token = strtok(NULL, ",");
  }
  // myFile.println("Received Values:");
  myFile.print(timestamp);
  myFile.print(", ");
  myFile.print(fault);
  myFile.print(", ");
  myFile.print(SOC);
  myFile.print(", ");
  myFile.print(Batt_voltage);
  myFile.print(", ");
  myFile.print(VBatt_ISNS);
  myFile.print(", ");
  myFile.print(HSFAN_ISNS);
  myFile.print(", ");
  myFile.print(CSFAN_ISNS);
  myFile.print(", ");
  myFile.print(temperatureCelsius[0]);
  myFile.print(", ");
  myFile.print(temperatureCelsius[1]);
  myFile.print(", ");
  myFile.print(temperatureCelsius[2]);
  myFile.print(", ");
  myFile.print(temperatureCelsius[3]);
  myFile.print(", ");
  myFile.print(temperatureCelsius[4]);
  myFile.print(", ");
  myFile.println(Flask_avg_temp);

  //Print received ADC values
  Serial.println("Received Values:");
  Serial.print("fault: ");
  Serial.println(fault);
  Serial.print("Batt_voltage: ");
  Serial.println(Batt_voltage);
  Serial.print("SOC: ");
  Serial.println(SOC);
  Serial.print("VBatt_ISNS: ");
  Serial.println(VBatt_ISNS);
  Serial.print("HSFAN_ISNS: ");
  Serial.println(HSFAN_ISNS);
  Serial.print("CSFAN_ISNS: ");
  Serial.println(CSFAN_ISNS);
  Serial.print("PCB Temperature : ");
  Serial.print(temperatureCelsius[0]);
  Serial.println(" °C");
  Serial.print("Flask Top Temperature : ");
  Serial.print(temperatureCelsius[1]);
  Serial.println(" °C");
  Serial.print("Hot Sink Temperature : ");
  Serial.print(temperatureCelsius[2]);
  Serial.println(" °C");
  Serial.print("Cold Sink Temperature : ");
  Serial.print(temperatureCelsius[3]);
  Serial.println(" °C");
  Serial.print("Flask bottom Temperature : ");
  Serial.print(temperatureCelsius[4]);
  Serial.println(" °C");
  Serial.print("Flask average Temperature : ");
  Serial.print(Flask_avg_temp);
  Serial.println(" °C");






  // Clear the receivedData for the next reading
  receivedData = "";
}


float map_algo(float x, float in_min, float in_max, float out_min, float out_max) {
  const float run = in_max - in_min;
  if (run == 0) {
    log_e("map(): Invalid input range, min == max");
    return -1;  // AVR returns -1, SAM returns 0
  }
  const float rise = out_max - out_min;
  const float delta = x - in_min;
  return (delta * rise) / run + out_min;
}
void PI_ALGO() {

  float temperature = Flask_avg_temp;

  if (temperature <= 5.1 && PI_STATE == 1) {
    // Calculate the duty cycle proportionally based on the temperature
    // For example, if the temperature is 10°C, the duty cycle might be 50%
    if (temperature >= 3 && temperature <= 3.2) {
      PI_STATE = 0;
    }
    HS_FAN_dutyCycle = map_algo(temperature, 2, MAX_TEMP, 0, 4095);           // Scale 12-bit resolution (0-4095)
    TEC_dutyCycle = map_algo(temperature, referenceTemp, MAX_TEMP, 0, 4095);  // Scale 12-bit resolution (0-4095)
    if (HS_FAN_dutyCycle > 4095) {
      HS_FAN_dutyCycle = 4095;  // Cap the duty cycle at 100%
    }
    if (TEC_dutyCycle > 4095) {
      TEC_dutyCycle = 4095;  // Cap the duty cycle at 100%
    }


  } else if (temperature <= 5 && PI_STATE == 0) {
    HS_FAN_dutyCycle = map(temperature, 2, MAX_TEMP, 0, 4095);  // Scale 12-bit resolution (0-4095)
    TEC_dutyCycle = 0;
  } else {
    if (temperature >= 5.1) {
      PI_STATE = 1;
    }
    HS_FAN_dutyCycle = 4095;
    TEC_dutyCycle = 4095;
  }


  HSFAN_Dutycycle = HS_FAN_dutyCycle;
  TEC_Dutycycle = TEC_dutyCycle;
  SerialPort.print(HSFAN_Dutycycle);
  SerialPort.print(",");
  SerialPort.println(TEC_Dutycycle);
  Serial.print("HSFAN_Dutycycle");
  Serial.println(HSFAN_Dutycycle);
  Serial.print("tec_dutycycle: ");
  Serial.println(TEC_Dutycycle);
  // Serial.println(temperature);
  // Serial.println(PI_STATE);
}


void led_indication() {
  if (fault == 1) {  //soc is less than equal to 10%
    for (int i = 0; i < NUMPIXELS; i++) {
      pixels.setPixelColor(i, pixels.Color(255, 0, 0));  // White (Red + Green + Blue)
    }
    pixels.show();  // Update the LEDs
    delay(1000);    // Wait for 1 second




    // Turn all LEDs off
    for (int i = 0; i < NUMPIXELS; i++) {
      pixels.setPixelColor(i, pixels.Color(0, 0, 0));  // Off
    }
    pixels.show();  // Update the LEDs
    delay(1000);    // Wait for 1 second
  }




  if (fault == 2) {  //HS fan error
    pixels.setPixelColor(0, pixels.Color(255, 255, 0));
    pixels.show();  // Update the LEDs
  }




  if (fault == 3)  //CS fan error
  {
    pixels.setPixelColor(5, pixels.Color(255, 255, 255));
    pixels.show();  // Update the LEDs
  }




  if (fault == 4)  //led open for  more than 20 sec
  {
    for (int i = NUMPIXELS; i > 0; i--) {
      pixels.setPixelColor(i, pixels.Color(255, 255, 255));
    }
    pixels.show();  // Update the LEDs
  }
  if (fault == 6) {
    pixels.setPixelColor(0, pixels.Color(255, 255, 255));
    pixels.show();
  }
}


void loop() {


  Temperature_reading();
  // Open the file for appending
  myFile = SD.open("/temperature_log.csv", FILE_APPEND);
  if (myFile) {
    // Get timestamp (milliseconds since program started)
    timestamp = millis();

    // Print to Serial Monitor for debugging
    Serial.print("Timestamp: ");
    Serial.println(timestamp);
    // Check if data is available
    while (SerialPort.available()) {
      char incomingChar = SerialPort.read();
      if (incomingChar == '\n') {
        // End of the string, process the received data
        processReceivedData();




      } else {
        // Append the incoming character to the receivedData
        receivedData += incomingChar;
      }
    }


    if (SOC >= 10) {
      PI_ALGO();
    }
    led_indication();

    myFile.close();
  } else {
    Serial.println("Error opening file for appending");
    // digitalWrite(LED_PIN,HIGH);
  }


  // Delay before the next reading (e.g., 10 seconds)
  delay(2000);  // 10 seconds
  // Close the file
}
