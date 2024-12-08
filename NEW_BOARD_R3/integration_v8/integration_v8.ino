#include <HardwareSerial.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <Arduino.h>
#include "driver/ledc.h"  // For ESP32 PWM control
#include <Adafruit_NeoPixel.h>

#define SD_MOSI 41
#define SD_MISO 40
#define SD_SCLK 39
#define SD_CS 42
// Define pin for WS2812B
#define WS2812B_PIN 16
#define NUM_LEDS 1
// Create an instance for WS2812B
Adafruit_NeoPixel ws2812b(NUM_LEDS, WS2812B_PIN, NEO_GRB + NEO_KHZ800);

// Define pins for the 74HC595
#define SER_PIN 13                 // SER - Data input
#define SRCLK_PIN 44               // SRCLK - Clock pin
#define RCLK_PIN 43                // RCLK - Latch pin
#define OPEN_CIRCUIT_VOLTAGE 0.03  // Voltage near zero indicates an open circuit
// Create a HardwareSerial object for UART communication with EC200U
HardwareSerial ec200uSerial(1);

const gpio_num_t LED_PIN = GPIO_NUM_48;       // Pin no 48
const gpio_num_t BUTTON_SW = GPIO_NUM_10;     // Pin no 10
const gpio_num_t BUZZER_PIN = GPIO_NUM_5;     // Pin no 5
const gpio_num_t BATT_VSNS = GPIO_NUM_4;      // Pin no 4
const gpio_num_t HS_FAN = GPIO_NUM_7;         // Pin no 7
const gpio_num_t CS_FAN = GPIO_NUM_21;        // Pin no 21
const gpio_num_t LID = GPIO_NUM_46;           // Pin no 46
const gpio_num_t HSFAN_ISNS = GPIO_NUM_15;    // Pin no 15
const gpio_num_t CSFAN_ISNS = GPIO_NUM_14;    // Pin no 14
const gpio_num_t TEC_ISNS = GPIO_NUM_6;       // Pin no 6
const gpio_num_t TEC_COLD_CTRL = GPIO_NUM_1;  // Pin no 1
const gpio_num_t RX_PIN = GPIO_NUM_18;
const gpio_num_t TX_PIN = GPIO_NUM_17;
const gpio_num_t COLDSINK_TEMP = GPIO_NUM_3;  //flask temperature_1
const gpio_num_t HEATSINK_TEMP = GPIO_NUM_9;  // Heat sink temperature
const gpio_num_t FLASKTOP_TEMP = GPIO_NUM_11;
const gpio_num_t FLASKBOTTOM_TEMP = GPIO_NUM_12;
const gpio_num_t PCB_TEMP = GPIO_NUM_8;  //onboard ambient temperature
const gpio_num_t CHARGER_DOCK = GPIO_NUM_45;
const int HS_FAN_Channel = 1;  // Channel 1
const int TEC_Channel = 3;
const int CS_FAN_Channel = 0;  // Channel 0
const int resolution = 12;     // Maximum resolution supported by ESP32 LEDC is 12 bits
int HS_dutyCycle = 3900;       // Hot fan Duty cycle
int CS_dutyCycle = 3900;       // Cold fan Duty cycle
int TEC_dutyCycle = 1000;
const int freq = 5000;  // 5kHz frequency
const float R1 = 10520;
const float R2 = 1000;
const int THERMISTOR_PIN[5] = { PCB_TEMP, COLDSINK_TEMP, HEATSINK_TEMP, FLASKTOP_TEMP, FLASKBOTTOM_TEMP };
int LID_Count = 0;
unsigned long lidLowStartTime = 0;
bool lidWasLow = true;
String receivedData = "";
int HS_count;
int CS_count;
int fault;
int soc;
float batteryVoltage;
const int BUZZER_channel = 4;  // PWM channel (0-15 on ESP32)
const int freq_b = 5000;
volatile bool buttonPressed = false;  // Flag to indicate button press
bool deviceActive = false;            // Flag to indicate if device is on
int SOC_CHECK = 1;
bool stopSOCCheck = false;
int SOC_SAMP = 0;
int SOC_SAMP_1 = 0;
String latitude = "";
String longitude = "";
// Threshold voltage for open circuit detection (in volts)

bool conditionMet[5] = { false, false, false, false, false };  // Track if condition has been executed
int led_count = 0;

// Constants
const float ADC_REF_VOLTAGE = 3.3;  // Reference voltage in volts
const int ADC_RESOLUTION = 4095;    // 12-bit ADC
const float HSFAN_RSENSE = 0.05;    // Hot-side Rsense value in ohms
const float TEC_COLD_CTRL_RSENSE = 0.005;
const float CSFAN_RSENSE = 0.3;  // Cold-side Rsense value in ohms
const float INA_GAIN = 100.0;    // INA181A3 Gain
const int PWM_FREQ = 5000;       // 5 kHz PWM frequency
const int PWM_RESOLUTION = 12;   // 12-bit PWM resolution (0-4095)
float TEC_ISNS_current;
float HSFAN_ISNS_current;
float CSFAN_ISNS_current;

const float vRef = 2.5;        // Reference voltage
const uint16_t adcMax = 4095;  // Max ADC value (12-bit ADC)
const float rFixed = 10000.0;  // Fixed resistor value in ohms (10k)
const float beta = 3000.0;     // Beta parameter of the NTC thermistor
const float r0 = 10000.0;      // Resistance at reference temperature (10k ohms)
const float t0 = 289.15;       // Reference temperature in Kelvin (4°C)
uint16_t adcValue[5];
float ntcResistance[5];
float temperatureCelsius[5];
float FinalFlaskAvgTemp = 4.5;

// Define the reference temperature
const float referenceTemp = 1.0;
const float MAX_TEMP = 4.0;

unsigned long timestamp;

float Flask_avg_temp;
float HSFAN_Dutycycle, CSFAN_Dutycycle, TEC_Dutycycle;
int PI_STATE = 1;
int HS_FAN_dutyCycle = 0;
// int TEC_dutyCycle = 0;

const float Vref = 3.3;     // Reference voltage for ADC (3.3V)
const int ADC_Max = 4095;   // 12-bit ADC max value
const float Gain = 10.0;    // Assume a gain of 10 for the op-amp circuit (adjust as needed)
const float Rsense = 1.01;  // Current sense resistor value in ohms (adjust based on your design)

// LED pattern to display (5 LEDs, MSB unused)
byte ledPattern[] = {
  0b00000000,  // All OFF
  0b00011111,  // All ON
  0b00011110,  // LED1 OFF
  0b00011100,  // LED2 OFF
  0b00011000,  // LED3 OFF
  0b00010000,  // LED4 OFF
};

int indici = 0;
File myFile;
long Locationtimer = 0;

// Function to be called when the button is pressed
void buttonISR() {
  buttonPressed = true;  // Set the flag indicating button press
}

hw_timer_t* timer = NULL;          // Timer for ADC synchronization
volatile bool sampleFlag = false;  // Flag to trigger ADC sampling

void IRAM_ATTR onTimer() {
  sampleFlag = true;  // Set flag to perform ADC sampling in the loop
}

void setup() {
  Serial.begin(115200);                                    // Initialize Serial communication
  ec200uSerial.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);  // RX on GPIO18, TX on GPIO17
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);  // Ensure LED is initially off
  pinMode(BUTTON_SW, INPUT_PULLUP);
  pinMode(CHARGER_DOCK, INPUT);
  // Attach interrupt for button press (rising edge, i.e., button release)
  attachInterrupt(digitalPinToInterrupt(BUTTON_SW), buttonISR, RISING);
  analogReadResolution(12);
  // Setup PWM
  ledcSetup(BUZZER_channel, freq_b, resolution);
  // Attach the buzzer pin to the PWM channel
  ledcAttachPin(BUZZER_PIN, BUZZER_channel);
  // Initialize PWM channels
  ledcSetup(CS_FAN_Channel, PWM_FREQ, PWM_RESOLUTION);  // Channel 0 for HSFAN
  ledcSetup(HS_FAN_Channel, PWM_FREQ, PWM_RESOLUTION);  // Channel 1 for CSFAN

  // Attach PWM pins
  ledcAttachPin(HS_FAN, HS_FAN_Channel);
  ledcAttachPin(CS_FAN, CS_FAN_Channel);

  // Set up timer for ADC sampling synchronization
  timer = timerBegin(0, 80, true);  // Timer 0, prescaler 80 (1 µs ticks)
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 200, true);  // Trigger every 200 µs (adjust as needed)
  timerAlarmEnable(timer);

  ledcSetup(TEC_Channel, freq, resolution);   // configure CS_FAN PWM functionality
  ledcAttachPin(TEC_COLD_CTRL, TEC_Channel);  // attach the channel
  pinMode(LID, INPUT);
  pinMode(SER_PIN, OUTPUT);
  pinMode(SRCLK_PIN, OUTPUT);
  pinMode(RCLK_PIN, OUTPUT);

  // Initialize pins to LOW
  digitalWrite(SER_PIN, LOW);
  digitalWrite(SRCLK_PIN, LOW);
  digitalWrite(RCLK_PIN, LOW);

  // ledcWrite(TEC_Channel, TEC_dutyCycle);
  //     ledcWrite(HS_FAN_Channel, HS_dutyCycle);  // HSFAN 50%
  //     ledcWrite(CS_FAN_Channel, CS_dutyCycle);  // CSFAN 50%

  // Initialize WS2812B
  ws2812b.begin();
  ws2812b.show();  // Turn off all LEDs
  // // Allow time for EC200U to initialize
  // delay(3000);
  // // Enable GPS
  // sendCommand("AT+QGPS=1");
  // delay(2000);
  // // Query GPS location
  // sendCommand("AT+QGPSLOC?");
  // Initialize SPI for SD card
  SPI.begin(SD_SCLK, SD_MISO, SD_MOSI, SD_CS);

  if (!SD.begin(SD_CS)) {
    Serial.println("SD Card MOUNT FAIL");
    return;
  } else {
    Serial.println("SD Card MOUNT SUCCESS");
    Serial.println("");

    // Open file to check its size
    myFile = SD.open("/data_logging_01.csv", FILE_APPEND);

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
        myFile.print("TEC_ISNS: ");
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

void Buzzer_LID() {
  // Turn on the buzzer (50% duty cycle)
  ledcWrite(BUZZER_channel, 2048);  // 2048 is 50% of 4095 (for 12-bit resolution)
  // Beep sound for 2 seconds
  delay(200);
  // Turn off the buzzer
  ledcWrite(BUZZER_channel, 0);
  // Buzzer off for 2 seconds
  delay(100);
  ledcWrite(BUZZER_channel, 2048);  // 2048 is 50% of 4095 (for 12-bit resolution)
  // Beep sound for 2 seconds
  delay(400);
  // Turn off the buzzer
  ledcWrite(BUZZER_channel, 0);
}

void Buzzer_LID_OFF() {
  // Turn on the buzzer (50% duty cycle)
  ledcWrite(BUZZER_channel, 2048);  // 2048 is 50% of 4095 (for 12-bit resolution)
  // Beep sound for 2 seconds
  delay(500);
  // Turn off the buzzer
  ledcWrite(BUZZER_channel, 0);
}

float readBatteryVoltage() {

  // int BATT_VSNS_adcValue = analogRead(BATT_VSNS);  // Read ADC value
  float adc_sample[10];
  float adc_volt_samp = 0;
  for (int i = 0; i < 10; i++) {
    adc_sample[i] = analogRead(BATT_VSNS);
    adc_volt_samp = adc_volt_samp + adc_sample[i];
    delay(10);
  }
  int BATT_VSNS_adcValue = adc_volt_samp / 10;

  // Convert ADC value to voltage (assuming Vref=3.3V)
  //Serial.println(BATT_VSNS_adcValue);
  float BATT_VSNS_voltage = BATT_VSNS_adcValue * (3.3 / 4095.0);
  float voltage = BATT_VSNS_voltage / (R2 / (R1 + R2));
  return voltage;
}

int sample_soc(float voltage) {
  if (voltage <= 12.2) {
    return 0;
  } else if (voltage >= 16.8) {
    return 100;
  } else {
    return (int)((voltage - 12.2) / (16.8 - 12.2) * 99) + 1;
  }
}

int calculateSOC(float voltage) {
  int soc_sam[10];
  int average = 0;
  for (int i = 0; i < 10; i++) {
    soc_sam[i] = sample_soc(voltage);
    average += soc_sam[i];
    delay(10);
  }
  return average / 10;
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
  for (int i = 0; i < 5; i++) {
    adcValue[i] = analogRead(THERMISTOR_PIN[i]);
  }

  // Calculate the resistance of the NTC thermistors
  for (int i = 0; i < 5; i++) {
    ntcResistance[i] = calculateNTCResistance(adcValue[i], vRef, adcMax, rFixed);
  }

  // Convert the NTC resistance to temperature in Celsius
  for (int i = 0; i < 5; i++) {
    temperatureCelsius[i] = ntcResistanceToTemperature(ntcResistance[i], beta, r0, t0);
  }

  // Serial.println(temperatureCelsius[3]);
  // Serial.println(temperatureCelsius[4]);
  Flask_avg_temp = (temperatureCelsius[3] + temperatureCelsius[4]) / 2;
  if (Flask_avg_temp <= FinalFlaskAvgTemp) {
    ws2812b.setPixelColor(0, ws2812b.Color(0, 255, 0));  // Green
    ws2812b.show();
  } else if (Flask_avg_temp > FinalFlaskAvgTemp) {
    ws2812b.setPixelColor(0, ws2812b.Color(255, 0, 0));  // Red
    ws2812b.show();
  }
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

  if (temperature <= 8.1 && PI_STATE == 1) {
    // Calculate the duty cycle proportionally based on the temperature
    if (temperature >= 2 && temperature <= 2.2) {
      PI_STATE = 0;
    }
    HS_dutyCycle = map_algo(temperature, 0, 5, 0, 5000);               // Scale 12-bit resolution (0-4095)
    TEC_dutyCycle = map_algo(temperature, referenceTemp, 6, 0, 5000);  // Scale 12-bit resolution (0-4095)
    if (HS_dutyCycle > 3900) {
      HS_dutyCycle = 3900;  // Cap the duty cycle at 100%
    }
    if (TEC_dutyCycle > 3900) {
      TEC_dutyCycle = 3900;  // Cap the duty cycle at 100%
    }

  } else if (temperature <= 8 && PI_STATE == 0) {
    HS_dutyCycle = map(temperature, 2, 30, 0, 4095);  // Scale 12-bit resolution (0-4095)
    TEC_dutyCycle = 0;
  } else {
    if (temperature >= 8.1) {
      PI_STATE = 1;
    }
    HS_dutyCycle = 3900;
    TEC_dutyCycle = 3900;
  }

  if (HS_dutyCycle < 0) {
    HS_dutyCycle = 0;
  }
  if (TEC_dutyCycle < 0) {
    TEC_dutyCycle = 0;
  }

  Serial.print("HS_dutyCycle");
  Serial.println(HS_dutyCycle);
  Serial.print("TEC_dutyCycle: ");
  Serial.println(TEC_dutyCycle);
}

// Function to read voltage from INA181 output
float readVoltage(int analogPin) {
  int adcValue = analogRead(analogPin);  // Read ADC value
  return adcValue * (3.3 / 4095.0);      // Convert to voltage (12-bit ADC and 3.3V reference)
}


bool HS_Fan_Check() {
  ledcWrite(HS_FAN_Channel, HS_dutyCycle);
  Serial.print("HS_dutyCycle: ");
  Serial.println(HS_dutyCycle);
  float hsfan_voltage = readVoltage(HSFAN_ISNS);
  // Serial.println(hsfan_voltage);
  // Detect HS Fan open circuit fault
  if (hsfan_voltage < OPEN_CIRCUIT_VOLTAGE) {
    fault = 2;
    Serial.println("HS-Fail");
    return 0;
  } else {
    fault = 0;
    return 1;
  }
}
bool CS_Fan_Check() {
  ledcWrite(CS_FAN_Channel, CS_dutyCycle);
  Serial.print("CS_dutyCycle: ");
  Serial.println(CS_dutyCycle);
  float csfan_voltage = readVoltage(CSFAN_ISNS);
  // Detect CS Fan open circuit fault
  if (csfan_voltage < OPEN_CIRCUIT_VOLTAGE) {
    fault = 3;
    Serial.println("CS-Fail");
    return 0;
  } else {
    fault = 0;
    return 1;
  }
}

void HS_Fan() {
  ledcWrite(HS_FAN_Channel, HS_dutyCycle);
  Serial.print("HS_dutyCycle: ");
  Serial.println(HS_dutyCycle);
}


void CS_Fan() {
  ledcWrite(CS_FAN_Channel, CS_dutyCycle);
  Serial.print("CS_dutyCycle: ");
  Serial.println(CS_dutyCycle);
}

void TEC_COLD() {
  ledcWrite(TEC_Channel, TEC_dutyCycle);
  Serial.print("TEC_dutyCycle: ");
  Serial.println(TEC_dutyCycle);
  // float tec_voltage = readVoltage(TEC_ISNS);
  // // Detect TEC cold ctrl open circuit fault
  // if (tec_voltage < OPEN_CIRCUIT_VOLTAGE) {
  //   fault = 4;
  //   Serial.println("TEC-Fail");
  //   return 0;
  // } else {
  //   fault = 0;
  //   return 1;
  // }
}



void Current_values() {
  if (sampleFlag) {
    sampleFlag = false;  // Reset flag

    // Read ADC for HSFAN
    int TEC_COLD_CTRL_AdcValue = analogRead(HSFAN_ISNS);  // Read ADC for HSFAN
    float TEC_COLD_CTRL_Voltage = TEC_COLD_CTRL_AdcValue * (ADC_REF_VOLTAGE / ADC_RESOLUTION);
    TEC_ISNS_current = TEC_COLD_CTRL_Voltage / (TEC_COLD_CTRL_RSENSE * 50);


    // Serial.print("TEC_COLD_CTRL Current: ");
    // Serial.print(TEC_ISNS, 3);  // 3 decimal places
    // Serial.println(" A");


    // Read ADC for HSFAN
    int hsfanAdcValue = analogRead(HSFAN_ISNS);  // Read ADC for HSFAN
    float hsfanVoltage = hsfanAdcValue * (ADC_REF_VOLTAGE / ADC_RESOLUTION);
    HSFAN_ISNS_current = hsfanVoltage / (HSFAN_RSENSE * INA_GAIN);


    // Serial.print("HSFAN Current: ");
    // Serial.print(HSFAN_ISNS, 3);  // 3 decimal places
    // Serial.println(" A");


    // Read ADC for CSFAN
    int csfanAdcValue = analogRead(CSFAN_ISNS);  // Read ADC for CSFAN
    float csfanVoltage = csfanAdcValue * (ADC_REF_VOLTAGE / ADC_RESOLUTION);
    CSFAN_ISNS_current = csfanVoltage / (CSFAN_RSENSE * INA_GAIN);


    // Serial.print("CSFAN Current: ");
    // Serial.print(CSFAN_ISNS, 3);  // 3 decimal places
    // Serial.println(" A");
  }
}

// float current_calc(int adc_pin) {
//   int adcValue = analogRead(adc_pin);           // Read ADC value
//   float voltage = (adcValue * Vref) / ADC_Max;  // Convert ADC value to voltage
//   // Calculate current (I = Vout / (Gain * Rsense))
//   float current = voltage / (Gain * Rsense);


//   Serial.print("Current: ");
//   Serial.print(current, 6);  // Print current with 6 decimal places
//   Serial.println(" A");
//   return current;
// }


// void Current_values() {


//   float HSFAN_ISNS_CURRENT = current_calc(HSFAN_ISNS);
//   Serial.print("HSFAN_ISNS_CURRENT : ");
//   Serial.print(HSFAN_ISNS_CURRENT, 6);
//   Serial.println("A");




//   float CSFAN_ISNS_CURRENT = current_calc(CSFAN_ISNS);
//   Serial.print("CSFAN_ISNS_CURRENT : ");
//   Serial.print(CSFAN_ISNS_CURRENT, 6);
//   Serial.println("A");




//   float TEC_ISNS_CURRENT = current_calc(TEC_ISNS);
//   Serial.print("TEC_ISNS_CURRENT : ");
//   Serial.print(TEC_ISNS_CURRENT, 6);
//   Serial.println("A");
// }


void soc_read() {
  batteryVoltage = readBatteryVoltage();
  soc = calculateSOC(batteryVoltage);
}


// Function to evaluate the SOC value and control LEDs
void evaluateSOC() {
  if (soc >= 80) {
    conditionMet[0] = true;
    executeLogic(6);  // Send 0x0 six times, then 0x1 six times
  } else if (soc >= 60 && soc < 80) {
    conditionMet[1] = true;
    executeLogic(5);  // Send 0x0 six times, then 0x1 five times
  } else if (soc >= 40 && soc < 60) {
    conditionMet[2] = true;
    executeLogic(4);  // Send 0x0 six times, then 0x1 four times
  } else if (soc >= 20 && soc < 40) {
    conditionMet[3] = true;
    executeLogic(3);  // Send 0x0 six times, then 0x1 three times
  } else if (soc >= 0 && soc < 20) {
    conditionMet[4] = true;
    executeLogic(2);  // Send 0x0 six times, then 0x1 two times
  }
}


void executeLogic(int onCount) {
  // Serial.println("Turning all LEDs OFF");
  for (int i = 0; i < 6; i++) {
    turnAllLEDsOff();
    delay(50);  // Delay for visual effect
  }

  // Serial.print("Turning LEDs ON for ");
  // Serial.print(onCount);
  // Serial.println(" times");
  for (int i = 0; i < onCount; i++) {
    turnAllLEDsOn();
    delay(50);  // Delay for visual effect
  }
}


// Function to reset the condition flags
void resetConditions() {
  for (int i = 0; i < 5; i++) {
    conditionMet[i] = false;  // Reset all conditions
  }
}

// Function to turn all LEDs ON
void turnAllLEDsOn() {
  shiftOut(SER_PIN, SRCLK_PIN, MSBFIRST, 0x1);  // Send 00000001 (One LED ON)
  latchData();
}

// Function to turn all LEDs OFF
void turnAllLEDsOff() {
  shiftOut(SER_PIN, SRCLK_PIN, MSBFIRST, 0x0);  // Send 00000000 (All LEDs OFF)
  latchData();
}

// Function to latch data and display it on LEDs
void latchData() {
  digitalWrite(RCLK_PIN, LOW);
  digitalWrite(RCLK_PIN, HIGH);
}


void LED_INDI() {
  led_count++;
  if (led_count == 1) {
    if (soc < 0 || soc > 100) {
      Serial.println("Invalid SOC value. Please enter a value between 0 and 100.");
    } else {
      resetConditions();  // Reset condition flags
      evaluateSOC();      // Evaluate the SOC value and control LEDs
    }
  } else if (led_count == 100) {
    if (soc < 0 || soc > 100) {
      Serial.println("Invalid SOC value. Please enter a value between 0 and 100.");
    } else {
      resetConditions();  // Reset condition flags
      evaluateSOC();      // Evaluate the SOC value and control LEDs
    }
  } else if (led_count >= 200) {
    led_count = 0;
  }
}




void wakeup_process() {
  if (buttonPressed) {
    delay(500);                               // Debounce delay
    Serial.println("Loop: Button pressed.");  // Print button press message

    // Toggle device state
    deviceActive = !deviceActive;

    if (deviceActive) {
      Serial.println("Loop: Device is active...");  // Print waking up message
      digitalWrite(LED_PIN, HIGH);
      soc_read();
      Buzzer_LID();
      ledcWrite(TEC_Channel, TEC_dutyCycle);
      ledcWrite(HS_FAN_Channel, HS_dutyCycle);  // HSFAN 50%
      ledcWrite(CS_FAN_Channel, CS_dutyCycle);  // CSFAN 50%
      if (soc <= 10) {
        SOC_CHECK == 0;
        fault = 1;
        stopSOCCheck = true;
      }
      delay(2000);

    } else {
      Serial.println("Loop: Device is inactive...");  // Print going to sleep message
      Buzzer_LID_OFF();
      while (!deviceActive) {
        buttonPressed = false;
        ledcWrite(CS_FAN_Channel, 0);
        ledcWrite(HS_FAN_Channel, 0);
        ledcWrite(TEC_Channel, 0);
        ledcWrite(BUZZER_channel, 0);
        ws2812b.setPixelColor(0, ws2812b.Color(0, 0, 0));  // Red
        ws2812b.show();
        digitalWrite(LED_PIN, LOW);
        soc_read();
        LED_INDI();
        if (soc <= 10) {
          fault = 1;
        }
        Serial.print("Timestamp: ");
        Serial.println(timestamp);
        Serial.print("TEC_dutyCycle: ");
        Serial.println(TEC_dutyCycle);
        Serial.print(fault);
        Serial.print(",");
        Serial.print(soc);
        Serial.print(",");
        Serial.println(batteryVoltage);
        if (buttonPressed) {
          delay(100);
          buttonPressed = false;
          deviceActive = true;
          Serial.println("device is reactive:");
          Buzzer_LID();
          if (soc <= 10) {
            SOC_CHECK == 0;
            fault = 1;
            stopSOCCheck = true;
          }
          break;
          // esp_restart();
        }
        timestamp = millis();
        myFile = SD.open("/data_logging_01.csv", FILE_APPEND);
        if (myFile) {
          myFile.print(timestamp);
          myFile.print(",");
          myFile.print(fault);
          myFile.print(",");
          myFile.print(soc);
          myFile.print(",");
          myFile.println(batteryVoltage);
          myFile.close();
        } else {
          Serial.println("Error opening file for appending");
        }
        // sendToShiftRegister(ledPattern[indici]);

        if (buttonPressed) {
          delay(100);
          buttonPressed = false;
          deviceActive = true;
          Serial.println("device is reactive:");
          Buzzer_LID();
          if (soc <= 10) {
            SOC_CHECK == 0;
            fault = 1;
            stopSOCCheck = true;
          }
          break;
        }
        // buttonPressed = false;
        delay(200);
      }
    }
    // Reset the button press flag
    buttonPressed = false;
  }
}

void loop() {
  // Open the file for appending
  myFile = SD.open("/data_logging_01.csv", FILE_APPEND);
  // if (myFile) {
  // Get timestamp (milliseconds since program started)
  timestamp = millis();

  // Print to Serial Monitor for debugging
  // Serial.print("Timestamp: ");
  // Serial.println(timestamp);

  if (!digitalRead(CHARGER_DOCK)) {
    wakeup_process();
    if (deviceActive) {
      if (SOC_CHECK && !stopSOCCheck) {
        if (HS_Fan_Check()) {
          if (CS_Fan_Check()) {
            while (digitalRead(LID) == HIGH) {
              lidWasLow = false;
              timestamp = millis();

              // Stop if the charger is connected
              if (digitalRead(CHARGER_DOCK)) {
                Serial.println("Charger connected. Halting operations...");
                ledcWrite(CS_FAN_Channel, 0);  // Stop all PWM channels
                ledcWrite(HS_FAN_Channel, 0);
                ledcWrite(TEC_Channel, 0);
                return;  // Exit loop to stop further execution
              }

              // Periodically check for GPS location every 10 seconds
              // Locationtimer++;
              // if (Locationtimer == 100000) {
              //   Locationtimer = 0;
              //   sendCommand("AT+QGPSLOC?");
              // }

              Temperature_reading();
              Current_values();
              PI_ALGO();
              HS_Fan();
              CS_Fan();
              TEC_COLD();
              LED_INDI();
              Serial.print("timestamp: ");
              Serial.println(timestamp);
              Serial.print("soc: ");
              Serial.println(soc);
              Serial.print("fault: ");
              Serial.println(fault);
              Serial.print("batteryVoltage: ");
              Serial.println(batteryVoltage);
              Serial.print("TEC_ISNS_current: ");
              Serial.println(TEC_ISNS_current);
              Serial.print("HSFAN_ISNS_current: ");
              Serial.println(HSFAN_ISNS_current);
              Serial.print("CSFAN_ISNS_current: ");
              Serial.println(CSFAN_ISNS_current);
              Serial.print("PCB_TEMP: ");
              Serial.println(temperatureCelsius[0]);
              Serial.print("FLASKTOP_TEMP: ");
              Serial.println(temperatureCelsius[3]);
              Serial.print("HEATSINK_TEMP: ");
              Serial.println(temperatureCelsius[2]);
              Serial.print("COLDSINK_TEMP: ");
              Serial.println(temperatureCelsius[1]);
              Serial.print("FLASKBOTTOM_TEMP: ");
              Serial.println(temperatureCelsius[4]);
              Serial.print("Flask_avg_temp: ");
              Serial.println(Flask_avg_temp);

              myFile = SD.open("/data_logging_01.csv", FILE_APPEND);
              if (myFile) {
                myFile.print(timestamp);
                myFile.print(",");
                myFile.print(fault);
                myFile.print(",");
                myFile.print(soc);
                myFile.print(",");
                myFile.print(batteryVoltage);
                myFile.print(",");
                myFile.print(TEC_ISNS_current);
                myFile.print(",");
                myFile.print(HSFAN_ISNS_current);
                myFile.print(",");
                myFile.print(CSFAN_ISNS_current);
                myFile.print(",");
                myFile.print(PCB_TEMP);
                myFile.print(",");
                myFile.print(FLASKTOP_TEMP);
                myFile.print(",");
                myFile.print(HEATSINK_TEMP);
                myFile.print(",");
                myFile.print(COLDSINK_TEMP);
                myFile.print(",");
                myFile.print(FLASKBOTTOM_TEMP);
                myFile.print(",");
                myFile.println(Flask_avg_temp);
                myFile.close();
              } else {
                Serial.println("Error opening file for appending");
              }

              delay(2000);

              if (soc <= 10 ) {
                SOC_SAMP++;
                if (SOC_SAMP >= 6) {
                  SOC_SAMP = 0;
                  SOC_CHECK == 0;
                  fault = 1;
                  stopSOCCheck = true;
                  break;
                }
              } else {
                SOC_CHECK == 1;
                fault = 0;
              }


              // digitalWrite(LED_PIN, LOW);
              if (TEC_dutyCycle == 0) {
                SOC_SAMP_1++;
                if (SOC_SAMP_1 >= 6) {
                  soc_read();
                  SOC_SAMP_1 = 0;
                }
              }

              // Check if the LID is OPEN, and if so, break out of the loop
              if (digitalRead(LID) == LOW) {
                break;
              }

              // wakeup_process();
              if (buttonPressed) {
                wakeup_process();
              }
            }

            Serial.println("LID_open");

            // If LID is high, handle this condition
            if (digitalRead(LID) == LOW) {
              ledcWrite(CS_FAN_Channel, 0);  // Stop all PWM channels
              ledcWrite(HS_FAN_Channel, 0);
              ledcWrite(TEC_Channel, 0);
              if (!lidWasLow) {
                lidWasLow = true;
                lidLowStartTime = millis();
              }
              if (millis() - lidLowStartTime >= 20000) {
                Buzzer_LID();
                delay(3000);  // Wait for 3 seconds before the next beep
              }
            }
          }
        }
      } else {
        fault = 1;

        ledcWrite(CS_FAN_Channel, 0);
        ledcWrite(HS_FAN_Channel, 0);
        ledcWrite(TEC_Channel, 0);
        ledcWrite(BUZZER_channel, 0);
        ws2812b.setPixelColor(0, ws2812b.Color(0, 0, 0));  // Red
        ws2812b.show();
        digitalWrite(LED_PIN, LOW);
        Serial.print("soc: ");
        Serial.println(soc);
        Serial.print("battery_voltage : ");
        Serial.println(batteryVoltage);
        Serial.println("Connect charger");
        LED_INDI();
        timestamp = millis();
        myFile = SD.open("/data_logging_01.csv", FILE_APPEND);
        if (myFile) {
          myFile.print(timestamp);
          myFile.print(",");
          myFile.print(fault);
          myFile.print(",");
          myFile.print(soc);
          myFile.print(",");
          myFile.println(batteryVoltage);
          myFile.close();
        } else {
          Serial.println("Error opening file for appending");
        }
        

          delay(2000);
      }
      soc_read();
      if (soc > 10) {
        stopSOCCheck = false;
        SOC_CHECK = 1;
      }
    }
  }
}


// // Function to send AT commands and read response from EC200U
// void sendCommand(String command) {
//   // Send command to EC200u module
//   ec200uSerial.println(command);


//   // Wait for response and print to Serial Monitor
//   delay(1000);  // Wait for response
//   if (ec200uSerial.available()) {
//     while (ec200uSerial.available()) {
//       String response = ec200uSerial.readString();
//       Serial.println("Response: " + response);


//       // Extract latitude and longitude if +QGPSLOC response is received
//       if (response.startsWith("AT+QGPSLOC?")) {
//         extractLatLong(response);  // Extract latitude and longitude


//         // Convert to decimal format for Google Maps
//         double latDecimal = convertToDecimal(latitude, true);     // true for latitude
//         double longDecimal = convertToDecimal(longitude, false);  // false for longitude


//         Serial.print("Latitude (Decimal): ");
//         Serial.println(latDecimal, 6);  // 6 decimal places
//         Serial.print("Longitude (Decimal): ");
//         Serial.println(longDecimal, 6);  // 6 decimal places
//       }
//     }
//   } else {
//     Serial.println("No response received.");
//   }
// }


// // Function to extract latitude and longitude from +QGPSLOC response
// void extractLatLong(String response) {
//   // Example: +QGPSLOC: 180629.000,1307.1481N,07736.7321E,...
//   int firstCommaIndex = response.indexOf(',');                        // Find first comma
//   int secondCommaIndex = response.indexOf(',', firstCommaIndex + 1);  // Find second comma (latitude)
//   int thirdCommaIndex = response.indexOf(',', secondCommaIndex + 1);  // Find third comma (longitude)
//   int fourthCommaIndex = response.indexOf(',', thirdCommaIndex + 1);  // End of longitude


//   latitude = response.substring(firstCommaIndex + 1, secondCommaIndex);   // Extract latitude
//   longitude = response.substring(secondCommaIndex + 1, thirdCommaIndex);  // Extract longitude
// }


// // Function to convert NMEA-style latitude/longitude to decimal degrees
// double convertToDecimal(String coord, bool isLatitude) {
//   // Latitude has 2 degrees, longitude has 3 degrees
//   int degreePartLength = isLatitude ? 2 : 3;  // 2 digits for latitude, 3 digits for longitude


//   // Extract degrees and minutes
//   double degrees = coord.substring(0, degreePartLength).toDouble();
//   double minutes = coord.substring(degreePartLength, coord.length() - 1).toDouble();


//   // Convert minutes to decimal and add to degrees
//   double decimal = degrees + (minutes / 60.0);


//   // Check for direction (N/S or E/W) and adjust sign accordingly
//   char direction = coord.charAt(coord.length() - 1);
//   if (direction == 'S' || direction == 'W') {
//     decimal *= -1;
//   }


//   return decimal;
// }
