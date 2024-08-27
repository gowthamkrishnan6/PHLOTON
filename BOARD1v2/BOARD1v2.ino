#include <esp_system.h>  // Include the necessary header for esp_restart()
#include <esp_sleep.h>
const gpio_num_t LED_PIN = GPIO_NUM_47;       // Pin no 47
const gpio_num_t BUTTON_SW = GPIO_NUM_10;     // Pin no 10
const gpio_num_t BUZZER_PIN = GPIO_NUM_42;    // Pin no 42
const gpio_num_t BATT_VSNS = GPIO_NUM_4;      // Pin no 4
const gpio_num_t HS_FAN = GPIO_NUM_7;         // Pin no 7
const gpio_num_t CS_FAN = GPIO_NUM_21;        // Pin no 21
const gpio_num_t LID = GPIO_NUM_10;           // Pin no 9
const gpio_num_t HSFAN_ISNS = GPIO_NUM_15;    // Pin no 15
const gpio_num_t CSFAN_ISNS = GPIO_NUM_14;    // Pin no 14
const gpio_num_t BATT_ISNS = GPIO_NUM_5;      // Pin no 5
const gpio_num_t TEC_ISNS = GPIO_NUM_6;       // Pin no 6
const gpio_num_t TEC_COLD_CTRL = GPIO_NUM_1;  // Pin no 1
const int HS_FAN_Channel = 1;                 // Channel 1
const int resolution = 12;                    // Maximum resolution supported by ESP32 LEDC is 12 bits
int HS_dutyCycle = 1000;                      // Hot fan Duty cycle
const int freq = 5000;                        // 5kHz frequency
const int CS_FAN_Channel = 0;                 // Channel 0
int CS_dutyCycle = 1000;                      // Cold fan Duty cycle
const float R1 = 10000;
const float R2 = 1000;
int LID_Count = 0;
unsigned long lidLowStartTime = 0;
bool lidWasLow = false;
String receivedData = "";
float Rec_TEC_DutyCycle, Rec_HSFAN_DutyCycle, Rec_CSFAN_DutyCycle;
const gpio_num_t RX_PIN = GPIO_NUM_18;
const gpio_num_t TX_PIN = GPIO_NUM_17;
int HS_count;
int CS_count;

volatile bool buttonPressed = false;  // Flag to indicate button press
bool deviceActive = false;            // Flag to indicate if device is on


const float Vref = 3.3;     // Reference voltage for ADC (3.3V)
const int ADC_Max = 4095;   // 12-bit ADC max value
const float Gain = 10.0;    // Assume a gain of 10 for the op-amp circuit (adjust as needed)
const float Rsense = 0.01;  // Current sense resistor value in ohms (adjust based on your design)


HardwareSerial SerialPort(2);

// Function to be called when the button is pressed
void buttonISR() {
  buttonPressed = true;  // Set the flag indicating button press
}

void setup() {
  Serial.begin(115200);                                  // Initialize Serial communication
  SerialPort.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);  // RX on GPIO18, TX on GPIO17
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);  // Ensure LED is initially off
  pinMode(BUTTON_SW, INPUT_PULLUP);

  // Attach interrupt for button press (rising edge, i.e., button release)
  attachInterrupt(digitalPinToInterrupt(BUTTON_SW), buttonISR, RISING);
  pinMode(BUZZER_PIN, OUTPUT);  // Initialize the buzzer pin as an output
  // Buzzer();
  ledcSetup(HS_FAN_Channel, freq, resolution);  // configure HS_FAN PWM functionality
  ledcAttachPin(HS_FAN, HS_FAN_Channel);        // attach the channel
  ledcSetup(CS_FAN_Channel, freq, resolution);  // configure CS_FAN PWM functionality
  ledcAttachPin(CS_FAN, CS_FAN_Channel);        // attach the channel
  pinMode(LID, INPUT);
}

void Buzzer() {
  // Turn on the buzzer
  digitalWrite(BUZZER_PIN, HIGH);

  // Wait for 2 seconds
  delay(2000);

  // Turn off the buzzer
  digitalWrite(BUZZER_PIN, LOW);
}

void Buzzer_LID() {
  digitalWrite(BUZZER_PIN, HIGH);
  delay(500);
  digitalWrite(BUZZER_PIN, LOW);
}

float readBatteryVoltage() {
  int BATT_VSNS_adcValue = analogRead(BATT_VSNS);  // Read ADC value
  // Convert ADC value to voltage (assuming Vref=3.3V)
  Serial.println(BATT_VSNS_adcValue);
  float BATT_VSNS_voltage = BATT_VSNS_adcValue * (3.3 / 4095.0);
  float voltage = BATT_VSNS_voltage / (R2 / (R1 + R2));
  return voltage;
}

int calculateSOC(float voltage) {
  if (voltage <= 13.0) {
    return 0;
  } else if (voltage >= 16.4) {
    return 100;
  } else {
    return (int)((voltage - 13.0) / (16.4 - 13.0) * 99) + 1;
  }
}

bool HS_Fan() {
  ledcWrite(HS_FAN_Channel, HS_dutyCycle);
  Serial.println("HS fan is ON");

  // Check if the PWM signal is set
  int dutyCycleSet = ledcRead(HS_FAN_Channel);

  if (dutyCycleSet == HS_dutyCycle) {
    Serial.println("HS fan is working fine.");
    HS_count = 0;  // Reset the counter if everything is fine
    return true;
  } else {
    HS_count++;
    if (HS_count == 5) {
      HS_count = 0;
      Serial.println("HS fan error: PWM signal not set correctly. Restarting...");
      // esp_restart();  // Restart the ESP32-S2
      return false;  // This return will never be reached, as the ESP will restart
    }
  }
  return true;  // Default return value
}

bool CS_Fan() {
  ledcWrite(CS_FAN_Channel, CS_dutyCycle);
  Serial.println("CS fan is ON");
  // Check if the PWM signal is set
  int dutyCycleSet = ledcRead(CS_FAN_Channel);
  if (dutyCycleSet == CS_dutyCycle) {
    CS_count = 0;  // Reset the counter if everything is fine
    Serial.println("CS fan is working fine.");
    return true;
  } else {
    CS_count++;
    if (CS_count == 5) {
      CS_count = 0;
      Serial.println("CS fan error: PWM signal not set correctly.");
      // esp_restart();  // Restart the ESP32-S2
      return false;
    }
  }
}

float current_calc(int adc_pin) {
  int adcValue = analogRead(adc_pin);           // Read ADC value
  float voltage = (adcValue * Vref) / ADC_Max;  // Convert ADC value to voltage

  // Calculate current (I = Vout / (Gain * Rsense))
  float current = voltage / (Gain * Rsense);

  Serial.print("Current: ");
  Serial.print(current, 6);  // Print current with 6 decimal places
  Serial.println(" A");
  return current;
}

void Current_values() {


  // Calculate the current in amps
  float BATT_ISNS_CURRENT = current_calc(BATT_ISNS);
  Serial.print("BATT_ISNS_CURRENT : ");
  Serial.print(BATT_ISNS_CURRENT, 6);
  Serial.println("A");
  SerialPort.print(BATT_ISNS_CURRENT);
  SerialPort.print(",");


  float HSFAN_ISNS_CURRENT = current_calc(HSFAN_ISNS);
  Serial.print("HSFAN_ISNS_CURRENT : ");
  Serial.print(HSFAN_ISNS_CURRENT, 6);
  Serial.println("A");
  SerialPort.print(HSFAN_ISNS_CURRENT);
  SerialPort.print(",");


  float CSFAN_ISNS_CURRENT = current_calc(CSFAN_ISNS);
  Serial.print("CSFAN_ISNS_CURRENT : ");
  Serial.print(CSFAN_ISNS_CURRENT, 6);
  Serial.println("A");
  SerialPort.print(CSFAN_ISNS_CURRENT);
  SerialPort.print("\n");


  float TEC_ISNS_CURRENT = current_calc(TEC_ISNS);
  Serial.print("TEC_ISNS_CURRENT : ");
  Serial.print(TEC_ISNS_CURRENT, 6);
  Serial.println("A");
  // SerialPort.print(TEC_ISNS_CURRENT);
  // SerialPort.print("\n");
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
  while (token != NULL && index < 3) {
    float value = atof(token);
    if (index == 0) {
      Rec_TEC_DutyCycle = value;
    } else if (index == 1) {
      Rec_HSFAN_DutyCycle = value;
    } else if (index == 2) {
      Rec_CSFAN_DutyCycle = value;
    }
    index++;
    token = strtok(NULL, ",");
  }

  // Print received ADC values
  Serial.println("Received DutyCycle Values:");
  Serial.print("Rec_TEC_DutyCycle: ");
  Serial.println(Rec_TEC_DutyCycle);
  Serial.print("Rec_HSFAN_DutyCycle: ");
  Serial.println(Rec_HSFAN_DutyCycle);
  Serial.print("Rec_CSFAN_DutyCycle: ");
  Serial.println(Rec_CSFAN_DutyCycle);
  HS_dutyCycle = Rec_TEC_DutyCycle;
  // Clear the receivedData for the next reading
  receivedData = "";
}

void wakeup_process() {
  if (buttonPressed) {

    delay(500);                               // Debounce delay
    Serial.println("Loop: Button pressed.");  // Print button press message

    // Toggle device state
    deviceActive = !deviceActive;

    // Toggle LED based on device state
    digitalWrite(LED_PIN, deviceActive ? HIGH : LOW);

    // If device is now active, stay awake
    if (deviceActive) {
      Serial.println("Loop: Device waking up...");  // Print waking up message
      // No need to sleep, device is active
    } else {
      Serial.println("Loop: Device going to sleep...");  // Print going to sleep message
      // Enter sleep mode
      esp_sleep_enable_ext0_wakeup(BUTTON_SW, LOW);
      esp_deep_sleep_start();
    }

    // Reset the button press flag
    buttonPressed = false;
  }
}

void loop() {
  wakeup_process();
  if (deviceActive) {


    float batteryVoltage = readBatteryVoltage();
    int soc = calculateSOC(batteryVoltage);
    Serial.println(batteryVoltage);
    Serial.print(soc);

    if (soc >= 10) {
      if (HS_Fan()) {
        // if (CS_Fan())
        while (digitalRead(LID) == HIGH) {
          lidWasLow = false;
          batteryVoltage = readBatteryVoltage();
          soc = calculateSOC(batteryVoltage);
          Serial.println(soc);
          // SerialPort.print("SOC : ");
          SerialPort.print(soc);
          SerialPort.print(",");
          SerialPort.print(batteryVoltage);
          SerialPort.print(",");
          digitalWrite(TEC_COLD_CTRL, HIGH);
          Current_values();
          HS_Fan();

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
          delay(1000);
        }

        if (digitalRead(LID) == LOW) {
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
    } else {
      digitalWrite(LED_PIN, HIGH);
      delay(1000);
      digitalWrite(LED_PIN, LOW);
      delay(100);
      // HS_dutyCycle=0;
      // CS_dutyCycle=0;
      // HS_Fan();
      // CS_Fan();
      Serial.println("Connect charger");
    }
  }
}
