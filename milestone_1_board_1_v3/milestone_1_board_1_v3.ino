#include <esp_system.h>                       // Include the necessary header for esp_restart()
const gpio_num_t LED_PIN = GPIO_NUM_47;       // Pin no 47
const gpio_num_t BUTTON_SW = GPIO_NUM_10;     // Pin no 10
const gpio_num_t BUZZER_PIN = GPIO_NUM_42;    // Pin no 42
const gpio_num_t BATT_VSNS = GPIO_NUM_4;      // Pin no 4
const gpio_num_t HS_FAN = GPIO_NUM_7;         // Pin no 7
const gpio_num_t CS_FAN = GPIO_NUM_21;        // Pin no 21
const gpio_num_t LID = GPIO_NUM_9;            // Pin no 9
const gpio_num_t HSFAN_ISNS = GPIO_NUM_15;    // Pin no 15
const gpio_num_t CSFAN_ISNS = GPIO_NUM_14;    // Pin no 14
const gpio_num_t BATT_ISNS = GPIO_NUM_5;      // Pin no 5
const gpio_num_t TEC_ISNS = GPIO_NUM_6;       // Pin no 6
const gpio_num_t TEC_COLD_CTRL = GPIO_NUM_1;  // Pin no 1
const gpio_num_t TEC_HOT_CTRL = GPIO_NUM_2;   // Pin no 2
const int HS_FAN_Channel = 1;                 // Channel 1
const int resolution = 12;                    // Maximum resolution supported by ESP32 LEDC is 12 bits
int HS_dutyCycle = 2000;                      // Hot fan Duty cycle
const int freq = 5000;                        // 5kHz frequency
const int CS_FAN_Channel = 5;                 // Channel 0
const int TEC_Channel = 2;
int CS_dutyCycle = 10;  // Cold fan Duty cycle
int TEC_dutyCycle = 3000;
const float R1 = 10520;
const float R2 = 1000;
int LID_Count = 0;
unsigned long lidLowStartTime = 0;
bool lidWasLow = true;
String receivedData = "";
float Rec_TEC_DutyCycle, Rec_HSFAN_DutyCycle, Rec_CSFAN_DutyCycle;
const gpio_num_t RX_PIN = GPIO_NUM_18;
const gpio_num_t TX_PIN = GPIO_NUM_17;
int HS_count;
int CS_count;
int fault;
int soc;
float batteryVoltage;
const int pwmChannel = 4;  // PWM channel (0-15 on ESP32)
const int freq_b = 5000;
volatile bool buttonPressed = false;  // Flag to indicate button press
bool deviceActive = false;            // Flag to indicate if device is on
int SOC_CHECK = 1;
bool stopSOCCheck = false;




const float Vref = 3.3;     // Reference voltage for ADC (3.3V)
const int ADC_Max = 4095;   // 12-bit ADC max value
const float Gain = 10.0;    // Assume a gain of 10 for the op-amp circuit (adjust as needed)
const float Rsense = 1.01;  // Current sense resistor value in ohms (adjust based on your design)




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

  // Setup PWM
  ledcSetup(pwmChannel, freq_b, resolution);

  // Attach the buzzer pin to the PWM channel
  ledcAttachPin(BUZZER_PIN, pwmChannel);
  ledcSetup(HS_FAN_Channel, freq, resolution);  // configure HS_FAN PWM functionalityz
  ledcAttachPin(HS_FAN, HS_FAN_Channel);        // attach the channel
  ledcSetup(CS_FAN_Channel, freq, resolution);  // configure CS_FAN PWM functionality
  ledcAttachPin(CS_FAN, CS_FAN_Channel);        // attach the channel
  ledcSetup(TEC_Channel, freq, resolution);     // configure CS_FAN PWM functionality
  ledcAttachPin(TEC_COLD_CTRL, TEC_Channel);    // attach the channel
  pinMode(LID, OUTPUT);
}




void Buzzer_LID() {
  // Turn on the buzzer (50% duty cycle)
  ledcWrite(pwmChannel, 2048);  // 2048 is 50% of 4095 (for 12-bit resolution)
  // Beep sound for 2 seconds
  delay(200);
  // Turn off the buzzer
  ledcWrite(pwmChannel, 0);
  // Buzzer off for 2 seconds
  delay(100);
  ledcWrite(pwmChannel, 2048);  // 2048 is 50% of 4095 (for 12-bit resolution)
  // Beep sound for 2 seconds
  delay(400);
  // Turn off the buzzer
  ledcWrite(pwmChannel, 0);
}




void Buzzer_LID_OFF() {
  // Turn on the buzzer (50% duty cycle)
  ledcWrite(pwmChannel, 2048);  // 2048 is 50% of 4095 (for 12-bit resolution)
  // Beep sound for 2 seconds
  delay(500);
  // Turn off the buzzer
  ledcWrite(pwmChannel, 0);
}




float readBatteryVoltage() {




  // int BATT_VSNS_adcValue = analogRead(BATT_VSNS);  // Read ADC value
  float adc_sample[5];
  float adc_volt_samp = 0;
  for (int i = 0; i < 5; i++) {
    adc_sample[i] = analogRead(BATT_VSNS);
    adc_volt_samp = adc_volt_samp + adc_sample[i];
    delay(10);
  }
  int BATT_VSNS_adcValue = adc_volt_samp / 5;




  // Convert ADC value to voltage (assuming Vref=3.3V)
  //Serial.println(BATT_VSNS_adcValue);
  float BATT_VSNS_voltage = BATT_VSNS_adcValue * (3.3 / 4095.0);
  float voltage = BATT_VSNS_voltage / (R2 / (R1 + R2));
  return voltage;
}




int sample_soc(float voltage) {
  if (voltage <= 13.0) {
    return 0;
  } else if (voltage >= 16.4) {
    return 100;
  } else {
    return (int)((voltage - 13.0) / (16.4 - 13.0) * 99) + 1;
  }
}




int calculateSOC(float voltage) {
  int soc_sam[5];
  int average = 0;
  for (int i = 0; i < 5; i++) {
    soc_sam[i] = sample_soc(voltage);
    average += soc_sam[i];
    delay(10);
  }
  return average / 5;
}




void HS_Fan() {
  ledcWrite(HS_FAN_Channel, HS_dutyCycle);
  // Check if the PWM signal is set
  int dutyCycleSet = ledcRead(HS_FAN_Channel);




  if (dutyCycleSet == HS_dutyCycle) {
    HS_count = 0;
    fault = 0;
  } else {
    HS_count++;
    if (HS_count == 6) {
      HS_count = 0;
      fault = 2;
    }
  }
}




bool HS_Fan_check() {
  ledcWrite(HS_FAN_Channel, HS_dutyCycle);
  Serial.println("HS fan is ON");




  // Check if the PWM signal is set
  int dutyCycleSet = ledcRead(HS_FAN_Channel);




  if (dutyCycleSet == HS_dutyCycle) {
    HS_count = 0;  // Reset the counter if everything is fine
    fault = 0;
    return true;
  } else {
    HS_count++;
    if (HS_count == 6) {
      HS_count = 0;
      fault = 2;
      SerialPort.print(fault);
      SerialPort.println(",");
      // // esp_restart();  // Restart the ESP32-S2
      return false;
    }
  }
  return true;  // Default return value
}




void CS_Fan() {
  ledcWrite(CS_FAN_Channel, CS_dutyCycle);
  int dutyCycleSet = ledcRead(CS_FAN_Channel);
  if (dutyCycleSet == CS_dutyCycle) {
    CS_count = 0;
    fault = 0;
  } else {
    CS_count++;
    if (CS_count == 5) {
      CS_count = 0;
      fault = 3;
    }
  }
}




bool CS_Fan_check() {
  ledcWrite(CS_FAN_Channel, CS_dutyCycle);
  Serial.println("CS fan is ON");
  int dutyCycleSet = ledcRead(CS_FAN_Channel);
  if (dutyCycleSet == CS_dutyCycle) {
    CS_count = 0;  // Reset the counter if everything is fine
    fault = 0;
    return true;
  } else {
    CS_count++;
    if (CS_count == 6) {
      CS_count = 0;
      fault = 3;
      SerialPort.print(fault);
      SerialPort.println(",");
      return false;
    }
  }
  return true;  // Default return value
}
void TEC_COLD() {
  ledcWrite(TEC_Channel, TEC_dutyCycle);
  int dutyCycleSet = ledcRead(TEC_Channel);
  if (dutyCycleSet == TEC_dutyCycle) {
    CS_count = 0;
    fault = 0;
  } else {
    CS_count++;
    if (CS_count == 6) {
      CS_count = 0;
      fault = 7;
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
  // digitalWrite(LED_PIN, HIGH);
  // Convert the receivedData to a mutable character array
  char receivedCharArray[receivedData.length() + 1];
  receivedData.toCharArray(receivedCharArray, receivedData.length() + 1);
  // Split the received data into individual numbers
  int index = 0;
  char *token = strtok(receivedCharArray, ",");
  while (token != NULL && index < 3) {
    float value = atof(token);
    if (index == 0) {
      Rec_HSFAN_DutyCycle = value;
    } else if (index == 1) {
      Rec_TEC_DutyCycle = value;
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
  Serial.println(CS_dutyCycle);
  HS_dutyCycle = Rec_HSFAN_DutyCycle;
  // CS_dutyCycle = Rec_CSFAN_DutyCycle;
  TEC_dutyCycle = Rec_TEC_DutyCycle;
  //Clear the receivedData for the next reading
  receivedData = "";
}


void soc_read() {
  batteryVoltage = readBatteryVoltage();
  soc = calculateSOC(batteryVoltage);
}




void wakeup_process() {
  if (buttonPressed) {
    delay(500);                               // Debounce delay
    Serial.println("Loop: Button pressed.");  // Print button press message


    // Toggle device state
    deviceActive = !deviceActive;
    // Toggle LED based on device state
    // buttonPressed = false;
    // If device is now active, stay awake
    if (deviceActive) {
      Serial.println("Loop: Device is active...");  // Print waking up message
      digitalWrite(LED_PIN, HIGH);
      soc_read();
      Buzzer_LID();
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
        // ledcWrite(CS_FAN_Channel, 1);
        digitalWrite(CS_FAN,LOW);
        ledcWrite(HS_FAN_Channel, 0);
        ledcWrite(TEC_Channel, 0);
        ledcWrite(pwmChannel, 0);
        digitalWrite(LED_PIN, LOW);
        soc_read();
        if(soc<=10)
        SerialPort.print(1);
        else
        SerialPort.print(9);
        SerialPort.print(",");
        SerialPort.print(soc);
        SerialPort.print(",");
        SerialPort.println(batteryVoltage);

        delay(500);




        if (buttonPressed) {
          delay(500);
          buttonPressed = false;
          deviceActive = false;
          Serial.println("device is reactive:");
          Buzzer_LID();
          // return 0;
          break;
          // esp_restart();
        }
        // buttonPressed = false;
        delay(1000);
      }
    }
    // Reset the button press flag
    buttonPressed = false;
  }
}




void loop() {
  wakeup_process();
  if (deviceActive) {
    if (SOC_CHECK && !stopSOCCheck) {
      if (HS_Fan_check()) {
        if (CS_Fan_check()) {
          while (digitalRead(LID) == LOW) {
            lidWasLow = false;
            SerialPort.print(fault);
            SerialPort.print(",");
            Serial.print("soc: ");
            Serial.println(soc);
            SerialPort.print(soc);
            SerialPort.print(",");
            SerialPort.print(batteryVoltage);
            SerialPort.print(",");
            Current_values();
            HS_Fan();
            CS_Fan();
            TEC_COLD();
            if (TEC_dutyCycle == 0) {
              soc_read();
            }
            if (soc <= 10 && TEC_dutyCycle == 0) {
              SOC_CHECK == 0;
              fault = 1;
              stopSOCCheck = true;
              break;
            } else {
              SOC_CHECK == 1;
              fault = 0;
            }
            while (SerialPort.available()) {
              digitalWrite(LED_PIN, HIGH);
              char incomingChar = SerialPort.read();
              if (incomingChar == '\n') {
                processReceivedData();
              } else {
                receivedData += incomingChar;
              }
            }
            // digitalWrite(LED_PIN, LOW);




            // Check if the LID is closed, and if so, break out of the loop
            if (digitalRead(LID) == HIGH) {
              break;
            }
            delay(1900);
            // wakeup_process();
            if (buttonPressed) {
              wakeup_process();
            }
          }




          // If LID is high, handle this condition
          if (digitalRead(LID) == HIGH) {
            if (!lidWasLow) {
              lidWasLow = true;
              lidLowStartTime = millis();
            }
            if (millis() - lidLowStartTime >= 20000) {
              Buzzer_LID();
              delay(3000);  // Wait for 3 seconds before the next beep
              SerialPort.print(fault);
              SerialPort.println(",");
            }
          }
        }
      }
    } else {
      fault = 1;
      SerialPort.print(fault);
      SerialPort.print(",");
      SerialPort.print(soc);
      SerialPort.print(",");
      SerialPort.println(batteryVoltage);


      // ledcWrite(CS_FAN_Channel, 0);
      digitalWrite(CS_FAN,LOW);
      ledcWrite(HS_FAN_Channel, 0);
      ledcWrite(TEC_Channel, 0);
      ledcWrite(TEC_Channel, 0);
      ledcWrite(pwmChannel, 0);
      digitalWrite(LED_PIN, LOW);
      Serial.print("soc: ");
      Serial.println(soc);
      Serial.print("battery_voltage : ");
      Serial.print(batteryVoltage);
      Serial.println("Connect charger");
      delay(2000);
    }
    soc_read();
    if (soc > 10) {
      stopSOCCheck = false;
      SOC_CHECK = 1;
    }
  }
}
