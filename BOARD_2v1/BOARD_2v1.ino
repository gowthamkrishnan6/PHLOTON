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
const gpio_num_t thermistorPin1 = GPIO_NUM_3;
const gpio_num_t thermistorPin2 = GPIO_NUM_9;
const gpio_num_t thermistorPin3 = GPIO_NUM_10;
const gpio_num_t thermistorPin4 = GPIO_NUM_11;
const gpio_num_t thermistorPin5 = GPIO_NUM_12;
const gpio_num_t thermistorPin6 = GPIO_NUM_8;
const gpio_num_t LED_PIN = GPIO_NUM_21;
const float BETA = 3950;
const float R0 = 10000.0;
int Temperature1, Temperature2, Temperature3, Temperature4, Temperature5, Temperature6;
String receivedData = "";
float CSFAN_ISNS, HSFAN_ISNS, VBatt_ISNS, SOC;
float HSFAN_Dutycycle, CSFAN_Dutycycle, TEC_Dutycycle;

File myFile;







float Kp = 200.0;      // Proportional gain (tune based on your system)
float Ki = 50.0;       // Integral gain (tune based on your system)
float setpoint = 4.0;  // Desired temperature in Celsius
                       // Variables
float integral = 0.0;
float previous_error = 0.0;
float dt = 1.0; // Time step in seconds


void setup() {
  Serial.begin(115200);
  SerialPort.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);  // RX on GPIO18, TX on GPIO17
  // delay(2000);
  // while (!Serial) {
  //   // Wait for serial port to connect. Needed for native USB port only
  //   ;
  // }
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

void Temperature_reading() {
  Temperature1 = temperature_calculation(thermistorPin1);
  Temperature2 = temperature_calculation(thermistorPin2);
  Temperature3 = temperature_calculation(thermistorPin3);
  Temperature4 = temperature_calculation(thermistorPin4);
  Temperature5 = temperature_calculation(thermistorPin5);
  Temperature6 = temperature_calculation(thermistorPin6);

  // Validate all thermistors
  if (Temperature1 == 0 || Temperature2 == 0 || Temperature3 == 0 || Temperature4 == 0 || Temperature5 == 0 || Temperature6 == 0) {
    Serial.println("Error: One or more thermistors are not working correctly. Check connections.");
    return;
  }
}

int temperature_calculation(int ThermistorPin) {
  const int numreading = 5;
  int analogsamples[numreading];

  // Take multiple readings for averaging
  for (int i = 0; i < numreading; ++i) {
    analogsamples[i] = analogRead(ThermistorPin);
    delay(10);
  }

  // Calculate the average of the readings
  int sum = 0;
  for (int i = 0; i < numreading; ++i) {
    sum += analogsamples[i];
  }
  int analogValue = sum / numreading;

  // Check for valid ADC range
  if (analogValue < 0 || analogValue > 4095) {
    Serial.println("Error: Invalid ADC reading");
    return 0;
  }

  // Convert ADC value to voltage
  float voltage = analogValue * (2.5 / 4095.0);

  // Calculate the resistance of the thermistor
  float resistance = (R0 * voltage) / (2.5 - voltage);

  // Check if resistance is within expected range
  if (resistance < 100.0 || resistance > 1000000.0) {
    Serial.println("Error: Resistance out of range, possible thermistor disconnection");
    return 0;
  }

  // Calculate the temperature using the Steinhart-Hart equation
  float temperature = (1.0 / ((log(resistance / R0) / BETA) + (1.0 / (25.0 + 273.15)))) - 273.15;

  // Check if temperature is within expected range
  if (temperature < -40.0 || temperature > 100.0) {
    Serial.println("Error: Temperature out of range");
    return 0;
  }

  return int(temperature);
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


  // Step 1: Measure the temperature
  float currentTemperature = Temperature6;  // Implement this function based on your sensor

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
  SerialPort.println(HSFAN_Dutycycle);

  Serial.print("Current Temp: ");
Serial.println(Temperature6);
Serial.print("Error: ");
Serial.println(error);
Serial.print("P_out: ");
Serial.println(P_out);
Serial.print("I_out: ");
Serial.println(I_out);
// Serial.print("PWM Duty Cycle: ");
// Serial.println(pwmDutyCycle);

}


void loop() {

  Temperature_reading();

  // If any thermistor has an error, skip the rest of the loop
  if (Temperature1 == 0 || Temperature2 == 0 || Temperature3 == 0 || Temperature4 == 0 || Temperature5 == 0) {
    delay(10000);  // Wait for 10 seconds before trying again
    return;
  }
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



    myFile.print("Temperature1 :");
    myFile.println(Temperature1);
    myFile.print("Temperature2 :");
    myFile.println(Temperature2);
    myFile.print("Temperature3 :");
    myFile.println(Temperature3);
    myFile.print("Temperature4 :");
    myFile.println(Temperature4);
    myFile.print("Temperature5 :");
    myFile.println(Temperature5);
    myFile.print("Temperature6 :");
    myFile.println(Temperature6);


    Serial.print("Temperature1: ");
    Serial.println(Temperature1);
    Serial.print("Temperature2: ");
    Serial.println(Temperature2);
    Serial.print("Temperature3: ");
    Serial.println(Temperature3);
    Serial.print("Temperature4: ");
    Serial.println(Temperature4);
    Serial.print("Temperature5: ");
    Serial.println(Temperature5);
    Serial.print("Temperature6: ");
    Serial.println(Temperature6);
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
