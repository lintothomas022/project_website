#include <Arduino.h>
#include <U8x8lib.h> //Display library
#include <Wire.h> //I2C protocol library (the display uses I2C to interact with MCU)
#include "LSM6DS3.h"
#include "Wire.h"
#include <ArduinoBLE.h>
#include <PCF8563.h>
#include <SPI.h>
#include <SD.h>
#include <PDM.h>

const int chipSelect = 2;

U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* clock=*/ PIN_WIRE_SCL, /* data=*/
PIN_WIRE_SDA, /* reset=*/ U8X8_PIN_NONE); // OLEDs without Reset of the Display
//Create a instance of class LSM6DS3
LSM6DS3 myIMU(I2C_MODE, 0x6A);    //I2C device address 0x6A

PCF8563 pcf;

const int ledPin = LED_BUILTIN;

// Resistor values in your voltage divider (in ohms)
const float R1 = 2880.0;  // 2.2kΩ + 680Ω resistor
const float R2 = 10000.0; // 10kΩ resistor

// Reference voltage of the ADC (usually 5V or 3.3V)
const float referenceVoltage = 3.3;

// Microphone
static const char channels = 1;
static const int frequency = 16000;
short sampleBuffer[512];
volatile int samplesRead;

// Bluetooth® Low Energy LED Service
BLEService ledService("19B10000-E8F2-537E-4F6C-D104768A1214"); 

// Bluetooth® Low Energy LED Switch Characteristic - custom 128-bit UUID, read and writable by central
BLEByteCharacteristic switchCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);

bool logging = false;

void setup() {
  // Initialize serial communication at 9600 bits per second:
  Serial.begin(9600);

  while (!Serial);

  // Microphone setup
  PDM.onReceive(onPDMdata);

  if (!PDM.begin(channels, frequency)) {
    Serial.println("Failed to start PDM!");
    while (1);
  }

  //Call .begin() to configure the IMUs
    if (myIMU.begin() != 0) {
        Serial.println("Device error");
    } else {
        Serial.println("Device OK!");
    }

  u8x8.begin();
  u8x8.setFlipMode(1); // set number from 1 to 3, the screen word will rotary 180

  // set LED pin to output mode
  pinMode(ledPin, OUTPUT);

  // BLE initialization
  if (!BLE.begin()) {
    Serial.println("starting Bluetooth® Low Energy module failed!");
    while (1);
  }

  // set advertised local name and service UUID:
  BLE.setLocalName("Linto-James");
  BLE.setAdvertisedService(ledService);

  // add the characteristic to the service
  ledService.addCharacteristic(switchCharacteristic);

  // add service
  BLE.addService(ledService);

  // set the initial value for the characeristic:
  switchCharacteristic.writeValue(0);

  // start advertising
  BLE.advertise();
  Serial.println("BLE LED Peripheral");

  Serial.print("Initializing SD card...");

  if (!SD.begin(chipSelect)) {
    Serial.println("initialization failed. Things to check:");
    Serial.println("1. is a card inserted?");
    Serial.println("2. is your wiring correct?");
    Serial.println("3. did you change the chipSelect pin to match your shield or module?");
    Serial.println("Note: press reset button on the board and reopen this Serial Monitor after fixing your issue!");
    while (true);
  }

  Serial.println("initialization done.");

  pcf.init();//initialize the clock
  pcf.stopClock();//stop the clock

  //set time to to 31/3/2018 17:33:0

  pcf.setYear(25);//set year
  pcf.setMonth(2);//set month
  pcf.setDay(5);//set dat
  pcf.setHour(14);//set hour
  pcf.setMinut(42);//set minut
  pcf.setSecond(0);//set second
  pcf.startClock();//start the clock
}

float mic() {
  // Wait for samples to be read
  float audio;
  if (samplesRead) {

    // Print samples to the serial monitor or plotter
    for (int i = 0; i < samplesRead; i++) {
      if(channels == 2) {
        Serial.print("L:");
        Serial.print(sampleBuffer[i]);
        Serial.print(" R:");
        i++;
      }
      // Serial.println(sampleBuffer[i]);
      audio = sampleBuffer[i];
    }

    // Clear the read count
    samplesRead = 0;
  }
  Serial.println(audio);
  return audio;
}


void loop() {
   // listen for Bluetooth® Low Energy peripherals to connect:
  BLEDevice central = BLE.central();

  // Set initial screen state
  u8x8.setCursor(0, 2); // (columns, row)
  u8x8.print("STATUS OFF "); 
  u8x8.setCursor(0, 3); // (columns, row)
  u8x8.print("Low Battery:");
  u8x8.setCursor(0, 4); // (columns, row)
  u8x8.print("Logging Stopped");

  // if a central is connected to peripheral:
  if (central) {
    Serial.print("Connected to central: ");
    // print the central's MAC address:
    Serial.println(central.address());

    // while the central is still connected to peripheral:
    while (central.connected()) {
      // send a value from remote device to control the LED and data logging:
      if (switchCharacteristic.written()) {
        // any value other than 0 turns LED on and starts logging data
        if (switchCharacteristic.value()) {   
          logging = true;
        } else {  
          // 0 value turns the LED off and stops logging data 
          logging = false;                          
          Serial.println(F("LED off"));        
        }
      }
      getData();
    }
    // when the central disconnects, print it out:
    // connected = false; 

    Serial.print(F("Disconnected from central: "));
    Serial.println(central.address());
  }
}


void getData() {

  if (logging) {
    // Read the input on analog pin A0:
    int sensorValue = analogRead(A0);

    // Convert the ADC value to voltage:
    float measuredVoltage = sensorValue * (referenceVoltage / 1023.0);

    // Account for the voltage divider to get the actual battery voltage:
    float batteryVoltage = measuredVoltage * ((R1 + R2) / R2);

    if (batteryVoltage > referenceVoltage) {
          digitalWrite(ledPin, HIGH);  // Turn ON if above threshold
          Serial.println("ON");
          u8x8.setCursor(0, 2); // (columns, row)
          u8x8.print("STATUS ON ");
          u8x8.clearLine(3);
          u8x8.clearLine(4);
      } else {
          digitalWrite(ledPin, LOW);   // Turn OFF if below threshold
          Serial.println("OFF");
          u8x8.setCursor(0, 2); // (columns, row)
          u8x8.print("STATUS OFF "); 
          u8x8.setCursor(0, 3); // (columns, row)
          u8x8.print("Low Battery:");
          u8x8.setCursor(0, 4); // (columns, row)
          u8x8.print("Logging Stopped");

          return;
      }

    //Accelerometer
      Serial.print("\nAccelerometer:\n");
      Serial.print(" X1 = ");
      Serial.println(myIMU.readFloatAccelX(), 4);
      Serial.print(" Y1 = ");
      Serial.println(myIMU.readFloatAccelY(), 4);
      Serial.print(" Z1 = ");
      Serial.println(myIMU.readFloatAccelZ(), 4);

      //Gyroscope
      Serial.print("\nGyroscope:\n");
      Serial.print(" X1 = ");
      Serial.println(myIMU.readFloatGyroX(), 4);
      Serial.print(" Y1 = ");
      Serial.println(myIMU.readFloatGyroY(), 4);
      Serial.print(" Z1 = ");
      Serial.println(myIMU.readFloatGyroZ(), 4);

      //Thermometer
      Serial.print("\nThermometer:\n");
      Serial.print(" Degrees C1 = ");
      Serial.println(myIMU.readTempC(), 4);
      Serial.print(" Degrees F1 = ");
      Serial.println(myIMU.readTempF(), 4);

      // Print out the battery voltage:
      Serial.print("\nBattery Voltage: ");
      Serial.print(batteryVoltage);
      Serial.println(" V");

      // Get microphone data
      float audio = mic();

      delay(500); // Delay in between reads for stability

      u8x8.setFont(u8x8_font_chroma48medium8_r); //try u8x8_font_px437wyse700a_2x2_r
      u8x8.setCursor(0, 0); // It will start printing from (0,0) location
      u8x8.print("BATTERY VOLTAGE");
      u8x8.setCursor(0, 1);
      u8x8.print(batteryVoltage);

      Time nowTime = pcf.getTime();//get current time

      //print current time
      Serial.println(" ");
      Serial.print(nowTime.day);
      Serial.print("/");
      Serial.print(nowTime.month);
      Serial.print("/");
      Serial.print(nowTime.year);
      Serial.print(" ");
      Serial.print(nowTime.hour);
      Serial.print(":");
      Serial.print(nowTime.minute);
      Serial.print(":");
      Serial.println(nowTime.second);
      Serial.println(" ");
      delay(500);
      
      // make a string for assembling the data to log:
      String dataString = "";

      dataString += String("BatteryVoltage: ") + String(batteryVoltage) + "\n";  // Battery voltage in format BatteryVoltage
      dataString += String("Accelometer X-axis: ") + String(myIMU.readFloatAccelX()) + "," + String("Accelometer Y-axis: ") + String(myIMU.readFloatAccelY()) + "," + String("Accelometer Z-axis: ") + String(myIMU.readFloatAccelZ()) + "\n";  // Accelerometer data in format AccelX,AccelY,AccelZ
      dataString += String("Gyroscope X- axis: ") + String(myIMU.readFloatGyroX()) + "," + String("Gyroscope Y- axis: ") + String(myIMU.readFloatGyroY()) + "," + String("Gyroscope z- axis: ") + String(myIMU.readFloatGyroZ()) + "\n";  // Gyroscope data in format GyroX,GyroY,GyroZ
      dataString += String("Tempreature: ") + String(myIMU.readTempC()) + String(" C") + "," + String(myIMU.readTempF()) + String(" F") + "\n";  // Temperature data in format TempC,Temp
      dataString += String("Microphone: ") + String(audio) + "\n";
      dataString += String("Date: ") + String(nowTime.day) + "/" + String(nowTime.month) + "/" + String(nowTime.year) + "\n";  // RTC in format date (day/month/year)
      dataString += String("Time: ") + String(nowTime.hour) + ":" + String(nowTime.minute) + ":" + String(nowTime.second) + "\n\n";  // Time in format HH:MM:SS

      // read three sensors and append to the string:
      for (int analogPin = 0; analogPin < 3; analogPin++) {
        int sensor = analogRead(analogPin);
        dataString += String(sensor);
        if (analogPin < 2) {
          dataString += ",";
        }
      }

      // open the file. note that only one file can be open at a time,
      // so you have to close this one before opening another.
      File dataFile = SD.open("datalog.txt", FILE_WRITE);

      // if the file is available, write to it:
      if (dataFile) {
        dataFile.println(dataString);
        dataFile.close();
        // print to the serial port too:
        // Serial.println(dataString);
      }
      // if the file isn't open, pop up an error:
      else {
        Serial.println("error opening datalog.txt");
      }
    }
}

void onPDMdata() {
  // Query the number of available bytes
  int bytesAvailable = PDM.available();

  // Read into the sample buffer
  PDM.read(sampleBuffer, bytesAvailable);

  // 16-bit, 2 bytes per sample
  samplesRead = bytesAvailable / 2;
}
