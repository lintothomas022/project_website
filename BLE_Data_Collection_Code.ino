#include "LSM6DS3.h"
#include "Wire.h"
#include <Arduino.h>
#include <U8x8lib.h>  //Display library
#include <PCF8563.h>
#include <SPI.h> 
#include <SD.h>
#include <ArduinoBLE.h>

BLEService ledService("19B10000-E8F2-537E-4F6C-D104768A1214"); // Bluetooth® Low Energy LED Service

// Bluetooth® Low Energy LED Switch Characteristic - custom 128-bit UUID, read and writable by central
BLEByteCharacteristic switchCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);

const int ledPin = LED_BUILTIN; // pin to use for the LED

PCF8563 pcf;

U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* clock=*/PIN_WIRE_SCL, /* data=*/PIN_WIRE_SDA, /* reset=*/U8X8_PIN_NONE);  // OLEDs without Reset of the Display

//Create a instance of class LSM6DS3
LSM6DS3 myIMU(I2C_MODE, 0x6A);  //I2C device address 0x6A

const int chipSelect = 2;
unsigned long startTime;
unsigned long endTime;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  u8x8.begin();
  pinMode(LED_BUILTIN, OUTPUT);
  // begin initialization
  if (!BLE.begin()) {
    Serial.println("starting Bluetooth® Low Energy module failed!");

    while (1);
  }

  // set advertised local name and service UUID:
  BLE.setLocalName("James_Linto");
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
  

  u8x8.setFlipMode(1);  // set number from 1 to 3, the screen word will rotary 180
                        //  while (!Serial);
  //Call .begin() to configure the IMUs
  if (myIMU.begin() != 0) {
    Serial.println("Device error");
  } else {
    Serial.println("Device OK!");
  }
  Wire.begin();
  pcf.init();        //initialize the clock

  pcf.stopClock();//start the clock

  pcf.setYear(25);//set year
  pcf.setMonth(2);//set month
  pcf.setDay(25);//set dat
  pcf.setHour(20);//set hour
  pcf.setMinut(0);//set minut
  pcf.setSecond(0);//set second

  pcf.startClock();//start the clock
 
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    while (1)
      ;
  }

 
  // Setting up the headers for the stored data!
  File dataFile = SD.open("datalog.txt", FILE_WRITE);
    dataFile.println("\n");
    dataFile.print("Time");
    dataFile.print(", ");
    dataFile.print("Accel_x");
    dataFile.print(", ");
    dataFile.print("Accel_y");
    dataFile.print(", ");
    dataFile.print("Accel_z");
    dataFile.print(", ");
    dataFile.print("Gyr_x");
    dataFile.print(", ");
    dataFile.print("Gyr_y");
    dataFile.print(", ");
    dataFile.print("Gyr_z");
    dataFile.print(", ");
    dataFile.println("Label");
  // You can add the label here just make sure to only use "println" for the last thing you want to print!
    dataFile.close();
}
void loop() {
  // Listen for Bluetooth® Low Energy peripherals to connect:
  BLEDevice central = BLE.central();

  // If a central is connected to peripheral:
  if (central) {
    Serial.print("Connected to central: ");
    // Print the central's MAC address:
    Serial.println(central.address());

    bool collectingData = false; // Data collection flag

    // While the central is still connected to peripheral:
    while (central.connected()) {
      // If the remote device wrote to the characteristic,
      // use the value to control the data collection:
      if (switchCharacteristic.written()) {
        if (switchCharacteristic.value() == 0) {   // If value is 0, start collecting data
          Serial.println("Data collection started");
          collectingData = true;
        } else if (switchCharacteristic.value() == 1) {  // If value is 1, stop collecting data
          Serial.println("Data collection stopped");
            File dataFile = SD.open("datalog.txt", FILE_WRITE);
            dataFile.println("\n");
            dataFile.print("Time");
            dataFile.print(", ");
            dataFile.print("Accel_x");
            dataFile.print(", ");
            dataFile.print("Accel_y");
            dataFile.print(", ");
            dataFile.print("Accel_z");
            dataFile.print(", ");
            dataFile.print("Gyr_x");
            dataFile.print(", ");
            dataFile.print("Gyr_y");
            dataFile.print(", ");
            dataFile.print("Gyr_z");
            dataFile.print(", ");
            dataFile.println("Label");
  // You can add the label here just make sure to only use "println" for the last thing you want to print!
            dataFile.close();
          collectingData = false;
        }
      }

      // Perform data collection if the flag is set
      if (collectingData) {
        Time nowTime = pcf.getTime();  // Get current time
        u8x8.setFont(u8x8_font_chroma48medium8_r);
        u8x8.clear();
        u8x8.setCursor(0, 0);
        u8x8.print("Data Collection");
        digitalWrite(LED_BUILTIN, HIGH);
        
        float accelxDataArray[1000];
        float accelyDataArray[1000];
        float accelzDataArray[1000];
        float gyrxDataArray[1000];
        float gyryDataArray[1000];
        float gyrzDataArray[1000];

        for (int i = 0; i < 1000; i++) {
          accelxDataArray[i] = myIMU.readFloatAccelX();
          accelyDataArray[i] = myIMU.readFloatAccelY();
          accelzDataArray[i] = myIMU.readFloatAccelZ();
          gyrxDataArray[i] = myIMU.readFloatGyroX();
          gyryDataArray[i] = myIMU.readFloatGyroY();
          gyrzDataArray[i] = myIMU.readFloatGyroZ();
          delay(7); // To ensure 10 seconds of data in each batch
        }

        File dataFile = SD.open("datalog.txt", FILE_WRITE);
        if (dataFile) {
          for (int j = 0; j < 1000; j++) {
            dataFile.print(nowTime.hour);
            dataFile.print(":");
            dataFile.print(nowTime.minute);
            dataFile.print(":");
            dataFile.print(nowTime.second);
            dataFile.print(", ");
            dataFile.print(accelxDataArray[j]);
            dataFile.print(", ");
            dataFile.print(accelyDataArray[j]);
            dataFile.print(", ");
            dataFile.print(accelzDataArray[j]);
            dataFile.print(", ");
            dataFile.print(gyrxDataArray[j]);
            dataFile.print(", ");
            dataFile.print(gyryDataArray[j]);
            dataFile.println(gyrzDataArray[j]);
          }
          dataFile.close();
          Serial.println("Batch recorded");
          digitalWrite(LED_BUILTIN, LOW); // Blink LED after recording
        }
      }
    }

    // When the central disconnects, print it out:
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }
}