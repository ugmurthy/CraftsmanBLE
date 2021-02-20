////////////////////////////////////
// Author: U G Murthy
// Date : 16-FEB-2021
// For : Kosha Designs
// Project : Craftsman
////////////////////////////////////
/*
 Video: https://www.youtube.com/watch?v=oCMOYS71NIU
    Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleNotify.cpp
    Ported to Arduino ESP32 by Evandro Copercini
 
*/
// #include <M5StickC.h>
// uncomment ^ here for M5StickC
// comment next line for M5SticlC
#include "M5StickCPlus.h"
#include "EasyButton.h"

// BLE Related
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

BLEServer *pServer = NULL;
BLECharacteristic * pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;
char message[100];

void dumpBLE(char* msg);

// BLE Related 

// find appropriate values for M5Stick for BUTTON A,B - see reverse
// of the device for numbers
#define BTN_A 37
#define BTN_B 39


EasyButton Button_A(BTN_A,40);
EasyButton Button_B(BTN_B,40);


int dest = 1;
int delta_T = 100;

// for future
float threshold = 1.2;
int scale = 10;
bool start_read = false;

float accX;
float accY;
float accZ;
float gyroX;
float gyroY;
float gyroZ;
// readings buffer
// 1+3+3=7 floats x 4 bytes x 4000 = 64k
float acc_buff[21000];
//float acc_buff[210];
int buf_size = sizeof(acc_buff);

long idx=0;
bool folded = false;
long seqno = 0;

// func declarations
void acc_init(int dest);
void acc_read(void);
void Display_readings(float X,float Y,float Z);
void on_B_Pressed();
void on_A_Pressed();

void acc_init(int dest) {
  // find appropriate line for initialising IMU for M5StickC
  M5.Imu.Init();
  if (dest==1) {
    Serial.println("AccX,AccY,AccZ,GyroX,GyroY,GyroZ");
    }
}


void acc_read(int seq,int dest) {

  //find appropriate line for reading X,Y,Z values from IMU
  static bool first = true;
  M5.Imu.getAccelData(&accX, &accY, &accZ);
  M5.Imu.getGyroData(&gyroX,&gyroY,&gyroZ);
  
  if (first) {
    first = false;
    sprintf(message,"{\"Device\":\"M5Craftsman\",\"period\":%d}\n",delta_T);
    Serial.print(message);
    sprintf(message,"SeqNo,AccX,AccY,AccZ,GyroX,GyroY,GyroZ\n");
    Serial.print(message);
  }

  if (dest == 1) {
    sprintf(message,"%04d, %+6.2f, %+6.2f, %+6.2f, %+6.2f, %+6.2f, %+6.2f\n",(int)seq,accX,accY,accZ,gyroX,gyroY,gyroZ);
    Serial.print(message);
    }
}

/// SCreen routines
void Display_readings(float X,float Y,float Z,float gX,float gY,float gZ,float seq){
  M5.Lcd.setCursor(40,30);
  M5.Lcd.printf("%+6.2f  ",X );
  M5.Lcd.setCursor(140,30);
  M5.Lcd.printf("%+6.2f  ",gX);
  
  M5.Lcd.setCursor(40,50);
  M5.Lcd.printf("%+6.2f  ",Y );
  M5.Lcd.setCursor(140,50);
  M5.Lcd.printf("%+6.2f  ",gY);
  M5.Lcd.setCursor(40,70);
  M5.Lcd.printf("%+6.2f  ",Z );
  M5.Lcd.setCursor(140,70);
  M5.Lcd.printf("%+6.2f  ",gZ);
  M5.Lcd.setCursor(70,90);
  M5.Lcd.printf("%04d",int(seq));
}

void on_A_Pressed() {
  Serial.println("Button A has been pressed! Toggle read");
  start_read = !start_read;
}

void on_B_Pressed() {
  // DUMP Data
  int seqno = 0;
  start_read = false;
  int upto=0;

  Serial.print("Size of Buff ");
  Serial.println(sizeof(acc_buff));
  Serial.print("folded? ");
  Serial.println(folded);
  Serial.print("idx ");
  Serial.println(idx);
  
  // Assess how much of the buffer is full of readings
  // check if we have folded
  if (folded) {
    upto = buf_size/4;
  } else {
    upto = idx ;
  }

  

  // Get a header out including json header
  sprintf(message,"{\"Device\":\"M5Craftsman\",\"period\":%d}\n",delta_T);
  dumpBLE(message);
  Serial.print(message);
  
  sprintf(message,"%s\n","SeqNo,AccX,AccY,AccZ,GyroX,GyroY,GyroZ");
  dumpBLE(message);
  Serial.print(message);
  // DUMP the buffer
  for (int i=0;i<upto;i=i+7) {
    seqno = (int) acc_buff[i];
    sprintf(message,"%04d, %+6.2f, %+6.2f, %+6.2f, %+6.2f, %+6.2f, %+6.2f\n",
        (int)seqno,
        acc_buff[i+1]*scale,acc_buff[i+2]*scale,acc_buff[i+3]*scale,
        acc_buff[i+4],acc_buff[i+5],acc_buff[i+6]);
    dumpBLE(message);
    
    Serial.print(message);
    // Lets not tax the BLE - give it some rest!
    
  
      delay(10);  
    }
}
// a9a778fe-c8d9-4f73-af1d-f4e407e9a78c
// BLE related classes and call backs
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

// on connect/disconnect callbacks
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println("BLE Connected");
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println("BLE Disconnected");
    }
};

// for future - not used now.
// for control purposes.
class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();
      std::string subsRx;
      
      if (rxValue.length() > 0) {
        
        Serial.print("Received Value: ");
        Serial.print(rxValue.c_str());
        subsRx = rxValue.substr(0,4);
        Serial.print(" Substring :");
        Serial.println(subsRx.c_str());
      }
    }
};
// BLE related classes and call backs - ENDS here

//
void setup() {
  
  
  Serial.begin(115200);
  Button_A.begin();
  Button_B.begin();

  Button_A.onPressed(on_A_Pressed);
  Button_B.onPressed(on_B_Pressed);
  

  M5.begin();
  M5.Lcd.setRotation(3);
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(10, 5);
  M5.Lcd.println("IMU Readings");
  M5.Lcd.setCursor(15,30);
  M5.Lcd.println("X");
  M5.Lcd.setCursor(15,50);
  M5.Lcd.println("Y");
  M5.Lcd.setCursor(15,70);
  M5.Lcd.println("Z");
  M5.Lcd.setCursor(15,90);
  M5.Lcd.println("Seq");
  Serial.print("Delta T     = ");
  Serial.println(delta_T);
  Serial.print("Threshold   = ");
  Serial.println(threshold);
  Serial.print("Destination = ");
  Serial.println(dest);
  Serial.print("BufferSz/4  = ");
  Serial.println(sizeof(acc_buff)/4);
  
  // Initialise Acceleromenter 1 means serial port
  acc_init(1);
  // initialise buf_ptr


  /// SETUP BLE SERVER
  // Create the BLE Device
  BLEDevice::init("Kosha CraftsMan");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pTxCharacteristic = pService->createCharacteristic(
                    CHARACTERISTIC_UUID_TX,
                    BLECharacteristic::PROPERTY_NOTIFY
                  );
                      
  pTxCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic * pRxCharacteristic = pService->createCharacteristic(
                       CHARACTERISTIC_UUID_RX,
                      BLECharacteristic::PROPERTY_WRITE
                    );

  pRxCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();
  Serial.println("Waiting for a client connection to send Acc data...");
  
  
}

/// SEND DATA via BLE
void dumpBLE(char* msg) {
  if (deviceConnected) {
        pTxCharacteristic->setValue((uint8_t*)msg, strlen(msg));
        pTxCharacteristic->notify();
  }
}

void loop() {
  Button_A.read();
  Button_B.read();
  
  if (start_read) {
    acc_read(seqno,dest);
    acc_buff[idx]=float(seqno);
    acc_buff[idx+1]=accX;
    acc_buff[idx+2]=accY;
    acc_buff[idx+3]=accZ;
    acc_buff[idx+4]=gyroX;
    acc_buff[idx+5]=gyroY;
    acc_buff[idx+6]=gyroZ;
    Display_readings(accX,accY,accZ,gyroX,gyroY,gyroZ,float(seqno)); 
    seqno++;   
    idx += 7; 

    if (idx >= buf_size/4) {
       // foldit 
      folded = true;
      idx = 0 ;
    }  

    delay(delta_T);
  }

  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
        delay(500); // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
        Serial.println("start advertising");
        oldDeviceConnected = deviceConnected;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected) {
  // do stuff here on connecting
      oldDeviceConnected = deviceConnected;
  }
  
}
