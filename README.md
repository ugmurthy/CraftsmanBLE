## CRAFTSMANBLE 

CraftsMan BLE is designed to send accelerometer&Gyroscope data over BLE and serial port toany BLE Client and/or computer with a serial port. 

Systems requirements:
1. M5 Stick-C Plus (the device that will send the data) as BLE Server
2. BLE Client a Mobile phone using **Bluefruit LE Connect** on iOS or Android
3. and a Laptop with Arduino IDE 

### Installation
1. Download the CraftsmanBLE.ino file to your Arduino sketches directory
2. Compile it (Make necessary changes in case you are using a M5-Stick instead of a M5 Stick C plus. Note the only changes needed with be the Button numbers for Button A,B, IMU initialisation and IMU read commands, and the LCD print commands.

### Usage
1. Reset the M5-StickC plus
2. Connect Mobile to "Kosha Craftsman" servie using the BLE software
3. Hit Connect
4. Click UART under modules section
5. On the M5-StickC Plus - Click Button A to start reading IMU  readings
6. After a few seconds click Button A to stop the accelerometer readings.
7. M5 Screen will show the number of readings taken(see last line)
8. Click button B - to dump the readings in CSV format to BLE Client device
9. Click "Settings" icon, Export, Copy as txt.

### Libraries used
1. M5 Stick C plus
2. ESP32 Arduino BLE

### Other info:
1. CraftsmanBLE is set to take readings every 100 ms. you can change that by changing delta_T
2. The current buffer size can accomodate a max of 3000 accelerometer readings, if the number of readings exceed 3000 then the buffer pointer will fold back to 0 and start writing from top of buffer. Acts like a ring buffer.
3. Sorting the data on seqno will provide you the right order of readings even if the buffer has folded.
4. a 10ms delay is inserted while writing to BLE client- this is ensure BLE server is not overwhelmed.

