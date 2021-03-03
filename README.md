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

10. Long Press A to restart device.

### Version 2.0 Usage instructions
Version 2.0 allows control of M5 from the BLE Client device as follows
- All commands are single letter command (case insensitive)
- An optional integer (max 3 digits) follows the command

|S.No|Commands|Argument|Remarks|
|:---: | :---: | :---:|:---:|
|1|S|None|Start reading IMU|
|2|S|None|Stop reading IMU|
|3|R|None|Restart M5|
|4|P|nnn|Set period to nnn millisecs|
|5|O|nnn|Set offset to nnn millisecs|


introduced a way to control M5 from the mobile phone 


### Libraries used
1. M5 Stick C plus
2. ESP32 Arduino BLE
3. Easybutton

### Other info:
1. CraftsmanBLE is set to take readings every 100 ms. you can change that by changing delta_T
2. The current buffer size can accomodate a max of 3000 accelerometer readings, if the number of readings exceed 3000 then the buffer pointer will fold back to 0 and start writing from top of buffer. Acts like a ring buffer.
3. Sorting the data on seqno will provide you the right order of readings even if the buffer has folded.
4. a 10ms delay is inserted while writing to BLE client- this is ensure BLE server is not overwhelmed.

### Version History
 1.0 :17/Feb/21 Output consisted of 3-axis Accelerometer readings 
 1.1 :19/Feb/21 Output now includes 3-axis Accelerometer and Gyro readings
 1.5 :23/Feb/21 
     1. SeqNo in BLE output is replaced by secs.999 (starts at 0.000) 
     2. The interval time is very close to desired delta_T using delta_offset
     3. Long Press A button till beep to restart device
     4. If connected to physical serial port you can still read old seqNo along with Secs 
     5. Device has a unique name derived for chip id and prefixed by KoshaCraftsman

