# SeizureDetectionWearable
Seizure detection wearable firmware. Designed around a SAMG55J19 ARM M4 microcontroller.
Firmware written by Carlos Hernandez for Senior Design Group Project. (SJSU '15)
Group Members: Daniel Graham (App for iOS), Valentin Reyes (custom PCB design)

Prototype built using a SAMG55J19 Xplained Pro Evaluation board (coded on Atmel Studio). Components include:
* Muscle Sensor v3 by Advancer Technologies.
* Powerboost 500c power supply/charging breakout by Adafruit.
* Bluefruit LE Bluetooth LE nRF8001 breakout by Adafruit.
* ADXL345 accelerometer breakout by Adafruit.
* TinyDuino microcontroller by TinyCircuits (handles Bluetooth).

These are connected to a custom PCB that also includes:
* Pulse sensor circuit modified from Pulse Sensor Amped design by World Famous Electronics
* EDA sensor circuit modified from E-Health Sensor Platform v2 design by Libelium
* Piezo buzzer circuit for generating local alerts
* TWI (I2C) bus for communicating with TinyDuino
* SPI traces for easy connecting to accelerometer breakout
* Power circuit for providing negative supply to Muscle Sensor v3
* Power rails for all other breakout boards
* Custom header for easy plugging into Xplained Pro Evaluation board
 
All components are placed on a custom compression sleeve worn around the arm with washable, fabric electrodes.
iOS app code and custom PCB Eagle files available on request.
