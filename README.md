# ZinCarRobot

This script has been developed with the following hardware specification
- Arduino Nano BLE 33 (https://store.arduino.cc/products/arduino-nano-33-ble)
- Arduino Nano Expansion Shield (https://shopee.com.my/product/40459773/1352927096)
- GPS Sensor (GY-GEO6MV2)
- Ultrasonic Sensor (HC-SR04)
- Servo motor (SG09)
- 5 Channel Infrared Sensor (TCRT5000)
- DC Motor Driver (NL9100S)
- 2 Wheels and Dual Axis TT Gear Motor
- Battery Casing with 3.7V Rechargeable LI-ION 18650 4200mAH

Arduino Nano BLE 33 contains built-in devices such as
- IMU Sensor (LSM9DS1)
- BLE device

## Pre requisite

- Most of the sensors require 5V to run, hence, to get 5V running at the Arduino Nano BLE 33, please solder the pad at the 5V IO.
Reference (https://github.com/ostaquet/Arduino-Nano-33-IoT-Ultimate-Guide)

## Port Mapping (Sensor device <-> Arduino Nano BLE 33)

#### Ultrasonic Sensor (HC-SR04)

- GND     - GND
- Echo    - A6
- Trigger - A5
- VCC     - 5V

#### GPS Sensor (GY-GEO6MV2)

- GND - GND
- RX - TX
- TX - RX
- VCC  - 5V

#### Servo motor (SG09)

- V - 5V
- G - GND
- S - D8

#### 5 Channel Infrared Sensor (TCRT5000)

- GND - GND
- 5V - 5V
- OUT1 - A4
- OUT2 - A3
- OUT3 - A2
- OUT4 - A1
- OUT5 - A0

#### DC Motor Driver (NL9100S)

- GND - GND
- VCC - 5V
- A-1A - A6
- A-2A - A7
- B-1A - A4
- B-2A - A5

## Software 

You can enable flags to activate to the sensor

1. ACTIVATE_BT : Built-in Bluetooth module
2. ACTIVATE_IMU : Built-in IMU Sensor module
3. ACTIVATE_GPS : GPS Sensor module
4. ACTIVATE_US : Ultrasonic Sensor module
5. ACTIVATE_LS : Infrared Line Tracing Sensor module
6. ACTIVATE_SERVO : Servo Motor module
7. ACTIVATE_MOVE : DC Motor driver module


## Images

#### Left View
![20230327_061921](https://user-images.githubusercontent.com/20044455/227808593-3ac7af19-8b87-46fa-a30f-7c9042bd0d9b.jpg)

#### Right View
![20230327_061929](https://user-images.githubusercontent.com/20044455/227808600-b414852c-4690-47fc-abcb-b0191fd68026.jpg)

#### Bottom View
![20230327_061939](https://user-images.githubusercontent.com/20044455/227808601-46d6abd8-d635-4feb-8aa2-9ee00a939094.jpg)

#### Front View
![20230327_061915](https://user-images.githubusercontent.com/20044455/227808603-108cc230-02ff-4b12-81ea-592fb03134c3.jpg)
