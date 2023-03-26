#include <Servo.h>

#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>
#include <Adafruit_GPS.h>

#define ACTIVATE_BT false
#define ACTIVATE_IMU false
#define ACTIVATE_GPS true
#define ACTIVATE_US false
#define ACTIVATE_LS false
#define ACTIVATE_SERVO false
#define ACTIVATE_MOVE false

#define GPSSerial Serial1
#define GPSECHO false

static const char* greeting = "Hello World!";

BLEService greetingService("180C");  // User defined service

BLEStringCharacteristic greetingCharacteristic("2A56",  // standard 16-bit characteristic UUID
    BLERead, 13); // remote clients will only be able to read this

Adafruit_GPS GPS(&GPSSerial);

Servo servo;

uint32_t timer = millis();

const int R_2 = PIN_A0;
const int R_1 = PIN_A1;
const int C_0 = PIN_A2;
const int L_1 = PIN_A3;
const int L_2 = PIN_A4;
const int trigPin = PIN_A5;
const int echoPin = PIN_A6;
const int usDriverPin = D8;
const int A1A = D6;
const int A2A = D7;
const int B1A = D4;
const int B2A = D5;

enum enDirection { LEFT, CENTER, RIGHT  };

int distance;
enDirection direction;

void setup() {
  
  Serial.begin(9600);    // initialize serial communication
  while (!Serial);

  if(ACTIVATE_BT) {
    pinMode(LED_BUILTIN, OUTPUT); // initialize the built-in LED pin

    if (!BLE.begin()) {   // initialize BLE
      Serial.println("starting BLE failed!");
      while (1);
    }

    BLE.setLocalName("Zin Robot");  // Set name for connection
    BLE.setAdvertisedService(greetingService); // Advertise service
    greetingService.addCharacteristic(greetingCharacteristic); // Add characteristic to service
    BLE.addService(greetingService); // Add service
    greetingCharacteristic.setValue(greeting); // Set greeting string

    BLE.advertise();  // Start advertising
    Serial.print("Peripheral device MAC: ");
    Serial.println(BLE.address());
    Serial.println("Waiting for connections...");
  }

  if(ACTIVATE_MOVE) {
    pinMode(A1A, OUTPUT);
    pinMode(A2A, OUTPUT);
    pinMode(B1A, OUTPUT);
    pinMode(B2A, OUTPUT);
  }

  if(ACTIVATE_IMU) {
    if (!IMU.begin()) {
      Serial.println("Failed to initialize IMU!");
      while (1);
    }
    Serial.print("Accelerometer sample rate = ");
    Serial.print(IMU.accelerationSampleRate());
    Serial.println("Hz");

    Serial.print("Gyroscope sample rate = ");
    Serial.print(IMU.gyroscopeSampleRate());
    Serial.println(" Hz");

    Serial.print("Magnetic field sample rate = ");
    Serial.print(IMU.magneticFieldSampleRate());
    Serial.println(" Hz");
  }

  if(ACTIVATE_US) {
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT); 
  }

  if(ACTIVATE_LS) {
    pinMode(R_2, INPUT); 
    pinMode(R_1, INPUT); 
    pinMode(C_0, INPUT); 
    pinMode(L_1, INPUT); 
    pinMode(L_2, INPUT);  
  }

  if(ACTIVATE_GPS) {
    // // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
    GPS.begin(9600);
    // // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    // // uncomment this line to turn on only the "minimum recommended" data
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
    // // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
    // // the parser doesn't care about other sentences at this time
    // // Set the update rate
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
    // // For the parsing code to work nicely and have time to sort thru the data, and
    // // print it out we don't suggest using anything higher than 1 Hz

    // // Request updates on antenna status, comment out to keep quiet
    // GPS.sendCommand(PGCMD_ANTENNA);

    // delay(1000);
    // // Ask for firmware version
    // GPSSerial.println(PMTK_Q_RELEASE);

  }

  if(ACTIVATE_SERVO) {
    servo.attach(usDriverPin);
  }
}

void loop() {
  
  // Bluetooth module
  if(ACTIVATE_BT) {
    BTManager();    
  }

  // IMU module
  if(ACTIVATE_IMU) {
    IMUManager();
  }

  // Ultrasonic module
  if(ACTIVATE_US) {
    UltrasonicManager();
  }

  // Line Tracing
  if(ACTIVATE_LS) {
    LineTracerManager();
  }

  // GPS Module
  if(ACTIVATE_GPS) {
    GPSManager();
  }

  // Ultrasonic controller
  if(ACTIVATE_SERVO) {
    SERVOManager();
  }

  // Motor for moving
  if(ACTIVATE_MOVE) {
    MOVEManager();
  }
  // delay(200);
}

void MOVEManager() {
  MoveForward();
  delay(500);
  Stop();
  delay(1000);
  MoveBackward();
  delay(500);
}

void MoveForward() {          //function of forward 
  analogWrite(A1A, 255);
  analogWrite(A2A, 0);
  analogWrite(B1A, 255);
  analogWrite(B2A, 0);
}

void MoveBackward() {         //function of backward
  analogWrite(A1A, 0);
  analogWrite(A2A, 210);
  analogWrite(B1A, 0);
  analogWrite(B2A, 210);
}

void Stop() {              //function of stop
  digitalWrite(A1A, LOW);
  digitalWrite(A2A, LOW);
  digitalWrite(B1A, LOW);
  digitalWrite(B2A, LOW);
}

void SERVOManager() {
  Serial.println("0");
  
  // delay(2000);
  // Serial.println("45");
  // servo.write(45);
  // delay(2000);
  // Serial.println("90");
  // servo.write(90);
  // delay(2000);
  if(distance < 10) {
    if (direction == CENTER) {
      int randNumber = random(0, 2);
      if (randNumber == 0) {
        LookLeft();
      } else {
        LookRight();
      }
      delay(2000);
    }
  } else {
    LookCenter();    
  }
}

void LookLeft() {
  servo.write(0);
  direction = LEFT;
}

void LookCenter() {
  servo.write(45);
  direction = CENTER;
}

void LookRight() {
  servo.write(90);
  direction = RIGHT;
}


void GPSManager() {
  // // read data from the GPS in the 'main loop'
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  // if (GPSECHO)
  //   if (c) Serial.print(c);
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
    // Serial.print(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) {
    
    timer = millis(); // reset the timer
    Serial.print("\nTime: ");
    if (GPS.hour < 10) { Serial.print('0'); }
    Serial.print(GPS.hour, DEC); Serial.print(':');
    if (GPS.minute < 10) { Serial.print('0'); }
    Serial.print(GPS.minute, DEC); Serial.print(':');
    if (GPS.seconds < 10) { Serial.print('0'); }
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    if (GPS.milliseconds < 10) {
      Serial.print("00");
    } else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
      Serial.print("0");
    }
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
    if (GPS.fix) {
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", ");
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
      Serial.print("Antenna status: "); Serial.println((int)GPS.antenna);
    }
  } 

}

void LineTracerManager() {
  int r2, r1, c0, l1, l2;

  r2 = digitalRead(R_2);
  r1 = digitalRead(R_1);
  c0 = digitalRead(C_0);
  l1 = digitalRead(L_1);
  l2 = digitalRead(L_2);

  Serial.print(l2);
  Serial.print('\t');
  Serial.print(l1);
  Serial.print('\t');
  Serial.print(c0);
  Serial.print('\t');
  Serial.print(r1);
  Serial.print('\t');
  Serial.println(r2);
}

void BTManager() {

  BLEDevice central = BLE.central();  // Wait for a BLE central to connect

  // if a central is connected to the peripheral:
  if (central) {
    Serial.print("Connected to central MAC: ");
    // print the central's BT address:
    Serial.println(central.address());
    // turn on the LED to indicate the connection:
    digitalWrite(LED_BUILTIN, HIGH);

    while (central.connected()){

    } // keep looping while connected
    
    // when the central disconnects, turn off the LED:
    digitalWrite(LED_BUILTIN, LOW);
    Serial.print("Disconnected from central MAC: ");
    Serial.println(central.address());
  }
}

void UltrasonicManager() {
  long duration;

   // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2;
  // Prints the distance on the Serial Monitor
  Serial.print("Distance: ");
  Serial.println(distance);
}

void IMUManager() {
  int degreesX = 0;
  int degreesY = 0;
  float ax, ay, az, gx, gy, gz, mx, my, mz;  

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(ax, ay, az);
    // Serial.print("ACC : ");
    // Serial.print(ax);
    // Serial.print('\t');
    // Serial.print(ay);
    // Serial.print('\t');
    // Serial.println(az);
  }

  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gx, gy, gz);
    // Serial.print("GYR : ");
    // Serial.print(gx);
    // Serial.print('\t');
    // Serial.print(gy);
    // Serial.print('\t');
    // Serial.println(gz);
  }

  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(mx, my, mz);
    // Serial.print("MAG : ");
    // Serial.print(mx);
    // Serial.print('\t');
    // Serial.print(my);
    // Serial.print('\t');
    // Serial.println(mz);
  }

  if (ax > 0.1) {
    ax = 100 * ax;
    degreesX = map(ax, 0, 97, 0, 90);
    Serial.print("Tilting up ");
    Serial.print(degreesX);
    Serial.println("  degrees");
  }
  if (ax < -0.1) {
    ax = 100 * ax;
    degreesX = map(ax, 0, -100, 0, 90);
    Serial.print("Tilting down ");
    Serial.print(degreesX);
    Serial.println("  degrees");
  }
  if (ay > 0.1) {
    ay = 100 * ay;
    degreesY = map(ay, 0, 97, 0, 90);
    Serial.print("Tilting left ");
    Serial.print(degreesY);
    Serial.println("  degrees");
  }
  if (ay < -0.1) {
    ay = 100 * ay;
    degreesY = map(ay, 0, -100, 0, 90);
    Serial.print("Tilting right ");
    Serial.print(degreesY);
    Serial.println("  degrees");
  }

}