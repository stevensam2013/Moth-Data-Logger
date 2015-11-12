



/* ------------------ Include Libraries --------------------
Include all libraries that the program uses here
*/

//GPS Libraries
#include <SoftwareSerial.h>
#include <Adafruit_GPS.h>

//MPU Libraries
#include <Wire.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include <I2Cdev.h>


/* --------------- Pre-Processor directives ----------------
The #define statements can be used instead of declaring variables
if the value will not change. This reduces the amount of program
memory used.
*/

//Define Pins
#define BUTTONX_PIN    2       //Set the pin that button X connects to

//GPS Definitions
#define GPS_BAUD       9600    //

//MPU6050 Definitions
#define PIN_INTERRUPT  0       //

//SD Card Definitions
#define LOG_INTERVAL   200     //

//Debug definitions
#define DEBUG          true     //
#define DEBUG_BAUD     115200  //

                           

/* ------------------- Global Variables --------------------
Variables created here will be accessible to all functions
throughout this program.
*/

//GPS Gloabals
Adafruit_GPS GPS(&Serial1);   //

//MPU6050 Globals
volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
MPU6050 mpu;


// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector



//SD Card Globals
uint32_t timer;               //



/* -------------------- Setup Function ---------------------
This function is run once at the start of the program and is
used to intialise values and setup resources, such as IO pins,
servos and serial ports
*/
void setup() {

  // --- Setup Debug ---
  
  //Serial debudgging
  #if DEBUG
    //Use the main serial port to output debug messages
    Serial.begin(DEBUG_BAUD);
    Serial.println("Adafruit GPS library basic test!");
  #endif
  
  //TODO: Setup status LED's

  
  // --- Setup GPS Module ---
  
  //Initialise serial GPS connection
  GPS.begin(GPS_BAUD);
  
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);
  
  // --- Setup MPU6050 ---

  mpu.initialize();
  
  #if DEBUG
    Serial.println(F("Initializing I2C devices..."));
    
    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));    
  #endif
  
  //TODO: Check connection is successful, continue if so, indicate if not!
  
  // load and configure the DMP
  #if DEBUG
    Serial.println(F("Initializing DMP..."));
  #endif
  
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
  
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
      // turn on the DMP, now that it's ready
      #if DEBUG
        Serial.println(F("Enabling DMP..."));
      #endif
      mpu.setDMPEnabled(true);

      // enable Arduino interrupt detection
      #if DEBUG
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
      #endif
      
      attachInterrupt(PIN_INTERRUPT, dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();

      // set our DMP Ready flag so the main loop() function knows it's okay to use it
      #if DEBUG
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
      #endif
      
      dmpReady = true;

      // get expected DMP packet size for later comparison
      packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      #if DEBUG
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
      #endif
  }
  
  // --- Setup Ultrasonic Module --- 
  
  
  // --- Setup SD card ---
   
  //Create new logging file

}


/* ------------------ Main Program Loop --------------------
This is the main loop of the program. After the setup function
has run, code in this function is run. when it has completed,
it will start again.
*/
void loop() {

    // --- GPS ---
  
    //Get all required data from the GPS object
    char cGPS = GPS.read();
    
    #if DEBUG
      if (cGPS) Serial.print(cGPS);
    #endif
    
    // if a sentence is received, we can check the checksum, parse it...
    if (GPS.newNMEAreceived()) {
      #if DEBUG
        Serial.println("GPS: Sentence Received");
      #endif
    
      if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
        return;  // we can fail to parse a sentence in which case we should just wait for another
        
      #if DEBUG
        gpsPrint();
      #endif
    }
    
    
    
    
    // --- Log to file ---
    // if millis() or timer wraps around, we'll just reset it
    if (timer > millis())  timer = millis();
  
    // approximately every 2 seconds or so, print out the current stats
    if (millis() - timer > LOG_INTERVAL) { 
      timer = millis(); // reset the timer
      
      Serial.println("TODO: Print to file");
      
    }
      
    
    
}

void gpsPrint()
{
  Serial.print("\nTime: ");
  Serial.print(GPS.hour, DEC); Serial.print(':');
  Serial.print(GPS.minute, DEC); Serial.print(':');
  Serial.print(GPS.seconds, DEC); Serial.print('.');
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
  }

}


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

void dmpDataReady() {
    mpuInterrupt = true;
}
