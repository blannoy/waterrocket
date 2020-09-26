#include <EEPROM.h>
#include <Wire.h>
#include <Servo.h>
#include <SFE_BMP180.h>
#include <Fat16.h>
#include <Fat16util.h> // use functions to print strings from flash memory

#define VERSION 2.5

const short int CHIP_SELECT = 10;  // SD card chip select pin.
SdCard card;
Fat16 file;

// Uncomment the below line to use this axis definition: 
   // X axis pointing forward
   // Y axis pointing to the right 
   // and Z axis pointing down.
// Positive pitch : nose up
// Positive roll : right wing down
// Positive yaw : clockwise
int SENSOR_SIGN[9] = {1,1,1,-1,-1,-1,1,1,1}; //Correct directions x,y,z - gyro, accelerometer, magnetometer
// Uncomment the below line to use this axis definition: 
   // X axis pointing forward
   // Y axis pointing to the left 
   // and Z axis pointing up.
// Positive pitch : nose down
// Positive roll : right wing down
// Positive yaw : counterclockwise
//int SENSOR_SIGN[9] = {1,-1,-1,-1,1,1,1,-1,-1}; //Correct directions x,y,z - gyro, accelerometer, magnetometer

//prototypes
boolean setupSensor();
double getPressure();
void I2C_Init();
void Accel_Init();
void Read_Accel();
void Compass_Init();
void Read_Compass();
int handle_button(int buttonNr);
void flashError(int signal);
double running_average(double);
void log_msg(const char* str);
void log_msg(const __FlashStringHelper* str);
void readSensor();

// servo is used to release chute
Servo myservo;  // create servo object to control a servo
#define ZEROPOS 0
int pos = ZEROPOS;
int setAngle=45;
int prevAngle=ZEROPOS;
#define SERVO_PIN 14

//button init: button to start calibration & measurement, button circuit to detect breakwire (lift off)
#define BUTTON_PIN 6
#define BREAKWIRE 8

//signaling LEDS
#define RED_LED 9
//#define GREEN_LED 13

//file output vars
#define FLUSH_LIMIT 1
int nr_entries=0;

byte buttons[2] = {BUTTON_PIN, BREAKWIRE};
boolean button_was_pressed[2] = {}; // previous state
int button_pressed_counter[2] = {}; // press running duration
enum buttonEvents { EV_NONE=0, EV_SHORTPRESS, EV_LONGPRESS };
#define LONGPRESS_LEN    15  // Min nr of loops for a long press

// flight states
enum flightState {INIT, READY, LIFTOFF, FLYING, MAXHEIGHT, CHUTE, GROUND};
flightState state=INIT;


// timing stuff
unsigned long currentMillis = 0;
unsigned long previousMillis = 0;
unsigned long startLog = 0;
unsigned long chute_timer;
unsigned long state_timer;
unsigned int counter=0;

boolean chute_closed = true;
boolean flying = false;
boolean ready_flight = false;
boolean doLogging = false;

// number of calibration loops
const int calLoops = 60;

//variable to store sensor reading using running average
#define numReadings 5
double readings[numReadings];   // the readings 
byte index = 0;                 // the index of the current reading
double total = 0;               // the running total
double sigma;                   // error sigma 

// initialisation data from EEPROM
// delay counter is stored in EEPROM; after breakwire & set amount of milliseconds the chute is released
struct varStruct {
  int delay_counter;
};

varStruct vars;

//sensor stuff
// LSM303 accelerometer: 8 g sensitivity
// 3.9 mg/digit; 1 g = 256
#define GRAVITY 256  //this equivalent to 1G in the raw data coming from the accelerometer 

#define ToRad(x) ((x)*0.01745329252)  // *pi/180
#define ToDeg(x) ((x)*57.2957795131)  // *180/pi


long timer=0;   //general purpuse timer
long timer_old;
int AN[6]; //array that stores the gyro and accelerometer data
int AN_OFFSET[6]={0,0,0,0,0,0}; //Array that stores the Offset of the sensors

int accel_x;
int accel_y;
int accel_z;

float Accel_Vector[3]= {0,0,0}; //Store the acceleration in a vector
SFE_BMP180 pressure;

double baseline; // baseline pressure, calculated during calibration phase
double P; // current pressure/altitude
double a; // relative altitude
double maximumHeight=0;

void setup()
{
  Serial.begin(9600);

  I2C_Init();

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(RED_LED, OUTPUT);
  pinMode(BREAKWIRE, INPUT_PULLUP);
  digitalWrite(RED_LED, LOW);



// read initialisation vars from beginning of EEPROM
  EEPROM.get(0, vars);
  
// Init SD card and open file for logging
 // initialize the SD card
  if (!card.begin(CHIP_SELECT,SPI_HALF_SPEED)){
    log_msg(F("Error card.begin"),true);
    log_msg(card.errorCode);
    log_msg(F("\n"),false);
    flashError(5);
  }
  
  // initialize a FAT16 volume
  if (!Fat16::init(&card)){
    log_msg(F("Error: Fat16::init\n Error "),true);
    log_msg(card.errorCode);
    log_msg(F("\n"),false);
    flashError(5);
  }
  if (!file.open("LOG.TXT", O_CREAT | O_APPEND | O_WRITE)) {
    log_msg(F("Error: Cannot open file\n Error "),true);
    log_msg(card.errorCode);
    log_msg(F("\n"),false);
    flashError(6);
  }
  log_msg(F("Software version "),true);
  log_msg(VERSION);
  log_msg(F("\n"),false);
  log_msg(F("---------------------------------\n"),false);
  log_msg(F("start\n"),true);

  // if button is still pressed on boot, setup delay time
  // LED flashes for every 500ms increment 
  if (!digitalRead(BUTTON_PIN)) {
      log_msg(F("Info: Button pressed on boot\n"),true);
      log_msg(F("Info: Configure delay\n"),true);
    
    vars.delay_counter = 0;
    digitalWrite(RED_LED, HIGH);
    delay(2000);

    // until button is  pressed, flash led. Every flash = .5 seconds delay
    while (digitalRead(BUTTON_PIN)) {
      delay(1000);
      digitalWrite(RED_LED, HIGH);
      delay(200);
      digitalWrite(RED_LED, LOW);
      vars.delay_counter += 500;
    }
    digitalWrite(RED_LED, LOW);
    // write delay to EEPROM
    EEPROM.put(0,vars);
  }

  log_msg(F("Info: Delay counter is "),true);
  log_msg(vars.delay_counter);
  log_msg(F("\n"),false);

  digitalWrite(RED_LED, LOW);
  delay(100);
  
// initialize sensors
  Accel_Init();
    if (pressure.begin())
      log_msg(F("Info: BMP180 init success\n"),true);
  else
  {
          log_msg(F("Error: BMP180 init fail (disconnected?)\n"),true);
    while(true){
          flashError(2);
    }
  }

// check delay counter
  if (vars.delay_counter == 0) {
    log_msg(F("Error: Chute delay is 0\n"),true);
    while (true) {
      flashError(3);
    }
  }

// setup servo
  log_msg(F("Info: attach servo\n"),true);
  myservo.attach(SERVO_PIN);  // attaches the servo on pin 14 to the servo object
  myservo.write(pos);
  
  // check if breakwire open
  // if breaking wire is open on setup, the servo can be switched from position using button
  // closing the breakwire will continue the rest of the code 
  while (digitalRead(BREAKWIRE)) {
    log_msg(F("Error: Breakwire is open\n"),true);
    delay(50);
    // if 
    if (!digitalRead(BUTTON_PIN)){
      log_msg(F("Info: switch servo\n"),true);
      if (pos==setAngle){
        pos=prevAngle;
        prevAngle=setAngle;
      } else {
        prevAngle=pos;
        pos=setAngle;
      }
      myservo.write(pos);
    }
    delay(50);
    flashError(4);
  }

  log_msg(F("Info: Ready\n"),true);
  digitalWrite(RED_LED, HIGH);
}

void loop()
{
  // handle button
  byte pushEvent = handle_button(0,true);
  byte breakEvent = handle_button(1,false);

  // button long pressed
  if (pushEvent == EV_LONGPRESS){
  // first press: calibrate sensor and setup for flight
      if (!ready_flight) {
        setupFlight();
        state=READY;
        state_timer=millis();
      } else if (!doLogging){
  // second press after flight setup, start logging
        log_msg(F("Info: start logging\n"),true);
        startLog = millis();
        log_msg(F("Info: sigma is "),true);
        log_msg(sigma);
        log_msg(F("\n"),false);
        log_msg(F("t;roll;pitch;yaw;alt;accx;accy;accz\n"),false);    
        doLogging = true;
      }
      pushEvent=EV_NONE;
   }

  // breaking wire broken, this means liftoff
  if ((breakEvent) && (!flying)) {
   log_msg(F("Info: Breakwire broken, we're flying\n"),true);
   flying = true;
   chute_timer = millis();
  }

// take readings if sensor was calibrated
  if (ready_flight) {
    readSensor();
  }

  // detect launch: relative altitude > 6*sigma
  if ((state == READY) && (a > 6*sigma)){
      state=LIFTOFF;
      state_timer=millis();
      log_msg(F("Info: Liftoff detected\n"),true);
  }

  // liftoff code
  if (state == LIFTOFF){
      state=FLYING;
      state_timer=millis();
  }
  
  // detect maximum height
  if (state==FLYING){
    if (a>maximumHeight){
      maximumHeight=a;
    } else {
      if ((maximumHeight-a)>6*sigma){
        state=MAXHEIGHT;
        state_timer=millis();
        log_msg(F("Info: Maximum height reached"),true);        
        log_msg(maximumHeight);
        log_msg(F("\n"),false);                        
      }
    }
  }

  if (state==MAXHEIGHT){
// deploy chute .5 seconds after maximum height
    if (millis()-state_timer > 500){   
        state=CHUTE;
        log_msg(F("Info: Deploy chute\n"),true);        
        state_timer=millis();
    }
  }
  
  // detect touchdown
  if (state==CHUTE){
      if (a<6*sigma){
        state=GROUND;
        log_msg(F("Info: Ground level reached\n"),true);        
      }
  }

// log data
  if (doLogging) {
  // flash led every second to show its working
    if (currentMillis - previousMillis > 1000) {
      if (currentMillis - previousMillis < 1200) {
        digitalWrite(RED_LED, HIGH);   // turn the LED on (HIGH is the voltage level)
      } else if (currentMillis - previousMillis >= 1200) {
        digitalWrite(RED_LED, LOW);   // turn the LED on (HIGH is the voltage level)
      previousMillis = currentMillis;
      }
    }

  // log all data with timestamp
    printdata();
  // stop logging after certain period. Currently fixed to 60 seconds
    if (millis()-startLog >= 60000) {
      doLogging = false;
      if (!file.close()) log_msg(F("Error: close\n"),true);
      digitalWrite(RED_LED, LOW);
      log_msg(F("Info: stopped logging\n"),true);
    }
  }
//if ((state==MAXHEIGHT)|| ((currentMillis - chute_timer) > vars.delay_counter)){} // open chute
  currentMillis = millis();

// chute is closed & we're flying; check if we are paste counter delay; if so move servo
  if ((chute_closed) && (flying) && ((currentMillis - chute_timer) > vars.delay_counter)) {
    if (pos == ZEROPOS) {
      log_msg(F("Info: Opening chute\n"),true);
      pos = setAngle;
      myservo.write(pos);
    }
    // after 2 seconds move servo back
    if ((pos == setAngle) && (currentMillis - chute_timer > (vars.delay_counter + 2000))) {
      log_msg(F("Info: Closing servo\n"),true);
      pos = ZEROPOS;
      myservo.write(pos);
      chute_closed = false;
      }
  }

  delay(5);
}

void setupFlight(){
    log_msg(F("Info: Get ready for flight\n"),true);
    digitalWrite(RED_LED, HIGH);
    delay(500);    
    digitalWrite(RED_LED, LOW);
    
    chute_closed = true;
    flying = false;
    ready_flight = setupSensor();
    maximumHeight=a;
    
    digitalWrite(RED_LED, LOW);
    delay(200);
    digitalWrite(RED_LED, HIGH);
}
