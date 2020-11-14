#include <EEPROM.h>
#include <Wire.h>
#include <Servo.h>
#include <SFE_BMP180.h>
#include <Fat16.h>
#include <Fat16util.h> // use functions to print strings from flash memory

#define VERSION 4.0
#define BLE

#ifdef BLE
#include <TimeLib.h>
#include <NeoSWSerial.h>
NeoSWSerial Bluetooth( 4, 5 );
#define CMDLENGTH 4 // BLE command length = 3
#define VALLENGTH 13 // BLE value length = 10
#define FULLLENGTH 16 // BLE full string length = CMD+":"+VAL+";"+null => CMD+VAL+3
char rState[VALLENGTH];
long lastPlotTime=0;
long lastHeartBeat = 5000;
#endif



//#define DEBUGPRINT

#ifdef DEBUGPRINT
#define DEBUG_PRINT(x)  Serial.print(x)
#define DEBUG_PRINTLN(x)  Serial.println(x)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#endif

const short int CHIP_SELECT = 10;  // SD card chip select pin.
SdCard card;
Fat16 file;

// servo is used to release chute
Servo myservo;  // create servo object to control a servo
#define ZEROPOS 0
int pos = ZEROPOS;
int setAngle = 45;
int prevAngle = ZEROPOS;
#define SERVO_PIN 14

//button init: button to start calibration & measurement, button circuit to detect breakwire (lift off)
#define BUTTON_PIN 6
#define BREAKWIRE 8
byte buttons[2] = {BUTTON_PIN, BREAKWIRE};
boolean button_was_pressed[2] = {}; // previous state
int button_pressed_counter[2] = {}; // press running duration
enum buttonEvents { EV_NONE = 0, EV_SHORTPRESS, EV_LONGPRESS };
#define LONGPRESS_LEN    15  // Min nr of loops for a long press


//signaling LEDS
#define RED_LED 9
//#define GREEN_LED 13

//file output vars
#define FLUSH_LIMIT 1
int nr_entries = 0;

// flight states
enum flightStates {INIT, CALIBRATE, READY, LIFTOFF, FLYING, MAXHEIGHT, DEPLOYCHUTE, CHUTE, GROUND};
static const char* flightStates_string[] = {"INIT", "CALIB", "READY", "LIFTOFF", "FLYING", "MAXH", "DPLCHUTE", "CHUTE", "GRND"};
flightStates state = INIT;
enum logStates {OFF, STARTLOG, LOGGING};
static const char* logStates_string[] = {"OFF", "START", "LOG"};
logStates logState = OFF;
// state string to send to bluetooth app
// Breakwire 0=open; 1=closed
#define BreakwireState 0
// Servo 0=open; 1=closed
#define ServoState 1
// Calibrate 0=uncalibrated; 1=calibrating;2=calibrated
#define CalibrateState 2
// Logging 0=off; 1=Startlog; 2=logging
#define LoggingState 3
// Plotdata sent to device 0=off; 1=on
#define PlotState 4

char rocketState[5] = {'1', '1', '0', '0','0'};

// timing stuff
unsigned long currentMillis = 0;
unsigned long previousMillis = 0;
unsigned long startLog = 0;
unsigned long chute_timer;
unsigned long state_timer;
unsigned int counter = 0;

boolean chute_closed = true;
boolean flying = false;
boolean ready_flight = false;
boolean doLogging = false;

// number of calibration loops
const int calLoops = 60;

double sigma;                   // error sigma

// initialisation data from EEPROM
struct varStruct {
  int delay_counter = 3000; // after breakwire & set amount of milliseconds the chute is released
  long lastFlightDate = 0; // flight date if received via Bluetooth
  double sigma = 0; // sigma of height measurement
  long liftoffTime = 0;
  double maxHeight = 0.0; // maximum height reached (heighest point after 6 sigma drop)
  long maxHeightTime = 0; // time of maximum height
  long chuteTime = 0; // time of chute opening
  long groundTime = 0; // time when ground touched
  int gravity=1;
};

varStruct vars;

//sensor stuff
int GRAVITY = 1;
int SENSOR_SIGN[3] = { 1, -1, -1}; //Correct directions rocket is along x axis


long timer = 0; //general purpuse timer
long timer_old;
int AN[3]; //array that stores the accelerometer data
int AN_OFFSET[3] = {0, 0, 0}; //Array that stores the Offset of the sensors
int accel_x;
int accel_y;
int accel_z;

SFE_BMP180 pressure;

double baseline=0; // baseline pressure, calculated during calibration phase
double P; // current pressure/altitude
double maximumHeight = -5000;
#define numReadings 5
struct runningAverage {
  double readings[numReadings];
  byte index = 0;
  double total = 0;
  double currentValue = 0;
};

runningAverage heightAverage;
runningAverage accelAverage;
// test
boolean test = true;
// test

void setup()
{
  Serial.begin(9600);
  I2C_Init();
#ifdef BLE
  Bluetooth.begin(9600);
#endif

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(RED_LED, OUTPUT);
  pinMode(BREAKWIRE, INPUT_PULLUP);
  digitalWrite(RED_LED, LOW);



  // read initialisation vars from beginning of EEPROM
  EEPROM.get(0, vars);
  // set all vars to 0
  long lastFlightDate = 0; // flight date if received via Bluetooth
  double sigma = 0; // sigma of height measurement
  long liftoffTime = 0;
  double maxHeight = 0.0; // maximum height reached (heighest point after 6 sigma drop)
  long maxHeightTime = 0; // time of maximum height
  long chuteTime = 0; // time of chute opening
  long groundTime = 0; // time when ground touched
  int gravity=1;

  // Init SD card and open file for logging
  // initialize the SD card
  if (!card.begin(CHIP_SELECT, SPI_HALF_SPEED)) {
    log_msg(F("Error card.begin"), true);
    log_msg(card.errorCode);
    log_msg(F("\n"), false);
    flashError(5);
  } else {
    // initialize a FAT16 volume
    if (!Fat16::init(&card)) {
      log_msg(F("Error: Fat16::init\n Error "), true);
      log_msg(card.errorCode);
      log_msg(F("\n"), false);
      flashError(5);
    } else {
      if (!file.open("LOG.TXT", O_CREAT | O_APPEND | O_WRITE)) {
        log_msg(F("Error: Cannot open file\n Error "), true);
        log_msg(card.errorCode);
        log_msg(F("\n"), false);
        flashError(6);
      }
    }
  }
  log_msg(F("Software version "), true);
  log_msg(VERSION);
  log_msg(F("\n"), false);
  log_msg(F("---------------------------------\n"), false);
  log_msg(F("start\n"), true);

  // if button is still pressed on boot, setup delay time
  // LED flashes for every 500ms increment
  if (!digitalRead(BUTTON_PIN)) {
    log_msg(F("Info: Button pressed on boot\n"), true);
    log_msg(F("Info: Configure delay\n"), true);

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
    EEPROM.put(0, vars);
  }

  log_msg(F("Info: Delay counter is "), true);
  log_msg(vars.delay_counter);
  log_msg(F("\n"), false);

  digitalWrite(RED_LED, LOW);
  delay(100);

  // initialize sensors
  Accel_Init();
  if (pressure.begin())
    log_msg(F("Info: BMP180 init success\n"), true);
  else
  {
    log_msg(F("Error: BMP180 init fail (disconnected?)\n"), true);
    while (true) {
      flashError(2);
    }
  }

  // check delay counter
  if (vars.delay_counter == 0) {
    log_msg(F("Error: Chute delay is 0\n"), true);
    while (true) {
      flashError(3);
    }
  }

  // setup servo
  log_msg(F("Info: attach servo\n"), true);
  myservo.attach(SERVO_PIN);  // attaches the servo on pin 14 to the servo object
  myservo.write(pos);
  setRocketState(ServoState, '1');

  // check if breakwire open
  // if breaking wire is open on setup, the servo can be switched from position using button
  // closing the breakwire will continue the rest of the code
  while (!handle_button(1, false)) {
    log_msg(F("Error: Breakwire is open\n"), true);
    setRocketState(BreakwireState, '0');
    delay(50);
    if (!digitalRead(BUTTON_PIN)) {
      log_msg(F("Info: switch servo\n"), true);
      if (pos == setAngle) {
        pos = prevAngle;
        prevAngle = setAngle;
        setRocketState(ServoState, '1');
      } else {
        prevAngle = pos;
        pos = setAngle;
        setRocketState(ServoState, '0');
      }
      myservo.write(pos);
    }
    delay(50);
    flashError(4);
  }
  setRocketState(BreakwireState, '1');
  log_msg(F("Info: Ready\n"), true);
  digitalWrite(RED_LED, HIGH);
}

void loop()
{
#ifdef BLE
  bluetoothLoop();
  receiveCommandLoop();
#endif

  buttonLoop();
  // take readings if sensor was calibrated
  if ((state != INIT) && (state != CALIBRATE) && (baseline > 0)) {
    readSensor();
#ifdef BLE
    if ((rocketState[PlotState]=='1') && (millis()-lastPlotTime>250)){
      sendCommand("PLT", heightAverage.currentValue);
    lastPlotTime=millis();
    }
#endif
  }


  if (state == CALIBRATE) {
    setupFlight();
    state_timer = millis();
  }

  if (state == READY) {
    test = true;
  }

  // breaking wire broken, this means liftoff
  if (state == LIFTOFF) {
///    log_msg(F("Info: Breakwire broken, we're flying\n"), true);
    chute_timer = millis();
    vars.lastFlightDate = now();
    vars.liftoffTime=millis();
    vars.sigma = sigma;
    vars.gravity = GRAVITY;    
    EEPROM.put(0, vars);
    setRocketState(BreakwireState, '0');
    changeState(FLYING);
    state_timer = millis();
  }

  //test liftoff detection & max height detection (-> deploy)
  // detect launch: relative altitude > 6*sigma + acceleration jump
  if (((state == READY) || (state == LIFTOFF) || (state == FLYING)) && test && (heightAverage.currentValue > 6 * sigma) && (abs(accelAverage.currentValue) > 5.0)) {
    //state = LIFTOFF;
    //state_timer = millis();
    test = false;
    log_msg(F("Info: Liftoff detected: "), true);
    log_msg(heightAverage.currentValue);
    log_msg(F(" - "));
    log_msg(accelAverage.currentValue);
  }
  
  // detect maximum height 
  if ((state == FLYING)||(state == DEPLOYCHUTE)||(state == CHUTE)) {
    if (heightAverage.currentValue > maximumHeight) {
      maximumHeight = heightAverage.currentValue;
      vars.maxHeightTime = millis() - chute_timer;
      vars.maxHeight = maximumHeight;
    } else {
      if ((maximumHeight - heightAverage.currentValue) > 6 * sigma) {
       // changeState(MAXHEIGHT);
        EEPROM.put(0, vars);
        state_timer = millis();
///        log_msg(F("Info: Maximum height reached ; "), true);
///        log_msg(maximumHeight);
///        log_msg(F("\n"), false);
      }
    }
  }

  if (state == MAXHEIGHT) {
    //state = DEPLOYCHUTE;
    //state_timer = millis();
    //log_msg(F("Info: Deploy chute\n"), true);
  }
  // test

  if (state == DEPLOYCHUTE) {
    if (pos == ZEROPOS) {
     /// log_msg(F("Info: Opening chute\n"), true);
      vars.chuteTime = millis() - chute_timer;
      EEPROM.put(0, vars);
      pos = setAngle;
      myservo.write(pos);
      setRocketState(ServoState, '0');
    }
    // after 2 seconds move servo back
    if ((pos == setAngle) && (currentMillis - chute_timer > (vars.delay_counter + 2000))) {
///      log_msg(F("Info: Closing servo\n"), true);
      pos = ZEROPOS;
      myservo.write(pos);
      setRocketState(ServoState, '1');
      changeState(CHUTE);
    }
  }

  // detect touchdown
  if (state == CHUTE) {
    if (heightAverage.currentValue < 6 * sigma) {
      changeState(GROUND);
      vars.groundTime = millis() - chute_timer;   
  ///    log_msg(F("Info: Ground level reached\n"), true);
    }
  }

  // chute timer
  currentMillis = millis();
  if (((state == LIFTOFF) || (state == FLYING) || (state == MAXHEIGHT)) && ((currentMillis - chute_timer) > vars.delay_counter)) {
    changeState(DEPLOYCHUTE);
  }

  // Logging states
  if (logState == STARTLOG) {
    startLog = millis();
///    log_msg(F("Info: start logging\n"), true);
///    log_msg(F("Info: sigma is "), true);
///    log_msg(sigma);
///    log_msg(F("\n"), false);
log_msg(F("t;height;acc;accx;accy;accz\n"), false);
    changeLogState(LOGGING);
    setRocketState(LoggingState, '2');
  }

  if (logState == LOGGING) {
    // log data
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
    if (millis() - startLog >= 60000) {
      changeLogState(OFF);
      setRocketState(LoggingState, '0');
      // write all flight data
              //itoa(vars.delay_counter, buff, 10);
       log_msg(F("Info: flight data\n"), true);
       log_msg(F("delay;date;sigma;gravity;liftoffTime;maxHeight;maxHeightTime;chuteTime;groundTime\n"));
       log_msg(vars.delay_counter);
       log_msg(F(";"));
       log_msg(vars.lastFlightDate);
       log_msg(F(";"));
       log_msg(vars.sigma);
       log_msg(F(";"));
       log_msg(vars.gravity);
       log_msg(F(";"));
       log_msg(vars.liftoffTime);
       log_msg(F(";"));
       log_msg(vars.maxHeight);
       log_msg(F(";"));
       log_msg(vars.maxHeightTime);
       log_msg(F(";"));
       log_msg(vars.chuteTime);
       log_msg(F(";"));
       log_msg(vars.groundTime);
       log_msg(F("\n"));
       // write flight data tot EEPROM
       EEPROM.put(0, vars);
      if (!file.close()) log_msg(F("Error: close\n"), true);
      digitalWrite(RED_LED, LOW);
///      log_msg(F("Info: stopped logging\n"), true);
    }
  }
  delay(10);
}

void setupFlight() {
///  log_msg(F("Info: Get ready for flight\n"), true);
  digitalWrite(RED_LED, HIGH);
  delay(500);
  digitalWrite(RED_LED, LOW);

  chute_closed = true;
  // flying = false;
  boolean ready_flight = setupSensor();
  if (ready_flight) {
    changeState(READY);
    // force heartbeat timer
    lastHeartBeat = millis();
    setRocketState(CalibrateState, '2');
  }
  maximumHeight = -5000;

  digitalWrite(RED_LED, LOW);
  delay(200);
  digitalWrite(RED_LED, HIGH);
}
