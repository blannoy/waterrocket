#include <EEPROM.h>
#include <Wire.h>
#include <Servo.h>
#include <SFE_BMP180.h>
#include <Fat16.h>
#include <Fat16util.h> // use functions to print strings from flash memory

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
void Compass_Heading();
void Normalize(void);
void Drift_correction(void);
void Matrix_update(void);
void Euler_angles(void);
void I2C_Init();
void Gyro_Init();
void Read_Gyro();
void Accel_Init();
void Read_Accel();
void Compass_Init();
void Read_Compass();
void Matrix_Multiply(float a[3][3], float b[3][3],float mat[3][3]);
float Vector_Dot_Product(float vector1[3],float vector2[3]);
void Vector_Cross_Product(float vectorOut[3], float v1[3],float v2[3]);
void Vector_Scale(float vectorOut[3],float vectorIn[3], float scale2);
void Vector_Add(float vectorOut[3],float vectorIn1[3], float vectorIn2[3]);
int handle_button(int buttonNr);
void flashError(int signal);
double running_average(double);
void log_msg(const char* str);
void log_msg(const __FlashStringHelper* str);

// servo
Servo myservo;  // create servo object to control a servo
int pos = 0;
int setAngle=30;
int prevAngle=0;
#define SERVO_PIN 14

//button init: button to start calibration & measurement, button to detect breakwire
#define BUTTON_PIN 7
#define BREAKWIRE 8
//signaling LEDS
#define RED_LED 9
#define GREEN_LED 13
#define BATT_CHECK 15

//file output vars
#define FLUSH_LIMIT 1
int nr_entries=0;

byte buttons[2] = {BUTTON_PIN, BREAKWIRE};
boolean button_was_pressed[2] = {}; // previous state
int button_pressed_counter[2] = {}; // press running duration
enum { EV_NONE=0, EV_SHORTPRESS, EV_LONGPRESS };
#define LONGPRESS_LEN    15  // Min nr of loops for a long press

// timing stuff
unsigned long currentMillis = 0;
unsigned long previousMillis = 0;
unsigned long startLog = 0;
unsigned long chute_timer;

boolean chute_closed = true;
boolean flying = false;
boolean ready_flight = false;
boolean calibrated = false;
boolean doLogging = false;

const int calLoops = 60;

//variable to store reading
#define numReadings 5
double readings[numReadings];      // the readings 
byte index = 0;                  // the index of the current reading
double total = 0;                  // the running total
//int average = 0;                // the average
//int normHeight;
double sigma;

// initialisation data from EEPROM
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

// L3G4200D gyro: 2000 dps full scale
// 70 mdps/digit; 1 dps = 0.07
#define Gyro_Gain_X 0.07 //X axis Gyro gain
#define Gyro_Gain_Y 0.07 //Y axis Gyro gain
#define Gyro_Gain_Z 0.07 //Z axis Gyro gain
#define Gyro_Scaled_X(x) ((x)*ToRad(Gyro_Gain_X)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Y(x) ((x)*ToRad(Gyro_Gain_Y)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Z(x) ((x)*ToRad(Gyro_Gain_Z)) //Return the scaled ADC raw data of the gyro in radians for second

// LSM303 magnetometer calibration constants; use the Calibrate example from
// the Pololu LSM303 library to find the right values for your board
#define M_X_MIN -1302
#define M_Y_MIN -3784
#define M_Z_MIN 267
#define M_X_MAX -1208
#define M_Y_MAX -3720
#define M_Z_MAX 287

#define Kp_ROLLPITCH 0.02
#define Ki_ROLLPITCH 0.00002
#define Kp_YAW 1.2
#define Ki_YAW 0.00002

/*For debugging purposes*/
//OUTPUTMODE=1 will print the corrected data, 
//OUTPUTMODE=0 will print uncorrected data of the gyros (with drift)
#define OUTPUTMODE 0

//#define PRINT_DCM 0     //Will print the whole direction cosine matrix
//#define PRINT_ANALOGS 0 //Will print the analog raw data
//#define PRINT_EULER 1   //Will print the Euler angles Roll, Pitch and Yaw

float G_Dt=0.02;    // Integration time (DCM algorithm)  We will run the integration loop at 50Hz if possible

long timer=0;   //general purpuse timer
long timer_old;
long timer24=0; //Second timer used to print values 
int AN[6]; //array that stores the gyro and accelerometer data
int AN_OFFSET[6]={0,0,0,0,0,0}; //Array that stores the Offset of the sensors

int gyro_x;
int gyro_y;
int gyro_z;
int accel_x;
int accel_y;
int accel_z;
int magnetom_x;
int magnetom_y;
int magnetom_z;
float c_magnetom_x;
float c_magnetom_y;
float c_magnetom_z;
float MAG_Heading;

float Accel_Vector[3]= {0,0,0}; //Store the acceleration in a vector
float Gyro_Vector[3]= {0,0,0};//Store the gyros turn rate in a vector
float Omega_Vector[3]= {0,0,0}; //Corrected Gyro_Vector data
float Omega_P[3]= {0,0,0};//Omega Proportional correction
float Omega_I[3]= {0,0,0};//Omega Integrator
float Omega[3]= {0,0,0};

// Euler angles
float roll;
float pitch;
float yaw;

float errorRollPitch[3]= {0,0,0}; 
float errorYaw[3]= {0,0,0};

unsigned int counter=0;
byte gyro_sat=0;

float DCM_Matrix[3][3]= {
  {
    1,0,0  }
  ,{
    0,1,0  }
  ,{
    0,0,1  }
}; 
float Update_Matrix[3][3]={{0,1,2},{3,4,5},{6,7,8}}; //Gyros here


float Temporary_Matrix[3][3]={
  {
    0,0,0  }
  ,{
    0,0,0  }
  ,{
    0,0,0  }
};

// You will need to create an SFE_BMP180 object, here called "pressure":

SFE_BMP180 pressure;

double baseline; // baseline pressure
double P; 
double a;

void setup()
{
  Serial.begin(9600);

  I2C_Init();

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(BREAKWIRE, INPUT_PULLUP);
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, LOW); 

// read initialisation vars from beginning of EEPROM
  EEPROM.get(0, vars);
  
//MOD delay(1000);

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
  log_msg(F("---------------------------------\n"),false);
  log_msg(F("start\n"),true);

  // if button pressed on boot, setup delay timer
  if (!digitalRead(BUTTON_PIN)) {
      log_msg(F("Info: Button pressed on boot\n"),true);
      log_msg(F("Info: Configure delay\n"),true);
    
    vars.delay_counter = 0;
    digitalWrite(RED_LED, HIGH);
    //digitalWrite(GREEN_LED, HIGH);
    delay(2000);
    //digitalWrite(GREEN_LED, LOW);   // turn the LED on (HIGH is the voltage level)

    // until button is  pressed, flash green led. Every flash = .5 seconds delay
    while (digitalRead(BUTTON_PIN)) {
      delay(1000);
      digitalWrite(RED_LED, HIGH);   // turn the LED on (HIGH is the voltage level)
      delay(200);
      digitalWrite(RED_LED, LOW);   // turn the LED on (HIGH is the voltage level)
      vars.delay_counter += 500;
    }
    // write delay to flash
    digitalWrite(RED_LED, LOW);
    EEPROM.put(0,vars);
  }

  log_msg(F("Info: Delay counter is "),true);
  log_msg(vars.delay_counter);
  log_msg(F("\n"),false);

  digitalWrite(RED_LED, LOW);
  delay(100);
  Accel_Init();
  Compass_Init();
  Gyro_Init();
    if (pressure.begin())
      log_msg(F("Info: BMP180 init success\n"),true);
  else
  {
          log_msg(F("Error: BMP180 init fail (disconnected?)\n"),true);
    while(1){
          flashError(2);
      // Pause forever.
    }
  }

  if (vars.delay_counter == 0) {
    log_msg(F("Error: Chute delay is 0\n"),true);
    while (true) {
      flashError(3);
    }
  }
    // setup servo
  log_msg(F("Info: attach servo\n"),true);
  myservo.attach(SERVO_PIN);  // attaches the servo on pin 15 to the servo object
  myservo.write(pos);
  
  // check if breaking wire open
  while (digitalRead(BREAKWIRE)) {
    log_msg(F("Error: Breakwire is open\n"),true);
    delay(50);
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

  //test battery
  /*int battVal[5];
  float avg=0;
  for (int iloop=0;iloop < 5;iloop++){
    battVal[iloop] = analogRead(A1);
    delay(200);
  }
  for (int iloop=0;iloop < 5;iloop++){
    avg+=(float)battVal[iloop];
  }
  avg=avg/5.0;
  if (avg < 1019){
    log_msg("Low battery",1);
    log_msg(avg,1);
    flashError(6);
  }*/
 /* PgmPrint("Appending to: ");
  Serial.println(fname);*/
  
  // clear write error
//  file.writeError = false;

  calibrated = false;
  //delay(500);
  log_msg(F("Info: Ready\n"),true);
  /*digitalWrite(RED_LED, LOW);
  //delay(500);    
  digitalWrite(RED_LED, HIGH);
  delay(800);    
  digitalWrite(RED_LED, LOW);
  delay(500);    */
  digitalWrite(RED_LED, HIGH);
  //digitalWrite(GREEN_LED, HIGH);
}

void loop()
{
//  freeRam();
  // flash led every second to show its working
  if (currentMillis - previousMillis > 1000) {
    if (doLogging) {
      if (currentMillis - previousMillis < 1200) {
        digitalWrite(RED_LED, HIGH);   // turn the LED on (HIGH is the voltage level)
      } else if (currentMillis - previousMillis >= 1200) {
        digitalWrite(RED_LED, LOW);   // turn the LED on (HIGH is the voltage level)
      previousMillis = currentMillis;
      }
    }
    /*if (currentMillis - previousMillis < 1200) {
      digitalWrite(GREEN_LED, HIGH);   // turn the LED on (HIGH is the voltage level)
    } else if (currentMillis - previousMillis >= 1200) {
      digitalWrite(GREEN_LED, LOW);   // turn the LED on (HIGH is the voltage level)
      previousMillis = currentMillis;
    }*/
  }


  // handle button
  byte pushEvent = handle_button(0,true);
  byte breakEvent = handle_button(1,false);


  // button long pressed: recalibrate sensor and setup for flight
  if ((pushEvent == EV_LONGPRESS)&& (!ready_flight)) {
	  log_msg(F("Info: Get ready for flight\n"),true);
    digitalWrite(RED_LED, HIGH);
    delay(500);    
    digitalWrite(RED_LED, LOW);
    chute_closed = true;
    flying = false;
    // calibrate sensor
    if (!calibrated) {
      calibrated = setupSensor();
    }
    ready_flight=true;
    pushEvent=EV_NONE;
    digitalWrite(RED_LED, LOW);
    delay(200);
    digitalWrite(RED_LED, HIGH);
    //freeRam();
  }

  // button pushed for second time after flight setup, start logging until log segment full
  if ((pushEvent == EV_LONGPRESS) && (!doLogging) && (ready_flight)) {
    log_msg(F("Info: start logging\n"),true);
    startLog = millis();
    log_msg(F("Info: sigma is "),true);
    log_msg(sigma);
    log_msg(F("\n"),false);
    log_msg(F("t;roll;pitch;yaw;alt;accx;accy;accz\n"),false);
    
    doLogging = true;
 //   delay(500);
  }
  //      unsigned long timenow=millis();
  if (calibrated) {
      if((millis()-timer)>=20)  // Main loop runs at 50Hz
  {
    counter++;
    timer_old = timer;
    timer=millis();
    if (timer>timer_old)
      G_Dt = (timer-timer_old)/1000.0;    // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
    else
      G_Dt = 0;
    
    // *** DCM algorithm
    // Data aquisition
    Read_Gyro();   // This read gyro data
    Read_Accel();     // Read I2C accelerometer
    // Get a new pressure reading:
    P = getPressure();
    // Show the relative altitude difference between
    // the new reading and the baseline reading:
    a = running_average(pressure.altitude(P,baseline));
    
    if (counter > 5)  // Read compass data at 10Hz... (5 loop runs)
      {
      counter=0;
      Read_Compass();    // Read I2C magnetometer
      Compass_Heading(); // Calculate magnetic heading  
      }
    
    // Calculations...
    Matrix_update(); 
    Normalize();
    Drift_correction();
    Euler_angles();
    // ***

    //average = read();
  }
  }
// main loop to log data
  if (doLogging) {
    // log all data with timestamp
//    Serial.println(millis()-startLog);
printdata();
 // TODO: stop logging after certain period
    if (millis()-startLog >= 60000) {
      doLogging = false;
      if (!file.close()) log_msg(F("Error: close\n"),true);
      digitalWrite(RED_LED, LOW);
      log_msg(F("Info: stopped logging\n"),true);
      //      Serial.println(millis()-startLog);
    //reset vars
    }
  }

  // breaking wire broken, this means liftoff
  if ((breakEvent) && (!flying)) {
   log_msg(F("Info: Breakwire broken, we're flying\n"),true);
   flying = true;
    chute_timer = millis();
  }

  // chute_timer is finished
  currentMillis = millis();
// chute is closed & we're flying; check if we are paste counter delay; if so move servo
  if ((chute_closed) && (flying) && ((currentMillis - chute_timer) > vars.delay_counter)) {
    
    if (pos == 0) {
   log_msg(F("Info: Opening chute\n"),true);
      pos = setAngle;
      myservo.write(pos);
    }
    // after 2 seconds move servo back
    if ((pos == setAngle) && (currentMillis - chute_timer > (vars.delay_counter + 2000))) {
      pos = 0;
      myservo.write(pos);
      chute_closed = false;
      }

  }

  delay(5);
}

