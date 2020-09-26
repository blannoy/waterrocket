#include <Wire.h>
#include <Servo.h>
#include <SFE_BMP180.h>

//signaling LEDS
#define RED_LED 9

//button init: button to start calibration & measurement, button circuit to detect breakwire (lift off)
#define BUTTON_PIN 7
#define BREAKWIRE 8

byte buttons[2] = {BUTTON_PIN, BREAKWIRE};
boolean button_was_pressed[2] = {}; // previous state
int button_pressed_counter[2] = {}; // press running duration
enum buttonEvents { EV_NONE=0, EV_SHORTPRESS, EV_LONGPRESS };
#define LONGPRESS_LEN    15  // Min nr of loops for a long press


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
void readSensor();

// flight states
enum flightState {INIT, READY, LIFTOFF, FLYING, MAXHEIGHT, CHUTE, GROUND};
flightState state=INIT;


// timing stuff
unsigned long currentMillis = 0;
unsigned long previousMillis = 0;
unsigned long startLog = 0;
unsigned long chute_timer;
unsigned long state_timer;


// number of calibration loops
const int calLoops = 60;

//variable to store sensor reading using running average
#define numReadings 5
double readings[numReadings];   // the readings 
byte index = 0;                 // the index of the current reading
double total = 0;               // the running total
double sigma;                   // error sigma 

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
#define M_X_MIN -1889
#define M_Y_MIN -3385
#define M_Z_MIN -212
#define M_X_MAX -1782
#define M_Y_MAX -3318
#define M_Z_MAX -189

#define Kp_ROLLPITCH 0.02
#define Ki_ROLLPITCH 0.00002
#define Kp_YAW 1.2
#define Ki_YAW 0.00002

/*For debugging purposes*/
//OUTPUTMODE=1 will print the corrected data, 
//OUTPUTMODE=0 will print uncorrected data of the gyros (with drift)
#define OUTPUTMODE 1

//#define PRINT_DCM 0     //Will print the whole direction cosine matrix
//#define PRINT_ANALOGS 0 //Will print the analog raw data
//#define PRINT_EULER 1   //Will print the Euler angles Roll, Pitch and Yaw

float G_Dt=0.02;    // Integration time (DCM algorithm)  We will run the integration loop at 50Hz if possible

long timer=0;   //general purpuse timer
long timer_old;
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
SFE_BMP180 pressure;

double baseline; // baseline pressure, calculated during calibration phase
double P; // current pressure/altitude
double a; // relative altitude
double maximumHeight=0;

void setup()
{
  Serial.begin(9600);

  I2C_Init();
 
// initialize sensors
  Accel_Init();
  Compass_Init();
  Gyro_Init();
    if (pressure.begin())
      log_msg(F("Info: BMP180 init success\n"),true);
  else
  {
          log_msg(F("Error: BMP180 init fail (disconnected?)\n"),true);
    while(true){
          flashError(2);
    }
  }
  setupSensor();

  log_msg(F("Info: Ready\n"),true);
}

void loop()
{
  readSensor();

  Serial.print("!ANG:");
  Serial.print(ToDeg(roll));
  Serial.print(F(","));
  Serial.print(ToDeg(pitch));
  Serial.print(F(","));
  Serial.print(ToDeg(yaw));
  Serial.print(F(","));
  Serial.print(F("acc:"));
  Serial.print(accel_x/256.0);
  Serial.print(F(","));
  Serial.print(accel_y/256.0);
  Serial.print(F(","));
  Serial.print(accel_z/256.0);
  Serial.println();   
}


