#include <L3G.h>
#include <LSM303.h>

LSM303 compass;


void I2C_Init()
{
  Wire.begin();
}

void Accel_Init()
{
  compass.init();
  compass.enableDefault();
  //  compass.writeReg(LSM303::CTRL2, 0x18); // 8 g full scale: AFS = 011; 1G=4096
  compass.writeReg(LSM303::CTRL2, 0x20); // 16 g full scale: AFS = 100; 1G=1365
}

// Reads x,y and z accelerometer registers
double Read_Accel()
{
  compass.readAcc();

  AN[0] = compass.a.x;
  AN[1] = compass.a.y;
  AN[2] = compass.a.z;
  accel_x = SENSOR_SIGN[0] * (AN[0] - AN_OFFSET[0]);
  accel_y = SENSOR_SIGN[1] * (AN[1] - AN_OFFSET[1]);
  accel_z = SENSOR_SIGN[2] * (AN[2] - AN_OFFSET[2]);
  //accel_x = SENSOR_SIGN[0] * AN[0];
  //accel_y = SENSOR_SIGN[1] * AN[1];
  //accel_z = SENSOR_SIGN[2] * AN[2];

  return (SENSOR_SIGN[0]*sqrt(sq(accel_x * 1.0) + sq(accel_y * 1.0) + sq(accel_z * 1.0)));
}

double getPressure()
{
  char status;
  double T, P, p0, a;

  // You must first get a temperature measurement to perform a pressure reading.

  // Start a temperature measurement:
  // If request is successful, the number of ms to wait is returned.
  // If request is unsuccessful, 0 is returned.

  status = pressure.startTemperature();
  if (status != 0)
  {
    // Wait for the measurement to complete:

    delay(status);

    // Retrieve the completed temperature measurement:
    // Note that the measurement is stored in the variable T.
    // Use '&T' to provide the address of T to the function.
    // Function returns 1 if successful, 0 if failure.

    status = pressure.getTemperature(T);
    if (status != 0)
    {
      // Start a pressure measurement:
      // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
      // If request is successful, the number of ms to wait is returned.
      // If request is unsuccessful, 0 is returned.

      status = pressure.startPressure(3);
      if (status != 0)
      {
        // Wait for the measurement to complete:
        delay(status);

        // Retrieve the completed pressure measurement:
        // Note that the measurement is stored in the variable P.
        // Use '&P' to provide the address of P.
        // Note also that the function requires the previous temperature measurement (T).
        // (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
        // Function returns 1 if successful, 0 if failure.

        status = pressure.getPressure(P, T);
        if (status != 0)
        {
          return (P);
        }
        else DEBUG_PRINTLN(F("error retrieving pressure measurement\n"));
      }
      else DEBUG_PRINTLN(F("error starting pressure measurement\n"));
    }
    else DEBUG_PRINTLN(F("error retrieving temperature measurement\n"));
  }
  else DEBUG_PRINTLN(F("error starting temperature measurement\n"));
}

// determine baseline pressure and sigma (deviation) using "running average"
// calibrate acceleration sensor: total acceleration = Gravity = 1G; assuming x axis is up (not entirely co
// LED flashes during calibration
boolean setupSensor() {
  for (byte loops = 0; loops < 10; loops++) {
    getPressure();
    delay(20);
  }
  RunningStat rs;
  for (byte loops = 0; loops < 60; loops++) {
    rs.Push(getPressure());
    digitalWrite(RED_LED, HIGH);
    delay(200);
    digitalWrite(RED_LED, LOW);
  }
  sigma = rs.StandardDeviation();
  baseline = rs.Mean();
  sigma = pressure.altitude(baseline - sigma, baseline);

  delay(20);

  RunningStat rsAx, rsAy, rsAz;
  for (byte i = 0; i < 32; i++) // We take some readings...
  {
    Read_Accel();
    rsAx.Push(AN[0]);
    rsAy.Push(AN[1]);
    rsAz.Push(AN[2]);
    delay(20);
  }
  AN_OFFSET[0] = rsAx.Mean();
  AN_OFFSET[1] = rsAy.Mean();
  AN_OFFSET[2] = rsAz.Mean();

  GRAVITY = (int)sqrt(sq(AN_OFFSET[0] * 1.0) + sq(AN_OFFSET[1] * 1.0) + sq(AN_OFFSET[2] * 1.0));

  return true;
}

void readSensor() {
  if ((millis() - timer) >= 20) { // Main loop runs at 50Hz
    counter++;
    timer_old = timer;
    timer = millis();

    running_average(accelAverage, Read_Accel());

    // Get a new pressure reading:
    P = getPressure();
    // Show the relative altitude difference between
    // the new reading and the baseline reading:
    // heightAverage.currentValue=pressure.altitude(P,baseline);
    running_average(heightAverage, pressure.altitude(P, baseline));
  }
}
