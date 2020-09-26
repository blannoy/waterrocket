// Statistics code for average and standard deviation
// from http://www.johndcook.com/blog/standard_deviation/
class RunningStat
    {
    public:
        RunningStat() : m_n(0) {}

        void Clear()
        {
            m_n = 0;
        }

        void Push(double x)
        {
            m_n++;

            // See Knuth TAOCP vol 2, 3rd edition, page 232
            if (m_n == 1)
            {
                m_oldM = m_newM = x;
                m_oldS = 0.0;
            }
            else
            {
                m_newM = m_oldM + (x - m_oldM)/m_n;
                m_newS = m_oldS + (x - m_oldM)*(x - m_newM);
    
                // set up for next iteration
                m_oldM = m_newM; 
                m_oldS = m_newS;
            }
        }

        int NumDataValues() const
        {
            return m_n;
        }

        double Mean() const
        {
            return (m_n > 0) ? m_newM : 0.0;
        }

        double Variance() const
        {
            return ( (m_n > 1) ? m_newS/(m_n - 1) : 0.0 );
        }

        double StandardDeviation() const
        {
            return sqrt( Variance() );
        }

    private:
        int m_n;
        double m_oldM, m_newM, m_oldS, m_newS;
    };
    
  // determine baseline and sigma (deviation) using "running average"
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
  sigma=rs.StandardDeviation();
  baseline=rs.Mean();
  sigma=pressure.altitude(baseline-sigma,baseline);
  
  delay(20);
  
  for(byte i=0;i<32;i++)    // We take some readings...
    {
 //   Read_Gyro();
    Read_Accel();
    for(int y=0; y<6; y++)   // Cumulate values
      AN_OFFSET[y] += AN[y];
    delay(20);
    }
    
  for(int y=0; y<6; y++)
    AN_OFFSET[y] = AN_OFFSET[y]/32;
    
  AN_OFFSET[5]-=GRAVITY*SENSOR_SIGN[5];

  return true;
}

void readSensor(){
    if((millis()-timer)>=20){  // Main loop runs at 50Hz
      counter++;
      timer_old = timer;
      timer=millis();

      Read_Accel();     // Read I2C accelerometer
      // Get a new pressure reading:
      P = getPressure();
      // Show the relative altitude difference between
      // the new reading and the baseline reading:
      a = running_average(pressure.altitude(P,baseline));
  }
}
