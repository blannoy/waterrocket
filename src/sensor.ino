
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
// determine baseline and sigma (deviation) using running average
boolean setupSensor() {
 /* int loops = 0;
  unsigned long cumulVal = 0;
  unsigned long lSigma = 0;
  long average = 0;
  
  sigma = 0;
  //normHeight = 0;
  double min=2000;
  double p=0;
  for (loops = 0; loops < 10; loops++) {
    p=getPressure();
    if (p < min){
      min=p;
    }
  }
  min=min*.95;
  loops = 0;

  while (loops < calLoops) {
    p=getPressure();
    Serial.print(p);
    Serial.print(",");
    average = (long)(p*100);
    Serial.print(average);
    average = average-(long)(min*100);
    Serial.print(",");
    Serial.print(average);
    
   // if (average != -999) {
      // running mean and sigma
      loops++; // increment the current number
      cumulVal += average;
      lSigma = lSigma + (average * average);
    Serial.print(",");
    Serial.print(cumulVal);
    Serial.print(",");
    Serial.println(lSigma);
      
  //  }
    if (loops % 5 == 0) {
      digitalWrite(GREEN_LED, HIGH);
      delay(100);
      digitalWrite(GREEN_LED, LOW);
    }
    if (loops % 10 == 0) {
      digitalWrite(RED_LED, HIGH);
      delay(100);
      digitalWrite(RED_LED, LOW);
    }
    delay(10);
  }
  Serial.println(cumulVal);
  cumulVal = (cumulVal * 100 / calLoops);
  unsigned long factor = (long)calLoops * (long)cumulVal * (long)cumulVal;
  Serial.println(cumulVal);
  Serial.println(factor);
  lSigma = sqrt((lSigma * 10000 - factor) / (long)calLoops);
  Serial.println(lSigma);
  //normHeight = (int)(cumulVal / 100);
  sigma = (int)(round(lSigma / 100.));

 // Serial.println(normHeight);
  Serial.println(sigma);  
  Serial.println((double)lSigma/10000.);  
//  Serial.println(cumulVal);
  
  baseline = (double)cumulVal/10000.+min;
  Serial.println(baseline);
  p=pressure.altitude(baseline-(double)lSigma/10000.,baseline);
  Serial.println(p);
 */
  for (byte loops = 0; loops < 10; loops++) {
    getPressure();
    delay(20);
  } 
  RunningStat rs;
  for (byte loops = 0; loops < 60; loops++) {
    //double p=getPressure();
    rs.Push(getPressure());
    //Serial.println(p);
    digitalWrite(RED_LED, HIGH); 
    delay(200);
    digitalWrite(RED_LED, LOW);  
  } 
  sigma=rs.StandardDeviation();
  baseline=rs.Mean();
  //Serial.println(sigma);
  //Serial.println(baseline);
  sigma=pressure.altitude(baseline-sigma,baseline);
  //Serial.println(sigma);
  
  delay(20);
  
  for(byte i=0;i<32;i++)    // We take some readings...
    {
    Read_Gyro();
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

