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
        m_newM = m_oldM + (x - m_oldM) / m_n;
        m_newS = m_oldS + (x - m_oldM) * (x - m_newM);

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
      return ( (m_n > 1) ? m_newS / (m_n - 1) : 0.0 );
    }

    double StandardDeviation() const
    {
      return sqrt( Variance() );
    }

  private:
    int m_n;
    double m_oldM, m_newM, m_oldS, m_newS;
};

void buttonLoop() {
  byte pushEvent = handle_button(0, true);
  byte breakEvent = handle_button(1, false);

  // button long pressed
  if (pushEvent == EV_LONGPRESS) {
    // first press: calibrate sensor and setup for flight
    if (state == INIT) {
      Serial.println("Calibrate");
      setRocketState(CalibrateState, '1');
      changeState(CALIBRATE);
    }
    if (state == READY && logState == OFF) {
      changeLogState(STARTLOG);
      setRocketState(LoggingState, '1');
    }
    pushEvent = EV_NONE;
  }
  if ((!breakEvent) && rocketState[0] == '1') {

    setRocketState(0, '0');
    changeState(LIFTOFF);
  } else if ((breakEvent) && rocketState[0] == '0') {
    setRocketState(0, '1');
  }
}

// button handling code for
// - long press event
// - flash LED when button is pressed
int handle_button(int buttonNr, boolean isButton)
{
  buttonEvents event;
  byte button_now_pressed = !digitalRead(buttons[buttonNr]); // pin low -> pressed
  // button: change event when released
  if (isButton) {
    if (!button_now_pressed && button_was_pressed[buttonNr]) {
      if (button_pressed_counter[buttonNr] < LONGPRESS_LEN)
        event = EV_SHORTPRESS;
      else
        event = EV_LONGPRESS;
    }
    else
      event = EV_NONE;
  } else {
    // no button: keep sending state
    if (button_was_pressed[buttonNr]) {
      if (button_pressed_counter[buttonNr] < LONGPRESS_LEN)
        event = EV_SHORTPRESS;
      else
        event = EV_LONGPRESS;
    }
    else
      event = EV_NONE;
  }
  if (button_now_pressed) {
    ++button_pressed_counter[buttonNr];
    // flash LED when longpress reached if its real button
    if (isButton && button_pressed_counter[buttonNr] >= LONGPRESS_LEN) {
      digitalWrite(RED_LED, HIGH);
      delay(100);
      digitalWrite(RED_LED, LOW);
    }
  }
  else
    button_pressed_counter[buttonNr] = 0;

  button_was_pressed[buttonNr] = button_now_pressed;
  return event;
}

int _handle_button(int buttonNr, boolean isButton)
{
  buttonEvents event;
  byte button_now_pressed = !digitalRead(buttons[buttonNr]); // pin low -> pressed

  if (!button_now_pressed && button_was_pressed[buttonNr]) {
    if (button_pressed_counter[buttonNr] < LONGPRESS_LEN)
      event = EV_SHORTPRESS;
    else
      event = EV_LONGPRESS;
  }
  else
    event = EV_NONE;

  if (button_now_pressed) {
    ++button_pressed_counter[buttonNr];
    // flash LED when longpress reached if its real button
    if (isButton && button_pressed_counter[buttonNr] >= LONGPRESS_LEN) {
      digitalWrite(RED_LED, HIGH);
      delay(100);
      digitalWrite(RED_LED, LOW);
    }
  }
  else
    button_pressed_counter[buttonNr] = 0;

  button_was_pressed[buttonNr] = button_now_pressed;
  return event;
}

// error conditions
// 2 x red : no sensor or sensor problems
// 3 x red : no delay setup
// 4 x red : breaking wire open
// 5 x red : no sdcard or SDCard problem

void flashError(int signal) {
#ifdef BLE

  sendCommand("ERR", signal);
#endif
  for (byte iSig = 0; iSig < signal; iSig++) {
#ifdef BLE
  bluetoothLoop();
  receiveCommandLoop();
#endif
    digitalWrite(RED_LED, HIGH);
    delay(200);
    digitalWrite(RED_LED, LOW);
    delay(200);
  }
  delay(300);
}

/* function to read with running average over 5 values */
void running_average(runningAverage &avg, double storeValue) {
  avg.total = avg.total - avg.readings[avg.index];
  avg.readings[avg.index] = storeValue;
  avg.total = avg.total + avg.readings[avg.index];
  avg.index = avg.index + 1;
  if (avg.index >= numReadings) {
    avg.index = 0;
  }
  avg.currentValue = avg.total / numReadings;
}

void setRocketState(int pos, char state) {
  rocketState[pos] = state;
#ifdef BLE
  sendState();
#endif
}

#ifdef BLE
void sendState(){
  memset(rState, 0, VALLENGTH);
  memcpy(rState, rocketState, 5);
  sendCommand("STA", rState);
}
#endif

void changeState(flightStates targetState) {
  state = targetState;
#ifdef BLE
  //  sendCommand("RST", flightStates_string[targetState]);
#endif
}

void changeLogState(logStates targetState) {
  logState = targetState;
  //#ifdef BLE
  //  sendCommand("LST",logStates_string[targetState]);
  //#endif
}
