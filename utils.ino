int handle_button(int buttonNr,boolean isButton)
{
  // mod int -> byte
  byte event;
  byte button_now_pressed = !digitalRead(buttons[buttonNr]); // pin low -> pressed
   
  if (!button_now_pressed && button_was_pressed[buttonNr]) {
    if (button_pressed_counter[buttonNr] < LONGPRESS_LEN)
      event = EV_SHORTPRESS;
    else
      event = EV_LONGPRESS;
  }
  else
    event = EV_NONE;

  if (button_now_pressed){
    ++button_pressed_counter[buttonNr];
    // flash LED when longpress reached if its real button
    if (isButton && button_pressed_counter[buttonNr] >= LONGPRESS_LEN){
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
// 2 x red : no sensor comms
// 3 x red : no delay
// 4 x red : breaking wire open
// 5 x red : no sdcard
// 6 x red : battery problem

void flashError(int signal) {
  //Serial.println("error");
  for (byte iSig = 0; iSig < signal; iSig++) {
    digitalWrite(RED_LED, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(200);
    digitalWrite(RED_LED, LOW);   // turn the LED on (HIGH is the voltage level)
    delay(200);
  }
  delay(300);
}

// determine current average height & sigma
/*void normalize() {
  int minHeightOffset = 9999;
  int maxHeightOffset = -9999;
  //int calLoops=30;
  int loops = 0;
  unsigned long cumulVal = 0;
  unsigned long lSigma = 0;

  sigma = 0;
  normHeight = 0;

  for (loops = 0; loops < 10; loops++) {
    read();
  }
  loops = 0;

  while (loops < calLoops) {
    average = read();
    if (average != -999) {
      // running mean and sigma
      loops++; // increment the current number
      cumulVal += average;
      lSigma = lSigma + ((long)average * (long)average);
    }
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
  //Serial.println(cumulVal);
  cumulVal = (cumulVal * 100 / calLoops);
  unsigned long factor = (long)calLoops * (long)cumulVal * (long)cumulVal;
  //Serial.println(cumulVal);
  //Serial.println(factor);
  lSigma = sqrt((lSigma * 10000 - factor) / (long)calLoops);
  //Serial.println(lSigma);
  normHeight = (int)(cumulVal / 100);
  sigma = (int)(round(lSigma / 100));
}*/
/*
long d2l(double x)
{
  return (long) x*100;
}
float l2f(long x)
{
  return x/100;
}
double l2d(long x)
{
  return x/100;
}*/

void freeRam() 
{
  extern int __heap_start, *__brkval; 
  int v, ram;
  ram= (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
  log_msg(F("RAM : "),true);
  log_msg(ram);
  log_msg(F("\n"),false);
}
