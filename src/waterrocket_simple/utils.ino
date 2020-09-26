// button handling code for
// - long press event
// - flash LED when button is pressed
int handle_button(int buttonNr,boolean isButton)
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
// 2 x red : no sensor or sensor problems 
// 3 x red : no delay setup
// 4 x red : breaking wire open
// 5 x red : no sdcard or SDCard problem

void flashError(int signal) {
  for (byte iSig = 0; iSig < signal; iSig++) {
    digitalWrite(RED_LED, HIGH);
    delay(200);
    digitalWrite(RED_LED, LOW);
    delay(200);
  }
  delay(300);
}

/* function to read with running average over 5 values */ 
double running_average(double value) {
  total = total - readings[index];
  readings[index] = value;
  total = total + readings[index];
  index = index + 1;
  if (index >= numReadings) {
    index = 0;
  }
  return total / numReadings;
}
