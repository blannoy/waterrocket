#include <Servo.h>
const int servoPin = 7;
const int buttonPin = 10;    // the number of the pushbutton pin
int buttonState;             // the current reading from the input pin
int lastButtonState = LOW;   // the previous reading from the input pin
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers
#include <NeoSWSerial.h>
NeoSWSerial Bluetooth( 8, 9 );

String voice;
Servo Servo1;
byte state = 0;

void setup() {
  Bluetooth.begin(9600);
  Serial.begin(9600);
  Serial.println("Waiting for command...");

  Servo1.attach(servoPin);
  Servo1.write(0);
  pinMode(buttonPin, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  int reading = digitalRead(buttonPin);
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;

      // only toggle the LED if the new button state is HIGH
      if (buttonState == HIGH) {
        state = 1;
      }
    }
  }

  //Bluetooth.listen();
  while (Bluetooth.available())
  {
    //Serial.println("read");
    delay(10);
    char c = Bluetooth.read();
    voice += c;
  }
  if (voice.length() > 0)
  {
    Serial.println(voice);
    if (voice == "launch")
    {
      state = 1;
    }
  }
  voice = "";
  if (state == 1) {
    Serial.println("Launch!");
    Bluetooth.listen();
    Bluetooth.print("Rocket away!");
    digitalWrite(LED_BUILTIN, HIGH);
    Servo1.write(180);
    delay(2000);
    Servo1.write(0);
    state = 0;
    digitalWrite(LED_BUILTIN, LOW);
  }

  lastButtonState = reading;
}
