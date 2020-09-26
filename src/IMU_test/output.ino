// log to console and file if file available
void log_msg(const char* str) {
  long curTime=millis();
  Serial.print(curTime);
  Serial.print(F(":"));
  Serial.print(str);
}

void log_msg(const __FlashStringHelper* str) {
  Serial.print(str);
}

// log message with timestamp if tStamp=true (to have timestamp at beginning of line)
void log_msg(const __FlashStringHelper* str, bool tStamp) {
 if (tStamp){
  long curTime=millis();
  Serial.print(curTime);
  Serial.print(F(":"));
 }
 log_msg(str);
}

void log_msg(int iVal) {
 // long curTime=millis();
//  Serial.print(curTime);
//  Serial.print(F(":"));
  Serial.print(iVal);
}

void log_msg(double dVal) {
 // long curTime=millis();
//  Serial.print(curTime);
//  Serial.print(F(":"));
  Serial.print(dVal);
}


