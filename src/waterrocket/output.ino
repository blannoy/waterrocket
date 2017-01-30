// log to console and file if file available
void log_msg(const char* str) {
  long curTime=millis();
  Serial.print(curTime);
  Serial.print(F(":"));
  Serial.print(str);
  if (file.isOpen()){
    file.print(curTime);
    file.print(F(":"));
    file.print(str);
    flush_file(); 
  }
}

void log_msg(const __FlashStringHelper* str) {
  Serial.print(str);
  if (file.isOpen()){
    file.print(str);
    flush_file(); 
  }
}

// log message with timestamp if tStamp=true (to have timestamp at beginning of line)
void log_msg(const __FlashStringHelper* str, bool tStamp) {
 if (tStamp){
  long curTime=millis();
  Serial.print(curTime);
  Serial.print(F(":"));
  if (file.isOpen()){
    file.print(curTime);
    file.print(F(":"));
  }  
 }
 log_msg(str);
}

void log_msg(int iVal) {
 // long curTime=millis();
//  Serial.print(curTime);
//  Serial.print(F(":"));
  Serial.print(iVal);
  if (file.isOpen()){
//    file.print(curTime);
//    file.print(F(":"));
    file.print(iVal);
    flush_file(); 
  }
}

void log_msg(double dVal) {
 // long curTime=millis();
//  Serial.print(curTime);
//  Serial.print(F(":"));
  Serial.print(dVal);
  if (file.isOpen()){
//    file.print(curTime);
//    file.print(F(":"));
    file.print(dVal);
    flush_file(); 
  }
}

void printdata(void)
{     file.print(millis()-startLog);
      file.print(F(";"));
      file.print(ToDeg(roll));
      file.print(F(";"));
      file.print(ToDeg(pitch));
      file.print(F(";"));
      file.print(ToDeg(yaw));
      file.print(F(";"));
      file.print(a);
      file.print(F(";"));
      file.print(accel_x/256.0);
      file.print(F(";"));
      file.print(accel_y/256.0);
      file.print(F(";"));
      file.print(accel_z/256.0);
      file.println(F(""));
     flush_file(); 
}

void flush_file(){
  nr_entries++;
  // keep track of number of outputs to file
  // sync after FLUSH_LIMIT entries
  if(nr_entries >= FLUSH_LIMIT){
    file.sync();
    nr_entries=0;
  }
}

