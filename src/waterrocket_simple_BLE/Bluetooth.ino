void bluetoothLoop() {
  while (Bluetooth.available())
  {
    delay(10);
    char c = Bluetooth.read();
    if (c == ':'){  
      nrCommands+=1;
    /*  Serial.print("command ");
      Serial.print(nrCommands);
      Serial.print(" - ");
      Serial.print(BTdata);
      Serial.print(" - ");*/
      commands[nrCommands-1]=BTdata;
      BTdata="";
    } else if (c == ';'){
            nrCommandsAvailable+=1;  
  /*    Serial.print(BTdata);
      Serial.print(" - ");
      Serial.print(nrCommandsAvailable);*/
      commandValues[nrCommands-1]=BTdata;
    
      BTdata="";
    } else {
       BTdata += c;
    }
  }
}

void processTime(String timeValue) {
  unsigned long pctime;
  pctime = timeValue.toInt();
  setTime(pctime); // Sync Arduino clock to the time received on the serial port
}

void sendCommand(String commandKey, String commandValue){
  String stringToSend="";
  stringToSend=commandKey+":"+commandValue+";";
  if (stringToSend.length()>20){
    stringToSend="Error:too long";
  }
  Bluetooth.print(stringToSend);
}

void commandLoop(){

  if (nrCommandsAvailable >0){
/*      Serial.print("Nr commands: ");
  Serial.println(nrCommandsAvailable);*/
    for (int iLoop=0;iLoop<nrCommandsAvailable;iLoop++){
      Serial.print("Command - ");
      Serial.println(commands[iLoop]);
      
  if (commands[iLoop] == "time"){
    processTime(commandValues[iLoop]);
   // sendCommand("OK", "");
  } else if (commands[iLoop] == "getChuteDelay"){
    dtostrf(vars.delay_counter, 1, 1, buff);
    sendCommand("chuteDelay", buff);
  }else if (commands[iLoop] == "setChuteDelay"){
    vars.delay_counter=commandValues[iLoop].toFloat();
    dtostrf(vars.delay_counter, 1, 1, buff);
    sendCommand("chuteDelay", buff);
 }
    }
    }

  // if partial command (max 1 diff between nrCommands & nrCommandsAvailable) copy content to first position and reset 
  if (nrCommands > nrCommandsAvailable){
    for (int iLoop=0;iLoop<(nrCommands-nrCommandsAvailable);iLoop++){
      commands[0]=commands[nrCommands-1];
      commandValues[0]=commandValues[nrCommands-1];
      nrCommands=1;
      nrCommandsAvailable=0;
      
    }
  } else {
    nrCommands=0;
    nrCommandsAvailable=0;
  }
}
