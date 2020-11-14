#ifdef BLE
#include <avr/pgmspace.h>
char buff[10];
byte isBLEConnected = 0;
char BTdata[FULLLENGTH];
char commands[3][CMDLENGTH];
char commandValues[3][VALLENGTH];
int nrCommandsAvailable = 0;
int nrCommands = 0;
long lastCommandTime = 0;


#define BLE_DELAY 30

void bluetoothLoop() {
  while (Bluetooth.available())
  {
    delay(10);
    char c = Bluetooth.read();
    if (c == ':') {
      nrCommands += 1;
      if (strlen(BTdata) > sizeof(commands[nrCommands - 1])) {
      }
      strcpy(commands[nrCommands - 1], BTdata);
      memset(BTdata, 0, sizeof(BTdata));
    } else if (c == ';') {
      nrCommandsAvailable += 1;
      if (strlen(BTdata) > sizeof(commandValues[nrCommands - 1])) {
      } else {
        strcpy(commandValues[nrCommands - 1], BTdata);
      }
      memset(BTdata, 0, sizeof(BTdata));
    } else {
      if ((strlen(BTdata) + 1) < sizeof(BTdata) - 1) {
        BTdata[strlen(BTdata)] = c;
        BTdata[strlen(BTdata) + 1] = 0;
      } else {
        memset(BTdata, 0, sizeof(BTdata));
      }
    }
  }
  if (millis() - lastHeartBeat > 5000) {
    if (rocketState[PlotState]=='1') {
      setRocketState(PlotState, '0');
    }
  }
}

void receiveCommandLoop() {
  if (nrCommandsAvailable > 0) {
    isBLEConnected = 1;
    lastHeartBeat = millis();
    for (int iLoop = 0; iLoop < nrCommandsAvailable; iLoop++) {
      if (!strcmp_P(commands[iLoop], PSTR("TIM"))) {
        processTime(commandValues[iLoop]);
        sendState();
      } else if (!strcmp_P(commands[iLoop], PSTR("CAL"))) {
        changeState(CALIBRATE);
        setRocketState(CalibrateState, '1');
      } else if (!strcmp_P(commands[iLoop], PSTR("SER"))) {
        // open / close servo
        if (pos == setAngle) {
          pos = prevAngle;
          prevAngle = setAngle;
          setRocketState(ServoState, '1');
        } else {
          prevAngle = pos;
          pos = setAngle;
          setRocketState(ServoState, '0');
        }
        myservo.write(pos);
      } else if (!strcmp_P(commands[iLoop], PSTR("LOG"))) {
        // Start/stop logging
        if (rocketState[LoggingState] == '0') {
          changeLogState(STARTLOG);
          setRocketState(LoggingState, '1');
        }
      } else if (!strcmp_P(commands[iLoop], PSTR("DTA"))) {
        sendCommand("DC", vars.delay_counter);
        sendCommand("DF", vars.lastFlightDate);
        sendCommand("DS", vars.sigma);
        sendCommand("DMH", vars.maxHeight);
        sendCommand("DMT", vars.maxHeightTime);
        sendCommand("DCT", vars.chuteTime);
        sendCommand("DGT", vars.groundTime);

        sendState();
      } else if (!strcmp_P(commands[iLoop], PSTR("GCD"))) {
        itoa(vars.delay_counter, buff, 10);
        sendCommand("CD", buff);
      } else if (!strcmp_P(commands[iLoop], PSTR("SCD"))) {
        vars.delay_counter = atoi(commandValues[iLoop]);
        EEPROM.put(0, vars);
        itoa(vars.delay_counter, buff, 10);
        sendCommand("CD", buff);
        sendState();
      } else if (!strcmp_P(commands[iLoop], PSTR("HTB"))) {
        lastHeartBeat = millis();
      } else if (!strcmp_P(commands[iLoop], PSTR("PLT"))) {
        if (!strcmp_P(commandValues[iLoop], PSTR("ST"))) {
          setRocketState(PlotState, '1');
        } else if (!strcmp_P(commandValues[iLoop], PSTR("SP"))) {
          setRocketState(PlotState, '0');
        }
      }
    }
  }

  // if partial command (max 1 diff between nrCommands & nrCommandsAvailable) copy content to first position and reset
  if (nrCommands > nrCommandsAvailable) {
    for (int iLoop = 0; iLoop < (nrCommands - nrCommandsAvailable); iLoop++) {
      strcpy(commands[0], commands[nrCommands - 1]);
      strcpy(commandValues[0], commandValues[nrCommands - 1]);
      memset(commands[nrCommands - 1], 0, sizeof(commands[nrCommands - 1]));
      memset(commandValues[nrCommands - 1], 0, sizeof(commandValues[nrCommands - 1]));
      nrCommands = 1;
      nrCommandsAvailable = 0;

    }
  } else {
    nrCommands = 0;
    nrCommandsAvailable = 0;
  }
}

void processTime(char* timeValue) {
  unsigned long pctime;
  pctime = atol(timeValue);
  setTime(pctime); // Sync Arduino clock to the time received on the serial port
}

void sendCommand(char* commandKey, double commandValue) {
  dtostrf(commandValue, 10, 1, buff);
  sendCommand(commandKey, buff);
}

void sendCommand(char* commandKey, long commandValue) {
  ltoa(commandValue, buff, 10);
  sendCommand(commandKey, buff);
}

void sendCommand(char* commandKey, int commandValue) {
  itoa(commandValue, buff, 10);
  sendCommand(commandKey, buff);
}

void sendCommand(char* commandKey, char* commandValue) {
    if (millis() - lastHeartBeat < 5000) {
    char stringToSend[FULLLENGTH];
    memset(stringToSend, 0, FULLLENGTH - 1);
    strcpy(stringToSend, commandKey);
    strcat_P(stringToSend, PSTR(":"));
    strcat(stringToSend, commandValue);
    strcat_P(stringToSend, PSTR(";"));
    if (strlen(commandKey) + strlen(commandValue) + 2 > FULLLENGTH) {
      ///DEBUG_PRINT(F("Error:too long - /"));
    } else {
      Bluetooth.print(stringToSend);
      delay(BLE_DELAY);
    }
    }else{
      DEBUG_PRINT(F("Dropped command, no Bluetooth connection - "));
       DEBUG_PRINT(commandKey);
             DEBUG_PRINT(F(" : "));
       DEBUG_PRINTLN(commandValue);
    }
}

#endif
