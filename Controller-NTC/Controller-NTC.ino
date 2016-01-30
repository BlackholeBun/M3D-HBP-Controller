#include <EEPROM.h>
#include <Servo.h>
#include <MsTimer2.h>
#include <PID_v1.h>

//Use for NTC Thermistor with 25 KOhm resistor
#define ThermistorCalc ((0.0000003 * pow(sensorValue, 3)) - (0.0004 * pow(sensorValue, 2)) + (0.245 * sensorValue) - 12.587)

//Use for old Thermistor with 25 KOhm resistor
//#define ThermistorCalc ((0.000000000000005 * pow(sensorValue, 6)) - (0.00000000001 * pow(sensorValue, 5)) + (0.00000001 * pow(sensorValue, 4)) - (0.000005 * pow(sensorValue, 3)) + (0.0011 * pow(sensorValue, 2)) + (0.0359 * sensorValue) - 2.3354)

//Use for NTC Thermistor with 100kOhm resistor
//#define ThermistorCalc ( pow(4.0953, (0.0034 * sensorValue)) )

const int analogInPin = A0;  // Analog input pin that the potentiometer is attached to

double temp = 0;
double tempLast = 0;
double goalTemp = 0;
double dbout = 0;
int sensorValue = 0;        // value read from the pot
int outputValue = 0;        // value output to the PWM (analog out)
String Command = "";
boolean CFlag = false;
boolean TFlag = false;
boolean Reverse = false;
boolean TWFlag = false;
boolean TOFlag = false;
int State = 0;
long int UpdateTime = 2000;
String Output = "";
int OutputTimer = 0;
int Override = 255;
int UpperBound = 170;
int LowerBound = 10;

Servo Heater;
PID Controller(&temp, &dbout, &goalTemp, 2, 0.6, 2, DIRECT);

void timerEvent() {
  TFlag = true;
}

void setup() {
  // initialize serial communications at 9600 bps:
  int RevTemp = 0;
  Serial.begin(115200);
  Heater.attach(9);
  Command.reserve(255);
  Output.reserve(255);
  EEPROM.get(0, LowerBound);
  EEPROM.get(2, UpperBound);
  EEPROM.get(4, RevTemp);
  if (LowerBound < 0) {
    LowerBound = 0;
    UpperBound = 180;
    Reverse = true;
    State = 1;
    Command = "?\r";
    CFlag = true;
  }
  if (RevTemp == 1) {
    Reverse = true;
  } else {
    Reverse = false;
  }
  MsTimer2::set(UpdateTime, timerEvent);
  initESC();
  MsTimer2::start();
  Controller.SetOutputLimits(LowerBound, UpperBound);
  Controller.SetMode(AUTOMATIC);
}

int GetInt(String &input, String &output) {
  int st = -1, en = -1, i = 0;
  char currentChar = 'x';

  while (currentChar != '\0') {
    currentChar = input.charAt(i);
    if (st < 0) {
      if (47 < currentChar && currentChar < 58) {
        st = i;
        en = i + 1;
      }
    } else {
      if (47 < currentChar && currentChar < 58) {
        en = i + 1;
      } else {
        currentChar = '\0';
      }
    }
    i++;
  }

  output = input.substring(st, en);

  return en - st;
}

void TimerUpdate(long tm) {
  MsTimer2::stop();
  MsTimer2::set(tm, timerEvent);
  MsTimer2::start();
}
void loop() {

  int loc = 0;
  // read the analog in value:
  sensorValue = analogRead(analogInPin);
  temp = ThermistorCalc;
  Controller.Compute();

  if (CFlag) {
    Command.toLowerCase();
    char startChar = Command.charAt(0);
    String newTemp = "0";
    int result = 0;
    switch (State) {
      case 0:
        switch (startChar) {
          case '?':
            Output = "Usable commands are the following:\n\r? Display this dialog\n\r(S)et Set the temp of the bed in C\n\r"
                     "(C)onfig Set up the esc\n\r(U)pdate Sets the update interval in Seconds from 1 to 60\n\r(M)onitor Rapid temp updates\n\r";
            Serial.print(Output);
            OutputTimer = 10000;
            break;
          case 's':
            result = GetInt(Command, newTemp);
            if (result > 0) {
              goalTemp = constrain(newTemp.toInt(), 0, 110);
              Output = "Temp set to :" + String(goalTemp);
            } else {
              Output = "Please enter a valid Temp <0 - 110>";
            }
            OutputTimer = 10000;
            Serial.println(Output);
            break;
          case 'm':
            State = 1;
            TimerUpdate(200);
            break;
          case 'u':
            result = GetInt(Command, newTemp);
            if (result > 0 ) {
              UpdateTime = newTemp.toInt() * 1000;
              TimerUpdate(UpdateTime);
              Output = "New update period set to: " + String(UpdateTime / 1000);
            } else {
              Output = "Please enter the new update time in sec <0 - 60>";
            }
            OutputTimer = 10000;
            Serial.println(Output);
            break;
          case 'c':
            State = 2;
            break;
          case 't':
            TOFlag = true;
          case 'i':
            State = 3;
            break;
          default:
            Output = "Invalid command. Please type ? for help.";
            OutputTimer = 10000;
            Serial.println(Output);
        }
        break;
      case 1:
        TimerUpdate(UpdateTime);
        State = 0;
        break;
      case 2:
        switch (startChar) {
          case '?':
            Output = "Config Mode:\n\r\n\r(O)veride <0-180> Manually set output to ESC No number will disable override\n\r(R)everse to reverse output to ESC\n\r"
                     "(U)pper <90-180> To set upper output boundary\n\r(L)ower <0-89> to set lower boundary\n\r (W)rite Save settings\n\r(E)xit\n\r";
            Serial.print(Output);
            OutputTimer = 10000;
            break;
          case 'o':
            result = GetInt(Command, newTemp);
            if (result > 0) {
              Override = constrain(newTemp.toInt(), 0, 180);
              Output = "Output set to : " + String(Override);
            } else {
              Override = 255;
              Output = "Override off";
            }
            OutputTimer = 10000;
            Serial.println(Output);
            break;
          case 'r':
            Reverse = !Reverse;
            Output = "Output reverse set to: " + String(Reverse);
            OutputTimer = 10000;
            Serial.println(Output);
            break;
          case 'u':
            result = GetInt(Command, newTemp);
            if (result > 0) {
              UpperBound = constrain(newTemp.toInt(), 90, 180);
              Output = "Upper Bound set to: " + String(UpperBound);
              Controller.SetOutputLimits(LowerBound, UpperBound);
            } else {
              Output = "Please enter a valid number after (U)pper";
            }
            OutputTimer = 10000;
            Serial.println(Output);
            break;
          case 'l':
            result = GetInt(Command, newTemp);
            if (result > 0) {
              LowerBound = constrain(newTemp.toInt(), 0, 89);
              Output = "Lower Bound set to: " + String(LowerBound);
              Controller.SetOutputLimits(LowerBound, UpperBound);
            } else {
              Output = "Please enter a valid number after (L)owerr";
            }
            OutputTimer = 10000;
            Serial.println(Output);
            break;
          case 'w':
            EEPROM.put(0, LowerBound);
            EEPROM.put(2, UpperBound);
            if (Reverse) {
              result = 1;
            } else {
              result = 0;
            }
            EEPROM.put(4, result);
            Output = "Data Stored.";
            OutputTimer = 10000;
            Serial.println(Output);
            break;
          case 'e':
            State = 0;
            break;
          default:
            Output = "Invalid command. Please type ? for help.";
            OutputTimer = 10000;
            Serial.println(Output);
        }
        break;
      case 3:
        switch (startChar) {
          case 's':
            result = GetInt(Command, newTemp);
            if (result > 0) {
              goalTemp = constrain(newTemp.toInt(), 0, 110);
            }
            break;
          case 't':
            TOFlag = true;
            TFlag = true;
            break;
          case 'e':
            State = 0;
            break;
          case 'w':
            result = GetInt(Command, newTemp);
            if (result > 0) {
              goalTemp = constrain(newTemp.toInt(), 0, 110);
            }
            TOFlag = true;
            TWFlag = true;
            tempLast = temp;
            TimerUpdate(500);
            break;
        }
        break;
      default:
        State = 0;
    }
    CFlag = false;
    Command = "";
  }

  if (Override <= 180 && State == 2) {
    outputValue = Override;
  } else {
    outputValue = dbout;
  }

  if (Reverse) {
    outputValue = UpperBound - outputValue + LowerBound;
  }

  // map it to the range of the analog out:
  //outputValue = map(sensorValue, 0, 1023, 0, 180);
  Heater.write(outputValue);
  // change the analog out value:
  //analogWrite(analogOutPin, outputValue);
  if (TFlag) {
    TFlag = false;
    if (State != 3) {
      // print the results to the serial monitor:
      Serial.write(27);
      Serial.print("[2J");
      Serial.write(27);
      Serial.print("[2H");
      Serial.print("\rSensor = " );
      Serial.println(sensorValue);
      Serial.print("Output = ");
      Serial.println(outputValue);
      Serial.print("Temp = ");
      Serial.println(temp);
      Serial.print("Goal Temp = ");
      Serial.println(goalTemp);
      if (OutputTimer > 0) {
        Serial.println(Output);
        OutputTimer -= UpdateTime;
      }
      Serial.print(Command);
    } else if (TOFlag) {
      Serial.print(temp);
      if (!TWFlag) {
        TOFlag = false;
      } else {
        tempLast = (tempLast + temp) / 2;
        if (tempLast >= goalTemp) {
          TWFlag = false;
          TOFlag = false;
          TimerUpdate(UpdateTime);
        }
      }
    }
  }


  // wait 2 milliseconds before the next loop
  // for the analog-to-digital converter to settle
  // after the last reading:
  delay(2);
}

void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    if (State !=3) Serial.print(inChar);
    // add it to the inputString:
    if (inChar == '\b') {
      Serial.print("\r");
      for (int i = 0; i <= Command.length(); i++){
        Serial.print(" ");
      }
      Command.remove(Command.length() - 1);
      Serial.print("\r" + Command);
    } else Command += inChar;
    // if the incoming character is a cr, set a flag
    // so the main loop can do something about it:
    if (inChar == '\r') {
      CFlag = true;
      Serial.println();
    }
  }
}

void initESC() {
  if (Reverse) {
    Heater.write(UpperBound);
  } else {
    Heater.write(LowerBound);
  }
}
