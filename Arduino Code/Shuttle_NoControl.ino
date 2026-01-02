#include <Wire.h>
#include <Adafruit_MotorShield.h>

// put your setup code here, to run once:

//#include "utility/Adafruit_PWMServoDriver.h"
// Establishes the motor controls using the 2 motor shields
Adafruit_MotorShield AFMSbottom = Adafruit_MotorShield(0x60);
Adafruit_DCMotor *RightFront = AFMSbottom.getMotor(1);
Adafruit_DCMotor *LeftFront = AFMSbottom.getMotor(2);
Adafruit_DCMotor *LeftBack = AFMSbottom.getMotor(3);
Adafruit_DCMotor *RightBack = AFMSbottom.getMotor(4);
Adafruit_MotorShield AFMStop = Adafruit_MotorShield(0x61);
Adafruit_DCMotor *Bwd = AFMStop.getMotor(1);
Adafruit_DCMotor *Fwd = AFMStop.getMotor(3);
char val; // Data received from the serial port
String ReadDataOld = "A100B50C200D0E0F0Z";
float t0;
int v[] = {0, 0, 0, 0, 0, 0}; //Motor speeds initialized at 0

void setup() {
  AFMStop.begin(); //Starts the connection with the motor shields
  AFMSbottom.begin();
  Serial2.begin(115200); //Starts the serial connections with Xbees on sets of TX and RX pins
  Serial1.begin(115200);
  Serial.begin(115200); //Starts communication over serial port for messages to be read on computer
}

void loop() {
  t0=millis();
  // Reset the data strings
  String ReadData = "";
  String Control_string = "";
  char dir;
  if (Serial2.available()) { //If there is a message from the controller Xbee
    dir = Serial2.read();
    if (dir == 'R') { //If the controller sent the signal 'R'
      RightBack->setSpeed(0);
      Fwd->setSpeed(0);
      Bwd->setSpeed(0);
      RightFront->setSpeed(0);
      LeftFront->setSpeed(255);
      LeftFront->run(FORWARD); //Run the motor forward at the set speed
      LeftBack->setSpeed(255); //Set motor 3 (on the right) to full power
      LeftBack->run(FORWARD); //Run the motor forward at the set speed
      Serial.println("Controller Right"); //Print to the terminal what control was received
    }
    if (dir == 'L') {
      LeftBack->setSpeed(0);
      Fwd->setSpeed(0);
      Bwd->setSpeed(0);
      LeftFront->setSpeed(0);
      RightFront->setSpeed(255);
      RightFront->run(FORWARD);
      RightBack->setSpeed(255);
      RightBack->run(FORWARD);
      Serial.println("Controller Left");
    }
    if (dir == 'U') {
      RightBack->setSpeed(0);
      LeftBack->setSpeed(0);
      Bwd->setSpeed(0);
      RightFront->setSpeed(0);
      LeftFront->setSpeed(0);
      Fwd->setSpeed(255);
      Fwd->run(FORWARD);
      Serial.println("Controller Forward");
    }
    if (dir == 'D') {
      RightBack->setSpeed(0);
      Fwd->setSpeed(0);
      LeftBack->setSpeed(0);
      RightFront->setSpeed(0);
      LeftFront->setSpeed(0);
      Bwd->setSpeed(255);
      Bwd->run(FORWARD);
      Serial.println("Controller Backward");
    }
    if (dir == 'Q') {
      RightBack->setSpeed(0);
      Fwd->setSpeed(0);
      Bwd->setSpeed(0);
      LeftFront->setSpeed(0);
      LeftBack->setSpeed(255);
      LeftBack->run(FORWARD);
      RightFront->setSpeed(255);
      RightFront->run(FORWARD);
      Serial.println("Controller Turning Left");
    }
    if (dir == 'E') {
      Fwd->setSpeed(0);
      Bwd->setSpeed(0);
      LeftBack->setSpeed(0);
      RightFront->setSpeed(0);
      RightBack->setSpeed(255);
      RightBack->run(FORWARD);
      LeftFront->setSpeed(255);
      LeftFront->run(FORWARD);
      Serial.println("Controller Turning Right");
    }
    if (dir == 'O') {
      RightBack->setSpeed(0);
      Fwd->setSpeed(0);
      Bwd->setSpeed(0);
      LeftBack->setSpeed(0);
      RightFront->setSpeed(0);
      LeftFront->setSpeed(0);
      Serial.println("Controller Not Engaged");
    }
    if (dir == 'X') {
      //If the toggle is set to auto-pilot control
      Serial.println("Computer Time"); // Print to terminal that the auto-pilot is taking over
      if (Serial1.available() > 0) { // If there is data from the Pi Xbee
        val = Serial1.read();
        Serial.print(val);
        if (val == 'A') { //Initiator symbol for a new command
          for (int i = 0; i < 100; ++i) { //Read in the characters of the string
            val = Serial1.read();
            if (val == 'Z') { //If the end symbol '>' is read, end the loop
              ReadData = String(ReadData + val);
              Serial.print("Read Data:");
              Serial.println(ReadData); // Print the full string to the console
              break;
            }
            else {
              ReadData = String(ReadData + val); // Concatenate the new character to the end of the string
            }
          }
          ReadDataOld = ReadData; //Save the newly read data
        }
      }
      else {
        ReadData = ReadDataOld; //If no new data, then use the same data
      }

      //With Data read in, set the delta values from the string 
      for (int i = 0; i < 50; ++i) { //Go through the data string until
        if (ReadData[i] == 'A' ) { //If the start character is read
          Control_string = "";
          ++i; //Skip the A character
        }
        else if (ReadData[i] == 'B') { // If b character is read, convert current string to v[0]
          v[0] = Control_string.toInt();
          Control_string = "";
          ++i; //Skip the X character to start reading the first v value
        }
        else if (ReadData[i] == 'C') { // If c character is read, convert current string
          v[1] = Control_string.toInt();
          Control_string = "";
          ++i; 
        }
        else if (ReadData[i] == 'D') { // If d character is read, convert current string 
          v[2] = Control_string.toInt();
          Control_string = "";
          ++i;
        }
        else if (ReadData[i] == 'E') { // If e character is read, convert current string 
          v[3] = Control_string.toInt();
          Control_string = "";
          ++i;
        }
        else if (ReadData[i] == 'F') { // If d character is read, convert current string 
          v[4] = Control_string.toInt();
          Control_string = "";
          ++i;
        }
        else if (ReadData[i] == 'Z') { // If z character is read, convert current string 
          v[5] = Control_string.toInt();
          Control_string = "";
          break; //Exit this loop as data has been extracted from the string
        }
        Control_string = String(Control_string + ReadData[i]); // Add the characters to the string
      }
      Serial.print("v = [");
      for (int i = 0; i < 5; ++i) {
        Serial.print(v[i]); //Print the motor speeds set by the control code
        Serial.print(", "); 
      }
      Serial.print(v[5]);
      Serial.println(" ]");
      RightFront->setSpeed(v[0]);
      RightFront->run(FORWARD);
      LeftFront->setSpeed(v[1]);
      LeftFront->run(FORWARD);
      RightBack->setSpeed(v[2]);
      RightBack->run(FORWARD);
      LeftBack->setSpeed(v[3]);
      LeftBack->run(FORWARD);
      Fwd->setSpeed(v[4]);
      Fwd->run(FORWARD);
      Bwd->setSpeed(v[5]);
      Bwd->run(FORWARD);
      }
  }
      
  else { //With no commands from the Controller Xbee, turn off fans
    RightBack->setSpeed(0);
    Fwd->setSpeed(0);
    Bwd->setSpeed(0);
    LeftBack->setSpeed(0);
    RightFront->setSpeed(0);
    LeftFront->setSpeed(0);
  }
  Serial.print("Elapsed Time (ms)= "); Serial.println(millis()-t0);
}
