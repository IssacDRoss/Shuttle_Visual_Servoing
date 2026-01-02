#include <Wire.h>
#include <Adafruit_MotorShield.h>

// put your setup code here, to run once:
// Sets thresholds for control
int theta_threshold = 5;
int pos_threshold = 1;
// theta in degrees, x/y in meters times conversion
String ReadDataOld = "AT-15X30Y20Z";
float conversion=25;
// Set Zeros and Poles for seperate controllers.
// 0 is theta, 1 is x, 2 is y
float Z[] = {0.25, 0.315, 0.21};
float P[] = {15.0, 5.0, 10.0};
float K[] = {0.5, 3, 6};
float dt = 0.02;
//Initialize control coefficient vectors
float a0[3];
float c1[3];
float c2[3];
float c3[3];
int t0;
// Initialize values for the control loop
float uOld[] = {0, 0, 0};
float u[] = {0, 0, 0};
float deltaOld[] = {0, 0, 0};
//Command line from python in form of dT,dX,dY
float delta[] = {0, 0, 0}; 
int m = 16600; //Slope of the fit line for command-v
float b = 49; //intercept of the fit line for command-v
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

void setup() {
  //Calculate control coefficients according to Tustin's rule
  for (int i = 0; i < 3; ++i) {
    a0[i] = P[i] * dt + 2;
    c1[i] = (dt * K[i] * Z[i] + 2.0 * K[i]) / a0[i];
    Serial.println("c1");
    Serial.println(c1[i]);
    c2[i] = (dt * K[i] * Z[i] - 2.0 * K[i]) / a0[i];
    Serial.println("c2");
    Serial.println(c2[i]);
    c3[i] = (P[i] * dt - 1.0) / a0[i];  
    Serial.println("c3");
    Serial.println(c3[i]);  }
  AFMStop.begin(); //Starts the connection with the motor shields
  AFMSbottom.begin();
  //Starts the serial connections with Xbees on sets of TX and RX pins
  Serial2.begin(115200); 
  Serial1.begin(115200);
  //Starts communication for messages to be read on computer
  Serial.begin(115200);} 

void loop() {
  t0=millis();//Initialize time stamp
  // Reset the data strings
  String ReadData = "";
  String Control_string = "";
  int v[] = {0, 0, 0, 0, 0, 0}; //Motor speeds initialized at 0
  char dir;
  Serial.println("Loop Begins");
  if (Serial2.available()) { 
    //If there is a message from the controller Xbee
    dir = Serial2.read();
    Serial.println(dir);
    if (dir == 'R') { //If the controller sent the signal 'R'
      RightBack->setSpeed(0);
      Fwd->setSpeed(0);
      Bwd->setSpeed(0);
      RightFront->setSpeed(0);
      //Set motor to full power
      LeftFront->setSpeed(255);
      //Run the motor forward at the set speed
      LeftFront->run(FORWARD); 
      LeftBack->setSpeed(255); 
      LeftBack->run(FORWARD); 
      Serial.println("Controller Right");} 
      //Print to the terminal what control was received
    if (dir == 'L') {
      LeftBack->setSpeed(0);
      Fwd->setSpeed(0);
      Bwd->setSpeed(0);
      LeftFront->setSpeed(0);
      RightFront->setSpeed(255);
      RightFront->run(FORWARD);
      RightBack->setSpeed(255);
      RightBack->run(FORWARD);
      Serial.println("Controller Left");    }
    if (dir == 'U') {
      RightBack->setSpeed(0);
      LeftBack->setSpeed(0);
      Bwd->setSpeed(0);
      RightFront->setSpeed(0);
      LeftFront->setSpeed(0);
      Fwd->setSpeed(255);
      Fwd->run(FORWARD);
      Serial.println("Controller Forward");    }
    if (dir == 'D') {
      RightBack->setSpeed(0);
      Fwd->setSpeed(0);
      LeftBack->setSpeed(0);
      RightFront->setSpeed(0);
      LeftFront->setSpeed(0);
      Bwd->setSpeed(255);
      Bwd->run(FORWARD);
      Serial.println("Controller Backward");    }
    if (dir == 'Q') {
      RightBack->setSpeed(0);
      Fwd->setSpeed(0);
      Bwd->setSpeed(0);
      LeftFront->setSpeed(0);
      LeftBack->setSpeed(255);
      LeftBack->run(FORWARD);
      RightFront->setSpeed(255);
      RightFront->run(FORWARD);
      Serial.println("Controller Turning Left");    }
    if (dir == 'E') {
      Fwd->setSpeed(0);
      Bwd->setSpeed(0);
      LeftBack->setSpeed(0);
      RightFront->setSpeed(0);
      RightBack->setSpeed(255);
      RightBack->run(FORWARD);
      LeftFront->setSpeed(255);
      LeftFront->run(FORWARD);
      Serial.println("Controller Turning Right");    }
    if (dir == 'O') {
      RightBack->setSpeed(0);
      Fwd->setSpeed(0);
      Bwd->setSpeed(0);
      LeftBack->setSpeed(0);
      RightFront->setSpeed(0);
      LeftFront->setSpeed(0);
      Serial.println("Controller Not Engaged");    }
    if (dir == 'X') {
      //If the toggle is set to auto-pilot control
      Serial.println("Computer Time"); 
      // Print to terminal that the auto-pilot is taking over
      if (Serial1.available() > 0) { 
        // If there is data from the Pi Xbee
        val = Serial1.read();
        Serial.print(val);
        if (val == 'A') { //Initiator symbol for a new command
          for (int i = 0; i < 50; ++i) { 
            //Read in the characters of the string
            val = Serial1.read();
            if (val == 'Z') { 
              //If the end symbol '>' is read, end the loop
              ReadData = String(ReadData + val);
              Serial.print("Read Data:");
              Serial.println(ReadData); 
              // Print the full string to the console
              break;            }
            else {
              ReadData = String(ReadData + val); }
              // Concatenate the new char to end of the string
          }
          ReadDataOld = ReadData; //Save the newly read data
        }
      }
      else {
        ReadData = ReadDataOld;} 
        //If no new data, then use the same data

      //With Data read in, set the delta values from the string
      for (int i = 0; i < 50; ++i) { //Go through the data string
        if (ReadData[i] == 'T' ) { 
          //If the start character is read
          Control_string = "";
          i = i + 1;} 
          //Skip the T character to reach the actual Theta value
        else if (ReadData[i] == 'X') { 
          // If X character is read, convert string to dTheta
          delta[0] = Control_string.toInt();
          Control_string = "";
          ++i;} //Skip the X character to start reading the X value
        else if (ReadData[i] == 'Y') {
          // If Y character is read, convert current string to dX
          delta[1] = Control_string.toInt()/conversion;
          Control_string = "";
          ++i;} //Skip the Y character to start reading the Y value
        else if (ReadData[i] == '>') { 
          // If end character is read, convert current string to dY
          delta[2] = Control_string.toInt()/conversion;
          //Exit this loop as data has been extracted from the string
          break;} 
        // Add the characters to the string
        Control_string = String(Control_string + ReadData[i]); }
      //Print the values of delta read in for control
      Serial.println("Delta= ");
      Serial.print(delta[0]); Serial.print(", "); 
      Serial.print(delta[1]); Serial.print(", "); 
      Serial.println(delta[2]);
      //With delta values (e) find the forces desired u=[T,Fx,Fy]
      //and the corresponding forces of each fan F
      if (abs(delta[0]) > theta_threshold) {
        //If theta is over a threshold, only consider that and 
        //set torque to rotate until orientation is correct.
        //calculate the command
        u[0] = c1[0] * delta[0] + c2[0] * deltaOld[0] - c3[0] * uOld[0]; 
        Serial.print("Theta U= "); Serial.println(u[0]);
        if (u[0] > 0) { 
          //If the command is to rotate CCW, trigger fans 0 & 3
          v[0]=m*u[0];
          v[3]=v[0];        }
        else { //Command is to rotate CW, trigger fans 1 & 2
          v[1] = -m*u[0];
          v[2]=v[1];        }
      }
      else if (abs(delta[1]) > pos_threshold || abs(delta[2]) > pos_threshold) {
        //Next, if X and or Y are over a threshold, 
        //correct the positioning while keeping an eye on theta 
        u[0] = c1[0] * delta[0] + c2[0] * deltaOld[0] - c3[0] * uOld[0]; 
        u[1] = c1[1] * delta[1] + c2[1] * deltaOld[1] - c3[1] * uOld[1];
        u[2] = c1[2] * delta[2] + c2[2] * deltaOld[2] - c3[2] * uOld[2];
        Serial.print("Theta U= "); Serial.print(u[0]);
        Serial.print(", X U= "); Serial.print(u[1]); 
        Serial.print(", Y U= "); Serial.println(u[2]);
        if (u[0] > 0) { 
          //If the command is to rotate CCW, trigger fans 0 & 3
          v[0] = m*u[0];
          v[3] = v[0];        }
        else { 
          //Command is to rotate CW, trigger fans 1 & 2
          v[1] = -m*u[0];
          v[2] = v[1];        }
        if (u[1] > 0) { 
          //if command is to move left
          v[0] = m * u[1] + b + abs(v[0]);
          v[2] = m * u[1] + b + abs(v[2]);        }
        else { 
          //command to move right
          v[1] = -m * u[1] + b + abs(v[1]);
          v[3] = -m * u[1] + b + abs(v[3]);        }
        if (u[2] > 0) { 
          //if command is to move BWD
          v[5] = m * u[2] + b;        }
        else { 
          //cmd to move FWD
          v[4] = -m * u[2] + b;        }
      }
      else { 
        //If both position and attitude have been corrected
        Serial.println("You are docked!");      }
      //Threshold values to the max and min motor speed value
      for (int i = 0; i < 6; ++i) {
        if (v[i] > 255 || v[i]<0) {
          //If over the max, or wrapped the max int value
          v[i] = 255;        }
        else if (v[i] > 0 && v[i] < 50) {
          v[i] = 50;        }
      }
      Serial.print("v= [");
      for (int i = 0; i < 6; ++i) {
        //Print the motor speeds set by the control code
        Serial.print(v[i]); 
        Serial.print(", ");       }
      Serial.println(" ]");
      for (int i = 0; i < 3; ++i) { 
        deltaOld[i]=delta[i];
        uOld[i]=u[i];       }
      //Set motor speeds and run all fans at said speeds
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
      Bwd->run(FORWARD);      }
  }
  else { //With no commands from the Controller Xbee, turn off fans
    RightBack->setSpeed(0);
    Fwd->setSpeed(0);
    Bwd->setSpeed(0);
    LeftBack->setSpeed(0);
    RightFront->setSpeed(0);
    LeftFront->setSpeed(0);  }
  Serial.print("Elapsed Time (ms)= "); Serial.println(millis()-t0);
}
