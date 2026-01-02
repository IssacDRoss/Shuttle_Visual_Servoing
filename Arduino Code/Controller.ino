void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); //Begins the serial port using TX and RX at Baud rate for Xbee
 pinMode(2, INPUT); //set up pins to receive input from joysticks
 pinMode(3, INPUT);
 pinMode(4, INPUT);
 pinMode(5, INPUT);
 pinMode(6, INPUT);
 pinMode(7, INPUT);
 pinMode(8, INPUT);
}

void loop() {
  if (digitalRead(8) == HIGH) { //If the Toggle is in the Down "Manual" position
    //Series of if statements to determine which direction joystick is aimed
    if (digitalRead(2) == HIGH) { 
     // Serial.println("Move Left");
      Serial.println("L");
    }
    if (digitalRead(3) == HIGH) {
     // Serial.println("Move Down");
      Serial.println("D");
    }
    if (digitalRead(4) == HIGH) {
     // Serial.println("Move Right");
      Serial.println("R");
    }
    if (digitalRead(5) == HIGH) {
     // Serial.println("Move Up");
      Serial.println("U");
    }
    if (digitalRead(6) == HIGH) {
     // Serial.println("Turn Right");
      Serial.println("Q");
    }
    if (digitalRead(7) == HIGH) {
     // Serial.println("Turn Left");
      Serial.println("E");
    }
  } else {
    //If the toggle is up "Autopilot" then send X, indicating that instructions should be taken from the other Xbee
    Serial.println("X");
  }
}
