 
#include <ODriveUART.h>
#define LED_GREEN PB0
#define LED_YELLOW PE1
#define LED_RED PB14

//This code designed to work with STM32 Nucleo-H723ZG Board

//---------------------Settings-----------------
const unsigned long baudrate = 115200; // Must match what you configure on the ODrive 
const long feedback_interval = 100; 
const int numMotors = 2;
//---------------------Settings-----------------

//Adding additional motors
//1. Make a new hardware serial
//2. Assign the new hardware serial to a HardwareSerial&
//3. Create a ODriveUART object
//4. Add the ODriveUART object to the motors array
//5. Begin serial communications inside setup

HardwareSerial Serial5(PB12, PB13); //Rx, Tx
HardwareSerial Serial7(PF6, PF7);   //Rx, Tx

HardwareSerial& serial_shooter_tilt = Serial5;
HardwareSerial& serial_shooter_pan = Serial7;

ODriveUART shooter_pan(serial_shooter_pan);
ODriveUART shooter_tilt(serial_shooter_tilt);

ODriveUART* motors[numMotors] = {
  &shooter_pan,
  &shooter_tilt,
};

int start_character = 65; //The character in ascii to start representation of motors with. A = 65
unsigned long previousMillis = 0;
unsigned long currentMillis = millis();
float positions[numMotors];
ODriveFeedback feedback_motors[numMotors];
String data;

void setup() {
  Serial.begin(baudrate);
  
  //LEDs for error messages
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);

  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_YELLOW, LOW);
  digitalWrite(LED_GREEN, LOW);

  //Initiate serial connection with odrive 
  serial_shooter_pan.begin(baudrate); 
  serial_shooter_tilt.begin(baudrate);
}

void loop() {
  unsigned long currentMillis = millis();

  // If not in closed state enter closed state
  for (int i = 0; i < numMotors; i++) {
    if (motors[i]->getState() != AXIS_STATE_CLOSED_LOOP_CONTROL) { 
      delay(1000);
      digitalWrite(LED_GREEN, LOW);
      odrive_setup(*motors[i]); 
    }
  }
  digitalWrite(LED_GREEN, HIGH);

  //Send commands to odrive
  if (Serial.available() > 0) { 
    data = Serial.readStringUntil('\n'); 
    parse(data, positions); 
    for (int i = 0; i < numMotors; i++) {
      motors[i]->setVelocity(positions[i]); 
    }
    
  }

  //Send position every feedback_interval
  if (currentMillis - previousMillis >= feedback_interval) {
    for (int i = 0; i < numMotors; i++) {
      feedback_motors[i] = motors[i]->getFeedback(); 
      Serial.write(start_character+i);
      Serial.print(feedback_motors[i].pos);
    }
    Serial.println();
    previousMillis = currentMillis;
  }

}

//Parse data received over serial communication
void parse(String data, float *inbound_feedback) {
  int motor_index[numMotors];

  //Finds the index of each character the represents a motor
  for (int i = 0; i < numMotors; i++) {
    motor_index[i] = data.indexOf(start_character+i)+1;
  }

  //Parse the string to obtain input values for motors
  for (int i = 0; i < numMotors-1; i++) {
    inbound_feedback[i] = data.substring(motor_index[i], motor_index[i+1]).toFloat();
  }
  inbound_feedback[numMotors-1] = data.substring(motor_index[numMotors-1]).toFloat();
}

//Run a setup procedure on drive motors
void odrive_setup(ODriveUART odrive) {
  // Waiting for Odrive to enter different state
  while (odrive.getState() == AXIS_STATE_UNDEFINED) {
    digitalWrite(LED_YELLOW, !digitalRead(LED_YELLOW));
    delay(100);
  }
  digitalWrite(LED_YELLOW, LOW);

  while (odrive.getState() != AXIS_STATE_CLOSED_LOOP_CONTROL) {
    odrive.clearErrors();
    odrive.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
    delay(10);
  }  
}



