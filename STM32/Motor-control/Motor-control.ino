 
#include <ODriveUART.h>
#define LED_GREEN PB0
#define LED_YELLOW PE1
#define LED_RED PB14

//This code designed to work with STM32 Nucleo-H723ZG Board

//---------------------Settings-----------------
const unsigned long baudrate = 115200; // Must match what you configure on the ODrive 
const long feedback_interval = 100; 
const int numMotors = 4;
//---------------------Settings-----------------

//Adding additional motors
//1. Make a new hardware serial
//2. Assign the new hardware serial to a HardwareSerial&
//3. Create a ODriveUART object
//4. Add the ODriveUART object to the motors array
//5. Begin serial communications inside setup

HardwareSerial Serial5(PB12, PB13); //Rx, Tx Tilt Sensor
HardwareSerial Serial7(PF6, PF7);   //Rx, Tx Pan Sensor
HardwareSerial Serial6(PC7, PC6);   //Rx, Tx Tilt shooter
HardwareSerial Serial9(PG0, PG1);   //Rx, Tx Pan shooter

HardwareSerial& serial_sensor_tilt = Serial5;
HardwareSerial& serial_sensor_pan = Serial7;
HardwareSerial& serial_shooter_tilt = Serial6;
HardwareSerial& serial_shooter_pan = Serial9;

ODriveUART sensor_pan(serial_sensor_pan);
ODriveUART sensor_tilt(serial_sensor_tilt);
ODriveUART shooter_pan(serial_shooter_pan);
ODriveUART shooter_tilt(serial_shooter_tilt);


ODriveUART* motors[numMotors] = {
  &sensor_pan, //A
  &sensor_tilt,//B
  &shooter_pan,//C
  &shooter_tilt//D
};

int start_character = 65; //The character in ascii to start representation of motors with. A = 65
unsigned long previousMillis = 0;
unsigned long currentMillis = millis();
float pos_current = 0;
float vel_current = 0;
float positions[numMotors] = {0, 0, 0, 0};
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
  serial_sensor_pan.begin(baudrate); 
  serial_sensor_tilt.begin(baudrate);
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

  //Send commands to odrive
  if (Serial.available() > 0) { 
    data = Serial.readStringUntil('\n'); 
    parse(data, positions); 
  }

  //Limit movement based on pan or tilt specs
  for (int i = 0; i < numMotors; i++) {
    // Movement limiter
    // Tilt maximum range 0-0.5. 0 is straight up and 0.5 straight down

    if (i == 0){ // Sensor Pan
      // If within limit, move freely
      if (feedback_motors[i].pos > -2.5 && feedback_motors[i].pos < 2.5) {
        motors[i]->setVelocity(positions[i]); 
      }
      // If above upper limit, only allow negative movement
      else if (feedback_motors[i].pos >= 2.5 && positions[i] < 0) {
        motors[i]->setVelocity(positions[i]); 
      }
      // If below lower limit, only allow positive movement
      else if (feedback_motors[i].pos <= -2.5 && positions[i] > 0) {
        motors[i]->setVelocity(positions[i]); 
      }
      else {
        motors[i]->setVelocity(0); 
      }
    }


    if (i == 1) { // Sensor Tilt
      // If within limit, move freely
      if (feedback_motors[i].pos < 0 && feedback_motors[i].pos > -0.40) {
        motors[i]->setVelocity(positions[i]); 
      }
      // If above upper limit, only allow negative movement
      else if (feedback_motors[i].pos >= 0 && positions[i] < 0) {
        motors[i]->setVelocity(positions[i]); 
      }
      // If below lower limit, only allow positive movement
      else if (feedback_motors[i].pos <= -0.4 && positions[i] > 0) {
        motors[i]->setVelocity(positions[i]); 
      }
      else {
        motors[i]->setVelocity(0); 
      }
    }

    if (i == 2){ // Shooter Pan
      // If within limit, move freely
      if (feedback_motors[i].pos > -2.5 && feedback_motors[i].pos < 2.5) {
        motors[i]->setVelocity(positions[i]); 
      }
      // If above upper limit, only allow negative movement
      else if (feedback_motors[i].pos >= 2.5 && positions[i] < 0) {
        motors[i]->setVelocity(positions[i]); 
      }
      // If below lower limit, only allow positive movement
      else if (feedback_motors[i].pos <= -2.5 && positions[i] > 0) {
        motors[i]->setVelocity(positions[i]); 
      }
      else {
        motors[i]->setVelocity(0); 
      }
    }


    if (i == 3) { // Shooter Tilt
      // If within limit, move freely
      if (feedback_motors[i].pos < 0 && feedback_motors[i].pos > -0.40) {
        motors[i]->setVelocity(positions[i]); 
      }
      // If above upper limit, only allow negative movement
      else if (feedback_motors[i].pos >= 0 && positions[i] < 0) {
        motors[i]->setVelocity(positions[i]); 
      }
      // If below lower limit, only allow positive movement
      else if (feedback_motors[i].pos <= -0.4 && positions[i] > 0) {
        motors[i]->setVelocity(positions[i]); 
      }
      else {
        motors[i]->setVelocity(0); 
      }
    }
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
void odrive_setup(ODriveUART& odrive) {
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



