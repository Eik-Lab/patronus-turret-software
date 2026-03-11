 
#include <ODriveUART.h>
#define LED_GREEN PB0
#define LED_YELLOW PE1
#define LED_RED PB14

//This code designed to work with STM32 Nucleo-H723ZG Board


//---------------------Settings-----------------
const unsigned long baudrate = 115200; // Must match what you configure on the ODrive 
const long timeout_interval = 500; 
const int numMotors = 2;
//---------------------Settings-----------------

//Adding additional motors
//1. Make a new hardware serial
//2. Assign the new hardware serial to a HardwareSerial&
//3. Create a ODriveUART object
//4. Add the ODriveUART object to the motors array
//5. Begin serial communications inside setup

HardwareSerial Serial5(PB12, PB13); //Rx, Tx
HardwareSerial Serial7(PF6, PF7); //Rx, Tx

HardwareSerial& serial_shooter_tilt = Serial5;
HardwareSerial& serial_shooter_pan = Serial7;

ODriveUART shooter_pan(serial_shooter_pan);
ODriveUART shooter_tilt(serial_shooter_tilt);

ODriveUART* motors[numMotors] = {
  &shooter_pan,
  &shooter_tilt,
};

unsigned long previousMillis = 0;
unsigned long currentMillis = millis();
long previousSpeed = 0;
float motor_x = 0.0;
float motor_y = 0.0;
float pos_change = 0.0;
float pos_y = 0.0;
float positions[2];
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
  ODriveFeedback feedback_shooter_pan = shooter_pan.getFeedback();  //Do not put outside void loop. Prevents arduino board from showing up in COM
  ODriveFeedback feedback_shooter_tilt = shooter_tilt.getFeedback();  //Do not put outside void loop. Prevents arduino board from showing up in COM

  // If not in closed state enter closed state
  for (int i = 0; i < numMotors; i++) {
    if (motors[i]->getState() != AXIS_STATE_CLOSED_LOOP_CONTROL) { 
      delay(1000);
      digitalWrite(LED_GREEN, LOW);
      odrive_setup(*motors[i]); 
    }
  }
  digitalWrite(LED_GREEN, HIGH);
  
  for (int i = 0; i < numMotors; i++) {
    motors[i]->setVelocity(1); 
  }

  if (Serial.available() > 0) { 
    data = Serial.readStringUntil('\n'); 
    parse(data, positions); 
    motor_x = positions[0]; 
    motor_y = positions[1]; 
  }

}

void parse(String data, float *inbound_feedback) {
  float X_index = data.indexOf("X")+1;
  float Y_index = data.indexOf("Y")+1;
  inbound_feedback[0] = data.substring(X_index, Y_index).toFloat();
  inbound_feedback[1] = data.substring(Y_index).toFloat();
}

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



