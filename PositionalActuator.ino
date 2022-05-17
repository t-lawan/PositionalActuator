#include <ArduinoQueue.h>

#include <Array.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <SPI.h>    

#include <OSCBundle.h>
#include <OSCBoards.h>

const int ID =1;
const byte POSITION_SIGNAL_PIN = 2;
const int MOTOR_PIN = 12;
const int BRAKE_PIN = 9;
const int SPEED_PIN = 3;
int testStartTime = 0;
//const int MAX_HALL_DISTANCE = 1710;
const int MAX_HALL_DISTANCE = 1700;
const int OPEN_STROKE_IN_MM = 145;
//const int OPEN_STROKE_IN_MM = 150;
//const int CLOSE_STROKE_IN_MM = 245;
//const int CLOSE_STROKE_IN_MM = 265;
const int CLOSE_STROKE_IN_MM = 205;
const int MAX_STROKE_IN_MM = 300;

const int UPDATE_POSITION_RATE = 10;
const int UPDATED_SPEED_RATE = 200;
int CURRENT_POSITION = -1; //-1 is retracted  0 is open || 1 is close
int distance = MAX_HALL_DISTANCE;
bool hasCurrent = false;
const int QUEUE_SIZE = 5;
ArduinoQueue<int> currentValues(QUEUE_SIZE);
ArduinoQueue<long> pastPos(QUEUE_SIZE);
// Ethernet stuff
byte mac[] = {
  0xA8, 0x61, 0x0A, 0xAE, 0x79, 0xD9
};



IPAddress ip(192, 168, 1, ID * 10);
IPAddress myDns(192, 168, 1, 1);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 0, 0);

EthernetUDP Udp;

byte interrupt = digitalPinToInterrupt(POSITION_SIGNAL_PIN);
long pos = 0; // Actuator Position
long prevPos = 0;  // Previous Position
volatile long steps = 0;                 // Pulses from  Hall Effect sensors
long prevSteps = 0; // Previous step counted
float conNum = 0.000285;        // Convert to Inches
bool dir = 0;                   // Direction of Actuator (0=Retract, 1=Extend)
int Speed = 0;
bool homeFlag = 0;
unsigned long prevTimer = -1;
unsigned long lastSpeedUpdated = -1;
unsigned long lastStepTime = 0;
//int trigDelay = 50;
//int trigDelay = 500;
//int trigDelay = 620;
int trigDelay = 550;
bool isMoving = false;

void countSteps() {
  if (micros() - lastStepTime > trigDelay) {
    steps++;
    lastStepTime = micros();
  }
}

void setupActuator() {
  pinMode(MOTOR_PIN, OUTPUT); //Initiates Motor Channel A pin
  pinMode(BRAKE_PIN, OUTPUT); //Initiates Brake Channel A pin

  pinMode(0, INPUT);
  pinMode(POSITION_SIGNAL_PIN, INPUT);

  digitalWrite(POSITION_SIGNAL_PIN, HIGH);
  attachInterrupt(interrupt, countSteps, RISING);
}

void setupEthernet() {
  if (ID == 2) {
    byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
  }

  if (ID == 3) {
    byte mac[] = { 0x32, 0xBD, 0x84, 0x96, 0x5F, 0x50 };
  }

  Ethernet.begin(mac,ip);
  Udp.begin(5000 + ID);
  Serial.println(ip);

}

/* Updates Position */
void updatePosition(void) {
  pos = pos + steps;
  steps = 0;
}

void openActuator(void) {

  //  if(millis() - lastSpeedUpdated > UPDATED_SPEED_RATE){
  //    lastSpeedUpdated = millis();
  //    updateSpeedValue();
  //  }

  Speed = 255;
  digitalWrite(MOTOR_PIN, HIGH); //Establishes forward direction of Channel A
  digitalWrite(BRAKE_PIN, LOW);   //Disengage the Brake for Channel A
  analogWrite(SPEED_PIN, 255);
}

void closeActuator(void) {
  //  updateSpeedValue();
  //  if(millis() - lastSpeedUpdated > UPDATED_SPEED_RATE){
  //    lastSpeedUpdated = millis();
  //    updateSpeedValue();
  //  }
  Speed = 255;
  digitalWrite(MOTOR_PIN, LOW); //Establishes backward direction of Channel A
  digitalWrite(BRAKE_PIN, LOW);   //Disengage the Brake for Channel A
  analogWrite(SPEED_PIN, 255);
}

void stopActuator(void) {
  digitalWrite(BRAKE_PIN, HIGH);
  analogWrite(SPEED_PIN, 0);
  isMoving = false;

  pos = 0;
  Serial.println("STOP");

  for (int i = 0; i < currentValues.maxQueueSize(); i++) {
    currentValues.dequeue();
  }
}

void moveActuator(int message) {
  
  //0 is open || 1 is close
  if (message == 0 || message == 1) {

//    if (message != CURRENT_POSITION) {
      //      if (message == 0 && CURRENT_POSITION ==  1) {
      if (message == 0) {
        // Retract to Open from Close
        retractToOpenFromCloseState();
      }

      //      if (message == 1 && CURRENT_POSITION == 0) {
      if (message == 1) {
        // Extend to Close from Open
        extend();
      }

      //      CURRENT_POSITION = message;
//    } else {
//      stopActuator();
//    }
  }

  if(message == 2){
    stopActuator();  
  }

  //   detachInterrupt(interrupt);
  if (message == 5) {
    retract();
  }

  if (message == 6) {
    extend();
    //17
  }

  if (message == 7) {
    extendToOpenState();
    //22.3
  }

  if (message == 8) {
    extendToCloseState();
  }
  //  delay(50);
  //    attachInterrupt(interrupt, countSteps, RISING);

}

void updateCurrent() {
  hasCurrent = true;
  // Check if queue object has 5
  if (currentValues.itemCount() == QUEUE_SIZE) {
    // Remove first item in queue
    currentValues.dequeue();
  }

  // Check if queue object is less than 5
  if (currentValues.itemCount() < QUEUE_SIZE) {
    // Add object to last item in queue
    currentValues.enqueue(analogRead(0));

    //Check first and last value in queue
    int maxValue = currentValues.getHead() + currentValues.dequeue();

    // Read queue
    currentValues.enqueue(analogRead(0));
    // If there is no current and currenValue isn't 5
    if (maxValue == 0 && currentValues.itemCount() == QUEUE_SIZE) {
      hasCurrent = false;
    }
  }
}

void comparePosition() {
  hasCurrent = true;
  if (pastPos.itemCount() == QUEUE_SIZE) {
    // Remove first item in queue
    pastPos.dequeue();
  }

  // Check if queue object is less than 5
  if (pastPos.itemCount() < QUEUE_SIZE) {
    // Add object to last item in queue
    pastPos.enqueue(pos);

    //Check first and last value in queue
    int maxValue = pastPos.getHead() + pastPos.dequeue();

    // Read queue
    currentValues.enqueue(pos);
    // If there is no current and currenValue isn't 5
    if ( pastPos.getHead() == pastPos.dequeue() && pastPos.itemCount() == QUEUE_SIZE) {
      hasCurrent = false;
    }
  }
}


void updateSpeedValue() {
  float minSpeed = 128.0;
  float maxSpeed = 255.0;
  int halfRange = (maxSpeed - minSpeed) / 2;

  // Gets th
  float radian = mapf((float) pos, 0.0, (float) distance, 0.00, 3.142);

  //  radian = constrain(radian, 0, 3.142);
  Speed = mapf(sin(radian), 0.0, 1.0, minSpeed, maxSpeed);
  //    Speed =  minSpeed + halfRange + sin(degree) * halfRange;

}

float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void retract() {
  dir = 0;
  Speed = 255;
  updateCurrent();

  if (hasCurrent) {
    closeActuator();
    if (millis() - prevTimer > UPDATE_POSITION_RATE) {                // Update the Position Every 1/10 seocond
      updatePosition();
      //      comparePosition();
      prevTimer = millis();
      //      if (pos == prevPos | pos == 0) {
      //        pos = 0; // Corrects Position
      //      }
      //      else {
      //        prevPos = pos;
      //      }
    }
  } else {
    CURRENT_POSITION = -1;
    stopActuator();
  }



}

void extend() {
  dir = 1;
  Speed = 255;
  updateCurrent();

  if (hasCurrent) {
    openActuator();
    if (millis() - prevTimer > UPDATE_POSITION_RATE) {                // Update the Position Every 1/10 seocond
      updatePosition();
      //      comparePosition();
      prevTimer = millis();

      //      if (pos == prevPos | pos == 49000) {
      //        pos = 49000; // Corrects Position
      //      }
      //      else {
      //        prevPos = pos;
      //      }
    }
  } else {
    stopActuator();
  }

}

void extendToCloseFromOpenState() {
  dir = 1;
  Speed = 255;
  float distanceToMove = (float) MAX_HALL_DISTANCE * ((float ) (CLOSE_STROKE_IN_MM - OPEN_STROKE_IN_MM) / (float) MAX_STROKE_IN_MM);
//  distance =  (int) (distanceToMove - 0.1);
  distance =  (int) (distanceToMove);

  if (pos <= distance) {

    openActuator();
    if (millis() - prevTimer > UPDATE_POSITION_RATE) {                // Update the Position Every 1/10 seocond
      updatePosition();
      prevTimer = millis();
      //      if (pos == prevPos | pos == 49000) {
      //        pos = 49000; // Corrects Position
      //      }
      //      else {
      //        prevPos = pos;
      //      }
    }
  }  else {
    CURRENT_POSITION = 1;
    Serial.print("POSITION: ");
    Serial.println(pos);
    stopActuator();
  }
}

void retractToOpenFromCloseState() {
  
  dir = 0;
  Speed = 255;
  float distanceToMove = (float) MAX_HALL_DISTANCE * ((float ) (CLOSE_STROKE_IN_MM - OPEN_STROKE_IN_MM) / (float) MAX_STROKE_IN_MM);
  distance =  (int) distanceToMove;
  if (pos <= distance) {

    closeActuator();
    if (millis() - prevTimer > UPDATE_POSITION_RATE) {                // Update the Position Every 1/10 seocond
      updatePosition();
      prevTimer = millis();
      //      if (pos == prevPos | pos == 49000) {
      //        pos = 49000; // Corrects Position
      //      }
      //      else {
      //        prevPos = pos;
      //      }
    }
  }  else {
    CURRENT_POSITION = 0;
    Serial.print("POSITION: ");
    Serial.println(pos);
    stopActuator();

  }
}

void extendToOpenState() {
  dir = 1;
  Speed = 255;
  float distanceToMove = (float) MAX_HALL_DISTANCE * ((float )OPEN_STROKE_IN_MM / (float) MAX_STROKE_IN_MM);
  distance =  (int) distanceToMove;

  if (pos <= distance) {
    openActuator();
    if (millis() - prevTimer > UPDATE_POSITION_RATE) {                // Update the Position Every 1/10 seocond
      updatePosition();
      prevTimer = millis();
      //      if (pos == prevPos | pos == 49000) {
      //        pos = 49000; // Corrects Position
      //      }
      //      else {
      //        prevPos = pos;
      //      }
    }
  }  else {
    CURRENT_POSITION = 0;
    Serial.print("POSITION: ");
    Serial.println(pos);
    stopActuator();

  }

}

void extendToCloseState() {
  dir = 1;
  Speed = 255;
  float distanceToMove = (float) MAX_HALL_DISTANCE * ((float )CLOSE_STROKE_IN_MM / (float) MAX_STROKE_IN_MM);
  distance =  (int) distanceToMove;
  if (pos <= distance) {
    openActuator();
    if (millis() - prevTimer > UPDATE_POSITION_RATE) {                // Update the Position Every 1/10 seocond
      updatePosition();
      prevTimer = millis();
      //      if (pos == prevPos | pos == 49000) {
      //        pos = 49000; // Corrects Position
      //      }
      //      else {
      //        prevPos = pos;
      //      }
    }
  }  else {
    CURRENT_POSITION = 1;
    Serial.print("POSITION: ");
    Serial.println(pos);
    stopActuator();
  }

}


void homeActuator(void) {
  if (hasCurrent && homeFlag == 0) {
    openActuator();
    if (millis() - prevTimer > UPDATE_POSITION_RATE) {                // Update the Position Every 1/10 seocond
      updatePosition();
      //      comparePosition();
      prevTimer = millis();

      //      if (pos == prevPos | pos == 49000) {
      //        pos = 49000; // Corrects Position
      //      }
      //      else {
      //        prevPos = pos;
      //      }
    }
  } else {
    homeFlag = 1;
    stopActuator();
  }
}

/* Converts Position to Inches */
float convertToInches(long pos) {
  return conNum * pos;
}

int convertToMillimetres(long pos) {
  float mm = convertToInches(pos) * 25.4;
  return (int) mm;
}

void setup() {
  Serial.begin(9600);
  // setupEthernet();
  setupActuator();
}

void handleMessage(OSCMessage &message, int addrOffset){
  char * strBuffer = "";
//  int val = message.getString(0,strBuffer);
  int val = message.getInt(0);

}

void loop () {
    if (homeFlag == 0) {
      extend();
      homeFlag = 1;
    }

    if (!isMoving && Serial.available() > 0) {
    // read the incoming byte:
    Serial.println("MSG RCVD");
    String message = "";
    message = Serial.readString();
//    int message = Serial.read() - '0';
    isMoving = true;
    // say what you got:
    Serial.print("I received: ");
    Serial.println(message);

     while (isMoving) {
        moveActuator(message.toInt());
     }
  }


};
