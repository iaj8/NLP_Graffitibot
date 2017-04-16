#include "Stepper.h"
#include <math.h>
/*#include "AccelStepper.h"*/

const int speed_in_rpm = 30;
const int num_steps_to_a_circle = 200;
const int singe_step_temp = 1;
const int stepper1_pin_coil_1 = D0;
const int stepper1_pin_coil_2 = D1;
const int stepper1_pin_coil_3 = D2;
const int stepper1_pin_coil_4 = D3;

const int stepper2_pin_coil_1 = A0;
const int stepper2_pin_coil_2 = A1;
const int stepper2_pin_coil_3 = A2;
const int stepper2_pin_coil_4 = A3;

const int servoPin = A4;

int a[][2] = {{1,2},{3,4}};

Servo servo;
int pos = 0;
char val;
bool up = false;
bool down = false;
bool left = false;
bool right = false;
bool pen_up = true;
int inputSize;


Stepper stepper1(num_steps_to_a_circle, stepper1_pin_coil_1, stepper1_pin_coil_2, stepper1_pin_coil_3, stepper1_pin_coil_4);
//
Stepper stepper2(num_steps_to_a_circle, stepper2_pin_coil_1, stepper2_pin_coil_2, stepper2_pin_coil_3, stepper2_pin_coil_4);
//

//ACCEL LIBRARY NOT WORKING RIGHT NOW
/*AccelStepper accelStepper(stepper2_pin_coil_3, stepper2_pin_coil_4);*/


int angle;
int step = 0;
int direction = 1;
int range;
String output;

void setup()
{
  Serial.begin(9600);
  stepper1.setSpeed(speed_in_rpm);
  stepper2.setSpeed(speed_in_rpm);

  /*accelStepper.setMaxSpeed(200);
  accelStepper.setSpeed(200);*/

  servo.attach(servoPin);

  pinMode(stepper1_pin_coil_1, OUTPUT);
  pinMode(stepper1_pin_coil_2, OUTPUT);
  pinMode(stepper1_pin_coil_3, OUTPUT);
  pinMode(stepper1_pin_coil_4, OUTPUT);

  pinMode(stepper2_pin_coil_1, OUTPUT);
  pinMode(stepper2_pin_coil_2, OUTPUT);
  pinMode(stepper2_pin_coil_3, OUTPUT);
  pinMode(stepper2_pin_coil_4, OUTPUT);
}

void loop()
{
  if (up){
    moveOneUnitInPositiveYDirection();
  }
  if (down){
    moveOneUnitInNegativeYDirection();
  }
  if (left){
    moveOneUnitInNegativeXDirection();
  }
  if (right){
    moveOneUnitInPositiveXDirection();
  }
  if (Serial.available() > 0){
    val = Serial.read();
    switch (val) {
      case 'u':
        Serial.println(a[1]);
        up = true;
        break;
      case 'd':
        down = true;
        break;
      case 'l':
        left = true;
        break;
      case 'r':
        right = true;
        break;
      case 'q':
        //one step up
        stopMovement();
        moveOneUnitInPositiveYDirection();
        break;
      case 'w':
        //one step down
        stopMovement();
        moveOneUnitInNegativeYDirection();
        break;
      case 'e':
        //one step left
        stopMovement();
        moveOneUnitInNegativeXDirection();
        break;
      case 't':
        //one step right
        stopMovement();
        moveOneUnitInPositiveXDirection();
        break;
      case 'p':
      //move the pen up and down
        if (pen_up){
          pen_up = !pen_up;
          liftPen();
        } else{
          pen_up = !pen_up;
          releasePen();
        }
        stopMovement();
        break;
      case 'c':
      //draw circle
        drawCircle();
        break;
      case 'y':
      //draw square
        inputSize = Serial.parseInt();
        if (inputSize < 500 && inputSize > 0){
          drawSquare(inputSize);
        } else{
          drawSquare(100);
        }
        break;
      case 'i':
      //draw triangle
        inputSize = Serial.parseInt();
        if (inputSize < 500 && inputSize > 0){
          drawTriangle(inputSize);
        } else{
          drawTriangle(100);
        }
        break;
      case 's':
      //stop moving in any direction
        stopMovement();
        break;

    }
    delay(50);
  }
}

void liftPen(){
  int angle = 0;
  while (angle < 90){
    Particle.process();
    servo.write(-1);
    delay(10);
    angle++;
  }
}
void releasePen(){
  int angle = 0;
  while (angle < 90){
    Particle.process();
    servo.write(1);
    delay(10);
    angle++;
  }
}

void moveOneUnitInNegativeXDirection(){
  //go left
  stepper1.step(-1*singe_step_temp);
  stepper2.step(-1*singe_step_temp);
}

void moveOneUnitInPositiveXDirection(){
  //go right
  stepper1.step(singe_step_temp);
  stepper2.step(singe_step_temp);
}

void moveOneUnitInPositiveYDirection(){
  //go up
  stepper1.step(-1*singe_step_temp);
  stepper2.step(singe_step_temp);
}

void moveOneUnitInNegativeYDirection(){
  //go down
  stepper1.step(singe_step_temp);
  stepper2.step(-1*singe_step_temp);
}

void stopMovement(){
  up = false;
  down = false;
  left = false;
  right = false;
}


void drawSquare(int size){
  int currSize = size;
  while(currSize > 0){
    Particle.process();
    moveOneUnitInPositiveYDirection();
    currSize--;
  }
  currSize = size;
  while(currSize > 0){
    Particle.process();
    moveOneUnitInPositiveXDirection();
    currSize--;
  }
  currSize = size;
  while(currSize > 0){
    Particle.process();
    moveOneUnitInNegativeYDirection();
    currSize--;
  }
  currSize = size;
  while(currSize > 0){
    Particle.process();
    moveOneUnitInNegativeXDirection();
    currSize--;
  }
}


void drawTriangle(int size){
  int currSize = size;
  while(currSize > 0){
    Particle.process();
    moveOneUnitInPositiveXDirection();
    currSize--;
  }
  currSize = size;
  while(currSize > 0){
    Particle.process();
    moveOneUnitInPositiveYDirection();
    moveOneUnitInNegativeXDirection();
    currSize--;
  }
  currSize = size;
  while(currSize > 0){
    Particle.process();
    moveOneUnitInNegativeYDirection();
    currSize--;
  }
}

void drawCircle(){
  float theta = 0.0; //current angle
  float h = 0.0; // x coordinate of circles center
  float k = 0.0; // y coordinate of circles center
  float stp = 0.8;
  float currX;
  float currY;
  float prevX = h;
  float prevY = k;
  float stepsInXDirection;
  float stepsInYDirection;
  float r = 15.0; //radius
  float pi = 22.0/7.0;
  int actualXSteps;
  int actualYSteps;
  while (theta < 360.0){
    Particle.process();
    float thetaInRadians = (theta*pi)/180.0;
    Serial.print("thetaInRadians: ");
    Serial.println(thetaInRadians);
    currX = h + r*cos(thetaInRadians);
    Serial.print("currX: ");
    Serial.println(currX);
    Serial.print("prevX: ");
    Serial.println(prevX);
    currY = k + r*sin(thetaInRadians);
    Serial.print("currY: ");
    Serial.println(currY);
    Serial.print("prevY: ");
    Serial.println(prevY);
    stepsInXDirection = currX - prevX;
    Serial.print("stepsInXDirection: ");
    Serial.println(stepsInXDirection);
    stepsInYDirection = currY - prevY;
    Serial.print("stepsInYDirection: ");
    Serial.println(stepsInYDirection);
    prevX = currX;
    prevY = currY;
    actualXSteps = (int)(10*stepsInXDirection);
    actualYSteps = (int)(10*stepsInYDirection);
    while (actualXSteps != 0 && actualYSteps != 0){
      Particle.process();
      if (actualXSteps < 0){
        moveOneUnitInNegativeXDirection();
        Serial.println("moving x -1");
        actualXSteps ++;
      }
      if (actualXSteps > 0){
        moveOneUnitInPositiveXDirection();
        Serial.println("moving x 1");
        actualXSteps --;
      }
      if (actualYSteps < 0){
        moveOneUnitInNegativeYDirection();
        Serial.println("moving y -1");
        actualYSteps ++;
      }
      if (actualYSteps > 0){
        moveOneUnitInPositiveYDirection();
        Serial.println("moving y 1");
        actualYSteps --;
      }
    }
    theta += stp;
  }

}
