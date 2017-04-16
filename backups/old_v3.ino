// MultiStepper.pde
// -*- mode: C++ -*-
// Use MultiStepper class to manage multiple steppers and make them all move to
// the same position at the same time for linear 2d (or 3d) motion.
#include <AccelStepper.h>
#include <MultiStepper.h>

//left motor stuff
const int l_motorDirPin = D0; //digital pin 0
const int l_motorStepPin = D1; //digital pin 1
AccelStepper stepper1(AccelStepper::FULL2WIRE, l_motorDirPin, l_motorStepPin);


//right motor stuff
const int r_motorDirPin = D2; //digital pin 2
const int r_motorStepPin = D3; //digital pin 3
AccelStepper stepper2(AccelStepper::FULL2WIRE, r_motorDirPin, r_motorStepPin);

MultiStepper steppers;
const int maxStepperSpeed = 100;
long positions[2];// Array to be populated with position to move to

//servo data
const int servoPin = A4; //analog pin 4
Servo servo;

const int baudSpeed = 9600;

float multiplier = 100;
const int delayBetweenAngleMovement = 30;

char val;


long origin[2] = {0,0};
float x_0 = 18.5;
float x_1 = 18.5;
float y = 28;
float a = sqrt(x_0*x_0 + y*y);
float b = sqrt(x_0*x_0 + y*y);
float a_0, a_1, b_0, b_1;
float px = 0;
float py = 0;

const int w = 37;
const int h = 28;

//The lengths of a and b at the start
long origin_a = sqrt(pow(px,2) + pow(h-py,2));
long origin_b = sqrt(pow(w-px,2) + pow(h-py,2));


void setup() {
  Serial.begin(baudSpeed);
  // Configure each stepper
  stepper1.setMaxSpeed(maxStepperSpeed);
  stepper2.setMaxSpeed(maxStepperSpeed);
  // Then give them to MultiStepper to manage
  steppers.addStepper(stepper1);
  steppers.addStepper(stepper2);
  steppers.moveTo(origin);
  px = 0;
  py = 0;
  servo.attach(servoPin);
  setPenToWriteMode();
}


void loop() {
  if (Serial.available() > 0){
    val = Serial.read();
    switch (val) {
      case 'o':
        moveToOrigin();
        break;
      case 's':
        drawSquareV2();
        break;
      case 'v':
        drawVerticalLine();
        break;
      case 'h':
        drawHorizontalLine();
        break;
      case 'c':
        //in this case it is coordinates
        moveToCoords();
        break;
      case 'l':
        adjustLengths();
        break;
      default:
        break;
    }
  }
}

void moveToCoords(){
  int x1,y1;
  int c = 0;
  String inString = "";
  int m = 1;
  while (Serial.available() > 0) {
    int inChar = Serial.read();
    if (isDigit(inChar)) {
      // convert the incoming byte to a char
      // and add it to the string:
      inString += (char)inChar;
    }
    if (inChar == '-'){
      m = -1;
    }
    if (inChar == ',' || inChar == '\n') {
      if (c == 0){
          x1 = inString.toInt()*m;
          Serial.print("x1: "); Serial.print(x1);
          c++;
      } else if (c == 1) {
          y1 = inString.toInt()*m;
          Serial.print(" y1: "); Serial.println(y1);
          c++;
          break;
      }
      inString = "";
      m = 1;
    }
  }
  Serial.print("go from "); Serial.print(px); Serial.print(","); Serial.print(py);
  Serial.print(" to "); Serial.print(x1); Serial.print(","); Serial.println(y1);
  moveStepper(x1, y1);
}

void adjustLengths(){
  int l1,l2;
  int c = 0;
  int m = 1;
  String inString = "";
  while (Serial.available() > 0) {
    int inChar = Serial.read();
    if (isDigit(inChar)) {
      // convert the incoming byte to a char
      // and add it to the string:
      inString += (char)inChar;
    }
    if (inChar == '-'){
      m = -1;
    }
    if (inChar == ',' || inChar == '\n') {
      if (c == 0){
          l1 = inString.toInt()*m;
          Serial.print("l1: "); Serial.print(l1);
          c++;
      } else if (c == 1) {
          l2 = inString.toInt()*m;
          Serial.print(" l2: "); Serial.println(l2);
          c++;
          break;
      }
      inString = "";
      m = 1;
    }
  }
  setAAndB(l1,l2);
}

void setAAndB(int l1, int l2){
  Serial.print("Moving from ");
  Serial.print(positions[0]); Serial.print(","); Serial.print(positions[1]);
  positions[0] = l1;
  positions[1] = l2;
  Serial.print(" to ");
  Serial.print(positions[0]); Serial.print(","); Serial.println(positions[1]);
  steppers.runSpeedToPosition(); // Blocks until all are in position
  steppers.moveTo(positions);
  steppers.runSpeedToPosition(); // Blocks until all are in position
  delay(1000);
}

void moveToOrigin(){
  setPenToNonWriteMode();
  Serial.println("moving to origin");
  steppers.moveTo(origin);
  steppers.runSpeedToPosition(); // Blocks until all are in position
  px = 0;
  py = 0;
  delay(1000);
  setPenToWriteMode();
}

void moveStepper(long new_coords_x, long new_coords_y) {
  long curr_a = sqrt(pow(px,2) + pow(h-py,2));
  long curr_b = sqrt(pow(w-px,2) + pow(h-py,2));

  long new_a = sqrt(pow(new_coords_x,2) + pow(h - new_coords_y,2));
  long new_b = sqrt(pow(w-new_coords_x,2) + pow(h-new_coords_y,2));

  long a_pos = new_a - origin_a;
  long b_pos = new_b - origin_b;

  positions[0] = -1 * multiplier * a_pos;
  positions[1] = multiplier * b_pos;
  //a goes to positions[0]
  //b goes to positions[1]
  Serial.print(" to ");
  Serial.print(positions[0]);
  Serial.print(",");
  Serial.println(positions[1]);
  steppers.moveTo(positions);
  steppers.runSpeedToPosition(); // Blocks until all are in position
  px = new_coords_x;
  py = new_coords_y;
  delay(1000);
}

void drawVerticalLine(){
  //at this point we are at origin so x_0 = 18.5 and y = 28
  //move to (0, 10)
  Serial.println("moving to 0,10");
  moveStepper(0,10);

  /*a_0 = a;
  b_0 = b;
  a_1 = sqrt(pow(x_0, 2) + pow(y+10,2));
  b_1 = sqrt(pow(x_1, 2) + pow(y+10,2));
  positions[0] = multiplier*(a_1-a_0);
  positions[1] = -multiplier*(b_1-b_0);
  steppers.moveTo(positions);
  steppers.runSpeedToPosition(); // Blocks until all are in position
  delay(1000);*/
}

void drawHorizontalLine(){
  //at this point we are at origin so x_0 = 18.5 and y = 28
  //move to (0, 10)
  Serial.println("moving to 10,0");
  moveStepper(10,0);
  /*a_0 = a;
  b_0 = b;
  a_1 = sqrt(pow(x_0+10, 2) + pow(y,2));
  b_1 = sqrt(pow(x_1-10, 2) + pow(y,2));
  positions[0] = multiplier*(a_1-a_0);
  positions[1] = multiplier*abs(b_1-b_0);
  steppers.moveTo(positions);
  steppers.runSpeedToPosition(); // Blocks until all are in position
  delay(1000);*/
}

void drawSquareV2(){
  //at this point we are at origin so x_0 = 18.5 and y = 28
  //move to (10, 0)
  Serial.println("moving to 10,0");
  a_0 = a;
  b_0 = b;
  a_1 = sqrt(pow(x_0+10, 2) + pow(y,2));
  b_1 = sqrt(pow(x_1-10, 2) + pow(y,2));
  positions[0] = multiplier*abs(a_1-a_0);
  positions[1] = -multiplier*abs(b_1-b_0);
  steppers.moveTo(positions);
  steppers.runSpeedToPosition(); // Blocks until all are in position
  delay(1000);

  //move to (10, 10)
  Serial.println("moving to 10,10");
  a_0 = a_1;
  b_0 = b_1;
  a_1 = sqrt(pow(x_0+10, 2) + pow(y+10,2));
  b_1 = sqrt(pow(x_1-10, 2) + pow(y+10,2));
  positions[0] = multiplier*abs(a_1-a_0);
  positions[1] = multiplier*abs(b_1-b_0);
  steppers.moveTo(positions);
  steppers.runSpeedToPosition(); // Blocks until all are in position
  delay(1000);
}

void drawSquare(){
  Serial.println("moving to origin");
  steppers.moveTo(origin);
  steppers.runSpeedToPosition(); // Blocks until all are in position
  delay(1000);
  //make first line
  Serial.println("drawing first line");
  long a_0 = a;
  long b_0 = b;
  long a_1 = 35.6;
  long b_1 = 31.3;
  positions[0] = multiplier*abs(a_1-a_0);
  positions[1] = multiplier*abs(b_1-b_0);
  steppers.moveTo(positions);
  steppers.runSpeedToPosition(); // Blocks until all are in position
  delay(1000);
  //make second line
  Serial.println("drawing second line");
  a_0 = a_1;
  b_0 = b_1;
  a_1 = 38.8;
  b_1 = 34.92;
  positions[0] = multiplier*abs(a_1-a_0);
  positions[1] = multiplier*abs(b_1-b_0);
  steppers.moveTo(positions);
  steppers.runSpeedToPosition(); // Blocks until all are in position
  delay(1000);
  //make third line
  Serial.println("drawing third line");
  a_0 = a_1;
  b_0 = b_1;
  a_1 = 36.7;
  b_1 = 38.8;
  positions[0] = multiplier*abs(a_1-a_0);
  positions[1] = multiplier*abs(b_1-b_0);
  steppers.moveTo(positions);
  steppers.runSpeedToPosition(); // Blocks until all are in position

  delay(1000);
  //make last line
  Serial.println("drawing last line");
  a_0 = a_1;
  b_0 = b_1;
  a_1 = a;
  b_1 = b;
  positions[0] = multiplier*abs(a_1-a_0);
  positions[1] = multiplier*abs(b_1-b_0);
  steppers.moveTo(positions);
  steppers.runSpeedToPosition(); // Blocks until all are in position
  delay(1000);
}

void setPenToWriteMode() {
  int angle = (int)servo.read();
  while (angle < 90){
    Particle.process();
    servo.write(angle);
    delay(delayBetweenAngleMovement);
    angle++;
  }
}

void setPenToNonWriteMode() {
  int angle = (int)servo.read();
  while (angle > 0){
    angle--;
    Particle.process();
    servo.write(angle);
    delay(delayBetweenAngleMovement);
  }
}


/*const int l_motorDirPin = D0; //digital pin 0
const int l_motorStepPin = D1; //digital pin 1

const int r_motorDirPin = D2; //digital pin 2
const int r_motorStepPin = D3; //digital pin 3

const int servoPin = A4; //analog pin 4

const int baudSpeed = 9600;

Servo servo;

const int delayBetweenAngleMovement = 30;
const int delayBetweenSteps = 800;

void setup() {
  pinMode(l_motorDirPin, OUTPUT);
  pinMode(l_motorStepPin, OUTPUT);
  servo.attach(servoPin);
  setPenToWriteMode();
  Serial.begin(baudSpeed);
}

void step_left_motor(boolean dir, int steps) {
  digitalWrite(l_motorDirPin,dir);
  delay(50);
  for(int i=0;i<steps;i++){
    digitalWrite(l_motorStepPin, HIGH);
    delayMicroseconds(delayBetweenSteps);
    digitalWrite(l_motorStepPin, LOW);
    delayMicroseconds(delayBetweenSteps);
  }
}

void step_right_motor(boolean dir, int steps) {
  digitalWrite(r_motorDirPin,dir);
  delay(50);
  for(int i=0;i<steps;i++){
    digitalWrite(r_motorStepPin, HIGH);
    delayMicroseconds(delayBetweenSteps);
    digitalWrite(r_motorStepPin, LOW);
    delayMicroseconds(delayBetweenSteps);
  }
}

void setPenToWriteMode() {
  int angle = 0;
  while (angle < 90){
    Particle.process();
    servo.write(angle);
    delay(delayBetweenAngleMovement);
    angle++;
  }
}

void setPenToNonWriteMode() {
  int angle = 90;
  while (angle > 0){
    angle--;
    Particle.process();
    servo.write(angle);
    delay(delayBetweenAngleMovement);
  }
}

void loop(){
  delay(1000);
  step_left_motor(true, 100);
  delay(1000);
  step_left_motor(false, 100);
}*/
