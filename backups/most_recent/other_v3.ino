#include "config.h"
#include <math.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <assert.h>

//------------------------------------------------------------------------------
// GLOBALS
//------------------------------------------------------------------------------

char  buffer[MAX_BUF];  // where we store the message until we get a newline
int   sofar;            // how much is in the buffer
bool IS_DEBUG = false;
String delimiter  = "*";
int delayTime = 50;
int ack  = 'Q';
bool first_time = true;
bool USING_HANDSHAKE = false;


// speeds
float fr =     0;  // human version
long  step_delay;  // machine version
const int delayBetweenAngleMovement = 30;

// settings
char mode_abs=1;   // absolute mode?

const float w = 66.5;//was 75
const float h = 32;
const float x_10 = 30;

// location
long origin[2] = {0,0};
float px = 0;
float py = 0;


//The lengths of a and b at the start
float origin_a = 43.86;
float origin_b = 48.54;//was 55.22

long lengths[2];// Array to be populated with position to move to

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

//servo data
const int servoPin = A4; //analog pin 4
Servo servo;

float multiplier = 180.0;
char val;


//------------------------------------------------------------------------------
// METHODS
//------------------------------------------------------------------------------


/**
 * delay for the appropriate number of microseconds
 * @input ms how many milliseconds to wait
 */
void pause(long ms) {
  delay(ms/1000);
  delayMicroseconds(ms%1000);  // delayMicroseconds doesn't work for values > ~16k.
}


/**
 * Set the feedrate (speed motors will move)
 * @input nfr the new speed in steps/second
 */
void feedrate(float nfr) {
  Serial.print(" at feedrate ");
  if(fr==nfr) {
    Serial.print(fr);
    return;  // same as last time?  quit now.
  }

  if(nfr>MAX_FEEDRATE || nfr<MIN_FEEDRATE) {  // don't allow crazy feed rates
    Serial.print(F("New feedrate must be greater than "));
    Serial.print(MIN_FEEDRATE);
    Serial.print(F("steps/s and less than "));
    Serial.print(MAX_FEEDRATE);
    Serial.println(F("steps/s."));
    return;
  }
  step_delay = 1000000.0/nfr;
  fr = nfr;
  Serial.print(fr);
}


/**
 * Set the logical position
 * @input npx new position x
 * @input npy new position y
 */
void position(float npx,float npy) {
  // here is a good place to add sanity tests
  px=npx;
  py=npy;
}


/**
 * @input newx the destination x position
 * @input newy the destination y position
 **/
void line(float newx,float newy) {
  Serial.print(" From point ");
  Serial.print(px); Serial.print(","); Serial.print(py);
  Serial.print(" To point ");
  Serial.print(newx); Serial.print(","); Serial.print(newy);
  moveStepper(newx, newy);
}

void moveStepper(float new_coords_x, float new_coords_y) {

  float new_a = sqrt(pow(x_10+new_coords_x,2.0) + pow(h-new_coords_y,2.0));
  float new_b = sqrt(pow((w-x_10)-new_coords_x,2.0) + pow(h-new_coords_y,2.0));

  float a_pos = new_a - origin_a;
  float b_pos = new_b - origin_b;

  float len1 = -1 * multiplier * a_pos;
  float len2 = multiplier * b_pos;

  lengths[0] = (long)len1;
  lengths[1] = (long)len2;
  //a goes to lengths[0]
  //b goes to lengths[1]
  Serial.print(" to ");
  Serial.print(lengths[0]);
  Serial.print(",");
  Serial.println(lengths[1]);
  steppers.runSpeedToPosition();
  steppers.moveTo(lengths);
  steppers.runSpeedToPosition(); // Blocks until all are in position
  px = new_coords_x;
  py = new_coords_y;
  delay(1000);
}

// returns angle of dy/dx as a value from 0...2PI
float atan3(float dy,float dx) {
  float a = atan2(dy,dx);
  if(a<0) a = (M_PI*2.0)+a;
  return a;
}


// This method assumes the limits have already been checked.
// This method assumes the start and end radius match.
// This method assumes arcs are not >180 degrees (PI radians)
// cx/cy - center of circle
// x/y - end position
// dir - ARC_CW or ARC_CCW to control direction of arc
/*void arc(float cx,float cy,float x,float y,float dir) {
  Serial.print("Make arc ");
  // get radius
  float dx = px - x; Serial.print("dx is "); Serial.println(dx);
  float dy = py - y; Serial.print("dy is "); Serial.println(dy);
  float radius=sqrt(dx*dx+dy*dy); Serial.print("radius is "); Serial.println(radius);

  // find angle of arc (sweep)
  float angle1=atan3(dy,dx); Serial.print("angle1 is "); Serial.println(angle1);
  float angle2=atan3(y-cy,x-cx); Serial.print("angle2 is "); Serial.println(angle2);
  float theta=angle2-angle1; Serial.print("theta is "); Serial.println(theta);

  if(dir>0 && theta<0) angle2+=2*M_PI;
  else if(dir<0 && theta>0) angle1+=2*M_PI;

  theta=(angle2-angle1);//%(2*M_PI);

  //maybe we can try if theta is greater than 1 then treat as G2 if was G3 or G3 if was G2
  if (abs(theta) > M_PI){
    reverseArcInstruction(angle1, angle2, theta, cx, cy, x, y, dir, radius);
  } else {
    regularArcInstruction(angle1, angle2, theta, cx, cy, x, y, dir, radius);
  }
}*/

void arc(float cx,float cy,float x,float y,float dir) {
  // get radius
  float dx = px - cx;
  float dy = py - cy;
  float radius=sqrt(dx*dx+dy*dy);

  // find angle of arc (sweep)
  float angle1=atan3(dy,dx);
  float angle2=atan3(y-cy,x-cx);
  float theta=angle2-angle1;

  if(dir>0 && theta<0) angle2+=2*PI;
  else if(dir<0 && theta>0) angle1+=2*PI;

  theta=angle2-angle1;

  // get length of arc
  // float circ=PI*2.0*radius;
  // float len=theta*circ/(PI*2.0);
  // simplifies to
  float len = abs(theta) * radius;

  int i, segments = ceil( len * MM_PER_SEGMENT );

  float nx, ny, angle3, scale;

  for(i=0;i<segments;++i) {
    // interpolate around the arc
    scale = ((float)i)/((float)segments);

    angle3 = ( theta * scale ) + angle1;
    nx = cx + cos(angle3) * radius;
    ny = cy + sin(angle3) * radius;
    // send it to the planner
    line(nx,ny);
  }

  line(x,y);
}

void regularArcInstruction(float angle1, float angle2, float theta, float cx,float cy,float x,float y,float dir, float radius){
  Serial.print("angle1 is "); Serial.println(angle1);
  Serial.print("angle2 is "); Serial.println(angle2);
  Serial.print("theta is "); Serial.println(theta);
  Serial.print("dir is"); Serial.println(dir);

  // get length of arc
  // float circ=PI*2.0*radius;
  // float len=theta*circ/(PI*2.0);
  // simplifies to
  float len = abs(theta) * radius;
  Serial.print("len is "); Serial.println(len);

  int segments = ceil( len * MM_PER_SEGMENT );

  float nx, ny, angle3, scale;
  Serial.print("number of segments: "); Serial.print(segments);
  for(int i=0;i<segments;++i) {
    Serial.print("done "); Serial.print(i); Serial.print(" of ");
    Serial.print(segments); Serial.print(" segments");
    // interpolate around the arc
    scale = ((float)i)/((float)segments);

    angle3 = ( theta * scale ) + angle1;
    nx = cx + cos(angle3) * radius;
    ny = cy + sin(angle3) * radius;
    // send it to the planner
    line(nx,ny);
  }
  line(x,y);
}

void reverseArcInstruction(float angle1, float angle2, float theta, float cx,float cy,float x,float y,float dir, float radius){
  dir = -1*dir;
  if (theta > 0) theta = 2 * M_PI - theta;
  else theta = 2 * M_PI + theta;
  Serial.print("angle1 is "); Serial.println(angle1);
  Serial.print("angle2 is "); Serial.println(angle2);
  Serial.print("theta is "); Serial.println(theta);
  Serial.print("dir is"); Serial.println(dir);

  // get length of arc
  // float circ=PI*2.0*radius;
  // float len=theta*circ/(PI*2.0);
  // simplifies to
  float len = abs(theta) * radius;
  Serial.print("len is "); Serial.println(len);

  int segments = ceil( len * MM_PER_SEGMENT );

  float nx, ny, angle3, scale;
  Serial.print("number of segments: "); Serial.print(segments);
  for(int i=0;i<segments;++i) {
    Serial.print("done "); Serial.print(i); Serial.print(" of ");
    Serial.print(segments); Serial.print(" segments");
    // interpolate around the arc
    scale = ((float)i)/((float)segments);

    angle3 = ( theta * scale ) + angle1;
    nx = cx + cos(angle3) * radius;
    ny = cy + sin(angle3) * radius;
    // send it to the planner
    line(nx,ny);
  }

  line(x,y);
}

/**
 * Look for character /code/ in the buffer and read the float that immediately follows it.
 * @return the value found.  If nothing is found, /val/ is returned.
 * @input code the character to look for.
 * @input val the return value if /code/ is not found.
 **/

float parsenumber(char code,float val) {
  String gCodeLine = buffer;
  int i = 0;
  int len = gCodeLine.length();
  char curr;
  while (i < len){
    Particle.process();
    curr = gCodeLine.charAt(i);
    if(curr==code) {
      return atof(gCodeLine.substring(i+1));
    }
    i++;
  }
  return val;
}


/**
 * write a string followed by a float to the serial line.  Convenient for debugging.
 * @input code the string.
 * @input val the float.
 */
void output(const char *code,float val) {
  Serial.print(code);
  Serial.println(val);
}


/**
 * print the current position, feedrate, and absolute mode.
 */
void where() {
  output("X",px);
  output("Y",py);
  output("F",fr);
  Serial.println(mode_abs?"ABS":"REL");
}


/**
 * display helpful information
 */
void help() {
  Serial.print(F("GcodeCNCDemo2AxisV1 "));
  Serial.println(VERSION);
  Serial.println(F("Commands:"));
  Serial.println(F("G00 [X(steps)] [Y(steps)] [F(feedrate)]; - line"));
  Serial.println(F("G01 [X(steps)] [Y(steps)] [F(feedrate)]; - line"));
  Serial.println(F("G02 [X(steps)] [Y(steps)] [I(steps)] [J(steps)] [F(feedrate)]; - clockwise arc"));
  Serial.println(F("G03 [X(steps)] [Y(steps)] [I(steps)] [J(steps)] [F(feedrate)]; - counter-clockwise arc"));
  Serial.println(F("G04 P[seconds]; - delay"));
  Serial.println(F("G90; - absolute mode"));
  Serial.println(F("G91; - relative mode"));
  Serial.println(F("G92 [X(steps)] [Y(steps)]; - change logical position"));
  Serial.println(F("M18; - disable motors"));
  Serial.println(F("M100; - this help message"));
  Serial.println(F("M114; - report position and feedrate"));
  Serial.println(F("All commands must end with a newline."));
}


/**
 * Read the input buffer and find any recognized commands.  One G or M command per line.
 */
void processCommand() {
  int cmd = parsenumber('G',-1);
  switch(cmd) {
  case  0:
  case  1: { // line
    Serial.print("Move ");
    feedrate(parsenumber('F',fr));
    line( parsenumber('X',(mode_abs?px:0)) + (mode_abs?0:px),
          parsenumber('Y',(mode_abs?py:0)) + (mode_abs?0:py) );
    Serial.println("...");
    break;
    }
  case 2:
  case 3: {  // arc
      feedrate(parsenumber('F',fr));
      arc(parsenumber('I',(mode_abs?px:0)) + (mode_abs?0:px),
          parsenumber('J',(mode_abs?py:0)) + (mode_abs?0:py),
          parsenumber('X',(mode_abs?px:0)) + (mode_abs?0:px),
          parsenumber('Y',(mode_abs?py:0)) + (mode_abs?0:py),
          (cmd==2) ? -1 : 1);
      break;
    }
  case  4:  pause(parsenumber('P',0)*1000);  break;  // dwell
  case 90:  mode_abs=1;  Serial.println("mode set to absolute"); break;  // absolute mode
  case 91:  mode_abs=0;  break;  // relative mode
  case 92:  // set logical position
    position( parsenumber('X',0),
              parsenumber('Y',0) );
    break;
  case 20: Serial.println("Unit set to inches"); break;
  case 21: Serial.println("Unit set to millimeters"); break;
  default:  break;
  }

  cmd = parsenumber('M',-1);
  switch(cmd) {
  case 18:  // disable motors
    disable();
    break;
  case 100:  help();  break;
  case 114:  where();  break;
  case 3: Serial.println("Move pen down (start writing)"); setPenToWriteMode(); break;
  case 5: Serial.println("Move pen up (stop writing)"); setPenToNonWriteMode(); break;
  default:  break;
  }
}


/**
 * prepares the input buffer to receive a new message and tells the serial connected device it is ready for more.
 */
void ready() {
  sofar=0;  // clear input buffer
  Serial.print(F(">"));  // signal ready to receive input
}


/**
 * First thing this machine does on startup.  Runs only once.
 */
void setup() {
  Serial.begin(BAUD);  // open coms

  setup_controller();
  px = 0; py = 0;
  position(0,0);  // set staring position
  feedrate((MAX_FEEDRATE + MIN_FEEDRATE)/2);  // set default speed
  // Configure each stepper
  stepper1.setMaxSpeed(maxStepperSpeed);
  stepper2.setMaxSpeed(maxStepperSpeed);
  // Then give them to MultiStepper to manage
  steppers.addStepper(stepper1);
  steppers.addStepper(stepper2);
  steppers.moveTo(origin);
  servo.attach(servoPin);
  setPenToWriteMode();
  while(Serial.available()){  //is there anything to read?
    Serial.read();  //if yes, read it
  }
  help();  // say hello
  ready();
}


/**
 * After setup() this machine will repeat loop() forever.
 */
void loop() {
  if (IS_DEBUG){
    debugLoop();
  } else {
    regularLoop();
  }
}

void processLine(){
  //wait for a line, it definitely will come
  char c;
  while(true) {
    Particle.process();
    c = Serial.read();  // read serial
    // if we are here serial is not empty
    if(sofar<MAX_BUF-1) {
      if (c != 'N') buffer[sofar++]=c;  // store it
      Serial.print("sofar: "); Serial.println(sofar);
    }
    String c_string = "";
    c_string += c;
    if((c == '\n') || (c == '\r') || ((int)c == -1) || ((int)c == 255) || ((int)c == 42) || (c_string.equals(delimiter)) || (sofar > 1000)) {
      // entire message received
      buffer[sofar]=0;  // end the buffer so string functions work right
      //Serial.print(F("\r\n"));  // echo a return character for humans
      break;
    }
  }
  processCommand();
  ready();
  Serial.println(buffer);
  Serial.print(delimiter);
}

/**
 * After setup() this machine will repeat loop() forever.
 */
 void regularLoop(){
   //step 1 step at a time till 50, then reverse
   //each step make a reading with the range finder
   //which will give you a mm value
   if (USING_HANDSHAKE){
     handshakeProtocol();
   } else {
     regularReadSerial();
   }
 }

void handshakeProtocol(){
  if (Serial.available()) {
    delay(delayTime);
    if (first_time) {
      Serial.print(delimiter);
      first_time = false;
    }
    processLine();
  }
}

void regularReadSerial(){
  //wait for a line, it definitely will come
  char c;
  while(Serial.available() > 0) {
    c = Serial.read();  // read serial
    // if we are here serial is not empty
    if(sofar<MAX_BUF-1) {
      if (c != 'N') buffer[sofar++]=c;  // store it
      Serial.print("sofar: "); Serial.println(sofar);
    }
    String c_string = "";
    c_string += c;
    if((c == '\n') || (c == '\r') || ((int)c == -1) || ((int)c == 255) || ((int)c == 42) || (c_string.equals(delimiter)) || (sofar > 1000)) {
      // entire message received
      buffer[sofar]=0;  // end the buffer so string functions work right
      //Serial.print(F("\r\n"));  // echo a return character for humans
      Serial.println(buffer);
      processCommand();
      ready();
    }
  }
}
void debugLoop(){
  if (Serial.available() > 0){
    val = Serial.read();
    switch (val) {
      case 'o':
        moveToOrigin();
        break;
      case 's':
        drawSquare();
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
      case 'd':
        setPenToWriteMode();
        break;
      case 'u':
        setPenToNonWriteMode();
        break;
      default:
        break;
    }
  }
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

void drawSquare(){
  moveStepper(0,0);
  moveStepper(5,0);
  moveStepper(5,5);
  moveStepper(0,5);
  moveStepper(5,5);
}

void drawVerticalLine(){
  //at this point we are at origin so x_0 = 18.5 and y = 28
  //move to (0, 10)
  Serial.println("moving to 0,1.5");
  moveStepper(0,1.5);
}

void drawHorizontalLine(){
  //at this point we are at origin so x_0 = 18.5 and y = 28
  //move to (0, 10)
  Serial.println("moving to 1.5,0");
  moveStepper(1.5,0);
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
  Serial.print(lengths[0]); Serial.print(","); Serial.print(lengths[1]);
  lengths[0] = l1;
  lengths[1] = l2;
  Serial.print(" to ");
  Serial.print(lengths[0]); Serial.print(","); Serial.println(lengths[1]);
  steppers.runSpeedToPosition(); // Blocks until all are in position
  steppers.moveTo(lengths);
  steppers.runSpeedToPosition(); // Blocks until all are in position
  delay(1000);
}
