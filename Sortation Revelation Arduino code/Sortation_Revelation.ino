
/*
 Sortation Revelation

 Created 4/2/2021
 modify 5/21/2021
 by junxing lin
 */
// Variables for Color Pulse Width Measurements
int redaveragevalue=0;
int greenaveragevalue=0;
int blueaveragevalue=0;
int redlimitvalue=0;
int greenlimitvalue=0;
int bluelimitvalue=0;
int redPW = 0;
int greenPW = 0;
int bluePW = 0;
String blockcolor = "";

//Belt1
#include <Stepper.h>
//NEMA 23 take 200 step per revolution
const int stepsPerRevolution = 200;
//laser data pin
const int LaserIn = 2;
//motor speed of the stepper
int motorSpeed = 100;
// initialize the stepper library on pins 8 through 11:
Stepper myStepper(stepsPerRevolution, 46, 47, 48, 49);

//Servo shield
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  160// this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  700// this is the 'maximum' pulse length count (out of 4096)
#define nintydegree 380
#define gripperclose 350
uint8_t colorservo = 0;
uint8_t firstarmservo1 = 1;
uint8_t firstarmservo2 = 2;
uint8_t firstarmservo3 = 3;
uint8_t firstarmservo4 = 4;
uint8_t firstarmservo5 = 5;
uint8_t firstarmservo6 = 6;
String posstring;
float pos = SERVOMIN;

// Define color sensor pins
//color sensor1
#define S0 26 //green
#define S1 25 //blue
#define S2 24 //orange
#define S3 23 //yellow
#define sensorOut 22 //red


/*  
  belt 1
  black block limits
  red-244
  green-220
  blue-160

  white block limits
  red-20
  green-7
  blue-20
*/


//Belt2

//Sensor output and input pins definition
const int pingTrig = A0;
const int pingEcho = A1;

//color sensor
// Define color sensor pins
#define S02 37
#define S12 36
#define S22 35
#define S32 34
#define sensorOut2 33
/*
  black block limits
  red-244
  green-220
  blue-160

  white block limits
  red-114
  green-107
  blue-104
*/

//servo shield
// called this way, it uses the default address 0x40
#define SERVOMIN2  150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX2  600 // this is the 'maximum' pulse length count (out of 4096)
#define NINTYDEGREE 380
#define gripperclose 350
uint8_t colorservo2 = 7;
uint8_t secondarmservo1 = 8;
uint8_t secondarmservo2 = 9;
uint8_t secondarmservo3 = 10;
uint8_t secondarmservo4 = 11;
uint8_t secondarmservo5 = 12;
uint8_t secondarmservo6 = 13;
String posstring2;
float pos2 = SERVOMIN2;

//stepper motor
const int stepsPerRevolution2 = 32;  // change this to fit the number of steps per revolution
// for your motor
const int gearReduction = 64;
const int outPutRevolution = gearReduction * stepsPerRevolution2;
// initialize the stepper library on pins 8 through 11:
Stepper myStepper2(stepsPerRevolution2, 50,51,52,53);

void setup() {
  pinMode(LaserIn, INPUT);
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  pwm.setPWM(colorservo,0, SERVOMIN);
  armintpos();
  
  //color sensor setup
  // Set S0 - S3 as outputs
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  // Set Sensor output as input
  pinMode(sensorOut, INPUT);
  // Set Pulse Width scaling to 20%
  digitalWrite(S0,HIGH);
  digitalWrite(S1,LOW);
//second Belt setup
  //ultrasonic setup
  // initialize sensor pins
  pinMode(pingTrig, OUTPUT); // change A0 pin mode to digital output
  pinMode(pingEcho,  INPUT); // change A1 pin mode to digital input

  //servo setup
  // attaches the servo on pin 9 to the servo object
//  myservo.attach(servoPin);

  //servo shield 
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  pwm.setPWM(colorservo2, 0, 140);
  armintpos2();
  
  //color sensor setup
  // Set S02 - S32 as outputs
  pinMode(S02, OUTPUT);
  pinMode(S12, OUTPUT);
  pinMode(S22, OUTPUT);
  pinMode(S32, OUTPUT);
  // Set Sensor output as input
  pinMode(sensorOut2, INPUT);
  // Set Pulse Width scaling to 20%
  digitalWrite(S02,HIGH);
  digitalWrite(S12,LOW);
  
  //stepper stepup
  // set the speed at 60 rpm:
  myStepper2.setSpeed(1000);
  
  Serial.begin(9600);
  
}

void loop() {
  
  // read the sensor value:
  int sensorReading = digitalRead(LaserIn);
  Serial.println(sensorReading);
  myStepper.setSpeed(motorSpeed);
  // set the motor speed:
  if (sensorReading == 1) 
  {
    for (int i =0; i <10;i++)
    {
      // step 1/100 of a revolution:
      myStepper.step(stepsPerRevolution / 20);
      delay (1000);
    } 
    while (sensorReading == 1)
    {
      myStepper.step(0 / 20);
      pwm.setPWM(colorservo,0, nintydegree);
      delay(500);
      readingColorValue();
      sensorReading = digitalRead(LaserIn);
    }
  }
  while (sensorReading == 0) 
  {
    // step 1/100 of a revolution:
    myStepper.step(stepsPerRevolution / 20);
    sensorReading = digitalRead(LaserIn);
  }
  
  
}
void sortblock(String blockcolor){
  if (blockcolor=="white" or blockcolor=="black")
  {
    pickupblockone();
    delay(1000);
    whiteblockone();
    armintpos();
    secondBeltSensor();
  }
  else 
  {
    pickupblockone();
    delay(1000);
    blackblockone();
  }
    
}

void readingColorValue(){
  
  blockcolor = ReadColor();
  Serial.println("the block is "+blockcolor);
  pwm.setPWM(colorservo,0, SERVOMIN);
  delay(1000);
  sortblock(blockcolor);
}

String ReadColor() {
  redaveragevalue=0;
  greenaveragevalue=0;
  blueaveragevalue=0;
  redlimitvalue=0;
  greenlimitvalue=0;
  bluelimitvalue=0;
  int repeatcount=10;
  for(int i = 0;i<repeatcount;i++){
  // Read Red Pulse Width
  redPW = getRedPW();
  // Delay to stabilize sensor
  delay(200);
  
  // Read Green Pulse Width
  greenPW = getGreenPW();
  // Delay to stabilize sensor
  delay(200);
  
  // Read Blue Pulse Width
  bluePW = getBluePW();
  // Delay to stabilize sensor
  delay(200);
  
  // Print output to Serial Monitor
  Serial.print("Red PW = ");
  Serial.print(redPW);
  
  Serial.print(" Red limit = ");
  Serial.println(redlimitvalue);
  Serial.print("Green PW = ");
  Serial.print(greenPW);
  
  Serial.print(" - Green limit = ");
  Serial.println(greenlimitvalue);
  Serial.print("Blue PW = ");
  Serial.print(bluePW);
  
  Serial.print(" - Blue limit = ");
  Serial.println(bluelimitvalue);
  
  }
  Serial.print("Red average PW = ");
  Serial.print(redaveragevalue/repeatcount);
  Serial.print(" - Green average PW = ");
  Serial.print(greenaveragevalue/repeatcount);
  Serial.print(" - Blue averagePW = ");
  Serial.println(blueaveragevalue/repeatcount);
  /*black block limits
  red-244
  green-220
  blue-160
  */
  if (blueaveragevalue/repeatcount >=8 and redaveragevalue/repeatcount >=8)
  {
    return "black";
    }
  /*  
  white block limits
  red-114
  green-107
  blue-104
  */
   else if ( blueaveragevalue/repeatcount <6 and redaveragevalue/repeatcount <6){
   return "white";
}
else
  return "unknown";
}

// Function to read Red Pulse Widths
int getRedPW() {

  // Set sensor to read Red only
  digitalWrite(S2,LOW);
  digitalWrite(S3,LOW);
  // Define integer to represent Pulse Width
  int PW;
  // Read the output Pulse Width
  PW = pulseIn(sensorOut, LOW);
  
  if (redaveragevalue==0)
  {
    redaveragevalue = PW;
    redlimitvalue= PW;
    }
  else {
    if (redlimitvalue<PW){
    redlimitvalue= PW;}
    redaveragevalue += PW;}
  // Return the value
  return PW;

}

// Function to read Green Pulse Widths
int getGreenPW() {

  // Set sensor to read Green only
  digitalWrite(S2,HIGH);
  digitalWrite(S3,HIGH);
  // Define integer to represent Pulse Width
  int PW;
  // Read the output Pulse Width
  PW = pulseIn(sensorOut, LOW);

  if (greenaveragevalue==0)
  {
    greenaveragevalue = PW;
    greenlimitvalue= PW;
    }
  else
  {
    if (greenlimitvalue<PW){
    greenlimitvalue= PW;}
    greenaveragevalue += PW;}
  // Return the value
  return PW;

}

// Function to read Blue Pulse Widths
int getBluePW() {

  // Set sensor to read Blue only
  digitalWrite(S2,LOW);
  digitalWrite(S3,HIGH);
  // Define integer to represent Pulse Width
  int PW;
  // Read the output Pulse Width
  PW = pulseIn(sensorOut, LOW);
  
  if (blueaveragevalue==0)
  {
    blueaveragevalue = PW;
    bluelimitvalue= PW;
    }
  else{
    if (bluelimitvalue<PW){
    bluelimitvalue= PW;}
    blueaveragevalue+= PW;}
  // Return the value
  return PW;

}
void whiteblockone(){
  
  pwm.setPWM(firstarmservo4, 0, SERVOMAX);
  delay(2000);
  pwm.setPWM(firstarmservo3, 0, SERVOMIN);
  delay(2000);
  pwm.setPWM(firstarmservo1, 0, 555);//555
  delay(2000);
  pwm.setPWM(firstarmservo2, 0, 120);//120
  delay(2000);
  pwm.setPWM(firstarmservo3, 0, 370); //370
  delay(2000);
  pwm.setPWM(firstarmservo5, 0, 300);
  delay(2000);
  pwm.setPWM(firstarmservo4, 0, 220);//200
  delay(2000);
  
  pwm.setPWM(firstarmservo6, 0, SERVOMIN);
  delay(2000);
  pwm.setPWM(firstarmservo3, 0, SERVOMIN);
  delay(2000);
  }
  void blackblockone(){
  
  pwm.setPWM(firstarmservo4, 0, SERVOMAX);
  delay(2000);
  pwm.setPWM(firstarmservo3, 0, SERVOMIN);
  delay(2000);
  pwm.setPWM(firstarmservo1, 0, 600);
  delay(2000);
  pwm.setPWM(firstarmservo3, 0, 340);
  delay(2000);
  pwm.setPWM(firstarmservo4, 0, 210);
  delay(2000);
  pwm.setPWM(firstarmservo6, 0, SERVOMIN);
  delay(2000);
  pwm.setPWM(firstarmservo4, 0, SERVOMAX);
  delay(2000);
  }
void pickupblockone(){
  /*
  1-400
  2-430
  3-350
  4-200
  5-700
  6-350*/
  pwm.setPWM(firstarmservo1, 0, 440);
  delay(2000);
  pwm.setPWM(firstarmservo2, 0, 360);
  delay(2000);
  pwm.setPWM(firstarmservo4, 0, 180);
  delay(2000);
  pwm.setPWM(firstarmservo5, 0, 180);
  delay(2000);
  pwm.setPWM(firstarmservo3, 0, 330);
  delay(2000);
  pwm.setPWM(firstarmservo3, 0, 350);
  delay(2000);
  pwm.setPWM(firstarmservo6, 0, gripperclose);
  delay(2000);
}
  

void armintpos(){
  
  pwm.setPWM(firstarmservo6, 0, SERVOMIN);
  delay(1000);
  pwm.setPWM(firstarmservo4, 0, SERVOMAX);
  delay(1000);
  pwm.setPWM(firstarmservo5, 0, SERVOMAX);
  delay(1000);
  pwm.setPWM(firstarmservo3, 0, SERVOMIN);
  delay(1000);
  pwm.setPWM(firstarmservo2, 0, SERVOMIN);
  delay(1000);
  pwm.setPWM(firstarmservo1, 0, SERVOMIN);
  delay(1000);}
  
void secondBeltSensor(){
  
  
  
  // establish variables for duration of the ping, 
  // and the distance result in inches and centimeters:
  long duration;
  int distance;
  int blockinmotion=0;
  while (blockinmotion==0){
  myStepper2.step(outPutRevolution);
  // The HC-SR04 is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse
  digitalWrite(pingTrig,  LOW);
  delayMicroseconds( 5);
  digitalWrite(pingTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(pingTrig,  LOW);

  // The Echo pin is used to read the signal from HC-SR04; a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  duration = pulseIn(pingEcho, HIGH);
  
  // convert the time into a distance
  distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  // Displays the distance on the Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  delay(500);
  if (distance <= 7 and distance> 0)
  {
    
    readingColorValue2();
    blockinmotion=1;
    
    //myStepper2.step(0);
  }
  //start the second belt 
  
  
  
  }
}
void readingColorValue2()
{
  pwm.setPWM(colorservo2, 0, NINTYDEGREE);
  arm2(ReadColor2());
  
  
  
  delay(1000);
  
}
String ReadColor2() {
  redaveragevalue=0;
  greenaveragevalue=0;
  blueaveragevalue=0;
  redlimitvalue=0;
  greenlimitvalue=0;
  bluelimitvalue=0;
  int repeatcount=10;
  for(int i = 0;i<repeatcount;i++){
  // Read Red Pulse Width
  redPW = getRedPW2();
  // Delay to stabilize sensor
  delay(200);
  
  // Read Green Pulse Width
  greenPW = getGreenPW2();
  // Delay to stabilize sensor
  delay(200);
  
  // Read Blue Pulse Width
  bluePW = getBluePW2();
  // Delay to stabilize sensor
  delay(200);
  
  // Print output to Serial Monitor
  Serial.print("Red PW = ");
  Serial.print(redPW);
  
  Serial.print(" Red limit = ");
  Serial.println(redlimitvalue);
  Serial.print("Green PW = ");
  Serial.print(greenPW);
  
  Serial.print(" - Green limit = ");
  Serial.println(greenlimitvalue);
  Serial.print("Blue PW = ");
  Serial.print(bluePW);
  
  Serial.print(" - Blue limit = ");
  Serial.println(bluelimitvalue);
  
  }
  Serial.print("Red average PW = ");
  Serial.print(redaveragevalue/repeatcount);
  Serial.print(" - Green average PW = ");
  Serial.print(greenaveragevalue/repeatcount);
  Serial.print(" - Blue averagePW = ");
  Serial.println(blueaveragevalue/repeatcount);
  /*black block limits
  red-244
  green-220
  blue-160
  */
  if (greenaveragevalue/repeatcount >107 and blueaveragevalue/repeatcount >104 and redaveragevalue/repeatcount >120)
  {
    return "black";
    }
  /*  
  white block limits
  red-114
  green-107
  blue-104
  */
   else if (greenaveragevalue/repeatcount <107 and blueaveragevalue/repeatcount <104 and redaveragevalue/repeatcount <120){
   return "white";
}
else
  return "unknown";
}


// Function to read Red Pulse Widths
int getRedPW2() {

  // Set sensor to read Red only
  digitalWrite(S22,LOW);
  digitalWrite(S32,LOW);
  // Define integer to represent Pulse Width
  int PW;
  // Read the output Pulse Width
  PW = pulseIn(sensorOut2, LOW);
  
  if (redaveragevalue==0)
  {
    redaveragevalue = PW;
    redlimitvalue= PW;
    }
  else {
    if (redlimitvalue<PW){
    redlimitvalue= PW;}
    redaveragevalue += PW;}
  // Return the value
  return PW;

}

// Function to read Green Pulse Widths
int getGreenPW2() {

  // Set sensor to read Green only
  digitalWrite(S22,HIGH);
  digitalWrite(S32,HIGH);
  // Define integer to represent Pulse Width
  int PW;
  // Read the output Pulse Width
  PW = pulseIn(sensorOut2, LOW);

  if (greenaveragevalue==0)
  {
    greenaveragevalue = PW;
    greenlimitvalue= PW;
    }
  else
  {
    if (greenlimitvalue<PW){
    greenlimitvalue= PW;}
    greenaveragevalue += PW;}
  // Return the value
  return PW;

}

// Function to read Blue Pulse Widths
int getBluePW2() {

  // Set sensor to read Blue only
  digitalWrite(S22,LOW);
  digitalWrite(S32,HIGH);
  // Define integer to represent Pulse Width
  int PW;
  // Read the output Pulse Width
  PW = pulseIn(sensorOut2, LOW);
  
  if (blueaveragevalue==0)
  {
    blueaveragevalue = PW;
    bluelimitvalue= PW;
    }
  else{
    if (bluelimitvalue<PW){
    bluelimitvalue= PW;}
    blueaveragevalue+= PW;}
  // Return the value
  return PW;

}
void arm2(String color) {
  // Drive each servo one at a time
  /*
  if (Serial.available()>0){
      posstring2 = Serial.readString();
      pos2 = posstring2.toInt();
      Serial.println(pos2);
      }
      
  gripperclose
  
  1-400
  2-430
  3-350
  4-200
  5-700
  
  */
  pwm.setPWM(colorservo2, 0, SERVOMIN2);
  Serial.println("the block is "+color);
  pickupblock2();
  if (color == "white"){
    whiteblock2();
    }
  else if (color == "black")
  {
    blackblock2();
    }
  
  armintpos2();
  delay(500);
}

void whiteblock2(){
  
  pwm.setPWM(secondarmservo4, 0, SERVOMAX2);
  delay(2000);
  pwm.setPWM(secondarmservo3, 0, SERVOMIN2);
  delay(2000);
  pwm.setPWM(secondarmservo1, 0, SERVOMIN2);
  delay(2000);
  pwm.setPWM(secondarmservo3, 0, 340);
  delay(2000);
  pwm.setPWM(secondarmservo4, 0, 210);
  delay(2000);
  pwm.setPWM(secondarmservo6, 0, SERVOMIN2);
  delay(2000);
  pwm.setPWM(secondarmservo4, 0, SERVOMAX2);
  delay(2000);
  }
  void blackblock2(){
  
  pwm.setPWM(secondarmservo4, 0, SERVOMAX2);
  delay(2000);
  pwm.setPWM(secondarmservo3, 0, SERVOMIN2);
  delay(2000);
  pwm.setPWM(secondarmservo1, 0, 600);
  delay(2000);
  pwm.setPWM(secondarmservo3, 0, 340);
  delay(2000);
  pwm.setPWM(secondarmservo4, 0, 210);
  delay(2000);
  pwm.setPWM(secondarmservo6, 0, SERVOMIN2);
  delay(2000);
  pwm.setPWM(secondarmservo4, 0, SERVOMAX2);
  delay(2000);
  }
void pickupblock2(){
  /*
  1-400
  2-430
  3-350
  4-200
  5-700
  6-350*/
  pwm.setPWM(secondarmservo1, 0, 400);
  delay(2000);
  pwm.setPWM(secondarmservo2, 0, 430);
  delay(2000);
  pwm.setPWM(secondarmservo3, 0, 340);
  delay(2000);
  pwm.setPWM(secondarmservo5, 0, 700);
  delay(2000);
  pwm.setPWM(secondarmservo4, 0, 210);
  delay(2000);
  pwm.setPWM(secondarmservo6, 0, gripperclose);
  delay(2000);}
  

void armintpos2(){
  
  pwm.setPWM(secondarmservo6, 0, SERVOMIN2);
  delay(1000);
  pwm.setPWM(secondarmservo4, 0, SERVOMAX2);
  delay(1000);
  pwm.setPWM(secondarmservo5, 0, SERVOMAX2);
  delay(1000);
  pwm.setPWM(secondarmservo3, 0, SERVOMIN2);
  delay(1000);
  pwm.setPWM(secondarmservo2, 0, SERVOMIN2);
  delay(1000);
  pwm.setPWM(secondarmservo1, 0, SERVOMIN2);
  delay(1000);}
