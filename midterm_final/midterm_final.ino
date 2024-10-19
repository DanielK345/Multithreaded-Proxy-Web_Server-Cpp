// IMU & Ultra Sonic Setup
#include <Adafruit_BNO055.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <utility/imumaths.h>
#include "TimerOne.h"
#include <NewPing.h>
#include <math.h>
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29, &Wire);
/*------Task 3 params-------*/
int readIMU = 0; 

//Encoder setup
volatile int leftEnCount = 0;
volatile int rightEnCount = 0;
int enLA = 2;
int enLB = 3;
int enRA = 18;
int enRB = 19;

const int K = 30;

// Line Follower Setup
// int threshold = 512; 

//Robot parameters
const float d = 0.18; //m   
const float diameter = 0.043;  //m

const int pulsePerRev = 700;
const float wheelCircumference = diameter * 3.14; 

//Position control for distance 1
const float T = 1.2;    
const float gamma = 0.26;  
const float lamda = 0.8; 
const float h = 0.21;   

float x = 0.0;     
float y = 0.0;     
float theta = 0.0; 

const float goalX = 1;  
const float goalY = 1;  
const float goalTheta = 1; //radian

bool reachedGoal1 = false;  

// Position 2
bool task3state = false;
const int distance2 = 2; //m
const float pulseNeededforDistance2 = (distance2 * pulsePerRev) / wheelCircumference;
bool reachedGoal2 = false;  

//Angle
float currentAngle2 = 0;  
const float targetAngle2 = 90;  // will change to 90 in final code
bool rotationComplete1 = false;
bool rotationComplete2 = false;
bool task1Completed = false;
bool allSensorsOn = false;
// Ultrasonic sensor pins (3 cảm biến cố định)
int trigFront = 26;  // Trig pin for front ultrasonic sensor
int echoFront = 28;  // Echo pin for front ultrasonic sensor
int trigLeft = 34;   // Trig pin for left ultrasonic sensor
int echoLeft = 36;   // Echo pin for left ultrasonic sensor
int trigRight = 32;  // Trig pin for right ultrasonic sensor
int echoRight = 30;  // Echo pin for right ultrasonic sensor

//Encoder Interrupt
// volatile int leftEnCount = 0;
// volatile int rightEnCount = 0;


//PID
const float Kp = 6.5;      // Proportional constant
const float Kd = 4;       // Derivative constant
const float Ki = 0.001;       // Integral constant

int previousError = 0;      // Store the previous error for D term
int integralError = 0;      // Accumulate the error for I term

// Motor Setup
const int enL = 7;    // Left motor PWM pin
const int inL1 = 10;   // Left motor a pin 1
const int inL2 = 11;   // Left motor a pin 2
const int enR = 12;   // Right motor PWM pin
const int inR1 = 8;  // Right motor a pin 1
const int inR2 = 9;  // Right motor a pin 2

// Define distance threshold
// const int threshold = 25; // Distance in cm to start avoiding
// const int wallDistance = 25; // Ideal distance from the wall while following

//Speed
const int baseSpeed = 70;   // Base motor speed
const int maxSpeed = 130;   // Maximum motor speed
const int minSpeed = 50;    // Minimum motor speed


//IR sensor
int IRvalue1 = 0;
int IRvalue2 = 0;
int IRvalue3 = 0;
int IRvalue4 = 0;
int IRvalue5 = 0;

const int pinOUT1 = A11;  // Leftmost sensor
const int pinOUT2 = A12;  // Left sensor
const int pinOUT3 = A13;  // Middle sensor
const int pinOUT4 = A14;  // Right sensor
const int pinOUT5 = A15;  // Rightmost sensor

int safeDistance = 20;
int safeDistance2 = 15;

// Speed constants
int normalSpeed1 = 75;   // Normal speed when following the line correctly
int slowSpeed1 = 75;     // Slow speed for corners or curves
int minSpeed1 = 60, maxSpeed1 = 90; // Minimum and maximum motor speeds


float distanceLeft;
float distanceFront;
float distanceRight;

bool isRight = 0, isLeft = 0;

// bool obstacle;
int a;
bool task1State = false;
bool task3State = false;
// -----------------------------------------------SET UP---------------------------------------------------------------
void setup() {
  Serial.begin(9600);
  pinMode(enL, OUTPUT);
  pinMode(inL1, OUTPUT);
  pinMode(inL2, OUTPUT);
  pinMode(enR, OUTPUT);
  pinMode(inR1, OUTPUT);
  pinMode(inR2, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(enLA), leftEnISRA, RISING);
  attachInterrupt(digitalPinToInterrupt(enLB), leftEnISRB, RISING);
  attachInterrupt(digitalPinToInterrupt(enRA), rightEnISRA, RISING);
  attachInterrupt(digitalPinToInterrupt(enRB), rightEnISRB, RISING);
  pinMode(pinOUT1, INPUT);
  pinMode(pinOUT2, INPUT);
  pinMode(pinOUT3, INPUT);
  pinMode(pinOUT4, INPUT);
  pinMode(pinOUT5, INPUT);

  pinMode(trigFront, OUTPUT);
  pinMode(echoFront, INPUT);
  pinMode(trigLeft, OUTPUT);
  pinMode(echoLeft, INPUT);
  pinMode(trigRight, OUTPUT);
  pinMode(echoRight, INPUT);
  // obstacle = false;
  // a = 0;

}
// -----------------------------------------------LOOP---------------------------------------------------------------
void loop() {  
  IRvalue1 = digitalRead(pinOUT1);
  IRvalue2 = digitalRead(pinOUT2);
  IRvalue3 = digitalRead(pinOUT3);
  IRvalue4 = digitalRead(pinOUT4);
  IRvalue5 = digitalRead(pinOUT5);
  int sensorCount = IRvalue1 + IRvalue2 + IRvalue3 + IRvalue4 + IRvalue5;
  // Get distances from the front, left, and right ultrasonic sensors
  distanceFront = getDistance(trigFront, echoFront);
  distanceLeft = getDistance(trigLeft, echoLeft);
  distanceRight = getDistance(trigRight, echoRight);

  // Debugging print for sensor readings
  Serial.print("Front: ");
  Serial.print(distanceFront);
  Serial.print(" cm, Left: ");
  Serial.print(distanceLeft);
  Serial.print(" cm, Right: ");
  Serial.print(distanceRight);
  Serial.println(" cm");

  if (sensorCount < 5){
      followLine();

      if (distanceFront < safeDistance) avoidObstacle(distanceLeft, distanceRight);
     
  }

  else if (sensorCount == 5){
   while(true){
      stop();
      delay(1000);
      runTask3();

      Serial.println("running 3");

      if (reachedGoal2 = true) {
        Serial.println("goal");
        break;
        stop();
      }
   }


  }

  // if (distanceFront > safeDistance && sensorCount < 5) {
  //   // moveForward();
  //   followLine();
  //   // return;
  //   // Obstacle avoidance logic 
  // }
  //     // REACH GOAL
  // // if (sensorCount == 5){
  // //   // task3state = true;
  // //   runTask3();
  // // }
  // // if(task3state == true){
  // //   runTask3();
  // //   return;
  // // }
  // else {
  //   avoidObstacle(distanceLeft, distanceRight);
  // }
  // // delay(300);
}
// -----------------------------------------------TASK2---------------------------------------------------------------
void moveForward() {
  digitalWrite(inL1, LOW);
  digitalWrite(inL2, HIGH);
  digitalWrite(inR1, HIGH);  
  digitalWrite(inR2, LOW); 
  analogWrite(enL, 70); // Speed for left motor (0-255)
  analogWrite(enR, 70); // Speed for right motor (0-255)
}

void avoidObstacle(int distanceLeft, int distanceRight) {
  // distanceLeft = getDistance(trigLeft, echoLeft);
  distanceRight = getDistance(trigRight, echoRight);
  moveRobot(0,0);
  delay(500);
  // goBackward(70,70); 
  // delay(200);
  turnL(60, 100);
  delay(500);
  moveRobot(150,150);
  delay(500);
  turnR(100, 60);
  delay(100);
  moveRobot(0,0);
  delay(500);
  while (true) {
      distanceRight = getDistance(trigRight, echoRight);
      isRight = 1;
      isLeft = 0;
        
      if (distanceRight <= safeDistance2 * 2 && lineDetected() == false) {
     
      wall_distance(distanceRight);
      Serial.print("distanceRight: ");
      Serial.println(distanceRight);
      } 
      else if ((distanceRight > safeDistance2 * 2 || distanceRight == 0)  && lineDetected() == false)  {
      stop();
      delay(500);
      // moveRobot(150,150);
      // delay(400);
      turnR(100, 60);  // Rẽ phải một chút
      delay(250);
      moveRobot(150, 150);
      delay(400);
      }

      // Check if the line is detected again
      else if (lineDetected()) {
        stop();
        delay(1000);
        IRvalue2 = digitalRead(pinOUT2);
        IRvalue3 = digitalRead(pinOUT3);
        IRvalue4 = digitalRead(pinOUT4);
        while (IRvalue2 == 0 && IRvalue3 == 0 && IRvalue4 == 0){
          IRvalue2 = digitalRead(pinOUT2);
          IRvalue3 = digitalRead(pinOUT3);
          IRvalue4 = digitalRead(pinOUT4);
          turnL(80,80);          
        }
        stop();
        delay(1000);
        // Small delay to stabilize
        return; // Exit obstacle handling
      }
    // }
  }
}

void wall_distance(float distance) {
  // PID control to maintain a safe distance from the wall
  float Kp1 = 40, Ki1 = 0 , Kd1 = 0;
  float setPoint = safeDistance2;
  int in_min = 0;
  int in_max =  (Kp1 * setPoint) + (Ki1 * setPoint) + (Kd1 * setPoint) ;
  int out_min = 0;
  int out_max = slowSpeed1 - minSpeed1;

  // Variables for PID control
  float error = 0;
  static float lastError = 0;
  float integral = 0;
  float derivative = 0;
  float motorSpeedCorrection = 0;

  // Calculate PID control
  distance = constrain(distance, 0, setPoint*2);
  error = setPoint - distance;       // Proportional term
  integral += error;                 // Integral term
  derivative = error - lastError;    // Derivative term
  motorSpeedCorrection = (Kp1 * error) + (Ki1 * integral) + (Kd1 * derivative) ;
        Serial.print("motorSpeedCorrection before: ");
        Serial.println(motorSpeedCorrection);
  // Ensure correction value stays within motor speed limits
  motorSpeedCorrection = map(motorSpeedCorrection, -in_max, in_max, -out_max, out_max);
        Serial.print("motorSpeedCorrection after: ");
        Serial.println(motorSpeedCorrection);
  // Control the motors to maintain distance
  int leftMotorSpeed = 0;
  int rightMotorSpeed = 0;

  if (isRight == 1 && isLeft == 0) {
    leftMotorSpeed = slowSpeed1 - motorSpeedCorrection;   // Adjust left motor speed
    rightMotorSpeed = slowSpeed1 + motorSpeedCorrection;  // Adjust right motor speed
  } else if (isRight == 0 && isLeft == 1) {
    leftMotorSpeed = slowSpeed1 + motorSpeedCorrection;   // Adjust left motor speed
    rightMotorSpeed = slowSpeed1 - motorSpeedCorrection;  // Adjust right motor speed
  } else {
    return;
  }

  leftMotorSpeed = constrain(leftMotorSpeed, minSpeed1, maxSpeed1);
  rightMotorSpeed = constrain(rightMotorSpeed, minSpeed1, maxSpeed1);
        Serial.print("mleftMotorSpeed: ");
      
        Serial.println(leftMotorSpeed);
        Serial.print("rightMotorSpeed ");
        Serial.println(rightMotorSpeed);
  moveRobot(leftMotorSpeed, rightMotorSpeed);

  // Update last error for the next cycle
  lastError = error;
}

bool lineDetected() {
  IRvalue1 = digitalRead(pinOUT1);
  IRvalue2 = digitalRead(pinOUT2);
  IRvalue3 = digitalRead(pinOUT3);
  IRvalue4 = digitalRead(pinOUT4);
  IRvalue5 = digitalRead(pinOUT5);

  // Return true if any of the middle line sensors detect the line
  return (IRvalue2 == 1 || IRvalue3 == 1 || IRvalue4 == 1);
}

void turnLeft() {
  // Left motor stops, right motor moves forward
  digitalWrite(inL1, HIGH);
  digitalWrite(inL2, LOW);  
  digitalWrite(inR1, HIGH);  
  digitalWrite(inR2, LOW);
  analogWrite(enL, 110);   // Stop left motor
  analogWrite(enR, 110);
  
}

void turnRight() {
  // Right motor stops, left motor moves forward
  digitalWrite(inL1, LOW);  // Move left motor forward
  digitalWrite(inL2, HIGH);
  digitalWrite(inR1, LOW);   // Stop right motor
  digitalWrite(inR2, HIGH);
  analogWrite(enL, 95); // Move left motor
  analogWrite(enR, 95);
}

void turnLeftSlight() {
  // Slight left turn to correct path
  digitalWrite(inL1, LOW);
  digitalWrite(inL2, HIGH);
  digitalWrite(inR1, HIGH);  
  digitalWrite(inR2, LOW);
  analogWrite(enL, 70);
  analogWrite(enR, 90);
}

void turnRightSlight() { 
  // Slight right turn to correct path
  digitalWrite(inL1, LOW);
  digitalWrite(inL2, HIGH);
  digitalWrite(inR1, HIGH);  
  digitalWrite(inR2, LOW);
  analogWrite(enL, 90);
  analogWrite(enR, 60);
}
// -----------------------------------------------TASK1---------------------------------------------------------------
void followLine() {
  task3state = false;
  IRvalue1 = digitalRead(pinOUT1);
  IRvalue2 = digitalRead(pinOUT2);
  IRvalue3 = digitalRead(pinOUT3);
  IRvalue4 = digitalRead(pinOUT4);
  IRvalue5 = digitalRead(pinOUT5);
  int sensorCount = IRvalue1 + IRvalue2 + IRvalue3 + IRvalue4 + IRvalue5;
    // Nếu tất cả các cảm biến đều không phát hiện vạch, lùi lại
  while (sensorCount == 0) {
    goBackward(50,50); // Lùi lại với tốc độ nhỏ
    delay(100); // Lùi lại trong 100ms

    // Cập nhật lại giá trị cảm biến sau khi lùi
    IRvalue1 = digitalRead(pinOUT1);
    IRvalue2 = digitalRead(pinOUT2);
    IRvalue3 = digitalRead(pinOUT3);
    IRvalue4 = digitalRead(pinOUT4);
    IRvalue5 = digitalRead(pinOUT5);
    sensorCount = IRvalue1 + IRvalue2 + IRvalue3 + IRvalue4 + IRvalue5;

    // Nếu ít nhất 1 cảm biến phát hiện lại vạch, thoát vòng lặp và tiếp tục đi
    if (sensorCount > 0) {
      break;
    }
  }

  int error = 0;
  if (IRvalue1 == 1 && IRvalue2 == 0 && IRvalue3 == 0 && IRvalue4 == 0 && IRvalue5 == 0) error = 4;
  if (IRvalue1 == 1 && IRvalue2 == 1 && IRvalue3 == 0 && IRvalue4 == 0 && IRvalue5 == 0) error = 3;
  if (IRvalue1 == 1 && IRvalue2 == 1 && IRvalue3 == 1 && IRvalue4 == 0 && IRvalue5 == 0) error = 3;
  if (IRvalue1 == 0 && IRvalue2 == 1 && IRvalue3 == 0 && IRvalue4 == 0 && IRvalue5 == 0) error = 2;
  if (IRvalue1 == 0 && IRvalue2 == 1 && IRvalue3 == 1 && IRvalue4 == 0 && IRvalue5 == 0) error = 1;
  if (IRvalue1 == 0 && IRvalue2 == 0 && IRvalue3 == 1 && IRvalue4 == 0 && IRvalue5 == 0) error = 0;
  if (IRvalue1 == 0 && IRvalue2 == 0 && IRvalue3 == 1 && IRvalue4 == 1 && IRvalue5 == 0) error = -1;
  if (IRvalue1 == 0 && IRvalue2 == 0 && IRvalue3 == 0 && IRvalue4 == 1 && IRvalue5 == 0) error = -2;
  if (IRvalue1 == 0 && IRvalue2 == 0 && IRvalue3 == 0 && IRvalue4 == 1 && IRvalue5 == 1) error = -3;
  if (IRvalue1 == 0 && IRvalue2 == 0 && IRvalue3 == 1 && IRvalue4 == 1 && IRvalue5 == 1) error = -3;
  if (IRvalue1 == 0 && IRvalue2 == 0 && IRvalue3 == 0 && IRvalue4 == 0 && IRvalue5 == 1) error = -4;
  int proportional = Kp * error;
  integralError += error; 
  int integral = Ki * integralError;
  int derivative = Kd * (error - previousError);
  previousError = error;
  int adjustment = proportional + integral + derivative;
  int leftSpeed = baseSpeed - adjustment;
  int rightSpeed = baseSpeed + adjustment;
  leftSpeed = constrain(leftSpeed, minSpeed, maxSpeed);
  rightSpeed = constrain(rightSpeed, minSpeed, maxSpeed);
  moveRobot(leftSpeed, rightSpeed );
  // delay(50); 
}

void moveRobot(int leftSpeed, int rightSpeed) {
  analogWrite(enL, leftSpeed);
  analogWrite(enR, rightSpeed);
  digitalWrite(inL1, HIGH);  // Left Motor forward
  digitalWrite(inL2, LOW); // Left Motor backward
  digitalWrite(inR1, HIGH); // Right Motor forward
  digitalWrite(inR2, LOW);  // Right Motor backward
}

void stop(){ //stop
  digitalWrite(inL1, LOW); //Left Motor backword Pin 
  digitalWrite(inL2, LOW); //Left Motor forword Pin 
  digitalWrite(inR1, LOW); //Right Motor forword Pin 
  digitalWrite(inR2, LOW); //Right Motor backword Pin 
  analogWrite(enL, 0);
  analogWrite(enR, 0);
}
// -----------------------------------------------TASK3---------------------------------------------------------------

void rotateUntilLine() {

    if (a == 1) { // Rotate left
        // Set motors to rotate left (clockwise rotation)
        digitalWrite(inL1, LOW);
        digitalWrite(inL2, HIGH);
        analogWrite(enL, 90);  // Adjust speed if necessary
        
        digitalWrite(inR1, HIGH);
        digitalWrite(inR2, LOW);
        analogWrite(enR, 90);  // Adjust speed if necessary

    } else if (a == 2) { // Rotate right
        // Set motors to rotate right (counterclockwise rotation)
        digitalWrite(inL1, HIGH);
        digitalWrite(inL2, LOW);
        analogWrite(enL, 90);  // Adjust speed if necessary
        digitalWrite(inR1, LOW);
        digitalWrite(inR2, HIGH);
        analogWrite(enR, 90);  // Adjust speed if necessary
    }
}

void forwardHalfRobot() {
    // Reset encoder count for the left motor
    leftEnCount = 0;  // Assuming leftendcount keeps track of the left encoder pulse count
    rightEnCount = 0; 
    // Move forward until the left encoder reaches the required pulse count for 10 cm
    while (leftEnCount + rightEnCount < 500) {
      Serial.println(leftEnCount);
      // Set motors to move forward
      digitalWrite(inL1, LOW);
      digitalWrite(inL2, HIGH);
      digitalWrite(inR1, HIGH);
      digitalWrite(inR2, LOW);
      analogWrite(enL, 80);
      analogWrite(enR, 80);  // Full speed, adjust if necessary
    }
}

void leftEnISRA() {
  leftEnCount++;
}

void leftEnISRB() {
  leftEnCount++;
}
void rightEnISRA() {
  rightEnCount++;
}

void rightEnISRB() {
  rightEnCount++;
}

void turnR(int leftSpeed, int rightSpeed) {
  // For PWM, the maximum possible value is 0 to 255
  analogWrite(enR, rightSpeed);
  analogWrite(enL, leftSpeed);

  // Set motors to move forward
  digitalWrite(inL1, LOW);
  digitalWrite(inL2, HIGH);
  digitalWrite(inR1, HIGH);
  digitalWrite(inR2, LOW);
}
void turnL(int leftSpeed, int rightSpeed) {
  // For PWM, the maximum possible value is 0 to 255
  analogWrite(enR, rightSpeed);
  analogWrite(enL, leftSpeed);

  // Set motors to move forward
  digitalWrite(inL1, HIGH);
  digitalWrite(inL2, LOW);
  digitalWrite(inR1, LOW);
  digitalWrite(inR2, HIGH);
}

void turn(int leftSpeed, int rightSpeed) {
  // For PWM, the maximum possible value is 0 to 255
  analogWrite(enR, rightSpeed);
  analogWrite(enL, leftSpeed);

  // Set motors to move forward
  digitalWrite(inL1, HIGH);
  digitalWrite(inL2, LOW);
  digitalWrite(inR1, HIGH);
  digitalWrite(inR2, LOW);
}


int getDistance(int trigPin, int echoPin) {
  // Send a 10us pulse to trigger the ultrasonic sensor
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read the echo time and calculate the distance in cm
  long duration = pulseIn(echoPin, HIGH);
  int distance = duration * 0.034 / 2; // Convert time to distance (speed of sound is ~340 m/s)
  return distance;
}

void set_speedL(float speed) {
  int pwmValue = constrain(speed, 0, 255); // Constrain speed to PWM range
  analogWrite(enL, pwmValue);
  digitalWrite(inL1, HIGH);
  digitalWrite(inL2, LOW);
}

// // Function to set the speed of the right wheel
void set_speedR(float speed) {
  int pwmValue = constrain(speed, 0, 255); // Constrain speed to PWM range
  analogWrite(enR, pwmValue);
  digitalWrite(inR1, HIGH);
  digitalWrite(inR2, LOW);
}

// // Function to get the current speed of the left wheel
float get_speedL() {
  // Convert encoder counts to speed in m/s
  float countsPerSecond = leftEnCount / T; // Counts per second
  float speedRPM = (countsPerSecond * 60) / pulsePerRev; // RPM
  float speedMetersPerSecond = speedRPM * (wheelCircumference / 60); // m/s
  leftEnCount = 0; // Reset count after reading
  return speedMetersPerSecond;
}

float get_speedR() {
  // Convert encoder counts to speed in m/s
  float countsPerSecond = rightEnCount / T; // Counts per second
  float speedRPM = (countsPerSecond * 60) / pulsePerRev; // RPM
  float speedMetersPerSecond = speedRPM * (wheelCircumference / 60); // m/s
  rightEnCount = 0; // Reset count after reading
  return speedMetersPerSecond;
}

void gotoGoal1(){
  
  float deltaX = goalX - x;
  float deltaY = goalY - y;
  float rho = sqrt(deltaX * deltaX + deltaY * deltaY);
  float phi = atan2(deltaY, deltaX) - goalTheta;
  float alpha = atan2(deltaY, deltaX) - theta;

  float v = gamma * cos(alpha) * rho;
  float w = lamda * alpha + gamma * cos(alpha) * sin(alpha) * (alpha + h * phi) / alpha;

  float vr = v + d * w / 2.0;
  float vl = v - d * w / 2.0;
  float wr = vr * 60.0 / wheelCircumference; // Angular velocity in RPM
  float wl = vl * 60.0 / wheelCircumference;
  set_speedL(wl);
  set_speedR(wr);

  delay(T * 1000); 
   // Update position
  float v1 = get_speedL();
  float v2 = get_speedR();
  x += (v1 + v2) / 2.0 * cos(theta) * T;
  y += (v1 + v2) / 2.0 * sin(theta) * T;
  theta += (v2 - v1) / d * T;
  if (fabs(goalX - x) <= 0.15 && fabs(goalY - y) <= 0.15) {
    stop();
    reachedGoal1 = true;
  }
 
}

void rotateToGoalAngle(){
  // delay(2000);
  sensors_event_t event;
  bno.getEvent(&event);
  currentAngle2 = event.orientation.x; 
  float angleError2 = currentAngle2 - targetAngle2;

  if (angleError2 > 1) { 
     // left wheel backward
      analogWrite(enL, 100);
      digitalWrite(inL1, LOW);
      digitalWrite(inL2, HIGH);
  
  } else if (angleError2 < -1) { 
          // left wheel forward
      analogWrite(enL, 100);
      digitalWrite(inL1, HIGH);
      digitalWrite(inL2, LOW);

  }
    else {
    stop(); 
    Serial.println("Rotation complete.");
    rotationComplete2 = true;
  }
    
  // Serial.print("Current angle: ");
  // Serial.println(currentAngle);

}

void goForwardtoGoal2(int baseSpeed) {
  // Reset encoder counters
  const int K = 30;
  leftEnCount = 0;
  rightEnCount = 0;
  // Serial.print("Pulse Needed for Distance2: ");
  // Serial.println(pulseNeededforDistance1);


  while (leftEnCount < pulseNeededforDistance2 && rightEnCount < pulseNeededforDistance2) {
    // Adjust motor speeds based on encoder counts
    int error = rightEnCount - leftEnCount;    
    int motor_L_speed = baseSpeed + K * error;  
    int motor_R_speed = baseSpeed - K * error;  

    // // Constrain speeds to the valid PWM range
    motor_L_speed = constrain(motor_L_speed, 0, 255);
    motor_R_speed = constrain(motor_R_speed, 0, 255);

    // Set motor speeds
    analogWrite(enL, motor_L_speed);
    analogWrite(enR, motor_R_speed);

  }
  delay(5000);
  stop();  
  reachedGoal2 = true;  

}

void moveForwardDistance(float distance) {
  leftEnCount = 0;
  rightEnCount = 0;

  long targetPulses = distance / wheelCircumference *pulsePerRev;

  
  while (leftEnCount < targetPulses && rightEnCount < targetPulses) {
    // Start the motors to move forward
    analogWrite(enL, 100);
    digitalWrite(inL1, HIGH);
    digitalWrite(inL2, LOW); 

    analogWrite(enR, 100);
    digitalWrite(inR1, HIGH);
    digitalWrite(inR2, LOW); 
  }
  analogWrite(enL, 0);
  analogWrite(enR, 0);
}


void runTask3(){
   if (!reachedGoal1) {
    gotoGoal1();
    delay(500);
  } else if (!rotationComplete2) {
    rotateToGoalAngle();
    delay(500);
  }
  if (!reachedGoal2) {
    Serial.println("GOal1");
    goForwardtoGoal2(200);
    // moveForwardDistance(50);
    delay(5000);
  // } else {
  //   stop();
  //   while(1);
    
  }
}

void goForward(float left, float right){
  analogWrite(enL, left);
  digitalWrite(inL1, HIGH);
  digitalWrite(inL2, LOW); 

  analogWrite(enR, right);
  digitalWrite(inR1, HIGH);
  digitalWrite(inR2, LOW); 

}


void goBackward(float left, float right) //rpm
{
  
  analogWrite(enL, left);
  digitalWrite(inL1, LOW);
  digitalWrite(inL2, HIGH); 

  analogWrite(enR, right);
  digitalWrite(inR1, LOW);
  digitalWrite(inR2, HIGH); 
}


