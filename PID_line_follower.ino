int enL = 7;     
int inL1 = 8;    
int inL2 = 9;    
int enR = 12;    
int inR1 = 10;  
int inR2 = 11;  

// Encoder pins
int enLA = 2;
int enLB = 3;
int enRA = 18;
int enRB = 19;

int mode = 0;

# define STOPPED 0
# define FOLLOWING_LINE 1
# define NO_LINE 2


int baseSpeedL = 80;
int baseSpeedR = 80;

int P;
int I;
int D;

float  Kp = 5;
float Ki = 0;
float Kd = 6;



int sensor1 = A11; // Far Left
int sensor2 = A12; // Middle Left
int sensor3 = A13; // Middle Right
int sensor4 = A14; // Far Right
int sensor5 = A15; // Middle Left

volatile int leftEnCount = 0;
volatile int rightEnCount = 0;



void PID_control() {
  // Read sensor values
  int leftSensor = digitalRead(sensor1); 
  int midLeftSensor = digitalRead(sensor2);
  int midSensor = digitalRead(sensor3);
  int midRightSensor = digitalRead(sensor4); 
  int rightSensor = digitalRead(sensor5); 
  int slow_speed = 0;

  boolean all_low = (midSensor == LOW) &&
                 (midLeftSensor == LOW) &&
                 (midRightSensor == LOW) &&
                 (leftSensor == LOW) &&
                 (rightSensor == LOW);

  stop();

  int error = 0;
  int lastError = error;
  float motorSpeedChange;

  if (midSensor == HIGH) {
    // 0 0 1 0 0 or 0 1 1 1 0
    error = 0;

    if (midLeftSensor == HIGH){
      error = -1;
    }

    if (midRightSensor == HIGH){
      error = 1;
    }
  } 

  else if (midLeftSensor == HIGH) {
    // 0 1 0 0 0
    error = -2;

    if (leftSensor == HIGH){
      error = -3;
      // slow_speed = 10;
    }

    if (midRightSensor == HIGH){
      error = 0;
    }
  } 

  else if (midRightSensor == HIGH) {
    // 0 0 0 1 0 
    error = 2;

    if (rightSensor == HIGH){
      error = 3;
      // slow_speed = 10;
    }

    if (midLeftSensor == HIGH){
      error = 0;
    }
  }

  else if (leftSensor == HIGH) {
    // 1 0 0 0 0 
    error = -5;
    // slow_speed = 40;

  } 

  else if (rightSensor == HIGH) {
    // 0 0 0 0 1
    error = 5;
    // slow_speed = 40;
  }

  P = error;
  I = error + I;
  D = error - lastError;

  motorSpeedChange = P*Kp  + I*Ki + D*Kd;

  int motorSpeedL = baseSpeedL + motorSpeedChange;
  int motorSpeedR  = baseSpeedR - motorSpeedChange;

  if (motorSpeedL > 150) {
    motorSpeedL =  150;
  }
  if (motorSpeedR > 150) {
    motorSpeedR = 150;
  }
  if  (motorSpeedL < 10) {
    motorSpeedL = 10;
  }
  if (motorSpeedR < 10)  {
    motorSpeedR = 10;
  }

  lastError = error;


  if (all_low == false){
    set_speedL(motorSpeedL);
    set_speedR(motorSpeedR);

     Serial.print("left:"); Serial.println(motorSpeedL);
     Serial.print("right:"); Serial.println(motorSpeedR);
  }

  // else if (all_low){
  //   stop();
  //   P = 0;
  //   I = 0;
  //   D = 0;
  // }
}

void PID2(){
  int s1 = digitalRead(sensor1); 
  int s2 = digitalRead(sensor2);
  int s3 = digitalRead(sensor3);
  int s4 = digitalRead(sensor4); 
  int s5 = digitalRead(sensor5); 
  int speedL = 0;
  int speedR = 0;
  int speedChange = 0;
  int total = s1 + s2 + s3 + s4 +s5;
  int error = 0;
  int lastError = error;
  float motorSpeedChange;

  if (total == 0 || total >=4){
    stop();
    P = 0;
    I = 0;
    D = 0;
  }

  else{
    error = ((-2*s1) + (-1*s2) + (1*s4) + (2*s5))/total;
    P = error;
    I = I + error;
    D = error - lastError;
    lastError = error;
    speedChange = P*Kp + I*Ki + D*Kd;

    if (speedChange > 100){
      speedChange = 100;
    }
    else if (speedChange < -100){
      speedChange = -100;
    }

    speedL = baseSpeedL + speedChange;
    speedR = baseSpeedR - speedChange;

    set_speedL(speedL);
    set_speedR(speedR);

    Serial.print("left:"); Serial.println(speedL);
    Serial.print("right:"); Serial.println(speedR);
  }

}

void goForward(float left, float right){
  int pwm_left = constrain(rpmToPWM(left), 0, 255);
  int pwm_right = constrain(rpmToPWM(right), 0, 255);

  analogWrite(enL, pwm_left);
  digitalWrite(inL1, HIGH);
  digitalWrite(inL2, LOW); 

  analogWrite(enR, pwm_right);
  digitalWrite(inL1, HIGH);
  digitalWrite(inL2, LOW); 

}

void goBackward(float rpm){
  int pwm_left = constrain(rpmToPWM(rpm), 0, 255);
  int pwm_right = constrain(rpmToPWM(rpm), 0, 255);

  analogWrite(enL, pwm_left);
  digitalWrite(inL1, LOW);
  digitalWrite(inL2, HIGH); 

  analogWrite(enR, pwm_right);
  digitalWrite(inL1, LOW);
  digitalWrite(inL2, HIGH); 
}

void set_speedL(float rpm) {
  int pwm = constrain(rpmToPWM(rpm), 0, 255);
  analogWrite(enL, pwm);
  digitalWrite(inL1, HIGH);
  digitalWrite(inL2, LOW); // Set left motor to forward
}

void set_speedR(float rpm) {
  int pwm = constrain(rpmToPWM(rpm), 0, 255);
  analogWrite(enR, pwm);
  digitalWrite(inR1, HIGH);
  digitalWrite(inR2, LOW); // Set right motor to forward
}

int rpmToPWM(float rpm) {
  return map(rpm, 0,120, 0, 255);
}


void stop() {
  digitalWrite(inR1, LOW);
  digitalWrite(inR2, LOW);
  digitalWrite(inL1, LOW);
  digitalWrite(inL2, LOW);

  leftEnCount = 0;
  rightEnCount = 0;
}


void setup()
{
  Serial.begin(9600);

  attachInterrupt(digitalPinToInterrupt(enLA), leftEnISRA, RISING);
  attachInterrupt(digitalPinToInterrupt(enLB), leftEnISRB, RISING);
  attachInterrupt(digitalPinToInterrupt(enRA), rightEnISRA, RISING);
  attachInterrupt(digitalPinToInterrupt(enRB), rightEnISRB, RISING);


  // Set all the motor control pins to outputs
	pinMode(enR, OUTPUT);
	pinMode(enL, OUTPUT);
	pinMode(inR1, OUTPUT);
	pinMode(inR2, OUTPUT);
	pinMode(inL1, OUTPUT);
	pinMode(inL2, OUTPUT);

  pinMode(sensor1, INPUT);
  pinMode(sensor2, INPUT);
  pinMode(sensor3, INPUT);
  pinMode(sensor4, INPUT);
  pinMode(sensor5, INPUT);


	
	// Turn off motors - Initial state
  stop();

}

// Interrupt Service Routines (ISRs)
void leftEnISRA() { leftEnCount++; }
void leftEnISRB() { leftEnCount++; }
void rightEnISRA() { rightEnCount++; }
void rightEnISRB() { rightEnCount++; }



void loop() {
  PID_control();
  // PID2();
}
