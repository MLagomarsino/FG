
// driver BL
#define enBL1 22 // black
#define enBL2 23 // white

// motorBL
#define encoderBLA 30 
#define encoderBLB 31

// driverFL
#define enFL1 24 //green
#define enFL2 25 //blue

// motorFL
#define encoderFLA 32
#define encoderFLB 33

// driverFR
#define enFR1 42 // red
#define enFR2 43 // orange

// motorFR
#define encoderFRA 50
#define encoderFRB 51

// driverBR
#define enBR1 44 // orange
#define enBR2 45 // yellow

// motorBR
#define encoderBRA 52
#define encoderBRB 53

// motor pins
#define motorPinBL 2 // grey
#define motorPinFL 3 // purple
#define motorPinFR 4 // brown
#define motorPinBR 5 // red

#define LOOPTIME 100  

int stateA = 0;
int stateB = 0;
int counter = -1;
unsigned long t_start;
double vel = 0;
String sw = "";
double actvel = 0;

int countAnt = 0;
int PWM_val = 0; 
unsigned long lastMilli = 0; // loop timing 
unsigned long lastMilliPrint = 0; // loop timing
double speed_req = 0.0; // speed (Set Point)
double speed_act = 0.0; // speed (actual value)
double Kp = 16.0; // PID proportional control Gain
double Kd = 4.0; // PID Derivitave control gain
double last_error = 0.0;


void setup() {
  Serial.begin(115200); // set up Serial library at 115200 bps
  // motor BL
  pinMode(motorPinBL, OUTPUT);
  pinMode(enBL1, OUTPUT); // quelli che vanno al driver
  pinMode(enBL2, OUTPUT);
  pinMode (encoderBLA, INPUT); // quelli che vengono dal motore
  pinMode (encoderBLB, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderBLA), checkBLA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderBLB), checkBLB, CHANGE);
  digitalWrite(enBL1, LOW);
  digitalWrite(enBL2, HIGH);

  // motor FL
  pinMode(motorPinFL, OUTPUT);
  pinMode(enFL1, OUTPUT); // quelli che vanno al driver
  pinMode(enFL2, OUTPUT);
  pinMode (encoderFLA, INPUT); // quelli che vengono dal motore
  pinMode (encoderFLB, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderFLA), checkFLA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderFLB), checkFLB, CHANGE);
  digitalWrite(enFL1, LOW);
  digitalWrite(enFL2, HIGH);

  // motor FR
  pinMode(motorPinFR, OUTPUT);
  pinMode(enFR1, OUTPUT); // quelli che vanno al driver
  pinMode(enFR2, OUTPUT);
  pinMode (encoderFRA, INPUT); // quelli che vengono dal motore
  pinMode (encoderFRB, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderFRA), checkFRA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderFRB), checkFRB, CHANGE);
  digitalWrite(enFR1, LOW);
  digitalWrite(enFR2, HIGH);

  // motor BR
  pinMode(motorPinBR, OUTPUT);
  pinMode(enBR1, OUTPUT); // quelli che vanno al driver
  pinMode(enBR2, OUTPUT);
  pinMode (encoderBRA, INPUT); // quelli che vengono dal motore
  pinMode (encoderBRB, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderFRA), checkBRA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderFRB), checkBRB, CHANGE);
  digitalWrite(enBR1, LOW);
  digitalWrite(enBR2, HIGH);
  
  t_start = millis();
  analogWrite(motorPinBL, 200); // era commentato
  analogWrite(motorPinFL, 200); // era commentato
  analogWrite(motorPinFR, 200); // era commentato
  analogWrite(motorPinBR, 200); // era commentato  
  
  Serial.println("<Arduino is ready>");
  Serial.flush();
}

void loop() {
  
  String serialResponse ="";

  if (Serial.available()) {
    serialResponse  = Serial.readString();
    vel = serialResponse.substring(2,5).toDouble();
    sw = serialResponse.substring(1,2);
    if(sw == "+"){
      digitalWrite(enBL1, LOW); // clockwise: comando al driver di andare avanti
      digitalWrite(enBL2, HIGH);

      digitalWrite(enFL1, LOW);
      digitalWrite(enFL2, HIGH);

      
      digitalWrite(enFR1, LOW);
      digitalWrite(enFR2, HIGH);

      
      digitalWrite(enBR1, LOW);
      digitalWrite(enBR2, HIGH);
    }
    else if (sw == "-"){
      digitalWrite(enBL1, HIGH); // counterclockwise
      digitalWrite(enBL2, LOW);

      digitalWrite(enFL1, HIGH); 
      digitalWrite(enFL2, LOW);
      
      digitalWrite(enFR1, HIGH); 
      digitalWrite(enFR2, LOW);

      digitalWrite(enBR1, HIGH); 
      digitalWrite(enBR2, LOW);
    }
    speed_req = vel;
  }
  if((millis()-lastMilli) >= LOOPTIME){ 
    lastMilli = millis();
    speed_act = (double)(((double)(counter - countAnt)*(1000.0/(double)LOOPTIME))/(double)(320.0)); 
    countAnt = counter;
    PWM_val = updatePid(PWM_val, speed_req, speed_act); // compute PWM value
    analogWrite(motorPinBL, PWM_val);  
    analogWrite(motorPinFL, PWM_val);  
    analogWrite(motorPinFR, PWM_val);  
    analogWrite(motorPinBR, PWM_val);  
   }
   printMotorInfo();  
}

void printMotorInfo(){  // display data
 if((millis()-lastMilliPrint) >= 500){                     
   lastMilliPrint = millis();
   Serial.print("< RPM:");          
   Serial.print(speed_act);         
   Serial.print("  PWM:");  
   Serial.print(PWM_val);   
   Serial.print(" > \n");          
 }
}

int updatePid(int command, double targetValue, double currentValue){ // compute PWM value
  double pidTerm = 0.0; // PID correction
  double error = 0.0;                                                               
  error = (double) (fabs(targetValue) - fabs(currentValue)); 
  pidTerm = (Kp * error) + (Kd * (error - last_error));                          
  last_error = error;
  return constrain(command + int(pidTerm), 0, 255);
}

void checkBLA() {
  stateA = digitalRead(encoderBLA);
  stateB = digitalRead(encoderBLB);
  
  if (stateA != stateB) {
    counter++;
  }
  else {
    counter--;
  }
}

void checkBLB(){
  stateA = digitalRead(encoderBLA);
  stateB = digitalRead(encoderBLB);
 
  if (stateA == stateB) {
    counter++;
  }
  else {
    counter--;
  }
}


void checkFLA() {
  stateA = digitalRead(encoderFLA);
  stateB = digitalRead(encoderFLB);
  
  if (stateA != stateB) {
    counter++;
  }
  else {
    counter--;
  }
}

void checkFLB(){
  stateA = digitalRead(encoderFLA);
  stateB = digitalRead(encoderFLB);
 
  if (stateA == stateB) {
    counter++;
  }
  else {
    counter--;
  }
}


void checkFRA() {
  stateA = digitalRead(encoderFRA);
  stateB = digitalRead(encoderFRB);
  
  if (stateA != stateB) {
    counter++;
  }
  else {
    counter--;
  }
}

void checkFRB(){
  stateA = digitalRead(encoderFRA);
  stateB = digitalRead(encoderFRB);
 
  if (stateA == stateB) {
    counter++;
  }
  else {
    counter--;
  }
}

void checkBRA() {
  stateA = digitalRead(encoderBRA);
  stateB = digitalRead(encoderBRB);
  
  if (stateA != stateB) {
    counter++;
  }
  else {
    counter--;
  }
}

void checkBRB(){
  stateA = digitalRead(encoderBRA);
  stateB = digitalRead(encoderBRB);
 
  if (stateA == stateB) {
    counter++;
  }
  else {
    counter--;
  }
}
