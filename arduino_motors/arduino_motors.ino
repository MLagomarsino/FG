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

int stateBLA = 0;
int stateBLB = 0;
int stateFLA = 0;
int stateFLB = 0;
int stateFRA = 0;
int stateFRB = 0;
int stateBRA = 0;
int stateBRB = 0;

int counterBL = 0;
int counterFL = 0;
int counterFR = 0;
int counterBR = 0;

int countAntBL = 0;
int countAntFL = 0;
int countAntFR = 0;
int countAntBR = 0;

unsigned long t_start;

String sw = "";

double velBL = 0;
double velFL = 0;
double velFR = 0;
double velBR = 0;

int PWM_valBL = 0;
int PWM_valFL = 0; 
int PWM_valFR = 0; 
int PWM_valBR = 0; 
 
unsigned long lastMilli = 0; // loop timing 
unsigned long lastMilliPrint = 0; // loop timing

double speed_reqBL = 0.0; // speed (Set Point)
double speed_actBL = 0.0; // speed (actual value)
double speed_reqFL = 0.0; 
double speed_actFL = 0.0; 
double speed_reqFR = 0.0; 
double speed_actFR = 0.0; 
double speed_reqBR = 0.0; 
double speed_actBR = 0.0;

double Kp = 16.0; // PID proportional control Gain
double Kd = 4.0; // PID Derivitave control gain

double last_errorBL = 0.0;
double last_errorFL = 0.0;
double last_errorFR = 0.0;
double last_errorBR = 0.0;

void setup() {
  Serial.begin(115200); // set up Serial library at 115200 bps

  // motor BL
  pinMode(motorPinBL, OUTPUT);
  pinMode(enBL1, OUTPUT); // quelli che vanno al driver
  pinMode(enBL2, OUTPUT);
  pinMode(encoderBLA, INPUT); // quelli che vengono dal motore
  pinMode(encoderBLB, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderBLA), checkBLA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderBLB), checkBLB, CHANGE);
  digitalWrite(enBL1, LOW);
  digitalWrite(enBL2, HIGH);

  // motor FL
  pinMode(motorPinFL, OUTPUT);
  pinMode(enFL1, OUTPUT); // quelli che vanno al driver
  pinMode(enFL2, OUTPUT);
  pinMode(encoderFLA, INPUT); // quelli che vengono dal motore
  pinMode(encoderFLB, INPUT);
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

  if(Serial.available()) {
    serialResponse  = Serial.readString();
		if(serialResponse.length() < 5){
			swBL = serialResponse.substring(1,2);
			swFL = swBL;
			swFR = swFL;
			swBR = swFR;

			velBL = serialResponse.substring(2,5).toDouble();
			velFL = velBL;
			velFR = velFL;
			velBR = velFR;
		}
		else{
	 		swBL = serialResponse.substring(1,2);
		  velBL = serialResponse.substring(2,5).toDouble();
	
			swFL = serialResponse.substring(6,7);
			velFL = serialResponse.substring(7,10).toDouble();
			 
			swFR = serialResponse.substring(11,12);
			velFR = serialResponse.substring(12,15).toDouble();

			swBR = serialResponse.substring(16,17);
			velBR = serialResponse.substring(17,20).toDouble();
		}

    if(swBL == "+"){
      digitalWrite(enBL1, LOW); // clockwise: comando al driver di andare avanti
      digitalWrite(enBL2, HIGH);
		else if(swBL == "-"){
			digitalWrite(enBL1, HIGH); // counterclockwise
      digitalWrite(enBL2, LOW);
		}
      
		if(swFL == "+"){
			digitalWrite(enFL1, LOW);
      digitalWrite(enFL2, HIGH);
		}
		else if(swFL == "-"){
			digitalWrite(enFL1, HIGH);
      digitalWrite(enFL2, LOW);
		}

		if(swFR == "+"){
			digitalWrite(enFR1, LOW);
      digitalWrite(enFR2, HIGH);
		}
		else if(swFR == "-"){
			digitalWrite(enFR1, HIGH);
      digitalWrite(enFR2, LOW);
		}

		if(swBR == "+"){
			digitalWrite(enBR1, LOW);
      digitalWrite(enBR2, HIGH);
		}
		else if(swBR == "-"){
			digitalWrite(enBR1, HIGH);
      digitalWrite(enBR2, LOW);
		}

    speedBL_req = velBL;
		speedBL_req = velFL;
		speedBL_req = velFR;
		speedBL_req = velBR;
  }

  if((millis()-lastMilli) >= LOOPTIME){ 
		lastMilli = millis();

    speed_actBL = (double)(((double)(counterBL - countAntBL)*(1000.0/(double)LOOPTIME))/(double)(320.0)); 
		speed_actFL = (double)(((double)(counterBL - countAntFL)*(1000.0/(double)LOOPTIME))/(double)(320.0)); 
		speed_actFR = (double)(((double)(counterBL - countAntFR)*(1000.0/(double)LOOPTIME))/(double)(320.0)); 
		speed_actBR = (double)(((double)(counterBL - countAntBR)*(1000.0/(double)LOOPTIME))/(double)(320.0)); 

    countAntBL = counterBL;
		countAntFL = counterFL;
		countAntFR = counterFR;
		countAntBR = counterBR;

    PWM_valBL = updatePid(1, PWM_valBL, speed_reqBL, speed_actBL); // compute PWM value
		PWM_valFL = updatePid(2, PWM_valFL, speed_reqFL, speed_actFL); 
		PWM_valFR = updatePid(3, PWM_valFR, speed_reqFR, speed_actFR); 
		PWM_valBR = updatePid(4, PWM_valBR, speed_reqBR, speed_actBR); 

    analogWrite(motorPinBL, PWM_valBL);  
    analogWrite(motorPinFL, PWM_valFL);  
    analogWrite(motorPinFR, PWM_valFR);  
    analogWrite(motorPinBR, PWM_valBR);  
	}
  printMotorInfo(1);  
	printMotorInfo(2);  
	printMotorInfo(3);  
	printMotorInfo(4);  
}

void printMotorInfo(int motor){  // display data
	if((millis()-lastMilliPrint) >= 500){                     
  	lastMilliPrint = millis();
  	
		switch(motor){
			case 1: 
				Serial.print("< BL -> RPM:"); 
				Serial.print(speed_actBL);
				Serial.print("  PWM:");  
			 	Serial.print(PWM_valBL);   
				break;
			case 1: 
				Serial.print("< FL -> RPM:"); 
				Serial.print(speed_actFL);
				Serial.print("  PWM:");  
			 	Serial.print(PWM_valFL);   
				break;
			case 1: 
				Serial.print("< FR -> RPM:"); 
				Serial.print(speed_actFR);
				Serial.print("  PWM:");  
			 	Serial.print(PWM_valFR);    
				break;				
			case 1: 
				Serial.print("< BR -> RPM"); 
				Serial.print(speed_actBR);
				Serial.print("  PWM:");  
			 	Serial.print(PWM_valBR);  
				break;       
			}         
      Serial.print(" > \n"); 
 }
}

int updatePid(int motor, int command, double targetValue, double currentValue){ // compute PWM value
  double pidTerm = 0.0; // PID correction
  double error = 0.0;                                                               
  error = (double) (fabs(targetValue) - fabs(currentValue)); 

	switch(motor){
		case 1: 
			pidTerm = (Kp * error) + (Kd * (error - last_errorBL));  
			last_errorBL = error;
			break;
		case 2:
			pidTerm = (Kp * error) + (Kd * (error - last_errorFL));  
			last_errorFL = error;
			break;
		case 3:
			pidTerm = (Kp * error) + (Kd * (error - last_errorFR));  
			last_errorFR = error;			
			break;
		case 4:
			pidTerm = (Kp * error) + (Kd * (error - last_errorBR));  
			last_errorBR = error;			
			break;
	}
                          
  return constrain(command + int(pidTerm), 0, 255);
}

void checkBLA() {
  stateBLA = digitalRead(encoderBLA);
  stateBLB = digitalRead(encoderBLB);
  
  if (stateBLA != stateBLB) {
    counterBL++;
  }
  else {
    counterBL--;
  }
}

void checkBLB(){
  stateBLA = digitalRead(encoderBLA);
  stateBLB = digitalRead(encoderBLB);
 
  if (stateBLA == stateBLB) {
    counterBL++;
  }
  else {
    counterBL--;
  }
}


void checkFLA() {
  stateFLA = digitalRead(encoderFLA);
  stateFLB = digitalRead(encoderFLB);
  
  if (stateFLA != stateFLB) {
    counterFL++;
  }
  else {
    counterFL--;
  }
}

void checkFLB(){
  stateFLA = digitalRead(encoderFLA);
  stateFLB = digitalRead(encoderFLB);
 
  if (stateFLA == stateFLB) {
    counterFL++;
  }
  else {
    counterFL--;
  }
}


void checkFRA() {
  stateFRA = digitalRead(encoderFRA);
  stateFRB = digitalRead(encoderFRB);
  
  if (stateFRA != stateFRB) {
    counterFR++;
  }
  else {
    counterFR--;
  }
}

void checkFRB(){
  stateFRA = digitalRead(encoderFRA);
  stateFRB = digitalRead(encoderFRB);
 
  if (stateFRA == stateFRB) {
    counterFR++;
  }
  else {
    counterFR--;
  }
}

void checkBRA() {
  stateBRA = digitalRead(encoderBRA);
  stateBRB = digitalRead(encoderBRB);
  
  if (stateBRA != stateBRB) {
    counterBR++;
  }
  else {
    counterBR--;
  }
}

void checkBRB(){
  stateBRA = digitalRead(encoderBRA);
  stateBRB = digitalRead(encoderBRB);
 
  if (stateBRA == stateBRB) {
    counterBR++;
  }
  else {
    counterBR--;
  }
}
