// driver BL
#define enBL1 22 // black
#define enBL2 23 // white

// motorBL
#define encoderBLA 30 
#define encoderBLB 31

// motor pins
#define motorPinBL 2 // grey

#define LOOPTIME 100  

int stateBLA = 0;
int stateBLB = 0;

int counterBL = 0;
int countAntBL = 0;

unsigned long t_start;

String swBL = "";

double velBL = 0;

int PWM_valBL = 0;
 
unsigned long lastMilli = 0; // loop timing 
unsigned long lastMilliPrint = 0; // loop timing

double speed_reqBL = 0.0; // speed (Set Point)
double speed_actBL = 0.0; // speed (actual value)

double Kp = 16.0; // PID proportional control Gain
double Kd = 4.0; // PID Derivitave control gain

double last_errorBL = 0.0;

void checkBLA();
void checkBLB();

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
  
  t_start = millis();
  analogWrite(motorPinBL, 200); // era commentato
  
  Serial.println("<Arduino is ready>");
  Serial.flush();
}

void loop() {
  
	String serialResponse ="";

  if(Serial.available()) {
    serialResponse  = Serial.readString();
		if(serialResponse.length() < 5){
			swBL = serialResponse.substring(1,2);

			velBL = serialResponse.substring(2,5).toDouble();
		}
		else{
	 		swBL = serialResponse.substring(1,2);
		  velBL = serialResponse.substring(2,5).toDouble();
		}

    if(swBL == "+"){
      digitalWrite(enBL1, LOW); // clockwise: comando al driver di andare avanti
      digitalWrite(enBL2, HIGH);
    }
		else if(swBL == "-"){
			digitalWrite(enBL1, HIGH); // counterclockwise
      digitalWrite(enBL2, LOW);
		}
     
    speed_reqBL = velBL;
  }

  if((millis()-lastMilli) >= LOOPTIME){ 
		lastMilli = millis();

    speed_actBL = (double)(((double)(counterBL - countAntBL)*(1000.0/(double)LOOPTIME))/(double)(320.0)); 
	
    countAntBL = counterBL;

    PWM_valBL = updatePid(1, PWM_valBL, speed_reqBL, speed_actBL); // compute PWM value
		
    analogWrite(motorPinBL, PWM_valBL);  
	}
  printMotorInfo(1);  
}

void printMotorInfo(int motor){  // display data
	if((millis()-lastMilliPrint) >= 500){                     
  	lastMilliPrint = millis();
  	
	Serial.print("< BL RPS:"); 
	Serial.print(speed_actBL);
	Serial.print("  PWM:");  
 	Serial.print(PWM_valBL);   
			        
    Serial.print(" > \n"); 
 }
}

int updatePid(int motor, int command, double targetValue, double currentValue){ // compute PWM value
  double pidTerm = 0.0; // PID correction
  double error = 0.0;                                                               
  error = (double) (fabs(targetValue) - fabs(currentValue)); 

	pidTerm = (Kp * error) + (Kd * (error - last_errorBL));  
	last_errorBL = error;
                          
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
