// driverFL
#define enFL1 44 //green
#define enFL2 45 //blue

// motorFL
#define encoderFLA 52
#define encoderFLB 53

// motor pins
#define motorPinFL 5 // purple

#define LOOPTIME 100  

int stateFLA = 0;
int stateFLB = 0;

int counterFL = 0;
int countAntFL = 0;

unsigned long t_start;

String swFL = "";

double velFL = 0;

int PWM_valFL = 0; 
 
unsigned long lastMilli = 0; // loop timing 
unsigned long lastMilliPrint = 0; // loop timing

double speed_reqFL = 0.0; 
double speed_actFL = 0.0; 

double Kp = 16.0; // PID proportional control Gain
double Kd = 4.0; // PID Derivitave control gain

double last_errorFL = 0.0;

void checkFLA();
void checkFLB();

void setup() {
  Serial.begin(115200); // set up Serial library at 115200 bps

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

  t_start = millis();
  analogWrite(motorPinFL, 200); 
  
  Serial.println("<Arduino is ready>");
  Serial.flush();
}

void loop() {
  
	String serialResponse ="";

  if(Serial.available()) {
    serialResponse  = Serial.readString();
		if(serialResponse.length() < 5){
			swFL = serialResponse.substring(1,2);

			velFL = serialResponse.substring(2,5).toDouble();
		}
		else{	
			swFL = serialResponse.substring(6,7);
			velFL = serialResponse.substring(7,10).toDouble();
		}

  
		if(swFL == "+"){
			digitalWrite(enFL1, LOW);
      digitalWrite(enFL2, HIGH);
		}
		else if(swFL == "-"){
			digitalWrite(enFL1, HIGH);
      digitalWrite(enFL2, LOW);
		}

		speed_reqFL = velFL;
  }

  if((millis()-lastMilli) >= LOOPTIME){ 
	lastMilli = millis();

	speed_actFL = (double)(((double)(counterFL - countAntFL)*(1000.0/(double)LOOPTIME))/(double)(320.0)); 

	countAntFL = counterFL;

	PWM_valFL = updatePid(2, PWM_valFL, speed_reqFL, speed_actFL); 

    analogWrite(motorPinFL, PWM_valFL);  
	}
	printMotorInfo(2);  
}

void printMotorInfo(int motor){  // display data
	if((millis()-lastMilliPrint) >= 500){                     
  	lastMilliPrint = millis();
  	
	Serial.print("< FL RPS:"); 
	Serial.print(speed_actFL);
	Serial.print("  PWM:");  
 	Serial.print(PWM_valFL);   
    Serial.print(" > \n"); 
 }
}

int updatePid(int motor, int command, double targetValue, double currentValue){ // compute PWM value
  double pidTerm = 0.0; // PID correction
  double error = 0.0;                                                               
  error = (double) (fabs(targetValue) - fabs(currentValue)); 
	pidTerm = (Kp * error) + (Kd * (error - last_errorFL));  
	last_errorFL = error;
                          
  return constrain(command + int(pidTerm), 0, 255);
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

