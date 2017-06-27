#include <PID_v1.h>

#define speedInput 2

#define brake A4
#define clutch A5
#define led 13
#define speedInput 2

#define PWM1 3
#define PWM2 11

#define PEDAL1MAX 220

#define pedal1 A0
#define pedal2 A1
#define button A3

//sensors values
int pedal1Val;
int pedal1MinVal = 255;
int pedal2Val;
int cruise1Val;
int cruise2Val;

int deviceButtonVal;
byte clutchVal;
byte brakeVal;

//speeds
unsigned long keepSpeed = 0;
unsigned long currentSpeed = 0;
unsigned long lastStoredSpeed = 0;
unsigned long speeds[5] = {0,0,0,0,0};

double Setpoint, Input, Output;

//misc
byte mode; //0 - bypass, 1 - cruise
unsigned long prevMicroTime = 0;
int tmp;

//buttons reading vars
int prevReadMode = 0;
unsigned long prevReadModeTime;

//double Kp=0.0005, Ki=0.05, Kd=0.001;
double Kp=0.001, Ki=0.05, Kd=0.001;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, REVERSE);

void setup() {
  Serial.begin(230400);
  pinMode(led, OUTPUT);
  pinMode(speedInput, INPUT);
  mode = 0;
  attachInterrupt(digitalPinToInterrupt(speedInput), measureSpeed, FALLING);
  myPID.SetSampleTime(10);
}

void measureSpeed(){
  for(int i=1; i<5; i++) { 
    speeds[i-1] = speeds[i]; 
  }
  
  currentSpeed = micros() - prevMicroTime;
  Input = (float) currentSpeed;
  speeds[4] = currentSpeed;
  prevMicroTime = micros();
  tmp++;
}


unsigned long debugPrevTime = 0;
void debugOut() {
    if (micros() - debugPrevTime < 500000) {
      return ;
    }
    debugPrevTime = micros();    
    Serial.print("cl:");
    Serial.print( clutchVal);
    Serial.print(" br:");
    Serial.print(brakeVal);
    Serial.print(" p:");
    Serial.print(pedal1Val);
    Serial.print("/");
    Serial.print(pedal2Val);
    Serial.print(" cruisePedal:");
    Serial.print(cruise1Val);
    Serial.print("/");
    Serial.print(cruise2Val);
    Serial.print(" m:");
    Serial.print(mode);
    Serial.print(" curSpd:");
    Serial.print(currentSpeed);
    Serial.print(" keepSpd:");
    Serial.print(keepSpeed);
    Serial.print(" lastStoredSpd:");
    Serial.print(lastStoredSpeed);
    Serial.print(" Input:");
    Serial.print(Input);
    Serial.print(" Output:");
    Serial.print(Output);
    Serial.print(" Setpoint:");
    Serial.print(Setpoint);
    Serial.println("");

}


void pedalRead() {
  pedal1Val = analogRead(pedal1) / 4;
  pedal2Val = analogRead(pedal2) / 4;
  if (pedal1Val < pedal1MinVal) {pedal1MinVal = pedal1Val;}
}

void pwmWrite() {
    float cruise2tmp;
    switch(mode) {
      case 1:
        cruise2tmp = float(cruise1Val) * float(pedal2Val+1) / float(pedal1Val+1); //+1 to avoid dividing by zero
        //cruise2tmp = cruise1Val * 250 / 125;    
        cruise2Val = (int) cruise2tmp;
        if (cruise1Val > pedal1Val && cruise2Val > pedal2Val) {
            analogWrite(PWM1, cruise1Val);  
            analogWrite(PWM2, cruise2Val);
          } else {
            analogWrite(PWM1, pedal1Val);  
            analogWrite(PWM2, pedal2Val);
          }
        break;
      case 0:
      default:
        analogWrite(PWM1, pedal1Val);  
        analogWrite(PWM2, pedal2Val);
        break;
    }
}

void querySwitches() {
    clutchVal = analogRead(clutch) / 4;
    brakeVal = analogRead(brake) / 4; 
    if (
      clutchVal > 50 || 
      brakeVal > 50 
    ) {
      mode = 0;
    }
}

void cruiseButtonsRead() {
    int val;
    int currentReadMode;
    unsigned long currentReadModeTime;
    unsigned long delta;
    bool longPress;
    bool tooShort;
    val = analogRead(button);
    currentReadModeTime = millis();
    currentReadMode = 0;
    if (val < 100) {
      currentReadMode = 1;
    } else if (val < 500) {
      //you can add your buttons here
    }

    longPress = (currentReadModeTime-prevReadModeTime>1300) ? true : false;
    tooShort = (currentReadModeTime-prevReadModeTime<100) ? true : false;
    
    if ((currentReadMode == 0 && prevReadMode == 1  || currentReadMode == 1 && longPress) && !tooShort) {
      if (longPress) {
        keepSpeed = lastStoredSpeed;
        //mode = 2;
        mode = 1;
        cruise1Val = pedal1Val;
      } else {
        if (mode > 0) {
          mode = 0;
        } else {
          lastStoredSpeed = speeds[4];
          keepSpeed = lastStoredSpeed;
          mode  = 1;
          cruise1Val = pedal1Val;
        }
      }
    }
    
    if (currentReadMode != prevReadMode  || currentReadMode == 0) {
      prevReadModeTime = currentReadModeTime;
    }
    
    prevReadMode = currentReadMode;
    if(mode > 0) {
      myPID.SetMode(AUTOMATIC);
      digitalWrite(led, HIGH);
    } else {
      myPID.SetMode(MANUAL);
      digitalWrite(led, LOW);
    }
} //ENDOF CruiseButtonsRead

void loop() {
  myPID.SetOutputLimits(pedal1MinVal, PEDAL1MAX);
  querySwitches();
  pedalRead();
  cruiseButtonsRead();
  Input = (float) currentSpeed;
  Setpoint = (float) keepSpeed;
  myPID.Compute();
  cruise1Val = (int) Output;
  pwmWrite();
  //debugOut();
}

