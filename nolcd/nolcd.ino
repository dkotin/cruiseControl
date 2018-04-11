#include <PID_v1.h>

#define speedInput 2

#define brake A4
#define clutch A5
#define led 13

#define PWM1 9
#define PWM2 11

#define pedal1 A0
#define pedal2 A1
#define button A3

//sensors values
int pedal1Val;
int pedal1MinVal = 50;
int pedal1MaxVal = 200;
int pedal2Val;
int cruise1Val;
int cruise2Val;

int deviceButtonVal;
byte clutchVal;
byte brakeVal;

//speeds
unsigned long keepSpeed = 0;
unsigned long lastStoredSpeed = 0;

volatile unsigned long currentSpeed = 0;
volatile unsigned long prevMicroTime = 0;

double Setpoint, Input, Output;

byte mode; //0 - bypass, 1 - cruise

//buttons reading vars
int prevReadMode = 0;
unsigned long prevReadModeTime;

//manual speed measuring
volatile int prevSpeedState = 0;
volatile unsigned long prevSpeedStateTime;
volatile unsigned long manuallyReadSpeed;

//double Kp=0.0005, Ki=0.05, Kd=0.001;
// double Kp=0.001, Ki=0.04, Kd=0.001; // -> these worked with interrupt-based speed reading. now too slow reaction
// double Kp=0.002, Ki=0.1, Kd=0.002; //-> slow reaction, about 1s before speed changing
// double Kp=0.001, Ki=0.04, Kd=0.004; // -> too fast reaction, keeps speed but operates the pedal too fast
// double Kp=0.001, Ki=0.04, Kd=0.003; // -> too much speed fluctuating, faster pedal reaction <-- driveable
// double Kp=0.001, Ki=0.07, Kd=0.003; // -> no checkengine finally, too fast and too fluctuating (t = 15)
//double Kp=0.001, Ki=0.03, Kd=0.004; // -> no checkengine finally, not bad  30 ms
//double Kp=0.025, Ki=0.03, Kd=0.004; // -> smooth 200 time
//double Kp=0.05, Ki=0, Kd=0;
//PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, REVERSE);

int timeslice=200;
double Tu=1.5;
double Ku = 0.04;

//Ziegler-Nichols method modified
PID myPID(
  &Input, 
  &Output, 
  &Setpoint, 
  0.6 * Ku,
  Ku * 0.6 * 2 / Tu,
  Ku * Tu / 70,
  REVERSE
  );

void setup() {
  analogWrite(PWM1, 36);  
  analogWrite(PWM2, 18);

  pedalRead();
  mode = 0;
  pwmWrite();
  pinMode(speedInput, INPUT_PULLUP); //INPUT or INPUT_PULLUP
  pinMode(led, OUTPUT);
  digitalWrite(led, HIGH);
//  attachInterrupt(digitalPinToInterrupt(speedInput), measureSpeed, FALLING);
  myPID.SetSampleTime(timeslice); 
  
  Serial.begin(230400);
}

void measureSpeed(){
  currentSpeed = micros() - prevMicroTime;
  prevMicroTime = micros();
}


unsigned long debugPrevTime = 0;
void debugOut() {
    if (micros() - debugPrevTime < 300000) {
      return ;
    }
    debugPrevTime = micros();  
    //Serial.println(manuallyReadSpeed);
}


void pedalRead() {
  pedal1Val = analogRead(pedal1) / 4;
  pedal2Val = analogRead(pedal2) / 4;
  if (pedal1Val < pedal1MinVal) {pedal1MinVal = pedal1Val;}
  if (pedal1Val > pedal1MaxVal) {pedal1MaxVal = pedal1Val;}
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
            Output = (double) pedal1Val;
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

void manualSpeedRead() {
  int currentState;
  currentState = digitalRead(speedInput);
  if (prevSpeedState == 0 && currentState == 1) {
    manuallyReadSpeed = micros() - prevSpeedStateTime;
    prevSpeedStateTime = micros();
  }
  prevSpeedState = currentState;
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
    tooShort = (currentReadModeTime-prevReadModeTime<70) ? true : false;
    
    if ((currentReadMode == 0 && prevReadMode == 1  || currentReadMode == 1 && longPress) && !tooShort) {
      if (longPress) {
        keepSpeed = lastStoredSpeed;
        mode = 1;
        cruise1Val = pedal1Val;
        Output = (double) pedal1Val;
      } else {
        if (mode > 0) {
          mode = 0;
        } else {
          lastStoredSpeed = currentSpeed;
          keepSpeed = lastStoredSpeed;
          mode  = 1;
          cruise1Val = pedal1Val;
          Output = (double) pedal1Val;
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
  myPID.SetOutputLimits(pedal1MinVal, pedal1MaxVal);
  manualSpeedRead();
  currentSpeed = manuallyReadSpeed;
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

