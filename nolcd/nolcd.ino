 #include <PID_v1.h>

#define speedInput 2

#define led 13

#define PWM1 9
#define PWM2 11

#define pedal1 A0
#define pedal2 A1
#define button A3
#define brake A4
#define clutch A5

#define correction1 26
#define correction2 25

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
volatile unsigned long manuallyReadSpeed;

int timeslice = 500;
double Ku = 25; //
double Tu = 900; //1.5

//before soldering: 45 750 - fast and fluctuating, 
//55 600 - fast and very agressive
//35 900 - smoother then above
// 500 / 25 / 900 works perfectly

int speedReadPeriod = 500;
volatile unsigned long prevReadTime = 0;
volatile unsigned long curReadTime = 0;
volatile unsigned long speedImps = 0;

volatile float pedal21coefficient = 250/125;

//Ziegler-Nichols method modified
PID myPID(
  &Input,
  &Output,
  &Setpoint,                                       //   1.3 1.3      4.5 20    45 200   90 400         45 600
  0.25 * Ku,               //proportional               0.325        1.125     smooth   slow           best
  0.001 * Tu         ,      //integral 0.001 .. 0.01    0.0013       0.02               fluctuations   so far
  0, 125 * Tu,            //differential                0.1625       2.5                               increase tu mb
  DIRECT
);
/*
// http://www.mstarlabs.com/control/znrule.html
// try diff pi / pd = 1 / 100 and pp = 1/5pd

worked acceptable with:
  0.6 * Ku,               //proportional             0.78
  0.6 * Ku * 3 / Tu,      //integral 0.6 * * 2 /    1.17
  Ku * Tu / 70,           //differential    0.037

double Tu = 2; //1.5
double Ku = 1.3; // 1
overshot means reduce proportional and reduce integral. laziness means increase both. dif should be 2.5 prop.
*/

void setup() {
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  
  // PWM setup https://alexgyver.ru/lessons/pwm-overclock/

  // pins D9 and D10 - 62.5 kHz
  // TCCR1A = 0b00000001;  // 8bit
  // TCCR1B = 0b00001001;  // x1 fast pwm

  // pins D3 and D11 - 62.5 kHz
  // TCCR2B = 0b00000001;  // x1
  // TCCR2A = 0b00000011;  // fast pwm

  // pins D9 and D10 - 31.4 kHz
  TCCR1A = 0b00000001;  // 8bit
  TCCR1B = 0b00000001;  // x1 phase correct

  // pins D3 and D11 - 31.4 kHz
  TCCR2B = 0b00000001;  // x1
  TCCR2A = 0b00000001;  // phase correct 
  
  pedalWrite(PWM1, 36); //36
  pedalWrite(PWM2, 18); //18

  mode = 0;
  //pedalRead();
  //pwmWrite();
  pinMode(speedInput, INPUT_PULLUP); //INPUT or INPUT_PULLUP
  pinMode(led, OUTPUT);
  digitalWrite(led, HIGH);
  //attachInterrupt(digitalPinToInterrupt(speedInput), measureSpeed, FALLING);
  myPID.SetSampleTime(timeslice);

  Serial.begin(230400);
}

void measureSpeed() {
  currentSpeed = micros() - prevMicroTime;
  prevMicroTime = micros();
}

unsigned long debugPrevTime = 0;
void debugOut() {
  if (micros() - debugPrevTime < 1000000) {
    return ;
  }
  debugPrevTime = micros();
  Serial.println("p" + String(pedal1Val) + "/" + String(pedal2Val) + " " + String(cruise1Val) + "/" + String(cruise2Val) + "         ");
  Serial.println("m" + String(mode) + " cs" + String(currentSpeed) + " r" + String(pedal1MinVal) + "/" + String(pedal1MaxVal) + "         ");
}

void pedalRead() {
  pedal1Val = analogRead(pedal1) / 4;
  pedal2Val = analogRead(pedal2) / 4;
  if (pedal1Val < pedal1MinVal && pedal1Val > 3) {
    pedal1MinVal = pedal1Val;
  }
  if (pedal1Val > pedal1MaxVal) {
    pedal1MaxVal = pedal1Val;
  }
}

void pwmWrite() {
  float cruise2tmp, pedal2tmp;
  //pedal2coefficient = float(cruise1Val) * float(pedal2Val + 1) / float(pedal1Val + 1); //+1 to avoid dividing by zero
  pedal2tmp = float(pedal1Val * pedal21coefficient);
  cruise2tmp = cruise1Val * pedal21coefficient;
  switch (mode) {
    case 1:
      cruise2Val = (int) cruise2tmp;
      if (cruise1Val > pedal1Val) {  //&& cruise2Val > pedal2Val
        pedalWrite(PWM1, cruise1Val);
        pedalWrite(PWM2, cruise2Val);
      } else {
        pedalWrite(PWM1, pedal1Val);
        pedalWrite(PWM2, (int) pedal2tmp); //pedal2Val
        Output = (double) pedal1Val;
      }
      break;
    case 0:
    default:
      pedalWrite(PWM1, pedal1Val);
      pedalWrite(PWM2, (int) pedal2tmp); //pedal2Val
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
    curReadTime = millis();
    speedImps ++;
    if (curReadTime - prevReadTime > speedReadPeriod) {
      prevReadTime = curReadTime;
      manuallyReadSpeed = speedImps;
      speedImps = 0;
    }
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

  longPress = (currentReadModeTime - prevReadModeTime > 1000) ? true : false;
  tooShort = (currentReadModeTime - prevReadModeTime < 50) ? true : false;

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
  if (mode > 0) {
    myPID.SetMode(AUTOMATIC);
    digitalWrite(led, HIGH);
  } else {
    myPID.SetMode(MANUAL);
    digitalWrite(led, LOW);
  }
} //ENDOF CruiseButtonsRead

void pedalWrite(byte output, int val) {
  byte correction;
  switch (output) {
    case PWM1:
      correction = correction1;
      break;
    case PWM2:
      correction = correction2;
      break;
  }
  //if (val < 3) correction = 0;  //ecu not yet powered up? no corrections. output zero volts.
  
  if (val + correction > 255) {
    analogWrite(output, 255);
  } else {
    analogWrite(output, val + correction);
  }
}


void loop() {
  //myPID.SetOutputLimits( (pedal1MinVal, pedal1MaxVal);
  myPID.SetOutputLimits( (pedal1MinVal - correction1) > 0 ? pedal1MinVal - correction1 : 1, pedal1MaxVal - correction1);
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
  debugOut();
}
