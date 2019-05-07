#define VERTICALSPEED     1     //vertical motor speed
#define DRILLSPEED        1     //Drill motor speed
#define SAFETYPERIOD      100   //period before stop
#define LIMITUP           20    //upper limit switch pin active low
#define LIMITDOWN         2    //lower limit switch pin active low
#define LIMITMID          3    //middle limit switch pin active low
#define EXTRACTIONTIME    3000
#define VALVE1            7
#define VALVE2            6
#define SIGNALLIGHT       5
#define WASHPUMP          4
long safetyTimer = 0;
int idx = 0;
long extractingHoldEnterTime = 0;
int limitTest = 0;
enum controlState{
  DRILLING,
  PULLING,
  EXTRACTING,
  EXTRACTINGHOLD,
  PAUSED,
  DRILLPENDING,
  DRILLFINISHING,
  MOVINGFILTRATION,
};

controlState DRILLSTATE = PAUSED;
controlState LASTDRILLSTATE;

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  Serial2.begin(115200);
  Serial3.begin(115200);
  commandMotor(1,0);
  commandMotor(2,0);
  resetSafetyTimer();
  TCCR4B &= ~0x7;
  TCCR4B |= 2;
  pinMode(LIMITUP, INPUT_PULLUP);
  pinMode(LIMITDOWN, INPUT_PULLUP);
  pinMode(LIMITMID, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LIMITUP), upLimitISR, FALLING); 
  attachInterrupt(digitalPinToInterrupt(LIMITDOWN), downLimitISR, FALLING); 
  attachInterrupt(digitalPinToInterrupt(LIMITMID), midLimitISR, FALLING); 
  LASTDRILLSTATE = DRILLING;
  pinMode(VALVE1, OUTPUT);
  digitalWrite(VALVE1,HIGH);
  pinMode(VALVE2, OUTPUT);
  digitalWrite(VALVE2,HIGH);
  pinMode(SIGNALLIGHT,OUTPUT);
  digitalWrite(SIGNALLIGHT,HIGH);
  pinMode(WASHPUMP, OUTPUT);
  digitalWrite(WASHPUMP, HIGH);
}

void loop() {
//  Serial.println(limitTest);/
  safetyCheck();
  switch(DRILLSTATE)
  {
    case PAUSED:
      commandMotor(1,0);
      commandMotor(2,0);
      Serial.println("PAUSED");
      break;
    case DRILLING:
      commandMotor(1,-VERTICALSPEED);
      commandMotor(2,DRILLSPEED);  
      LASTDRILLSTATE = DRILLING;
      digitalWrite(SIGNALLIGHT,LOW);
      break;
    case PULLING:
      commandMotor(1, VERTICALSPEED);
      commandMotor(2, DRILLSPEED);
      LASTDRILLSTATE = PULLING;
      digitalWrite(SIGNALLIGHT,LOW);
      break;
    case EXTRACTING:
      commandMotor(1, -VERTICALSPEED);
      commandMotor(2, DRILLSPEED);
      LASTDRILLSTATE = EXTRACTING;
      break;
    case EXTRACTINGHOLD:
      commandMotor(1,0);
      commandMotor(2, DRILLSPEED);
      if (millis() - extractingHoldEnterTime > EXTRACTIONTIME)
      {
        DRILLSTATE = DRILLFINISHING;
      }
      LASTDRILLSTATE = EXTRACTINGHOLD;
      break;
    case DRILLPENDING:
      commandMotor(1,0);
      commandMotor(2,0);
      LASTDRILLSTATE = DRILLPENDING;
      digitalWrite(SIGNALLIGHT,HIGH);
      break;
    case MOVINGFILTRATION:
      commandMotor(1,0);
      commandMotor(2,0);
      //send command to dynamixel controller to move the filtration inplace
      //if feedback says filtration tower inplace
      //DRILLSTATE = EXTRACTING;
      //place holder: wait for 10000 iterations second
      idx ++;
      if (idx == 1000)
      {
        DRILLSTATE = EXTRACTING;
        idx = 0;
      }
      LASTDRILLSTATE = MOVINGFILTRATION;
      break;
    case DRILLFINISHING:
      commandMotor(1, VERTICALSPEED);
      commandMotor(2, DRILLSPEED);
      LASTDRILLSTATE = DRILLFINISHING;
      break;
  }
//  reAttachLimitSwitchOnConditions();/
}

//between -1 and 1, zero is stop
//motor ID: 1 for vertical and 2 for drill
void commandMotor(int id, float speedFrac)
{
  int motorSpeed = speedFrac * 2047;
  if (id == 1)
  {
    Serial1.print("M1: ");
    Serial1.println(motorSpeed);
  }
  if (id == 2)
  {
    Serial1.print("M2: ");
    Serial1.println(motorSpeed);
  }
}

void serialEvent()
{
  if(Serial.available())
  {
    char inChar = (char)Serial.read();
    Serial.write(inChar); //echo
    
    if (inChar == 'd')
    {
      DRILLSTATE = DRILLING;
      resetSafetyTimer();
    }
    else if (inChar == 'p')
    {
      DRILLSTATE = PAUSED;
      resetSafetyTimer();
    }
    else if (inChar == 'u')
    {
      DRILLSTATE = PULLING;
      resetSafetyTimer();
    }
    else if (inChar == 'h')
    {
      DRILLSTATE = EXTRACTINGHOLD;
      extractingHoldEnterTime = millis();
      resetSafetyTimer();
    }
    else if (inChar == 'a')
    {
      if (DRILLSTATE == PAUSED)
      {
        DRILLSTATE = LASTDRILLSTATE;
      }
      Serial.println(DRILLSTATE);
      resetSafetyTimer();
    }
    else if (inChar == '1')
    {
      Serial2.print('1');
      Serial3.print('1');
    }
    else if (inChar == '2')
    {
      Serial2.print('2');
      Serial3.print('2');
    }
    else if (inChar == '3')
    {
      Serial2.print('3');
      Serial3.print('3');
    }
    else if (inChar == '4')
    {
      Serial2.print('4');
      Serial3.print('4');
    }
    else if (inChar == '5')
    {
      Serial2.print('5');
      Serial3.print('5');
    }
    else if (inChar == '6')
    {
      Serial2.print('6');
      Serial3.print('6');
    }
    else if (inChar == 'r')
    {
      Serial2.print('R');
    }
    else if (inChar == 'p')
    {
      Serial2.print('P');
    }
    else if (inChar == 'i')
    {
      Serial3.print('i');
    }
    else if (inChar == 'o')
    {
      Serial3.print('o');
    }
    else if (inChar == '[')
    {
      digitalWrite(VALVE1, LOW);
    }
    else if (inChar == ']')
    {
      digitalWrite(VALVE1, HIGH);
    }
    else if (inChar == '{')
    {
      digitalWrite(VALVE2, LOW);
    }
    else if (inChar == '}')
    {
      digitalWrite(VALVE2, HIGH);
    }
  }
}
void safetyCheck()
{
  int timeLaps =  millis() - safetyTimer;
  if(timeLaps > SAFETYPERIOD)
  {
    DRILLSTATE = PAUSED;
  }
}
void resetSafetyTimer()
{
  safetyTimer = millis();
}

void upLimitISR()
{
  if (DRILLSTATE == PULLING)
  {
    DRILLSTATE = MOVINGFILTRATION;
  }
  else if (DRILLSTATE == DRILLFINISHING)
  {
    DRILLSTATE = DRILLPENDING;
  }
  limitTest = 1;
//  detachInterrupt(digitalPinToInterrupt(LIMITUP));/
}

void downLimitISR()
{
  DRILLSTATE = PULLING;
  limitTest = 3;
//  detachInterrupt(digitalPinToInterrupt(LIMITDOWN));/
}

void midLimitISR()
{
  if (DRILLSTATE == EXTRACTING)
  {
    DRILLSTATE = EXTRACTINGHOLD;
    extractingHoldEnterTime = millis();
  }
  limitTest = 2;
//  detachInterrupt(digitalPinToInterrupt(LIMITMID));/
}

///
