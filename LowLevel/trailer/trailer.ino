#include "trailer.h"

void setup()
{
  rosInit();
  resetSafetyTimer();
  Serial1.begin(9600); //connect to drill motor controller
  Serial2.begin(115200); //connect to Centrifuge
  Serial3.begin(115200);
  GPIOInit();
}

void loop()
{
  safetyCheck();
  stateRun();
  updateROSPubData();
  nh.spinOnce();
  delay(50);
}

void stateRun()
{
  switch(CONTROLSTATE)
  {
    case PAUSED:
    //Drill
    commandMotor(1,0);
    commandMotor(2,0);
    //arm
//    armProfile_idx = 0;
    //Centrifuge
    stopCentrifuge();
    //relays
    digitalWrite(VALVE1,HIGH);
    digitalWrite(VALVE2,HIGH);
    digitalWrite(SIGNALLIGHT,HIGH);
    digitalWrite(WASHPUMP,HIGH);
    //tower
    //index
    //LASTSTATE

    break;

    case MOVEOUTTOWER:
    //Drill
    commandMotor(1,0);
    commandMotor(2,0);
    //arm
    armProfile_idx = 0;
    //Centrifuge
    stopCentrifuge();
    //relays
    digitalWrite(VALVE1,HIGH);
    digitalWrite(VALVE2,HIGH);
    digitalWrite(SIGNALLIGHT,HIGH);
    digitalWrite(WASHPUMP,HIGH);
    //tower
    Serial3.print('o');
    delay(5);
    if (TOWERPLACE == OUT)
      CONTROLSTATE = DRILLING;
    //index
    //LASTSTATE
    LASTSTATE = MOVEOUTTOWER;
    break;

    case DRILLING:
    //Drill
    commandMotor(1,-VERTICALSPEED);
    commandMotor(2,DRILLSPEED);
    //arm
    armProfile_idx = 0;
    //Centrifuge
    Serial2.print(sample_idx);
    delay(5);
    // stopCentrifuge();
    //relays
    digitalWrite(VALVE1,HIGH);
    digitalWrite(VALVE2,HIGH);
    digitalWrite(SIGNALLIGHT,LOW);
    digitalWrite(WASHPUMP,HIGH);
    //tower
    //index
    Serial3.print(sample_idx);
    delay(5);

    //LASTSTATE
    LASTSTATE = DRILLING;
    break;

    case PULLING:
    //Drill
    commandMotor(1, 1);
    commandMotor(2, DRILLSPEED);
    //arm
    armProfile_idx = 0;
    //Centrifuge
    Serial2.print(sample_idx);
    delay(5);

    //relays
    digitalWrite(VALVE1,HIGH);
    digitalWrite(VALVE2,HIGH);
    digitalWrite(SIGNALLIGHT,LOW);
    digitalWrite(WASHPUMP,HIGH);
    //tower
    //index
    //LASTSTATE
    LASTSTATE = PULLING;
    break;

    case DRILLWAITING:
    //Drill
    commandMotor(1,0);
    commandMotor(2,0);
    //arm
    armProfile_idx = 0;
    //Centrifuge
    Serial2.print(sample_idx);
    delay(5);
    //relays
    digitalWrite(VALVE1,HIGH);
    digitalWrite(VALVE2,HIGH);
    digitalWrite(SIGNALLIGHT,HIGH);
    digitalWrite(WASHPUMP,HIGH);
    //tower
    Serial3.print('i');
    delay(5);
    //index
    //LASTSTATE
    if (TOWERPLACE == IN)
      CONTROLSTATE = RUNARMPROFILE1;
    LASTSTATE = DRILLWAITING;

    break;

    case RUNARMPROFILE1:
    //Drill
    commandMotor(1,0);
    commandMotor(2,0);
    //arm
    armProfile_idx = 1; 
    //Centrifuge
    Serial2.print(sample_idx);
    delay(5);

    //relays
    digitalWrite(VALVE1,HIGH);
    digitalWrite(VALVE2,HIGH);
    digitalWrite(SIGNALLIGHT,LOW);
    digitalWrite(WASHPUMP,HIGH);
    //tower
    //index
    //LASTSTATE
    if (armProfile_finished)
    {
      CONTROLSTATE = EXTRACTING;
      armProfile_idx = 0;
    }
    LASTSTATE = RUNARMPROFILE1;

    break;

    case EXTRACTING:
    //Drill
    commandMotor(1,-VERTICALSPEED);
    commandMotor(2,DRILLSPEED);
    //arm
    // armProfile_idx = 1;
    //Centrifuge
    // Serial2.print(sample_idx);
    //relays
    digitalWrite(VALVE1,LOW); //
    digitalWrite(VALVE2,HIGH);
    digitalWrite(SIGNALLIGHT,LOW);
    digitalWrite(WASHPUMP,HIGH);
    //tower
    //index
    //LASTSTATE
    LASTSTATE = EXTRACTING;
    //state change handled by limit switch
    break;


    case EXTRACTINGHOLD:
    //Drill
    commandMotor(1,0);
    commandMotor(2,DRILLSPEED);
    //arm
    // armProfile_idx = 1;
    //Centrifuge
    // Serial2.print(sample_idx);
    //relays
    if (millis() - extractingHoldEnterTime_water > EXTRACTINGWATERTIME)
      digitalWrite(VALVE1,HIGH); //
    else
      digitalWrite(VALVE1,LOW); //
    digitalWrite(VALVE2,HIGH);
    digitalWrite(SIGNALLIGHT,HIGH);
    digitalWrite(WASHPUMP,HIGH);
    //tower
    //index
    if (millis() - extractingHoldEnterTime > EXTRACTINGTIME)
      CONTROLSTATE = DRILLFINISHING;
    //LASTSTATE
    LASTSTATE = EXTRACTINGHOLD;
    break;

    case DRILLFINISHING:
    //Drill
    if (!drillAtTop)
      commandMotor(1,1);
    else
      commandMotor(1,0);
    commandMotor(2,0);
    //arm
    armProfile_idx = 2;
    //Centrifuge
    // Serial2.print(sample_idx);
    //relays
    digitalWrite(VALVE1,HIGH); //
    digitalWrite(VALVE2,HIGH);
    digitalWrite(SIGNALLIGHT,HIGH);
    digitalWrite(WASHPUMP,HIGH);
    //tower
    //index
    if (armProfile_finished)
    {
      CONTROLSTATE = MOVEOUTTOWERAGAIN;
      armProfile_idx = 0;
      centrifugeStartTime = millis();
      backwashEnterTime = millis();
    }
    //LASTSTATE
    LASTSTATE = DRILLFINISHING;
    break;

    case MOVEOUTTOWERAGAIN:
    //Drill
    commandMotor(1,0);
    commandMotor(2,0);
    //arm
    // armProfile_idx = 2;
    //Centrifuge
    Serial2.print('R');
    delay(5);

    //relays
    digitalWrite(VALVE1,HIGH); //
    if (TOWERPLACE = OUT)
    {
      if (millis() - backwashEnterTime < BACKWASHTIME)
      {
        digitalWrite(VALVE2,LOW);
      }
      else
      {
            digitalWrite(VALVE2,HIGH);
      }
    }
    else
    {
      digitalWrite(VALVE2,HIGH);
    }
    digitalWrite(SIGNALLIGHT,HIGH);
    digitalWrite(WASHPUMP,HIGH);
    //tower
    Serial3.print('o');
    delay(5);

    //index
    if (millis() - centrifugeStartTime > CENTRIFUGETIME)
    {
      CONTROLSTATE = STOPPINGCENTRIFUGE;
    }
    //LASTSTATE
    LASTSTATE = MOVEOUTTOWERAGAIN;
    break;

    case STOPPINGCENTRIFUGE:
    //Drill
    commandMotor(1,0);
    commandMotor(2,0);
    //arm
    // armProfile_idx = 2;
    //Centrifuge
    Serial2.print(sample_idx);
    delay(5);

    //relays
    digitalWrite(VALVE1,HIGH); //
    digitalWrite(VALVE2,HIGH);
    digitalWrite(SIGNALLIGHT,HIGH);
    digitalWrite(WASHPUMP,HIGH);
    //tower
    Serial3.print('i');
    delay(5);
    //index
    ////////////////////////////////////////////////////////////////
    //ask for current speed and advance to next stage
    //tower in place or not
    //CONTROLSTATE = RUNARMPROFILE3;

    if (TOWERPLACE == IN)
    {
      CONTROLSTATE = RUNARMPROFILE3;
      smallPumpStartTime = millis();
    }
    //LASTSTATE
    
    LASTSTATE = STOPPINGCENTRIFUGE;
    break;

    case RUNARMPROFILE3:
    //Drill
    commandMotor(1,0);
    commandMotor(2,0);
    //arm
    armProfile_idx = 3;
    //Centrifuge
    // Serial2.print('R');
    //relays
    digitalWrite(VALVE1,HIGH); //
    digitalWrite(VALVE2,HIGH);
    digitalWrite(SIGNALLIGHT,HIGH);
    if (millis()-smallPumpStartTime > SMALLPUMPTIME)
    {
      digitalWrite(WASHPUMP, HIGH);
    }
    else
    {
      digitalWrite(WASHPUMP,LOW);
    }
    //tower
    // Serial3.print('o');
    //index
    if (armProfile_finished)
    {
      armProfile_idx = 0;
      CONTROLSTATE = WAITING; 
    }

    //LASTSTATE
    LASTSTATE = RUNARMPROFILE3;
    break;


    case WAITING:
    //Drill
    commandMotor(1,0);
    commandMotor(2,0);
    //arm
    armProfile_idx = 0;
    //Centrifuge
    // Serial2.print('R');
    //relays
    digitalWrite(VALVE1,HIGH); //
    digitalWrite(VALVE2,HIGH);
    digitalWrite(SIGNALLIGHT,HIGH);
    digitalWrite(WASHPUMP,HIGH);
    //tower
    // Serial3.print('o');
    //index
    processBegin = false;
//    extractingHoldEnterTime = 0;
//    armProfile_finished = false;
//    drillAtTop = false;
//    extractingHoldEnterTime_water = 0;
//    centrifugeStartTime = 0;
//    smallPumpStartTime = 0;
    //LASTSTATE
    LASTSTATE = WAITING;
    break;    
  }
}

void serialEvent3()
{
  nh.loginfo("serial3Received");
  if (Serial3.available())
  {
    char inChar = (char)Serial3.read();
    if (inChar == 'o')
    {
//      nh.loginfo("serial3Received_OUT");

      TOWERPLACE = OUT;
    }
    else if (inChar == 'i')
    {
//      nh.loginfo("serial3Received_IN");

      TOWERPLACE = IN;
    }
  }
}
void trailerCommand_cb( const std_msgs::Char& data)
{
  char inChar = (char)data.data;
  if (inChar == 'a')
  {
    if (CONTROLSTATE == PAUSED)
    {
      CONTROLSTATE = LASTSTATE;
    }
    resetSafetyTimer();
  }
}
void armState_cb(const std_msgs::Bool& data)
{
  armProfile_finished = data.data;
}
void trailerIDX_cb(const std_msgs::Byte& data)
{
  uint8_t idx = (uint8_t)data.data;
  if (idx != sample_idx)
  {
    if(!processBegin)
    {
      processBegin = true;
      CONTROLSTATE = MOVEOUTTOWER;
      sample_idx = idx;
    }
  }
}
void downLimitISR()
{
  CONTROLSTATE = PULLING;
}
void upLimitISR()
{
  if (CONTROLSTATE == PULLING)
  {
    CONTROLSTATE = DRILLWAITING;
  }
  else if (CONTROLSTATE == DRILLFINISHING)
  {
    drillAtTop = true;
  }
}

void midLimitISR()
{
  if (CONTROLSTATE == EXTRACTING)
  {
    CONTROLSTATE = EXTRACTINGHOLD;
    extractingHoldEnterTime = millis();
    extractingHoldEnterTime_water = millis();
  }
}
