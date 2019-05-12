#include "trailer.h"

void setup()
{
  rosInit();
  resetSafetyTimer();
  Serial1.begin(9600); //connect to drill motor controller
  Serial2.begin(115200); //connect to Centrifuge
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
    armProfile_idx = 0;
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
    // stopCentrifuge();
    //relays
    digitalWrite(VALVE1,HIGH);
    digitalWrite(VALVE2,HIGH);
    digitalWrite(SIGNALLIGHT,LOW);
    digitalWrite(WASHPUMP,HIGH);
    //tower
    //index
    Serail3.print(sample_idx)
    //LASTSTATE
    LASTSTATE = DRILLING;
    break;

    case PULLING:
    //Drill
    commandMotor(1, VERTICALSPEED);
    commandMotor(2, DRILLSPEED);
    //arm
    armProfile_idx = 0;
    //Centrifuge
    Serial2.print(sample_idx);
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
    //relays
    digitalWrite(VALVE1,HIGH);
    digitalWrite(VALVE2,HIGH);
    digitalWrite(SIGNALLIGHT,HIGH);
    digitalWrite(WASHPUMP,HIGH);
    //tower
    Serial3.print('i');
    //index
    //LASTSTATE
    LASTSTATE = DRILLWAITING;
    if (TOWERPLACE = IN)
      CONTROLSTATE = RUNARMPROFILE1;
    break;

    case RUNARMPROFILE1:
    //Drill
    commandMotor(1,0);
    commandMotor(2,0);
    //arm
    armProfile_idx = 1; //bug here!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! communication delay, need to change on high level
    //Centrifuge
    Serial2.print(sample_idx);
    //relays
    digitalWrite(VALVE1,HIGH);
    digitalWrite(VALVE2,HIGH);
    digitalWrite(SIGNALLIGHT,HIGH);
    digitalWrite(WASHPUMP,HIGH);
    //tower
    //index
    //LASTSTATE
    LASTSTATE = RUNARMPROFILE1;
    if (armProfile_finished)
      CONTROLSTATE = EXTRACTING;
    break;

    case EXTRACTING:
    //Drill
    commandMotor(1,0);
    commandMotor(2,0);
    //arm
    armProfile_idx = 1;
    //Centrifuge
    Serial2.print(sample_idx);
    //relays
    digitalWrite(VALVE1,HIGH);
    digitalWrite(VALVE2,HIGH);
    digitalWrite(SIGNALLIGHT,HIGH);
    digitalWrite(WASHPUMP,HIGH);
    //tower
    //index
    //LASTSTATE
    LASTSTATE = RUNARMPROFILE1;
    if (armProfile_finished)
      CONTROLSTATE = EXTRACTING;
    break;
  }
}

void serialEvent3()
{
  if Serial3.avaliable()
  {
    char inChar = (char)Serial3.read();
    if (inChar == 'o')
    {
      TOWERPLACE = OUT;
    }
    else if (inChar == 'i')
    {
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
    CONTROLSTATE = DRILLPENDING;
  }
}

void midLimitISR()
{
  if (CONTROLSTATE == EXTRACTING)
  {
    CONTROLSTATE = EXTRACTINGHOLD;
    extractingHoldEnterTime = millis();
  }
}
