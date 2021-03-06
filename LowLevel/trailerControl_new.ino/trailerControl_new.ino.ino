#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Char.h>

#define LIMITUP           20    //upper limit switch pin active low
#define LIMITDOWN         2    //lower limit switch pin active low
#define LIMITMID          3    //middle limit switch pin active low
#define VALVE1            7
#define VALVE2            6
#define SIGNALLIGHT       5
#define WASHPUMP          4
#define FILTRATIONIN      303
#define FILTRATIONOUT     703
#define FILTRATIONUP      643
#define FILTRATIONTILT    309
#define ARMPROFILE1STEPS  100
#define ARMPROFILE2STEPS  100
#define ARMPROFILE3STEPS  100
#define VERTICALSPEED     0.5     //vertical motor speed
#define DRILLSPEED        1     //Drill motor speed
#define SAFETYPERIOD      100   //period before stop

ros::NodeHandle nh;
std_msgs::UInt16 armProfile_idx_ros;
std_msgs::UInt16MultiArray dxlGoalPos_ros;
std_msgs::UInt8 controlState_ros;

std_msgs::UInt16MultiArray openCM_fb;
byte armProfile_idx;
volatile uint16_t armStep = 0;
long safetyTimer = 0;
long extractingHoldEnterTime = 0;
int sampel_idx = 0;
uint8_t storagePos[6] = {46, 220, 395, 570, 744, 919};
enum controlStates
{
  PAUSED,
  DRILLING,
  PULLING,
  DRILLWAITING,
  RUNARMPROFILE1,
  EXTRACTING,
  EXTRACTINGHOLD,
  DRILLPENDING,
  DRILLFINISHING,
};
controlStates CONTROLSTATE = PAUSED;
controlStates LASTSTATE = DRILLING;

void setup() {
  ros::Publisher pub_dxlGoalPos("/dxlGoalPos", &dxlGoalPos_ros);
  ros::Publisher pub_armProfile_idx("/armProfile_idx", &armProfile_idx_ros);
  ros::Publisher pub_controlState("/controlState", &controlState_ros);
  ros::Subscriber<std_msgs::UInt16MultiArray> curPos_sub("/dxlCurPos_All", &dxlCurPos_cb );
  ros::Subscriber<std_msgs::Char> command_sub("/trailerCommand", &trailerCommand_cb);
  ros::Subscriber<std_msgs::UInt16> armStep_sub("/armStep", &armStep_cb);
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  dxlGoalPos_ros.data = (uint16_t *)malloc(sizeof(uint16_t)*4);
  dxlGoalPos_ros.data_length = 4; 
  openCM_fb.data = (uint16_t *)malloc(sizeof(uint16_t)*10);
  openCM_fb.data_length = 10;
  nh.advertise(pub_dxlGoalPos);
  nh.advertise(pub_armProfile_idx);
  nh.advertise(pub_controlState);
  nh.subscribe(curPos_sub);
  nh.subscribe(command_sub);
  nh.subscribe(armStep_sub);
  resetSafetyTimer();
  Serial1.begin(9600); //connect to drill motor controller
  Serial2.begin(115200); //connect to Centrifuge
  
  GPIOInit();

  while(1)
  {
    safetyCheck();
    stateRun();
    updateROSPubData();
    pub_dxlGoalPos.publish(&dxlGoalPos_ros);
    pub_armProfile_idx.publish(&armProfile_idx_ros);
    pub_controlState.publish(&controlState_ros);
    nh.spinOnce();
    delay(50);
  }
  
}

void resetSafetyTimer()
{
  safetyTimer = millis();
}
void safetyCheck()
{
  int timeLaps =  millis() - safetyTimer;
  if(timeLaps > SAFETYPERIOD)
  {
    CONTROLSTATE = PAUSED;
  }
}
void GPIOInit()
{
  pinMode(LIMITUP, INPUT_PULLUP);
  pinMode(LIMITDOWN, INPUT_PULLUP);
  pinMode(LIMITMID, INPUT_PULLUP);
  pinMode(VALVE1, OUTPUT);
  digitalWrite(VALVE1,HIGH);
  pinMode(VALVE2, OUTPUT);
  digitalWrite(VALVE2,HIGH);
  pinMode(SIGNALLIGHT,OUTPUT);
  digitalWrite(SIGNALLIGHT,HIGH);
  pinMode(WASHPUMP, OUTPUT);
  digitalWrite(WASHPUMP, HIGH);
  attachInterrupt(digitalPinToInterrupt(LIMITUP), upLimitISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(LIMITDOWN), downLimitISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(LIMITMID), midLimitISR, FALLING);
}
void loop() {}
void stateRun()
{
  switch(CONTROLSTATE)
  {
    case PAUSED:
    commandMotor(1, 0);
    commandMotor(2, 0);
    stopCentrifuge();
    stopDxl();
    break;
    case DRILLING:
      if(openCM_fb.data[2] > (FILTRATIONOUT+10) || openCM_fb.data[2] < (FILTRATIONOUT-10))
      {
        dxlGoalPos_ros.data[2] = FILTRATIONOUT;
        dxlGoalPos_ros.data[3] = FILTRATIONUP;
      }
      else
      {
        commandMotor(1,-VERTICALSPEED);
        commandMotor(2,DRILLSPEED);
        digitalWrite(SIGNALLIGHT,LOW);
      }
      dxlGoalPos_ros.data[0] = storagePos[sampel_idx];
      dxlGoalPos_ros.data[1] = storagePos[sampel_idx];
    LASTSTATE = DRILLING;
    break;
    case PULLING:
      commandMotor(1, VERTICALSPEED);
      commandMotor(2, DRILLSPEED);
      LASTSTATE = PULLING;
      digitalWrite(SIGNALLIGHT,LOW);
    break;
    case DRILLWAITING:
      commandMotor(1, 0);
      commandMotor(2, 0);
      if(openCM_fb.data[2] > (FILTRATIONIN+5) || openCM_fb.data[2] < (FILTRATIONIN-5))
      {
        dxlGoalPos_ros.data[2] = FILTRATIONIN;
        dxlGoalPos_ros.data[3] = FILTRATIONUP;
      }
      else
      {
        CONTROLSTATE = RUNARMPROFILE1;
      }
      LASTSTATE = DRILLWAITING;
      digitalWrite(SIGNALLIGHT,HIGH);
    break;
    case RUNARMPROFILE1:
      armProfile_idx = 1;
      if (armStep == ARMPROFILE1STEPS)
      {
        CONTROLSTATE = EXTRACTING;
      }
      LASTSTATE = RUNARMPROFILE1;
      break;
    case EXTRACTING:
      commandMotor(1, -VERTICALSPEED);
      commandMotor(2, DRILLSPEED);
      digitalWrite(VALVE1,LOW);//start valve 1
      LASTSTATE = EXTRACTING;
      digitalWrite(SIGNALLIGHT,LOW);
    break;
    case DRILLPENDING:
    break;
  }

}
void dxlCurPos_cb( const std_msgs::UInt16MultiArray& data)
{
//  for (int i = 0; i < 10; i++)
//  {
//    openCM_fb.data[i] = data.data[i];
//  }
    openCM_fb = data;
}
void armStep_cb( const std_msgs::UInt16& data)
{
  armStep = (uint16_t)data.data;
}

void updateROSPubData()
{
  armProfile_idx_ros.data = armProfile_idx;
  controlState_ros.data = CONTROLSTATE;
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
void stopCentrifuge()
{
  Serial2.print('P');
}
void stopDxl()
{
  armProfile_idx = 0;
  for (int i = 0; i < 4; i++)
  {
    dxlGoalPos_ros.data[i] = 0;
  }
}
