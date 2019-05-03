#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Char.h>
// rosrun rosserial_python serial_node.py /dev/ttyACM0 _baud:=115200

#define VERTICALSPEED     1     //vertical motor speed
#define DRILLSPEED        1     //Drill motor speed
#define SAFETYPERIOD      100   //period before stop
#define LIMITUP           20    //upper limit switch pin active low
#define LIMITDOWN         2    //lower limit switch pin active low
#define LIMITMID          3    //middle limit switch pin active low
#define VALVE1            7
#define VALVE2            6
#define SIGNALLIGHT       5
#define WASHPUMP          4
#define EXTRACTIONTIME    3000
#define CENTRIFUGERUNINGTIME 5000
#define FILTRATIONIN      303
#define FILTRATIONOUT     703
#define FILTRATIONUP      643
#define FILTRATIONTILT    309

////////////////////////////////////Prototype Functions///////////////////////

void dxlCurPos_cb( const std_msgs::UInt16MultiArray& data);
void trailerCommand_cb( const std_msgs::Char& data);
void updateROSPubData();
/////////////////////////////////////////////////////////////////////////////

//////////////////////////////////Global Variables////////////////////////////
long safetyTimer = 0;
int idx = 0;
int sample_idx = 0;
long extractingHoldEnterTime = 0;
long centrifugeStartTime = 0;
uint8_t centrifugeSpeed = 0;
//unsigned int armGoalPos[6] = {0,0,0,0,0,0};
//unsigned int armCurPos[6] = {0,0,0,0,0,0};
//unsigned int armStep_idx = 0;
unsigned int armProfile_idx = 0;
unsigned int openCM_cmd[10] = {0,0,0,0,0,0,0,0,0,0};
unsigned int openCM_fb[10] = {0,0,0,0,0,0,0,0,0,0};


//////////////////////////////////ROS Variables///////////////////////////////

ros::NodeHandle nh;
std_msgs::UInt16 armProfile_idx_ros;
std_msgs::UInt16MultiArray dxlGoalPos_ros;
std_msgs::UInt8 drillState_ros;
std_msgs::UInt8 servoState_ros;




ros::Publisher pub_dxlGoalPos("/dxlGoalPos", &dxlGoalPos_ros);
ros::Publisher pub_armProfile_idx("/armProfile_idx", &armProfile_idx_ros);
//ros::Publisher pub_armStep_idx("/armStep_idx", &armStep_idx_ros);
ros::Publisher pub_drillState("/drillState", &drillState_ros);
ros::Publisher pub_servoState("/servoState", &servoState_ros);
ros::Subscriber<std_msgs::UInt16MultiArray> curPos_sub("/dxlCurPos_All", &dxlCurPos_cb );
ros::Subscriber<std_msgs::Char> command_sub("/trailerCommand", &trailerCommand_cb);

enum drillState{
  DRILLPAUSED,
  DRILLING,
  PULLING,
  EXTRACTING,
  EXTRACTINGHOLD,
  DRILLPENDING,
  DRILLFINISHING,
};
drillState DRILLSTATE = DRILLPAUSED;
drillState LASTDRILLSTATE = DRILLING;

enum servoState{
  SERVOPAUSED,
  SERVOPENDING,
  MOVINGINFILTRATION,
  MOVINGOUTFILTRATION,
  TEST,
  ARMRUNPROFILE01,
  ARMRUNPROFILE02,
  CENTRIFUGEROTATE,
  CENTRIFUGESTOP,
  CENTRIFUGEPOS,
};
servoState SERVOSTATE = SERVOPAUSED;
servoState LASTSERVOSTATE = TEST;

void setup() {
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  dxlGoalPos_ros.data = (uint16_t *)malloc(sizeof(uint16_t)*8);
  dxlGoalPos_ros.data_length = 10; //???????????????
  nh.advertise(pub_dxlGoalPos);
  nh.advertise(pub_armProfile_idx);
//  nh.advertise(pub_armStep_idx);
  nh.advertise(pub_drillState);
  nh.advertise(pub_servoState);
  nh.subscribe(curPos_sub);
  nh.subscribe(command_sub);

  Serial1.begin(9600); //connect to drill motor controller
  Serial2.begin(115200); //connect to Centrifuge
  Serial3.begin(115200); //connect to opencm



  commandMotor(1,0);
  commandMotor(2,0);
  resetSafetyTimer();
  TCCR4B &= ~0x7;
  TCCR4B |= 2;
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

void loop() {
  safetyCheck(); //need to reset safety timer
  runState();
  updateROSPubData();
  pub_dxlGoalPos.publish(&dxlGoalPos_ros);
  pub_armProfile_idx.publish(&armProfile_idx_ros);
//  pub_armStep_idx.publish(&armStep_idx_ros);
  pub_drillState.publish(&drillState_ros);
  pub_servoState.publish(&servoState_ros);
  nh.spinOnce();
  delay(50);
}

//void openCM_sendData()
//{
//  Serial3.print('A');
//  for (int i = 0; i < 10; i++)
//  {
//    Serial3.write((openCM_cmd[i]>>8) & 0xFF);
//    Serial3.write(openCM_cmd[i] & 0xFF);
//  }
//}
void dxlCurPos_cb( const std_msgs::UInt16MultiArray& data)
{
  for (int i = 0; i < 10; i++)
  {
    openCM_fb[i] = data.data[i];
  }
}


void updateROSPubData()
{
  for (int i = 0; i < 10; i++)
  {
    dxlGoalPos_ros.data[i] = openCM_cmd[i];
  }
//  armProfile_idx_ros.data = armProfile_idx;
//  armStep_idx_ros.data = armStep_idx;
  drillState_ros.data = DRILLSTATE;
  servoState_ros.data = SERVOSTATE;
}

void midLimitISR()
{
  if (DRILLSTATE == EXTRACTING)
  {
    DRILLSTATE = EXTRACTINGHOLD;
    extractingHoldEnterTime = millis();
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
    DRILLSTATE = DRILLPAUSED;
    SERVOSTATE = SERVOPAUSED;
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

void turnOnValve1()
{
  digitalWrite(VALVE1,LOW);
}
void turnOffValve1()
{
  digitalWrite(VALVE1,HIGH);
}
void turnOnValve2()
{
  digitalWrite(VALVE2,LOW);
}
void turnOffValve2()
{
  digitalWrite(VALVE2,HIGH);
}
void turnOnSignal()
{
  digitalWrite(SIGNALLIGHT,LOW);
}
void turnOffSignal()
{
  digitalWrite(SIGNALLIGHT,HIGH);
}
void turnOnWashPump()
{
  digitalWrite(WASHPUMP,LOW);
}
void turnOffWashPump()
{
  digitalWrite(WASHPUMP,HIGH);
}
void runCentrifuge()
{
  Serial2.print('R');
}
void stopCentrifuge()
{
  Serial2.print('P');
}
void cmdCentrifugePos(int pos)
{
  if(pos >= 0 && pos <=5)
  Serial2.print(pos);
}
void downLimitISR()
{
  DRILLSTATE = PULLING;
}
void upLimitISR()
{
  if (DRILLSTATE == PULLING)
  {
    DRILLSTATE = DRILLPENDING;
    SERVOSTATE = MOVINGINFILTRATION;
  }
  else if (DRILLSTATE == DRILLFINISHING)
  {
    DRILLSTATE = DRILLPENDING;
//    SERVOSTATE = ARMRUNPROFILE02;
  }
}
void trailerCommand_cb( const std_msgs::Char& data)
{
  char inChar = (char)data.data;
  if (inChar == 'p')
  {
    DRILLSTATE = DRILLPAUSED;
    SERVOSTATE = SERVOPAUSED;
    resetSafetyTimer();
  }
  else if (inChar == 'a')
  {
    if (DRILLSTATE == DRILLPAUSED)
    {
      DRILLSTATE = LASTDRILLSTATE;
    }
    resetSafetyTimer();
  }
  else if (inChar == '1')
  {
    resetSafetyTimer();
    sample_idx = 0;
  }
  else if (inChar == '2')
  {
    resetSafetyTimer();
    sample_idx = 1;
  }
  else if (inChar == '3')
  {
    resetSafetyTimer();
    sample_idx = 2;
  }
  else if (inChar == '4')
  {
    resetSafetyTimer();
    sample_idx = 3;
  }
  else if (inChar == '5')
  {
    resetSafetyTimer();
    sample_idx = 4;
  }
  else if (inChar == '6')
  {
    resetSafetyTimer();
    sample_idx = 5;
  }
}
void runState()
{
  switch(DRILLSTATE)
  {
    case DRILLPAUSED:
      commandMotor(1,0);
      commandMotor(2,0);
      turnOffValve1();
      turnOffValve2();
      turnOffWashPump();
      turnOffSignal();
      break;
    case DRILLING:
      if(openCM_fb[2] > (FILTRATIONOUT+10) || openCM_fb[2] < (FILTRATIONOUT-10))
      {
        openCM_cmd[2] = FILTRATIONOUT;
      }
      else
      {
        commandMotor(1,-VERTICALSPEED);
        commandMotor(2,DRILLSPEED);
        turnOnSignal();
      }
      LASTDRILLSTATE = DRILLING;
      break;
    case PULLING:
      commandMotor(1, VERTICALSPEED);
      commandMotor(2, DRILLSPEED);
      LASTDRILLSTATE = PULLING;
      turnOnSignal();
      break;
    case EXTRACTING:
      commandMotor(1, -VERTICALSPEED);
      commandMotor(2, DRILLSPEED);
      //start valve 1
      LASTDRILLSTATE = EXTRACTING;
      turnOnSignal();
      break;
    case EXTRACTINGHOLD:
      commandMotor(1,0);
      commandMotor(2, DRILLSPEED);
      if (millis() - extractingHoldEnterTime > EXTRACTIONTIME)
      {
        DRILLSTATE = DRILLFINISHING;
      }
      LASTDRILLSTATE = EXTRACTINGHOLD;
      turnOnSignal();
      break;
    case DRILLPENDING:
      commandMotor(1,0);
      commandMotor(2,0);
      LASTDRILLSTATE = DRILLPENDING;
      turnOffSignal();
      armProfile_idx = 1;
      break;
    // case MOVINGINFILTRATION:
    //   commandMotor(1,0);
    //   commandMotor(2,0);
    //   //send command to dynamixel controller to move the filtration inplace
    //   //if feedback says filtration tower inplace
    //   //DRILLSTATE = EXTRACTING;
    //   //place holder: wait for 10000 iterations second
    //   idx ++;
    //   if (idx == 1000)
    //   {
    //     DRILLSTATE = EXTRACTING;
    //     idx = 0;
    //   }
    //   LASTDRILLSTATE = MOVINGINFILTRATION;
    //   break;
    case DRILLFINISHING:
      commandMotor(1, VERTICALSPEED);
      commandMotor(2, DRILLSPEED);
      //trun off valve 1
      LASTDRILLSTATE = DRILLFINISHING;
      turnOffSignal();
      break;
  }
  switch(SERVOSTATE)
  {
    case MOVINGINFILTRATION:
      //move filtration in inplace
      openCM_cmd[2] = FILTRATIONIN;
      openCM_cmd[3] = FILTRATIONUP;
      if (openCM_fb[2] > (FILTRATIONIN - 5) && openCM_fb[2] < (FILTRATIONIN + 5))
      {
        if (openCM_fb[3] > (FILTRATIONUP - 5) && openCM_fb[3] < (FILTRATIONUP + 5))
        {
          SERVOSTATE = ARMRUNPROFILE01;
        }
      }
      //move tower in place
      //if filtration and tower feedback is closed to goal point
      // SERVOSTATE = ARMRUNPROFILE01;
      // arm run profile 1 (from rest, pickup meshtube, move under filtration tower)
    break;
    case MOVINGOUTFILTRATION:
      //move tower out
      //if tower is in place
      //move filtration
      //if filtration is in place
      //SERVOSTATE = PENDING;
    break;
    case SERVOPENDING:
      //do nothing
    break;
    case SERVOPAUSED:
      stopCentrifuge();
//      for (int i = 0; i < 10; i++)
//      {
//        openCM_cmd[i] = 0;
//      }
      //send all 0 to opencm
      //send stop running to centrifugeSpeed
    break;
    case CENTRIFUGEROTATE:
      runCentrifuge();
      if ((millis()-centrifugeStartTime) > CENTRIFUGERUNINGTIME)
      {
        SERVOSTATE = CENTRIFUGESTOP;
      }
    break;
    case CENTRIFUGESTOP:
      runCentrifuge();
      if (centrifugeSpeed < 10)
      {
        SERVOSTATE = CENTRIFUGEPOS;
      }
    break;
    case CENTRIFUGEPOS:
      runCentrifuge();
    break;
    case ARMRUNPROFILE01:
    armProfile_idx = 1;
    // arm run profile 1 (from rest, pickup meshtube, move under filtration tower)
    //if arm in final position
    //SERVOSTATE = PENDING;
    //DRILLSTATE = EXTRACTING;
    break;
    case ARMRUNPROFILE02:
    armProfile_idx = 2;

    //arm run profile 2 (from under the filtration tower to the centrifuge, slowly put int mesh tube,
    //then move to rest pos)
    break;
  }
}
