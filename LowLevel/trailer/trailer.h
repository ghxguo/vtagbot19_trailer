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
#define EXTRACTINGTIME    10000
#define EXTRACTINGWATERTIME 3000
#define CENTRIFUGETIME    10000

void rosInit();
void resetSafetyTimer();
void safetyCheck();
void GPIOInit();
void trailerCommand_cb( const std_msgs::Char& data);
void downLimitISR();
void upLimitISR();
void midLimitISR();
void updateROSPubData();
void commandMotor(int id, float speedFrac);
void stopCentrifuge();
void stateRun();
void armState_cb(const std_msgs::Bool& data);


ros::NodeHandle nh;

static std_msgs::UInt16 armProfile_idx_ros;
static std_msgs::UInt8 controlState_ros;
static uint8_t armProfile_idx;
volatile uint16_t armStep = 0;
static long safetyTimer = 0;
static long extractingHoldEnterTime = 0;
static uint8_t sample_idx = 1;
static bool armProfile_finished = false;
static bool drillAtTop = false;

ros::Publisher pub_armProfile_idx("/armProfile_idx", &armProfile_idx_ros);
ros::Publisher pub_controlState("/controlState", &controlState_ros);
ros::Subscriber<std_msgs::Char> command_sub("/trailerCommand", &trailerCommand_cb);
ros::Subscriber<std_msgs::Bool> armState_sub("/armState", &armState_cb);

enum towerPlace{
  IN,
  OUT,
  UNKNOWN,
};
towerPlace TOWERPLACE = UNKNOWN;

enum controlStates
{
  PAUSED,
  MOVEOUTTOWER,
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

void updateROSPubData()
{
  armProfile_idx_ros.data = armProfile_idx;
  controlState_ros.data = CONTROLSTATE;
  pub_armProfile_idx.publish(&armProfile_idx_ros);
  pub_controlState.publish(&controlState_ros);
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

void rosInit()
{
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(pub_armProfile_idx);
  nh.advertise(pub_controlState);
  nh.subscribe(command_sub);
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
