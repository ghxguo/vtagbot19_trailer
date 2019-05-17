char inChar;
void setup() {
Serial.begin(9600);
Serial1.begin(9600);
Serial3.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (inChar == 'd')
  {
    commandMotor(1,-0.5);
  }
  else if (inChar == 'u')
  {
    commandMotor(1,0.5);
  }
  else if (inChar == ' ')
  {
    commandMotor(1,0);
  }
}

void serialEvent()
{
  inChar = Serial.read();
  if (inChar == 'd')
  {
  }
  else if (inChar == 'u')
  {
  }
  else if (inChar == ' ')
  {
    
  }
  else
  {
    Serial3.write(inChar);
  }
}
void serialEvent3()
{
  char inChar = Serial3.read();
  Serial.write(inChar);
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
