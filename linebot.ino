#include "BluetoothSerial.h"
BluetoothSerial BT;

#define IR1 36
#define IR2 39
#define IR3 34
#define IR4 35
#define IR5 32
#define IR6 33
#define IR7 25
#define IR8 26

#define AIN1 12
#define AIN2 27
#define PWMA 14
#define BIN1 23
#define BIN2 22
#define PWMB 13
#define STBY 4

int pins[8] = {IR1, IR2, IR3, IR4, IR5, IR6, IR7, IR8};
int weight[8] = {-13, -7, -3, -1, 1, 3, 7, 13};

float Kp = 30;
float Ki = 0;
float Kd = 50;
float lastError = 0;
float integral = 0;
int baseSpeed = 170;
int recoverySpeed = 90;
int turnSpeed = 120;
int lastDirection = 0;
int lastEndSensor = 0;
bool running = false;

// --- NEW SCORING SYSTEM VARIABLES ---
int rightScore = 0;
int leftScore = 0;
const int SCORE_THRESHOLD = 4;           // Frames needed to commit to a turn
unsigned long lastTurnTime = 0;          // Tracks when the last turn ended
const unsigned long COOLDOWN_TIME = 300; // ms to ignore new junctions after turning
const int SCOOT_TIME = 60;               // ms to drive forward before pivoting

void setup()
{
  Serial.begin(115200);
  BT.begin("LineBot");
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);
  ledcAttach(PWMA, 20000, 8);
  ledcAttach(PWMB, 20000, 8);
  for (int i = 0; i < 8; i++)
    pinMode(pins[i], INPUT);
  stopMotors();
  BT.println("LineBot Ready");
  BT.println("RIGHT PRIORITY with SCORING");
}

void loop()
{
  handleBluetooth();
  if (running)
    lineFollow();
  else
    stopMotors();
}

void handleBluetooth()
{
  if (!BT.available())
    return;
  String cmd = BT.readStringUntil('\n');
  cmd.trim();
  cmd.toUpperCase();
  if (cmd == "START")
  {
    running = true;
    lastError = 0;
    integral = 0;
    rightScore = 0;
    leftScore = 0;
    BT.println("OK: Started");
  }
  else if (cmd == "STOP")
  {
    running = false;
    stopMotors();
    BT.println("OK: Stopped");
  }
  else if (cmd == "STATUS")
  {
    sendStatus();
  }
  else if (cmd == "SENSORS")
  {
    sendSensors();
  }
  else if (cmd.startsWith("KP:"))
  {
    Kp = cmd.substring(3).toFloat();
    BT.print("OK: Kp = ");
    BT.println(Kp);
  }
  else if (cmd.startsWith("KI:"))
  {
    Ki = cmd.substring(3).toFloat();
    integral = 0;
    BT.print("OK: Ki = ");
    BT.println(Ki);
  }
  else if (cmd.startsWith("KD:"))
  {
    Kd = cmd.substring(3).toFloat();
    BT.print("OK: Kd = ");
    BT.println(Kd);
  }
  else if (cmd.startsWith("BASE:"))
  {
    baseSpeed = constrain((int)cmd.substring(5).toFloat(), 0, 255);
    BT.print("OK: Base Speed = ");
    BT.println(baseSpeed);
  }
  else if (cmd.startsWith("RECOVERY:"))
  {
    recoverySpeed = constrain((int)cmd.substring(9).toFloat(), 0, 255);
    BT.print("OK: Recovery Speed = ");
    BT.println(recoverySpeed);
  }
  else if (cmd.startsWith("TURN:"))
  {
    turnSpeed = constrain((int)cmd.substring(5).toFloat(), 0, 255);
    BT.print("OK: Turn Speed = ");
    BT.println(turnSpeed);
  }
  else
  {
    BT.print("ERR: Unknown command: ");
    BT.println(cmd);
  }
}

void sendStatus()
{
  BT.println("--- LineBot Status ---");
  BT.print("State          : ");
  BT.println(running ? "RUNNING" : "STOPPED");
  BT.print("Kp             : ");
  BT.println(Kp);
  BT.print("Ki             : ");
  BT.println(Ki);
  BT.print("Kd             : ");
  BT.println(Kd);
  BT.print("Base Speed     : ");
  BT.println(baseSpeed);
  BT.print("Turn Speed     : ");
  BT.println(turnSpeed);
  BT.println("----------------------");
}

void sendSensors()
{
  BT.print("SENSORS: ");
  for (int i = 0; i < 8; i++)
  {
    BT.print(digitalRead(pins[i]));
    if (i < 7)
      BT.print(",");
  }
  BT.println();
}

void lineFollow()
{
  int bin[8];
  long numerator = 0;
  long denominator = 0;
  
  for (int i = 0; i < 8; i++)
  {
    bin[i] = digitalRead(pins[i]);
    numerator += bin[i] * weight[i];
    denominator += bin[i];
  }

  if (bin[0] == 1) lastEndSensor = -1;
  if (bin[7] == 1) lastEndSensor = 1;

  // --- PHASE 4: Cooldown Check ---
  // Only evaluate junctions if we are outside the cooldown window
  if (millis() - lastTurnTime > COOLDOWN_TIME)
  {
    // --- PHASE 1: Accumulate Scores ---
    if (isRightJunction(bin)) {
      rightScore++;
    } else {
      rightScore = 0; // Decay immediately if no longer seen
    }

    if (isLeftJunction(bin)) {
      leftScore++;
    } else {
      leftScore = 0;
    }

    // --- PHASE 2: Commit (Right Priority) ---
    if (rightScore >= SCORE_THRESHOLD)
    {
      executeTurn('R');
      return;
    }
    else if (leftScore >= SCORE_THRESHOLD)
    {
      executeTurn('L');
      return;
    }
  }
  else
  {
    // Keep scores zeroed during cooldown
    rightScore = 0;
    leftScore = 0;
  }

  // --- Normal Line Following (Runs if no turn was triggered) ---
  if (denominator == 0)
  {
    recoverLine();
    return;
  }

  float error = (float)numerator / denominator;
  error = constrain(error, -5, 5);
  
  if (error > 0.5) lastDirection = 1;
  if (error < -0.5) lastDirection = -1;
  
  if (abs(error) < 2) integral += error;
  integral = constrain(integral, -100, 100);
  
  float derivative = error - lastError;
  derivative = constrain(derivative, -3, 3);
  lastError = error;
  
  float correction = (Kp * error) + (Ki * integral) + (Kd * derivative);
  correction = constrain(correction, -120, 120);
  
  int speedDrop = abs(correction) * 0.6;
  int leftSpeed = constrain(baseSpeed - correction - speedDrop, -255, 255);
  int rightSpeed = constrain(baseSpeed + correction - speedDrop, -255, 255);
  
  moveMotors(leftSpeed, rightSpeed);
  delayMicroseconds(1500);
}

// Helper function to manage Phase 2 and transitioning to Phase 3 & 4
void executeTurn(char dir)
{
  // 1. Scoot Forward: Gets the wheels aligned with the junction
  moveMotors(baseSpeed, baseSpeed);
  delay(SCOOT_TIME); 

  // 2. Execute the turn (Phase 3)
  if (dir == 'R') turnRight();
  else turnLeft();

  // 3. Setup Cooldown (Phase 4)
  lastTurnTime = millis();
  rightScore = 0;
  leftScore = 0;
  integral = 0; // Reset PID memory so it doesn't snap wildly after turning
}

bool isRightJunction(int s[])
{
  int centerCount = s[2] + s[3] + s[4] + s[5];
  int rightCount = s[6] + s[7];
  return (centerCount >= 3 && rightCount >= 1);
}

bool isLeftJunction(int s[])
{
  int centerCount = s[2] + s[3] + s[4] + s[5];
  int leftCount = s[0] + s[1];
  return (centerCount >= 3 && leftCount >= 1);
}

void turnRight()
{
  moveMotors(-turnSpeed, +turnSpeed);
  unsigned long turnStart = millis();
  const unsigned long minTurnTime = 250;
  int confirmCount = 0;
  const int confirmNeeded = 3;
  
  while (running)
  {
    int s1 = digitalRead(IR1);
    int s2 = digitalRead(IR2);
    int s4 = digitalRead(IR4);
    int s5 = digitalRead(IR5);
    int s7 = digitalRead(IR7);
    int s8 = digitalRead(IR8);

    bool centerOnLine = (s4 || s5);
    bool outerClear = !(s1 || s2 || s7 || s8);

    if (millis() - turnStart >= minTurnTime && centerOnLine && outerClear)
    {
      confirmCount++;
      if (confirmCount >= confirmNeeded)
        break;
    }
    else
    {
      confirmCount = 0;
    }
  }
  delay(20);
}

void turnLeft()
{
  moveMotors(+turnSpeed, -turnSpeed);
  unsigned long turnStart = millis();
  const unsigned long minTurnTime = 250;
  int confirmCount = 0;
  const int confirmNeeded = 3;
  
  while (running)
  {
    int s1 = digitalRead(IR1);
    int s2 = digitalRead(IR2);
    int s4 = digitalRead(IR4);
    int s5 = digitalRead(IR5);
    int s7 = digitalRead(IR7);
    int s8 = digitalRead(IR8);

    bool centerOnLine = (s4 || s5);
    bool outerClear = !(s1 || s2 || s7 || s8);

    if (millis() - turnStart >= minTurnTime && centerOnLine && outerClear)
    {
      confirmCount++;
      if (confirmCount >= confirmNeeded)
        break;
    }
    else
    {
      confirmCount = 0;
    }
  }
  delay(20);
}

void recoverLine()
{
  if (lastEndSensor != 0)
  {
    if (lastEndSensor > 0)
      moveMotors(-recoverySpeed, recoverySpeed);
    else
      moveMotors(recoverySpeed, -recoverySpeed);
  }
  else
  {
    if (lastDirection > 0)
      moveMotors(-recoverySpeed, recoverySpeed);
    else
      moveMotors(recoverySpeed, -recoverySpeed);
  }
}

void moveMotors(int leftSpeed, int rightSpeed)
{
  if (leftSpeed >= 0)
  {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  }
  else
  {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    leftSpeed = -leftSpeed;
  }
  
  if (rightSpeed >= 0)
  {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  }
  else
  {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    rightSpeed = -rightSpeed;
  }
  
  ledcWrite(PWMA, leftSpeed);
  ledcWrite(PWMB, rightSpeed);
}

void stopMotors()
{
  ledcWrite(PWMA, 0);
  ledcWrite(PWMB, 0);
}
