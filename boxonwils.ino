#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#define Wire Wire1
#include <Motoron.h>
#include <Servo.h>

// Motor controllers
MotoronI2C mc1(0x10); // Front
MotoronI2C mc2(0x11); // Back

// Network
const char ssid[] = "PhaseSpaceNetwork_2.4G";
const char password[] = "8igMacNet";
IPAddress ip;
WiFiUDP Udp;
const unsigned int localPort = 55500;
char incomingPacket[255];

// Flags and state variables
bool killSwitchActivated = false; // system starts off (until physical button pressed)
bool udpKilled = false;           // becomes true after UDP "Stop"
bool lastKillButtonState = HIGH;  // set to true when "stop" command received via udp
bool printFlag = false;           // makes the robot print sensor values to serial

// Pins
const int killPin = 10;                                                  // Physical kill switch
const int sensorPins[11] = {31, 41, 49, 39, 47, 37, 45, 35, 43, 33, 51}; // Reflectance sensors
bool reflectanceValuesBinary[11];                                        // Binary version of reflectance values
int baseSpeed = 600;                                                     // Base speed for motors
const int maxSpeed = 800;                                                // Maximum speed for motors

// PID for line-following
float lineKp = 6, lineKi = 0, lineKd = 0.1;

// Line-following global variables
int reflectanceValues[11]; // Reflectance sensor values
float lastErrorLf = 0;     // Last error for PID
float integralErrorLf = 0; // Integral term for PID
int lineThreshold = 300;   // Tuned experimentally
int lostLineCounter = 0;   // Counter for lost line detection

// PID for wall-following
float wallKp = 100, wallKi = 0, wallKd = 5, wallKa = 60; // Ka is the alignment factor (proportional to the difference between left sensors)

// Wall-following global variables
unsigned long turnStartTime = 0;
unsigned long turnDuration = 2400; // Adjust to calibrate 90-degree turn
String turnDirection = "";         // "LEFT" or "RIGHT"
float distanceError;               // desiredDistance - averageLeft
float lastDistanceError = 0;       // for derivative term
float integralErrorWf = 0;         // for integral term
float alignmentError;              // filteredLeft1 - filteredLeft2
unsigned long prevTime = 0;        // for dt

// Sonar pins
const int trigL1 = 5, echoL1 = 4, trigL2 = 9, echoL2 = 8;
const int trigF = 38, echoF = 36;
const int trigR = 52, echoR = 50;
// Sonar distances initialisation
float distL = 0;
float distL1 = 0;
float distL2 = 0;
float distF = 0;
float distR = 0;

// Filtering Variables for Sonars
float filteredLeft1 = 0, filteredLeft2 = 0;
float filteredFront = 0;
float filteredRight = 0;
const float alpha = 0.2; // Smoothing factor

// Servos
Servo front, back, bracket;

// Set the start and end angles of the servos
const int frontS = 20;
const int frontF = 90;
const int backS = 17;
const int backF = 125;
const int bracketS = 25;
const int bracketF = 65;

// Photoresistor
const int photoPin = A5;

void setup()
{
  Serial.begin(9600);             // Serial monitor for debugging
  pinMode(killPin, INPUT_PULLUP); // Physical kill switch pull-up resistor

  // Initialise I2C communication for Motoron controllers
  Wire1.begin();
  mc1.reinitialize();
  mc1.disableCrc();
  mc1.clearResetFlag();
  mc2.reinitialize();
  mc2.disableCrc();
  mc2.clearResetFlag();
  for (int m = 1; m <= 3; m += 2)
  {
    mc1.setMaxAcceleration(m, 140);
    mc1.setMaxDeceleration(m, 300);
    mc2.setMaxAcceleration(m, 140);
    mc2.setMaxDeceleration(m, 300);
  }

  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Udp.begin(localPort);
  Serial.println("\nWiFi connected.");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  // Initialise servos
  front.attach(10);
  back.attach(11);
  bracket.attach(12);
  front.write(frontS);
  back.write(backS);
  bracket.write(0);
}

void parseUDPCommand(String command)
{
  command.trim();

  // Check for kill command
  if (command == "Stop")
  {
    udpKilled = true;
    killSwitchActivated = false;
    Serial.println("🔴 REMOTE KILL RECEIVED — SYSTEM STOPPED 🔴");
    return;
  }

  // Check for line values command
  else if (command.startsWith("LINE_THRESHOLD"))
  {
    int lineThresholdComma = command.indexOf(',');
    if (lineThresholdComma == -1)
    {
      Serial.println("⚠️ Invalid line threshold format.");
      return;
    }
    lineThreshold = command.substring(lineThresholdComma + 1).toInt();
    Serial.println("Line Threshold = ");
    Serial.println(lineThreshold);
  }

  // Check for base speed command
  else if (command.startsWith("BASE_SPEED"))
  {
    int basesfirstComma = command.indexOf(',');
    if (basesfirstComma == -1)
    {
      Serial.println("⚠️ Invalid base speed format.");
      return;
    }
    String mode = command.substring(0, basesfirstComma);
    baseSpeed = command.substring(basesfirstComma + 1).toInt();
    Serial.print("✅ base speed updated: speed = ");
    Serial.println(baseSpeed);
  }

  // Check for turn duration command
  else if (command.startsWith("TURN_DURATION"))
  {
    int basesfirstComma = command.indexOf(',');
    if (basesfirstComma == -1)
    {
      Serial.println("⚠️ Invalid turn duration format.");
      return;
    }
    String mode = command.substring(0, basesfirstComma);
    turnDuration = command.substring(basesfirstComma + 1).toInt();
    Serial.print("✅ turn duration updated: duration = ");
    Serial.println(turnDuration);
  }

  // Check for toggle print flag command
  else if (command == "TOGGLE_PRINT_FLAGS")
  {
    printFlag = !printFlag;
    Serial.print("✅ print flag toggled: ");
    Serial.println(printFlag);
  }

  // Check for PID command
  else if (command.startsWith("LINE_PID") || command.startsWith("WALL_PID"))
  {
    int firstComma = command.indexOf(',');
    int secondComma = command.indexOf(',', firstComma + 1);
    int thirdComma = command.indexOf(',', secondComma + 1);

    // Retrieving Ka value for wall_pid
    if (command.startsWith("WALL_PID"))
    {
      int fourthComma = command.indexOf(',', thirdComma + 1);
      if (fourthComma == -1)
      {
        Serial.println("⚠️ Invalid PID format.");
        return;
        float ka = command.substring(fourthComma + 1).toFloat();
        wallKa = ka;
      }
    }

    if (firstComma == -1 || secondComma == -1 || thirdComma == -1)
    {
      Serial.println("⚠️ Invalid PID format.");
      return;
    }

    String mode = command.substring(0, firstComma);
    float kp = command.substring(firstComma + 1, secondComma).toFloat();
    float ki = command.substring(secondComma + 1, thirdComma).toFloat();
    float kd = command.substring(thirdComma + 1).toFloat();

    if (mode == "LINE_PID")
    {
      lineKp = kp;
      lineKi = ki;
      lineKd = kd;
      Serial.print("✅ Line-Following PID updated: P = ");
      Serial.println(lineKp);
      Serial.println(" | I = ");
      Serial.println(lineKi);
      Serial.println(" | D = ");
      Serial.println(lineKd);
    }
    else if (mode == "WALL_PID")
    {
      wallKp = kp;
      wallKi = ki;
      wallKd = kd;
      Serial.print("✅ Wall-Following PID updated: P = ");
      Serial.println(wallKp);
      Serial.println(" | I = ");
      Serial.println(wallKi);
      Serial.println(" | D = ");
      Serial.println(wallKd);
      Serial.println(" | A = ");
      Serial.println(wallKa);
    }
  }
  else
  {
    Serial.println("⚠️ Unknown UDP command received.");
  }
}

// Function to receive and parse UDP commands
void receiveUDPCommand()
{
  int packetSize = Udp.parsePacket();
  if (packetSize)
  {
    int len = Udp.read(incomingPacket, 255);
    if (len > 0)
    {
      incomingPacket[len] = 0;
      String command = String(incomingPacket);
      parseUDPCommand(command);
    }
  }
}

void checkPhysicalKillSwitch()
{
  static bool buttonState = HIGH;            // Current stable state
  static bool lastReading = HIGH;            // Last read from pin
  static unsigned long lastDebounceTime = 0; // Last time the button state changed
  const unsigned long debounceDelay = 50;    // ms debounce delay

  bool reading = digitalRead(killPin); // Read the current state of the kill switch

  if (reading != lastReading)
  {
    lastDebounceTime = millis(); // Reset the debounce timer
  }

  if ((millis() - lastDebounceTime) > debounceDelay)
  {
    if (reading != buttonState)
    {
      buttonState = reading; // Update the stable state

      if (buttonState == LOW)
      {
        // Button pressed — toggle the kill switch
        if (udpKilled)
        {
          Serial.println("🔁 Resetting system after remote kill.");
          udpKilled = false;
          killSwitchActivated = true;
        }
        else
        {
          killSwitchActivated = !killSwitchActivated;
          Serial.print("Kill switch toggled: ");
          Serial.println(killSwitchActivated ? "ON ✅" : "OFF ❌");

          // Handle servos
          if (!killSwitchActivated)
          {
            stopServos();
          }
          else
          {
            startServos();
          }
        }
      }
    }
  }

  lastReading = reading;
}

// Function to read sonar with filtering
float readSonarFiltered(int trigPin, int echoPin, float &filteredValue)
{
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Trigger the ultrasonic sensor
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Send a 10us pulse to trigger the sensor
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  // Stop the trigger pulse
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 12500); // Timeout: 12.5ms
  float distanceCm = (duration * 0.0343) / 2;    // Convert to cm

  if (distanceCm < 0 || distanceCm > 300)
    return filteredValue; // Invalid distance, return last filtered value
  if (filteredValue == 0)
    filteredValue = distanceCm; // Initialise on first read
  else
    filteredValue = alpha * distanceCm + (1 - alpha) * filteredValue; // Apply low-pass filter

  return filteredValue;
}

// Function to read sonar without filtering
// This is used for the front sensor, which is not filtered
long readSonar(int echoPin, int trigPin)
{
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000); // 30ms timeout
  long distance = duration * 0.034 / 2;          // cm
  if (duration == 0)
    return -1; // No echo
  return distance;
}

void stopServos()
{
  front.detach();
  back.detach();
  bracket.detach();
}

void startServos()
{
  front.attach(10);
  back.attach(11);
  bracket.attach(12);
}

void forward()
{
  mc1.setSpeed(1, baseSpeed); // Left 1
  mc2.setSpeed(1, baseSpeed); // Left 2
  mc1.setSpeed(3, baseSpeed); // Right 1
  mc2.setSpeed(3, baseSpeed); // Right 2
}

void fullSend()
{
  mc1.setSpeed(1, maxSpeed); // Left 1
  mc2.setSpeed(1, maxSpeed); // Left 2
  mc1.setSpeed(3, maxSpeed); // Right 1
  mc2.setSpeed(3, maxSpeed); // Right 2
}

void backward()
{
  mc1.setSpeed(1, -baseSpeed); // Left 1
  mc2.setSpeed(1, -baseSpeed); // Left 2
  mc1.setSpeed(3, -baseSpeed); // Right 1
  mc2.setSpeed(3, -baseSpeed); // Right 2
}

void leftTurn()
{
  mc1.setSpeed(1, -baseSpeed); // Left 1
  mc1.setSpeed(1, baseSpeed);  // Left 2
  mc2.setSpeed(3, -baseSpeed); // Right 1
  mc2.setSpeed(3, baseSpeed);  // Right 2
}

void rightTurn()
{
  mc1.setSpeed(1, baseSpeed);  // Left 1
  mc1.setSpeed(1, -baseSpeed); // Left 2
  mc2.setSpeed(3, baseSpeed);  // Right 1
  mc2.setSpeed(3, -baseSpeed); // Right 2
}

// Stop all motors
void stopMotors()
{
  mc1.setSpeed(1, 0);
  mc1.setSpeed(3, 0);
  mc2.setSpeed(1, 0);
  mc2.setSpeed(3, 0);
}

void readReflectanceSensors()
{
  // Reset reflectance values
  for (int i = 0; i < 11; i++)
  {
    reflectanceValues[i] = 0;
    pinMode(sensorPins[i], OUTPUT);
    digitalWrite(sensorPins[i], HIGH);
  }
  delayMicroseconds(10);

  // Set pins to input mode for reading
  for (int i = 0; i < 11; i++)
  {
    pinMode(sensorPins[i], INPUT);
  }

  // Read reflectance sensors
  uint32_t startTime = micros();
  bool active = true;
  while (active && micros() - startTime < 3000)
  {
    active = false;
    for (int i = 0; i < 11; i++)
    {
      if (reflectanceValues[i] == 0 && digitalRead(sensorPins[i]) == LOW)
      {
        reflectanceValues[i] = micros() - startTime;
      }
      else if (reflectanceValues[i] == 0)
      {
        active = true;
      }
    }
  }

  // Ensure all sensors have a value
  for (int i = 0; i < 11; i++)
  {
    if (reflectanceValues[i] == 0)
    {
      reflectanceValues[i] = 3001;
    }

    // Populate binary version based on lineThreshold
    reflectanceValuesBinary[i] = reflectanceValues[i] > lineThreshold;
  }
}

float computeLineError()
{
  long weightedSum = 0;      // Weighted sum of sensor values
  long totalValue = 0;       // Total value for normalisation
  const int centreIndex = 5; // Index of the centre sensor (6th sensor)

  for (int i = 1; i < 10; i++)
  {
    long value = reflectanceValues[i]; // Reflectance value for sensor i
    if (value < 0)
      value = 0;

    int offset = i - centreIndex;                                         // Offset from the centre sensor
    int weightBias = (i == centreIndex) ? 3 : (abs(offset) == 1 ? 2 : 1); // 3x for centre, 2x for neighbours of centre
    weightedSum += offset * value * weightBias;                           // Weighted sum calculation
    totalValue += value * weightBias;                                     // Total value for normalisation
  }

  if (printFlag)
  {
    Serial.print("Reflectance array: ");
    for (int i = 0; i < 11; i++)
    {
      Serial.print(reflectanceValues[i]);
      Serial.print(" ");
    }
    Serial.println();
    for (int i = 0; i < 11; i++)
    {
      Serial.print(reflectanceValuesBinary[i]);
      Serial.print(" ");
    }
    Serial.println();
  }

  if (totalValue == 0)
    return 0.0;                                   // No line detected
  return (float)weightedSum / totalValue * 100.0; // Scale for sensitivity
}

float computeLinePID(float error)
{
  float derivative = error - lastErrorLf; // Change in error
  integralErrorLf += error;               // Accumulate error for integral term
  lastErrorLf = error;                    // Update last error for next iteration

  return lineKp * error + lineKi * integralErrorLf + lineKd * derivative;
}

void applyMotorSpeeds(float correction)
{
  int leftSpeed = constrain(baseSpeed - correction, -maxSpeed, maxSpeed);  // Constrain left speed
  int rightSpeed = constrain(baseSpeed + correction, -maxSpeed, maxSpeed); // Constrain right speed

  if (printFlag)
  {
    Serial.println("");
    Serial.print(leftSpeed);
    Serial.print(" - ");
    Serial.print(rightSpeed);
    Serial.println("");
  }

  mc1.setSpeed(1, leftSpeed);  // Left 1
  mc2.setSpeed(1, leftSpeed);  // Left 2
  mc1.setSpeed(3, rightSpeed); // Right 1
  mc2.setSpeed(3, rightSpeed); // Right 2
}

// Check if all reflectance sensors are off (values are under lineThreshold)
bool allSensorsOff()
{
  for (int i = 0; i < 11; i++)
  {
    if (reflectanceValuesBinary[i])
      return false;
  }
  return true;
}

// Check if all reflectance sensors are on (values are above lineThreshold)
bool allSensorsOn()
{
  for (int i = 0; i < 11; i++)
  {
    if (!reflectanceValuesBinary[i])
      return false;
  }
  return true;
}

// Check if the central two of three sensors are on (values are above lineThreshold)
bool centralTwoOfThreeOn()
{
  // Check sensors 4, 5, and 6 (indices 3, 4, and 5)
  return reflectanceValuesBinary[3] || reflectanceValuesBinary[4] || reflectanceValuesBinary[5];
}

void followLine()
{
  readReflectanceSensors();

  // Check if we have lost the line
  if (allSensorsOff())
  {
    lostLineCounter++;

    if (printFlag)
      Serial.println("🔁 Turning right to reacquire line after intersection...");

    while (!centralTwoOfThreeOn())
    {
      // Keep turning right until we reacquire the line
      rightTurn(); // All turns are right turns
      readReflectanceSensors();
      ; // Refresh sensors while turning
    }

    if (printFlag)
      Serial.println("✅ Line reacquired — resuming line following");
    stopMotors(); // Stop motors to avoid overshooting
    delay(200);   // Short delay to stabilise after turning

    return; // skip the rest of line-following while turning
  }

  float error = computeLineError();
  float correction = computeLinePID(error);
  applyMotorSpeeds(correction);
}

void turn90(String direction, unsigned long duration)
{
  int speed = 800; // Speed for turning
  if (direction == "LEFT")
    speed = -speed;        // Reverse speed for left turn
  stopMotors();            // Stop motors before turning
  delay(200);              // Short delay to ensure motors are stopped
  mc1.setSpeed(1, speed);  // Left 1
  mc2.setSpeed(1, speed);  // Left 2
  mc1.setSpeed(3, -speed); // Right 1
  mc2.setSpeed(3, -speed); // Right 2
  delay(duration);         // Wait for the turn duration
  stopMotors();            // Stop motors after turning
  delay(200);              // Short delay to ensure motors are stopped
}

void followLeftWall()
{
  const float targetDistance = 8.0;             // in cm
  unsigned long currentTime = millis();         // Get current time in ms
  float dt = (currentTime - prevTime) / 1000.0; // Convert ms to seconds
  prevTime = currentTime;                       // Update previous time

  float averageLeft = (filteredLeft1 + filteredLeft2) / 2.0; // Average of left sensors
  distanceError = targetDistance - averageLeft;              // Calculate distance error
  alignmentError = filteredLeft1 - filteredLeft2;            // Calculate alignment error

  integralErrorWf += distanceError * dt;                            // Integrate distance error over time
  float derivativeError = (distanceError - lastDistanceError) / dt; // Calculate derivative of distance error
  lastDistanceError = distanceError;                                // Update previous distance error for next iteration

  float pidOutput =
      wallKp * distanceError +
      wallKi * integralErrorWf +
      wallKd * derivativeError; // PID output for distance error

  float alignmentCorrection = wallKa * alignmentError; // Correction based on alignment error

  float controlOutput = pidOutput + alignmentCorrection; // Total control output

  // Calculate left and right motor speeds
  int leftSpeed = baseSpeed + controlOutput;
  int rightSpeed = baseSpeed - controlOutput;

  // Constrain to maxSpeed
  leftSpeed = constrain(leftSpeed, -maxSpeed, maxSpeed);
  rightSpeed = constrain(rightSpeed, -maxSpeed, maxSpeed);

  // Print speeds fro debugging if printFlag is set
  if (printFlag)
  {
    Serial.println("");
    Serial.print(leftSpeed);
    Serial.print(" - ");
    Serial.print(rightSpeed);
    Serial.println("");
  }

  mc1.setSpeed(1, leftSpeed);  // Left 1
  mc2.setSpeed(1, leftSpeed);  // Left 2
  mc1.setSpeed(3, rightSpeed); // Right 1
  mc2.setSpeed(3, rightSpeed); // Right 2
}

// Function to navigate around walls (Wall-following with turning)
void navWall()
{
  if (distF < 18.0)
  {
    if (printFlag)
      Serial.println("Obstacle ahead");
    stopMotors();
    turn90("RIGHT", turnDuration);
  }
  else
  {
    followLeftWall();
  }
}

// Servo functions

// Deploys claw from start angle to end angle to attach to objects over the body.
// Deploys L-bracket to a functional line-following position.
void deployClaw_Bracket(int endAngle, Servo &motor)
{ // use &motor so that the actual servo object can be referenced
  motor.write(endAngle);
}

// Retracts claw from end angle to start angle to detach from objects over the body.
// Retracts L-bracket to a non-functional position for obstacle clearance.
void retractClaw_Bracket(int startangle, Servo &motor)
{
  motor.write(startangle);
}

void loop()
{
  checkPhysicalKillSwitch();
  receiveUDPCommand();

  // Check if the kill switch is activated or if the system has been killed via UDP
  if (!killSwitchActivated || udpKilled)
  {
    stopMotors();
    stopServos();
    delay(400);
    return;
  }

  // Read sonar sensors
  distL1 = readSonarFiltered(trigL1, echoL1, filteredLeft1);
  distL2 = readSonarFiltered(trigL2, echoL2, filteredLeft2);
  distF = readSonar(echoF, trigF);
  distR = readSonarFiltered(trigR, echoR, filteredRight);
  if (printFlag)
  {
    Serial.print("Left 1: ");
    Serial.print(distL1);
    Serial.print(" cm | ");
    Serial.print("Left 2: ");
    Serial.print(distL2);
    Serial.print(" cm | ");
    Serial.print("Front: ");
    Serial.print(distF);
    Serial.print(" cm | ");
    Serial.print("Right: ");
    Serial.print(distR);
    Serial.println(" cm");
  }

  // Insert main loop logic below

  // Section 1 only (theoretically)
  // followLine();

  // Section 2 only (theoretically)
  // navWall();

  // Section 3 only (theoretically)
  // run the All Sections logic with counter at 2 to skip the first section
  // lostLineCounter = 3;

  // All Sections (theoretically)
  if (lostLineCounter < 2)
    followLine(); // clears Section 1, counter becomes 1 at the first 90deg bend, then 2 when the line stops

  else if (lostLineCounter == 2)
  {
    retractClaw_Bracket(bracketS, bracket); // retract bracket for Section 2 obstacle clearance
    lostLineCounter++;                      // increment lostLineCounter artificially to avoid re-entering this section, makes it 3
  }

  else if (distL1 < 20)
    navWall(); // clears Section 2 + the first ramp of Section 3 (walls are only present until then)

  else if (distL1 > 20 and lostLineCounter == 3) // if the left sensor is clear, we are at the end of the Section 3 ramp
  {
    deployClaw_Bracket(bracketF, bracket); // deploys bracket for Section 3
    delay(400);
    lostLineCounter++; // increment lostLineCounter artificially to avoid re-entering this section, makes it 4
  }

  else if ((lostLineCounter == 7) || (lostLineCounter == 8 && allSensorsOn())) // Go across the pit or descend the zipline (the line stops before the pit, a black line across teh track is detected before the zipline)
  {                                                                            // two 90deg turns get the counter to 6, the end of the line before the pit makes it 7, crossing the pit makes it 8
    deployClaw_Bracket(frontF, front);                                         // deploys front claw
    delay(400);
    deployClaw_Bracket(backF, back); // deploys back claw
    delay(400);
    fullSend();                         // go full speed to cross the pit or after attaching to the zipline
    delay(3000);                        // wait for 3 seconds to cross the pit or descend the zipline
    retractClaw_Bracket(frontS, front); // retracts front claw
    delay(400);
    retractClaw_Bracket(backS, back); // retracts back claw
    delay(400);
    stopMotors();      // stop motors to avoid overshooting
    delay(200);        // Short delay to stabilise after stopping
    lostLineCounter++; // increment lostLineCounter artificially to avoid re-entering this section
  }

  else if (lostLineCounter >= 9)
    stopMotors(); // lostLineCounter >= 8 means all Section are cleared, so the robot is stopped

  else
    followLine(); // clears the rest of Section 3
}