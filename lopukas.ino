#include <Tic.h>

#ifdef SERIAL_PORT_HARDWARE_OPEN
#define ticSerial SERIAL_PORT_HARDWARE_OPEN
#else
#include <SoftwareSerial.h>
SoftwareSerial ticSerial(10, 11);
#endif

TicSerial tic(ticSerial);

#define ENCA 20
#define ENCB 21
#define IN2 8
#define IN1 5

// Z-axis control
const byte encoder0pinAz = 2; 
const byte encoder0pinBz = 3;
int E_leftz = 6; 
int M_leftz = 7; 

// Variables for encoder and motor control
boolean Direction;             // The rotation direction
double durationz = 0;          // The number of pulses
int currentposz = 0;           // Current Z position
int currentposx = 0;           // Current X position
int nextposz;                  // Next Z position
int nextposx;                  // Next X position
int currentpos;
int nextpos;
volatile int posi = 0;         // X axis encoder position
long prevT = 0;                // previous time for PID-calculations
float eprev = 0;               // previous error for PID-calculations
float eintegral = 0;           // previous integral error for PID-calculations
bool reachedTarget = false;    // boolean if x-axis next position has been reached

// Function declarations
void resetCommandTimeout();
void delayWhileResettingCommandTimeout(uint32_t ms);
void waitForPosition(int32_t targetPosition);
void advance(int Speed, int E, int M, boolean polarity, int timePosition);
void axis_stop(int E, int M);
void setup();
void loop();
void showCurrentPosition();
void moveToPosition();
void penUp();
void penDown();
void controlMotor(int target);
void setMotor(int dir, int pwmVal, int in1, int in2);
void setMotorBack(int dir, int pwmVal, int in1, int in2);
void readEncoder();

void resetCommandTimeout()
{
  tic.resetCommandTimeout(); //resets timeout
}

void delayWhileResettingCommandTimeout(uint32_t ms)
{
  uint32_t start = millis();
  do
  {
    resetCommandTimeout();
  } while ((uint32_t)(millis() - start) <= ms);
}

void waitForPosition(int32_t targetPosition)
{
  do
  {
    resetCommandTimeout();
  } while (tic.getCurrentPosition() != targetPosition); //wait until manipulator has reached the correct position
}

void advance(int Speed, int E, int M, boolean polarity, int timePosition) // move the z-axis motor, polarity decides if going up or down
{
 if (polarity == true) {
  digitalWrite(M,LOW);
  analogWrite(E,Speed);
  delay(timePosition);
  axis_stop(E, M);
 }
 else {
  digitalWrite(M,HIGH);
  analogWrite(E,Speed);
  delay(timePosition);
  axis_stop(E, M);
 }
}

void axis_stop(int E, int M) { //stop the motor
 digitalWrite(M,LOW);
 analogWrite(E,0);
}

void setup() {
  ticSerial.begin(9600);
  Serial.begin(9600);
  delay(20);
  tic.haltAndSetPosition(0); //set starting position to 0
  tic.exitSafeStart();
  tic.setTargetVelocity(400);
  tic.setStartingSpeed(2000);
  for(int i=3;i<9;i++)
    pinMode(i,OUTPUT);
  for(int i=11;i<13;i++)
    pinMode(i,OUTPUT);
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(M_leftz, OUTPUT);
  pinMode(E_leftz, OUTPUT);
  Direction = true;   // default -> Forward
  pinMode(encoder0pinBz, INPUT);
}

void loop() {
  prevT = 0;
  eprev = 0;
  eintegral = 0;
  reachedTarget = false;
  
  Serial.println("Vali soovitud käsk:");
  Serial.println("1 - liikumine uuele koordinaadile:");
  Serial.println("2 - kuva praegust koordinaati:");
  Serial.println("3 - langeta marker alla:");
  Serial.println("4 - tõsta marker üles:");
  Serial.println("5 - vaheta samm-mootori kiirust:");
  Serial.println("6 - välju programmist:");
  
  // Flag to indicate whether input has been received
  bool inputReceived = false;

  // Wait until there is input available from the serial interface
  while (!inputReceived) {
    if (Serial.available() > 0) {
      inputReceived = true;
    }
  }
  int choice = Serial.parseInt();
  switch (choice) {
    case 1:
      moveToPosition();
      break;
    case 2:
      showCurrentPosition();
      break;
    case 3:
      penDown();
      Serial.println("Tööriist on all.");
      break;
    case 4:
      penUp();
      Serial.println("Tööriist on üleval.");
      break;
    case 5:
      changeStepperSpeed();
      break;
    case 6:
      Serial.println("Programm suletud.");
      delay(100);
      exit(0);
      break;
    default:
      Serial.println("Sellist käsku menüüs ei ole, vali uus.");
      break;
  }
}

void showCurrentPosition(){
  Serial.print("Olete hetkel positsioonil (");
  Serial.print(int(posi / 15.44));
  Serial.print(", ");
  Serial.print(tic.getCurrentPosition() / 4);
  Serial.println(")");
}

void changeStepperSpeed(){
  Serial.println("Sisesta soovitud samm-mootori kiirus (tavakiirus on 200, maksimaalne 400): ");
  int desired_speed = Serial.parseInt();
  tic.setTargetVelocity(desired_speed);
  Serial.print("Samm-mootori kiirus seatud ");
  Serial.print(desired_speed);
  Serial.println(" peale.");
}

void moveToPosition() {
  int xPosition, yPosition;
  
  Serial.println("Sisesta soovitud koordinaadid (x (0-544), y (0-350), nt. 100 200):");
  
  // Wait until input is received from the user
  while (Serial.available() == 0) {
    // Wait for input
  }
  
  // Read the input position from the user
  String input = Serial.readStringUntil('\n');
  sscanf(input.c_str(), "%d %d", &xPosition, &yPosition);

  // Constrain the values within the valid range
  xPosition = constrain(xPosition, 0, 544);
  nextposx = xPosition;
  yPosition = constrain(yPosition, 0, 350);

  Serial.print("Liigud positsioonile: [");
  Serial.print(xPosition);
  Serial.print(",");
  Serial.print(yPosition);
  Serial.println("]");

  // Convert x and y positions accordingly
  xPosition = int(15.52 * xPosition);
  yPosition = yPosition * 4;
  delay(2); // Small delay for stability
  
  if(currentposx != nextposx) {
    tic.setTargetPosition(yPosition);
    while(reachedTarget == false) {
    controlMotor(xPosition);
    }
  }

  currentpos = posi;
  currentposx = nextposx;
  Serial.println("Liikumine lõpetatud, ootan uut käsku.");
}

void penUp() {
  advance(80, E_leftz, M_leftz, true, 1000);
}

void penDown() {
  advance(80, E_leftz, M_leftz, false, 1000);
}

void controlMotor(int target){
  // PID constants

  float kp = 10;
  float kd = 200;
  float ki = 0.11;

  // time difference
  long currT = micros();
  float deltaT = ((float)(currT - prevT)) / (1.0e6);
  prevT = currT;

  // Read the position
  int pos = 0;
  noInterrupts(); // disable interrupts temporarily while reading
  pos = posi;
  interrupts();  // turn interrupts back on

  int e;
  e = target - pos;

  // Check if the motor has reached the target
  if (abs(e) < 9) { // Adjust this threshold according to your application
    reachedTarget = true;
  }

  // Stop the motor if the target is reached
  if (reachedTarget) {
    setMotor(0, 0, IN1, IN2);
    return; // Exit loop
  }

  float dedt = (e - eprev) / (deltaT);   // derivative
  eintegral = eintegral + e * deltaT;   // integral
  float u = kp * e + kd * dedt + ki * eintegral; 
  float pwr = fabs(u);  // motor power
  if (pwr > 255) {
    pwr = 255;
  }

  // motor direction
  int dir = 1;
  if (u < 0) {
    dir = -1;
  }
  
  setMotor(dir, pwr, IN1, IN2);
  
  // store previous error
  eprev = e;
}

void setMotor(int dir, int pwmVal, int in1, int in2) {
  // Set motor speed and direction
  analogWrite(IN1, pwmVal / 2);
  if (dir == 1) {
    digitalWrite(IN2, LOW);
    return;
  } else {
    digitalWrite(IN2, HIGH);
    return;
  }
}

void readEncoder() {
  int b = digitalRead(ENCB);
  if (b > 0) {
    posi++;
  } else {
    posi--;
  }
}
