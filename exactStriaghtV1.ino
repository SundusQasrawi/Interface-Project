#include <Wire.h>
#include <MPU6050_light.h>
#include <PID_v1.h>

// MPU6050
MPU6050 mpu(Wire);

float angle = 0.0, rawAngle = 0.0,prevAngle = 0.0;

// Motor Pins
#define motor1_in1 12
#define motor1_in2 14
#define motor2_in3 27
#define motor2_in4 26
#define ENA 13
#define ENB 25

// Encoder Pins
#define encoder1_a 4
#define ENCA_B 18
#define encoder2_a 19
#define ENCB_B 23

// Movement Parameters
#define wheel_circumference 150.0
#define pulses_per_revolution 210
#define cell_distance 180.0
#define pulses_per_cell (int)((cell_distance / wheel_circumference) * pulses_per_revolution)

// Variables
volatile int encoder1_count = 0;
volatile int encoder2_count = 0;
volatile int target_count = 0;

// MPU6050 Variables
float currentAngle = 0.0;
float targetAngle = 0.0;

int maxSpeed = 255;
int minSpeed = 160;
int leftSpeedVal;
int rightSpeedVal;

// Function Prototypes
void IRAM_ATTR encoder1_isr();
void IRAM_ATTR encoder2_isr();
void moveForwardOneCell();
void turnRight();
void stopMotors();
void forward();
void driving();
void controlSpeed();
void getAngel();

void setup() {
  // Serial Monitor
  Serial.begin(115200);

  // Motor and Encoder Pin Setup
  pinMode(motor1_in1, OUTPUT);
  pinMode(motor1_in2, OUTPUT);
  pinMode(motor2_in3, OUTPUT);
  pinMode(motor2_in4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(encoder1_a, INPUT_PULLUP);
  pinMode(encoder2_a, INPUT_PULLUP);

  // Attach Interrupts
  attachInterrupt(digitalPinToInterrupt(encoder1_a), encoder1_isr, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder2_a), encoder2_isr, RISING);

  
    // Initialize MPU6050
  Wire.begin();
  mpu.begin();
  mpu.calcOffsets();

  mpu.update();

  // Reset Encoder Counts
  encoder1_count = 0;
  encoder2_count = 0;
  target_count = pulses_per_cell;
  
}

void loop() {
  moveForwardOneCell();
  delay(2000);
}
  
void moveForward() {
  encoder1_count = 0;
  encoder2_count = 0;  // Reset encoder counts
  while ((encoder1_count < target_count) && (encoder2_count < target_count)) {
    digitalWrite(motor1_in1, HIGH);
    digitalWrite(motor1_in2, LOW);
    digitalWrite(motor2_in3, HIGH);
    digitalWrite(motor2_in4, LOW);
    analogWrite(ENA, 200);  // Adjust speed (0-255)
    analogWrite(ENB, 200);  // Adjust speed (0-255)
  }
  stopMotors();
  Serial.println("Moved one cell forward.");
  delay(500);  // Small delay before turning
}
  
void moveForwardOneCell() {
  targetAngle = -mpu.getAngleZ(); 
  Serial.print("Angel at start of moving: "); 
  Serial.println(targetAngle);
  encoder1_count = 0;
  encoder2_count = 0; // Reset encoder counts

  leftSpeedVal = 200; // Initial motor speed
  rightSpeedVal = 200;

  while ((encoder1_count < target_count) && (encoder2_count < target_count)) {
    driving();
  }

  stopMotors();
  Serial.println("Moved one cell forward. Counts:");
  Serial.print("Ecoder 1 count");
  Serial.println(encoder1_count);
  Serial.print("Ecoder 2 count");
  Serial.println(encoder2_count);
  delay(500); // Stabilize before the next action
}



void driving() {
  mpu.update();
  currentAngle = -mpu.getAngleZ();
  int deltaAngle = round(targetAngle - currentAngle);
 

  if (deltaAngle != 0) {
    controlSpeed();
     forward();
    analogWrite(ENA, leftSpeedVal);
    analogWrite(ENB, rightSpeedVal);
  }
}

void controlSpeed() {
  int deltaAngle = round(targetAngle - currentAngle);
  Serial.print("Current angel: "); 
  Serial.println(currentAngle); 
  Serial.print("Taget angel: "); 
  Serial.println(targetAngle); 
  Serial.print("Delta angel: "); 
  Serial.println(deltaAngle); 
  int targetGyroZ;

  if (deltaAngle > 30) {
    targetGyroZ = 60;
  } else if (deltaAngle < -30) {
    targetGyroZ = -60;
  } else {
    targetGyroZ = 2 * deltaAngle;
  }
  Serial.print("target Gyro z: ");
  Serial.println(targetGyroZ);
  if (round(targetGyroZ) > 0) {
    leftSpeedVal = max(leftSpeedVal - 1, minSpeed); // Adjust to increase speed
  } else {
    leftSpeedVal = min(leftSpeedVal + 1, maxSpeed); // Adjust to decrease speed
  }
  rightSpeedVal = maxSpeed; // Right motor runs at full speed
  
  Serial.print("target Gyro z: ");
  Serial.println(targetGyroZ);
}

void forward() {
  digitalWrite(motor1_in1, HIGH);
  digitalWrite(motor1_in2, LOW);
  digitalWrite(motor2_in3, HIGH);
  digitalWrite(motor2_in4, LOW);
}
///////////////////////////////////////////////////////////////////////////

///// this part is correct 
const float correctionFactor = 2.3; // Adjust this value as needed
const int baseSpeed = 220; // Adjust the base motor speed as needed

void turnRight() {
  Wire.begin();
  mpu.begin();
  mpu.calcOffsets();  // Initial sensor calibration
  mpu.update();
  
  rawAngle = mpu.getAngleZ();
  angle = abs(rawAngle);

  float targetAngle = prevAngle + 60.0; // Calculate the target angle
  
  while (angle < targetAngle) {
    mpu.update();
    rawAngle = mpu.getAngleZ();
    angle = abs(rawAngle);

    Serial.print("Angle: ");
    Serial.println(angle);

    // Calculate the correction factor
    float correction = (targetAngle - angle) * correctionFactor;
    

    // Turn Right
    digitalWrite(motor1_in1, HIGH);
    digitalWrite(motor1_in2, LOW);
    digitalWrite(motor2_in3, LOW);
    digitalWrite(motor2_in4, HIGH);
    analogWrite(ENA, max((int)(baseSpeed + correction),100));
    analogWrite(ENB, max((int)(baseSpeed - correction),100));

  }
  
  stopMotors();
  
  prevAngle = angle; // Update the previous angle
}


void stopMotors() {
  digitalWrite(motor1_in1, LOW);
  digitalWrite(motor1_in2, LOW);
  digitalWrite(motor2_in3, LOW);
  digitalWrite(motor2_in4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

void IRAM_ATTR encoder1_isr() {
  encoder1_count++;
}

void IRAM_ATTR encoder2_isr() {
  encoder2_count++;
}
