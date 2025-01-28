#include <Wire.h>
#include <MPU6050_light.h>


float angle = 0.0, rawAngle = 0.0,prevAngle = 0.0;
// MPU6050
MPU6050 mpu(Wire);

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

// Function Prototypes
void IRAM_ATTR encoder1_isr();
void IRAM_ATTR encoder2_isr();
void moveForwardOneCell();
void turnRight();
void stopMotors();

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
  //Wire.begin();
  //mpu.begin();
  //mpu.calcOffsets(); // Calibrate MPU
  //Serial.println("MPU6050 Initialized and Calibrated");

  // Reset Encoder Counts
  encoder1_count = 0;
  encoder2_count = 0;
  target_count = pulses_per_cell;
}

void loop() {
  moveForwardOneCell();
  moveForwardOneCell();
  turnRight();
  delay(700);
}

void moveForwardOneCell() {
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

void turn() {
  mpu.update();  // Update MPU readings
  prevAngle = -mpu.getAngleZ();  // Get the current Z angle
  targetAngle = prevAngle + 90.0;  // Set the target to 90 degrees more
  Serial.print("Starting Turn. Current Angle: ");
  Serial.println(prevAngle);

  float tolerance = 2.0;  // Allowable error in degrees
while (currentAngle < targetAngle - tolerance) {
    mpu.update();  // Update MPU readings
    currentAngle = -mpu.getAngleZ();
    Serial.print("Current Angle: ");
    Serial.println(currentAngle);

    // Turn Right
    digitalWrite(motor1_in1, HIGH);
    digitalWrite(motor1_in2, LOW);
    digitalWrite(motor2_in3, LOW);
    digitalWrite(motor2_in4, HIGH);
    analogWrite(ENA, 100);
    analogWrite(ENB, 100);
    delay(500);
  }

  stopMotors();
  Serial.println("Turn completed.");
}



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
