#define motor1_in1 13
#define motor1_in2 12
#define motor2_in3 14
#define motor2_in4 27
#define ENA 26
#define ENB 25

// Encoder pins
#define encoder1_a 4
#define ENCA_B 18
#define encoder2_a 19
#define ENCB_B 23

#define wheel_circumference 150.0  // heel circumference in mm
#define pulses_per_revolution 210
#define cell_distance 180.0  // distance for one cell in mm
#define pulses_per_cell (int)((cell_distance / wheel_circumference) * pulses_per_revolution)

// Track encoder pulses for each motor
volatile int encoder1_count = 0;
volatile int encoder2_count = 0;

// Target encoder count to stop the motors
volatile int target_count = 0;

// Function prototypes
void IRAM_ATTR encoder1_isr();
void IRAM_ATTR encoder2_isr();
void moveForwardOneCell();
void stopMotors();

void setup() {
  // Reset encoder counts
  encoder1_count = 0;
  encoder2_count = 0;
  target_count = pulses_per_cell;

  // Motor and encoder pin setup
  pinMode(motor1_in1, OUTPUT);
  pinMode(motor1_in2, OUTPUT);
  pinMode(motor2_in3, OUTPUT);
  pinMode(motor2_in4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  // Encoder pins setup
  pinMode(encoder1_a, INPUT_PULLUP);
  pinMode(encoder2_a, INPUT_PULLUP);

  // Attach interrupts for encoders
  attachInterrupt(digitalPinToInterrupt(encoder1_a), encoder1_isr, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder2_a), encoder2_isr, RISING);

  // Initialize Serial Monitor
  Serial.begin(9600);
}

void loop() {
  if ((encoder1_count < target_count) && (encoder2_count < target_count)) {
    moveForwardOneCell();
  } else {
    stopMotors();
    Serial.print("Moved one cell forward. Encoder1: ");
    Serial.print(encoder1_count);
    Serial.print(", Encoder2: ");
    Serial.println(encoder2_count);
    // Reset for next movement after a delay
      // Wait for 5 seconds
    delay(5000);
    encoder1_count = 0;
    encoder2_count = 0;
  }
}

void moveForwardOneCell() {
  digitalWrite(motor1_in1, HIGH);
  digitalWrite(motor1_in2, LOW);
  digitalWrite(motor2_in3, HIGH);
  digitalWrite(motor2_in4, LOW);
  analogWrite(ENA, 200);  // Adjust speed (0-255)
  analogWrite(ENB, 200);  // Adjust speed (0-255)
  
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
