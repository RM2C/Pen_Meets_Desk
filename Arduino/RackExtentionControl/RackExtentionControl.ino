#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

// --- Constant Definitions ---
#define SPEED 250         // Maximum motor speed
#define ENCODERPIN1_1 2   // Pin for Encoder A output
#define ENCODERPIN1_2 4   // Pin for Encoder B output
#define DEAD_ZONE 4       // Dead zone for stopping the motor (errors within this range are ignored)

// --- Motor control pins ---
const int DIR_PIN = 12;   // Pin to control rotation direction
const int BRAKE_PIN = 9;  // Pin to control the brake
const int PWM_PIN = 3;    // Pin to control spped via PWM

// --- Global variables ---
Encoder myEnc(ENCODERPIN1_1, ENCODERPIN1_2);

long pos = 0;       // Current encoder position
long targetPos = 0; // Target encoder position

// Array for the oving average filter
int speeds[100];
const int speeds_len = 10;
int speeds_index = 0;

// Variables for the trapezoidal velocity profile 
long move_startingPos = 0; // Position at which the current movement started

// --- Function to compute the moving average ---
int moving_average(int data) {
  speeds[speeds_index] = data;
  speeds_index++;
  if (speeds_index >= speeds_len) {
    speeds_index = 0;
  }

  long sum = 0; // Use long type since the sum can become large
  for (int i = 0; i < speeds_len; i++) {
    sum += speeds[i];
  }
  return sum / speeds_len;
}

void setup() {
  Serial.begin(115200);

  // Set motor control pins as outputs
  pinMode(DIR_PIN, OUTPUT);
  pinMode(BRAKE_PIN, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);

  // Initialize the moving average array with zeros
  for (int i = 0; i < speeds_len; i++) {
    speeds[i] = 0;
  }


  // --- Motor initial positioning (homing) ---
  // Determine the origin by pressing the motor against a physical stop (e.g., a wall)
  // Set to HIGH to match the rotation direction used in the loop
  digitalWrite(DIR_PIN, HIGH);
  digitalWrite(BRAKE_PIN, LOW);
  analogWrite(PWM_PIN, 80);    // Rotate at low speed

  long time = millis();
  long lastPos = myEnc.read();

  while (true) {
    long currentPos = myEnc.read();
    // Reset the timer if the encoder value changes
    if (currentPos != lastPos) {
      lastPos = currentPos;
      time = millis();
    }
    // If there is no change in the encoder value for more than 100 ms, assume the motor has stopped
    if (millis() - time > 100) {
      digitalWrite(BRAKE_PIN, HIGH); // Activate brake
      analogWrite(PWM_PIN, 0);       // Stop PWM output
      myEnc.write(0);                // Reset the current position to the origin (0)
      break;
    }
  }

  Serial.println("Initialized");
  pos = myEnc.read();
  Serial.println(pos);
}

void loop() {
  // Read the current encoder position
  pos = myEnc.read();

  // Receive the target position from serial communication (semicolon-separated)
  if (Serial.available() > 0) {
    String str = Serial.readStringUntil(';');
    long newTargetPos = str.toInt();
    // Limit the movable range (from 0 to -1800)
    newTargetPos = constrain(newTargetPos, -2500, 0);

    // If a new target value is set, start a new movement
    if (newTargetPos != targetPos) {
        targetPos = newTargetPos;
        move_startingPos = pos; // Record the current position as the movement start position
    }
  }

  // --- Motor control ---
  // Calculate the difference between the target position and the current position
  int diff = targetPos - pos;

  // Drive the motor only if the difference exceeds the dead zone
  if (abs(diff) > DEAD_ZONE) {
    // 1. Determine rotation direction
    if (diff > 0) {
      digitalWrite(DIR_PIN, HIGH); // Forward rotation
    } else {
      digitalWrite(DIR_PIN, LOW);  // Reverse rotation
    }
    digitalWrite(BRAKE_PIN, LOW); // Release brake

    // Calculate speed using a trapezoidal velocity profile
    const int ACCEL_DECEL_DISTANCE = 400; // Distance used for acceleration and decceleration
    const int MIN_SPEED = 60;             // Minimum speed

    long distance_traveled = abs(pos - move_startingPos); // Distance traveled since movement started
    long distance_remaining = abs(targetPos - pos);      // Remaining distance to the target
    
    int changeSpeed = SPEED; // Default is maximum speed (constant velocity phase)

    // Deceleration phase: when approaching the target position
    if (distance_remaining < ACCEL_DECEL_DISTANCE) {
      changeSpeed = map(distance_remaining, DEAD_ZONE, ACCEL_DECEL_DISTANCE, MIN_SPEED, SPEED);
    } 
    // Acceleration phase: immediately after movement starts
    else if (distance_traveled < ACCEL_DECEL_DISTANCE) {
      changeSpeed = map(distance_traveled, 0, ACCEL_DECEL_DISTANCE, MIN_SPEED, SPEED);
    }
    
    changeSpeed = constrain(changeSpeed, MIN_SPEED, SPEED); // Constrain the speed between MIN_SPEED and SPEED

    // 3. Smooth the speed using a moving average
    int smoothedSpeed = moving_average(changeSpeed);
    
    // 4. Output to the motor
    analogWrite(PWM_PIN, smoothedSpeed);

  } else {
    // Apply the brake once the target position is reached
    digitalWrite(BRAKE_PIN, HIGH);
    analogWrite(PWM_PIN, 0);
    // Reset the moving average array when stopped
    for (int i = 0; i < speeds_len; i++) {
        speeds[i] = 0;
    }
  }

  // Output debug information every 100 ms
  static unsigned long lastPrintTime = 0;
  if (millis() - lastPrintTime > 100) {
    Serial.print("Target: "); Serial.print(targetPos);
    Serial.print(", Current: "); Serial.print(pos);
    Serial.print(", Diff: "); Serial.print(diff);
    Serial.print(", Speed: "); Serial.println(analogRead(PWM_PIN));
    lastPrintTime = millis();
  }
}

