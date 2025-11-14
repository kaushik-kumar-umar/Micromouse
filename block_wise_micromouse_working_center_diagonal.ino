#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <queue>

// Maze Constants
#define MAZE_SIZE 18
#define CENTER_X 9
#define CENTER_Y 9

// I2C Multiplexer Address
#define TCAADDR 0x70

// Motor driver pins
#define LEFT_MOTOR_A 26
#define LEFT_MOTOR_B 25
#define LEFT_PWM 19
#define RIGHT_MOTOR_A 27
#define RIGHT_MOTOR_B 14
#define RIGHT_PWM 18
#define distcheck 140
// Encoder pins
#define LEFT_ENCODER_A 13
#define RIGHT_ENCODER_A 15

// Movement Constants
#define CELL_DISTANCE 620
#define TURN_90_DEGREES 140
#define TURN_90_DEGREES_R 140
#define BASE_SPEED_INIT 140
#define TURN_SPEED 180
#define TURN_180_DEGREES 400        // 180 turn encoder ticks
#define TURN_180_DEGREES_set 170
#define TURN_90_DEGREES_set 40
// Wall Detection Thresholds
#define WALL_THRESHOLD_SIDE 140
#define WALL_THRESHOLD_FRONT 150
#define LEFT_WALL_SETPOINT 55

// Wall following PID constants
#define Kp 0.4
#define Ki 0.0
#define Kd 1.8
#define INTEGRAL_LIMIT 20

// Distance PID constants
#define Kp_dist 0.6
#define Ki_dist 0.0
#define Kd_dist 0.8

// PWM setup
#define PWM_FREQ 5000
#define PWM_RES 8
#define LEFT_PWM_CHANNEL 0
#define RIGHT_PWM_CHANNEL 1
// Speed profile parameters
#define MIN_SPEED 80                // Starting/ending speed
#define MAX_SPEED 140               // Cruising speed
#define ACCEL_DISTANCE 200          // Encoder ticks to accelerate
#define DECEL_DISTANCE 200          // Encoder ticks to decelerate
int rightThres=55;
int leftThres=55;

// Cell Structure
struct Cell {
    int x;
    int y;
    bool northWall;
    bool eastWall;
    bool southWall;
    bool westWall;
    int floodValue;
};

// Global Variables
Cell maze[MAZE_SIZE][MAZE_SIZE];
int currX = 17; 
int currY = 17;
int nextX = 17;
int nextY = 17;
int currDirection = 0; // START FACING NORTH

// Encoder Variables
volatile long leftCount = 0;
volatile long rightCount = 0;
bool leftMovingForward = true;
bool rightMovingForward = true;

// Wall following PID variables
float prevError = 0;
float integral = 0;

// Distance PID variables
float integral_dist = 0;
float previousError_dist = 0;

// Sensor Variables
Adafruit_VL53L0X sensorLeft = Adafruit_VL53L0X();
Adafruit_VL53L0X sensorFront = Adafruit_VL53L0X();
Adafruit_VL53L0X sensorRight = Adafruit_VL53L0X();
int leftDistance = 0;
int frontDistance = 0;
int rightDistance = 0;

// Current base speed
float BASE_SPEED = BASE_SPEED_INIT;
int cycleCount = 0;
bool movingForward = true;


#define button 2

// Function Declarations
void IRAM_ATTR leftEncoderISR();
void IRAM_ATTR rightEncoderISR();
void tcaSelect(uint8_t channel);
void initializeSensors();
void readSensors();
void updateDistancePID(long targetCount, long currentCount);
void forwardOneCell();
void turn90DegreesRight();
void turn90DegreesLeft();
void turn180Degrees();
void stopMotors();
void initializeMaze();
void resetMaze();
void setDestination();
bool isValidNeighbor(int x, int y);
void floodFill();
void findNextCell();
void setWall(int x, int y, int dir);
bool isTopWall();
bool isLeftWall();
bool isRightWall();
void setWalls();
void nextTurn();
void printMaze();
void printDebugInfo();

// Encoder ISRs
void IRAM_ATTR leftEncoderISR() {
    if (leftMovingForward) leftCount++;
    else leftCount--;
}

void IRAM_ATTR rightEncoderISR() {
    if (rightMovingForward) rightCount++;
    else rightCount--;
}

void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("\n\n=== Micromouse Starting ===");
    
    Wire.begin();

    // Motor setup
    pinMode(LEFT_MOTOR_A, OUTPUT);
    pinMode(LEFT_MOTOR_B, OUTPUT);
    pinMode(RIGHT_MOTOR_A, OUTPUT);
    pinMode(RIGHT_MOTOR_B, OUTPUT);
    pinMode(LEFT_PWM, OUTPUT);
    pinMode(RIGHT_PWM, OUTPUT);
    pinMode(button,INPUT);

    // Encoder setup
    pinMode(LEFT_ENCODER_A, INPUT_PULLUP);
    pinMode(RIGHT_ENCODER_A, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A), leftEncoderISR, RISING);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A), rightEncoderISR, RISING);

    // PWM setup
    ledcSetup(LEFT_PWM_CHANNEL, PWM_FREQ, PWM_RES);
    ledcSetup(RIGHT_PWM_CHANNEL, PWM_FREQ, PWM_RES);
    ledcAttachPin(LEFT_PWM, LEFT_PWM_CHANNEL);
    ledcAttachPin(RIGHT_PWM, RIGHT_PWM_CHANNEL);

    // Initialize sensors
    Serial.println("Initializing sensors...");
    initializeSensors();
    Serial.println("Sensors ready!");
    while (digitalRead(button) == LOW){
        delay(50);
    }
    readSensors();
    rightThres = rightDistance;
    leftThres = leftDistance;
    while (digitalRead(button) == HIGH){
        delay(50);
    }
     while (digitalRead(button) == LOW){
        delay(50);
    }

    // Initialize maze
    initializeMaze();
    setDestination();
    
    Serial.println("Taking initial sensor readings...");
    delay(500);
    readSensors();
    printDebugInfo();
    
    // Set initial walls
    setWalls();
    
    // Initial flood fill
    floodFill();
    findNextCell();
    printMaze();
    
    Serial.println("\nStarting position:");
    Serial.print("Current: ("); Serial.print(currX); Serial.print(","); Serial.print(currY); Serial.println(")");
    Serial.print("Next: ("); Serial.print(nextX); Serial.print(","); Serial.print(nextY); Serial.println(")");
    Serial.print("Direction: "); Serial.println(currDirection);
    
    delay(2000);
    Serial.println("\n=== STARTING MOVEMENT ===\n");
    
    // Make initial turn decision
    nextTurn();
}

void loop() {
    // Check if we've reached the center
    if (maze[currX][currY].floodValue == 0) {
        stopMotors();
        Serial.println("\n!!! CENTER REACHED !!!");
        printMaze();
        while(1) delay(1000);
    }
    
    Serial.println("\n--- Moving Forward ---");
    // Move forward one cell with PID control
    // forwardOneCell();
    
    // Update position
    // switch(currDirection) {
    //     case 0: currX--; break;
    //     case 1: currY++; break;
    //     case 2: currX++; break;
    //     case 3: currY--; break;
    // }
    
    // Serial.print("New Position: (");
    // Serial.print(currX);
    // Serial.print(", ");
    // Serial.print(currY);
    // Serial.print(") Direction: ");
    // Serial.println(currDirection);
    
    // // Read walls and update maze
    // readSensors();
    // printDebugInfo();
    // setWalls();
    
    // // Re-flood fill and find next move
    // resetMaze();
    // floodFill();
    // findNextCell();
    // Serial.print("Next target: (");
    // Serial.print(nextX);
    // Serial.print(",");
    // Serial.print(nextY);
    // Serial.println(")");
    
    // Decide next turn
    nextTurn();
    
    delay(100);
}

void printDebugInfo() {
    Serial.print("Sensors - L:");
    Serial.print(leftDistance);
    Serial.print("mm R:");
    Serial.print(rightDistance);
    Serial.print("mm F:");
    Serial.print(frontDistance);
    Serial.println("mm");
}

void initializeSensors() {
    tcaSelect(0);
    if (!sensorRight.begin()) {
        Serial.println("ERROR: Failed to start RIGHT sensor on channel 0");
        while(1) delay(1000);
    }
    Serial.println("Right sensor OK");
    
    tcaSelect(1);
    if (!sensorLeft.begin()) {
        Serial.println("ERROR: Failed to start LEFT sensor on channel 1");
        while(1) delay(1000);
    }
    Serial.println("Left sensor OK");

    tcaSelect(2);
    if (!sensorFront.begin()) {
        Serial.println("ERROR: Failed to start FRONT sensor on channel 2");
        while(1) delay(1000);
    }
    Serial.println("Front sensor OK");
    
    // Set faster measurement timing
    tcaSelect(0);
    sensorRight.setMeasurementTimingBudgetMicroSeconds(20000);
    tcaSelect(1);
    sensorLeft.setMeasurementTimingBudgetMicroSeconds(20000);
    tcaSelect(2);
    sensorFront.setMeasurementTimingBudgetMicroSeconds(20000);
}

void tcaSelect(uint8_t channel) {
    Wire.beginTransmission(TCAADDR);
    Wire.write(1 << channel);
    Wire.endTransmission();
}

void readSensors() {
    VL53L0X_RangingMeasurementData_t measure;

    tcaSelect(0);
    sensorRight.rangingTest(&measure, false);
    if (measure.RangeStatus != 4) {
        rightDistance = measure.RangeMilliMeter;
    }
    
    tcaSelect(1);
    sensorLeft.rangingTest(&measure, false);
    if (measure.RangeStatus != 4) {
        leftDistance = measure.RangeMilliMeter;
    }

    tcaSelect(2);
    sensorFront.rangingTest(&measure, false);
    if (measure.RangeStatus != 4) {
        frontDistance = measure.RangeMilliMeter;
    }
}

// void updateDistancePID(long targetCount, long currentCount) {
//     float error = targetCount - currentCount;
//     integral_dist += error;
//     float derivative = error - previousError_dist;
//     previousError_dist = error;

//     BASE_SPEED = (Kp_dist * error) + (Ki_dist * integral_dist) + (Kd_dist * derivative);
//     BASE_SPEED = constrain(BASE_SPEED, 80, 180);
// }

void updateDistancePID(long targetCount, long currentCount) {
    // Trapezoidal speed profile for smooth movement
    long distanceRemaining = targetCount - currentCount;
    
    if (currentCount < ACCEL_DISTANCE) {
        // Acceleration phase - ramp up smoothly
        float accelRatio = (float)currentCount / ACCEL_DISTANCE;
        BASE_SPEED = MIN_SPEED + (MAX_SPEED - MIN_SPEED) * accelRatio;
    } 
    else if (distanceRemaining < DECEL_DISTANCE) {
        // Deceleration phase - slow down smoothly
        float decelRatio = (float)distanceRemaining / DECEL_DISTANCE;
        BASE_SPEED = MIN_SPEED + (MAX_SPEED - MIN_SPEED) * decelRatio;
    } 
    else {
        // Cruise phase - maintain max speed
        BASE_SPEED = MAX_SPEED;
    }
    
    BASE_SPEED = constrain(BASE_SPEED, MIN_SPEED, MAX_SPEED);
}


void backwardOneCell() {
    // Reset for new cell
    leftCount = 0;
    rightCount = 0;
    prevError = 0;
    integral = 0;
    integral_dist = 0;
    previousError_dist = 0;
    BASE_SPEED = BASE_SPEED_INIT;
    
    leftMovingForward = true;
    rightMovingForward = true;
    
    unsigned long startTime = millis();
    unsigned long lastPrint = millis();
    
    Serial.println("Moving...");
    
    // Move one cell with PID control
    while (leftCount < TURN_180_DEGREES_set && rightCount < TURN_180_DEGREES_set) {
        long avgCount = (leftCount + rightCount) / 2;
        updateDistancePID(TURN_180_DEGREES_set, avgCount);
        
        readSensors();
        
        // Print progress every 300ms
        // if (millis() - lastPrint > 300) {
        //     Serial.print("  Progress: ");
        //     Serial.print((avgCount * 100) / CELL_DISTANCE);
        //     Serial.print("% | Encoders L:");
        //     Serial.print(leftCount);
        //     Serial.print(" R:");
        //     Serial.print(rightCount);
        //     Serial.print(" | Speed:");
        //     Serial.print((int)BASE_SPEED);
        //     Serial.print(" | Walls L:");
        //     Serial.print(leftDistance);
        //     Serial.print(" R:");
        //     Serial.print(rightDistance);
        //     Serial.println("mm");
        //     lastPrint = millis();
        // }
        
        int leftSpeed, rightSpeed;
        
        // Wall following logic
        if (leftDistance > distcheck && rightDistance > distcheck) {
            // No walls - go straight
            leftSpeed = BASE_SPEED;
            rightSpeed = BASE_SPEED;
        } else if (rightDistance < distcheck && leftDistance > distcheck) {
            // Right wall only
            float error = rightDistance - rightThres;
            integral = constrain(integral + error, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
            float derivative = error - prevError;
            prevError = error;
            float correction = (Kp * error) + (Ki * integral) + (Kd * derivative);
            leftSpeed = BASE_SPEED - correction;
            rightSpeed = BASE_SPEED + correction;
        } else if (leftDistance < (distcheck-10)) {
            // Left wall (preferred)
            float error = leftDistance - leftThres;
            integral = constrain(integral + error, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
            float derivative = error - prevError;
            prevError = error;
            float correction = (Kp * error) + (Ki * integral) + (Kd * derivative);
            leftSpeed = BASE_SPEED + correction;
            rightSpeed = BASE_SPEED - correction;
        } else {
            // Default
            leftSpeed = BASE_SPEED;
            rightSpeed = BASE_SPEED;
        }
        
        leftSpeed = constrain(leftSpeed, 80, 140);
        rightSpeed = constrain(rightSpeed, 80, 140);
        
        // Apply motor commands
        digitalWrite(LEFT_MOTOR_A, LOW);
        digitalWrite(LEFT_MOTOR_B, HIGH);
        digitalWrite(RIGHT_MOTOR_A, LOW);
        digitalWrite(RIGHT_MOTOR_B, HIGH);
        ledcWrite(LEFT_PWM_CHANNEL, leftSpeed);
        ledcWrite(RIGHT_PWM_CHANNEL, rightSpeed);
        
        delay(100);
        
        // Safety timeout (5 seconds)
        // if (millis() - startTime > 5000) {
        //     Serial.println("\n⚠ TIMEOUT! Movement took too long.");
        //     break;
        // }
    }
    
    unsigned long movementTime = millis() - startTime;
    
    stopMotors();
    delay(30);
    
    // Print final statistics
    Serial.println("\n  ✓ Cell movement complete!");
    Serial.print("  Time taken: "); Serial.print(movementTime); Serial.println("ms");
    Serial.print("  Final encoder counts - L:"); Serial.print(leftCount);
    Serial.print(" R:"); Serial.println(rightCount);
    Serial.print("  Average: "); Serial.println((leftCount + rightCount) / 2);
    Serial.print("  Difference: "); Serial.println(abs(leftCount - rightCount));
}


void backwardOneCell_turn() {
    // Reset for new cell
    leftCount = 0;
    rightCount = 0;
    prevError = 0;
    integral = 0;
    integral_dist = 0;
    previousError_dist = 0;
    BASE_SPEED = BASE_SPEED_INIT;
    
    leftMovingForward = true;
    rightMovingForward = true;
    
    unsigned long startTime = millis();
    unsigned long lastPrint = millis();
    
    Serial.println("Moving...");
    
    // Move one cell with PID control
    while (leftCount < TURN_90_DEGREES_set && rightCount < TURN_90_DEGREES_set) {
        long avgCount = (leftCount + rightCount) / 2;
        updateDistancePID(TURN_90_DEGREES_set, avgCount);
        
        readSensors();
        
        // Print progress every 300ms
        // if (millis() - lastPrint > 300) {
        //     Serial.print("  Progress: ");
        //     Serial.print((avgCount * 100) / CELL_DISTANCE);
        //     Serial.print("% | Encoders L:");
        //     Serial.print(leftCount);
        //     Serial.print(" R:");
        //     Serial.print(rightCount);
        //     Serial.print(" | Speed:");
        //     Serial.print((int)BASE_SPEED);
        //     Serial.print(" | Walls L:");
        //     Serial.print(leftDistance);
        //     Serial.print(" R:");
        //     Serial.print(rightDistance);
        //     Serial.println("mm");
        //     lastPrint = millis();
        // }
        
        int leftSpeed, rightSpeed;
        
        // Wall following logic
        if (leftDistance > distcheck && rightDistance > distcheck) {
            // No walls - go straight
            leftSpeed = BASE_SPEED;
            rightSpeed = BASE_SPEED;
        } else if (rightDistance < distcheck && leftDistance > distcheck) {
            // Right wall only
            float error = rightDistance - rightThres;
            integral = constrain(integral + error, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
            float derivative = error - prevError;
            prevError = error;
            float correction = (Kp * error) + (Ki * integral) + (Kd * derivative);
            leftSpeed = BASE_SPEED - correction;
            rightSpeed = BASE_SPEED + correction;
        } else if (leftDistance < (distcheck-10)) {
            // Left wall (preferred)
            float error = leftDistance - leftThres;
            integral = constrain(integral + error, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
            float derivative = error - prevError;
            prevError = error;
            float correction = (Kp * error) + (Ki * integral) + (Kd * derivative);
            leftSpeed = BASE_SPEED + correction;
            rightSpeed = BASE_SPEED - correction;
        } else {
            // Default
            leftSpeed = BASE_SPEED;
            rightSpeed = BASE_SPEED;
        }
        
        leftSpeed = constrain(leftSpeed, 80, 140);
        rightSpeed = constrain(rightSpeed, 80, 140);
        
        // Apply motor commands
        digitalWrite(LEFT_MOTOR_A, LOW);
        digitalWrite(LEFT_MOTOR_B, HIGH);
        digitalWrite(RIGHT_MOTOR_A, LOW);
        digitalWrite(RIGHT_MOTOR_B, HIGH);
        ledcWrite(LEFT_PWM_CHANNEL, leftSpeed);
        ledcWrite(RIGHT_PWM_CHANNEL, rightSpeed);
        
        delay(100);
        
        // Safety timeout (5 seconds)
        // if (millis() - startTime > 5000) {
        //     Serial.println("\n⚠ TIMEOUT! Movement took too long.");
        //     break;
        // }
    }
    
    unsigned long movementTime = millis() - startTime;
    
    stopMotors();
    delay(30);
    
    // Print final statistics
    Serial.println("\n  ✓ Cell movement complete!");
    Serial.print("  Time taken: "); Serial.print(movementTime); Serial.println("ms");
    Serial.print("  Final encoder counts - L:"); Serial.print(leftCount);
    Serial.print(" R:"); Serial.println(rightCount);
    Serial.print("  Average: "); Serial.println((leftCount + rightCount) / 2);
    Serial.print("  Difference: "); Serial.println(abs(leftCount - rightCount));
}

void forwardOneCell() {
    Serial.println("Starting forward movement...");
    
    // Reset for new cell
    leftCount = 0;
    rightCount = 0;
    prevError = 0;
    integral = 0;
    integral_dist = 0;
    previousError_dist = 0;
    BASE_SPEED = BASE_SPEED_INIT;
    
    leftMovingForward = true;
    rightMovingForward = true;
    
    unsigned long startTime = millis();
    unsigned long lastPrint = 0;
    
    // Move one cell with PID control
    while (leftCount < CELL_DISTANCE && rightCount < CELL_DISTANCE) {
        long avgCount = (leftCount + rightCount) / 2;
        updateDistancePID(CELL_DISTANCE, avgCount);
        
        readSensors();
        
        // Print progress every 200ms
        // if (millis() - lastPrint > 200) {
        //     Serial.print("Enc L:");
        //     Serial.print(leftCount);
        //     Serial.print(" R:");
        //     Serial.print(rightCount);
        //     Serial.print(" | Speed:");
        //     Serial.print((int)BASE_SPEED);
        //     Serial.print(" | Dist L:");
        //     Serial.print(leftDistance);
        //     Serial.print(" R:");
        //     Serial.println(rightDistance);
        //     lastPrint = millis();
        // }
        
        int leftSpeed, rightSpeed;
        
        // Wall following logic
        if (leftDistance > distcheck && rightDistance > distcheck) {
            // No walls - go straight
            leftSpeed = BASE_SPEED;
            rightSpeed = BASE_SPEED;
        } else if (rightDistance < 100 && leftDistance > 100) {
            // Right wall only
            float error = rightDistance - rightThres;
            integral = constrain(integral + error, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
            float derivative = error - prevError;
            prevError = error;
            float correction = (Kp * error) + (Ki * integral) + (Kd * derivative);
            leftSpeed = BASE_SPEED + correction;
            rightSpeed = BASE_SPEED - correction;
        } else if (leftDistance < 90) {
            // Left wall (prefer this)
            float error = leftDistance - leftThres;
            integral = constrain(integral + error, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
            float derivative = error - prevError;
            prevError = error;
            float correction = (Kp * error) + (Ki * integral) + (Kd * derivative);
            leftSpeed = BASE_SPEED - correction;
            rightSpeed = BASE_SPEED + correction;
        } else {
            // Default
            leftSpeed = BASE_SPEED;
            rightSpeed = BASE_SPEED;
        }
        
        leftSpeed = constrain(leftSpeed, 70, 130);
        rightSpeed = constrain(rightSpeed, 70, 130);
        
        digitalWrite(LEFT_MOTOR_A, HIGH);
        digitalWrite(LEFT_MOTOR_B, LOW);
        digitalWrite(RIGHT_MOTOR_A, HIGH);
        digitalWrite(RIGHT_MOTOR_B, LOW);
        ledcWrite(LEFT_PWM_CHANNEL, leftSpeed);
        ledcWrite(RIGHT_PWM_CHANNEL, rightSpeed);
        
        delay(10);
        
        // Safety timeout
        // if (millis() - startTime > 5000) {
        //     Serial.println("TIMEOUT - stopping");
        //     break;
        // }
    }
    
    Serial.print("Movement complete. L:");
    Serial.print(leftCount);
    Serial.print(" R:");
    Serial.println(rightCount);
    
    stopMotors();
    delay(30);
}

void turn90DegreesRight() {
    Serial.println(">>> Turning RIGHT");
    leftCount = 0;
    rightCount = 0;
    leftMovingForward = true;
    rightMovingForward = false;

    // readSensors();
    // if(frontDistance < 150){
    // while(frontDistance > 30){
    // digitalWrite(LEFT_MOTOR_A, HIGH);
    // digitalWrite(LEFT_MOTOR_B, LOW);
    // digitalWrite(RIGHT_MOTOR_A, HIGH);
    // digitalWrite(RIGHT_MOTOR_B, LOW);
    // ledcWrite(LEFT_PWM_CHANNEL, BASE_SPEED_INIT);
    // ledcWrite(RIGHT_PWM_CHANNEL, BASE_SPEED_INIT);
    // readSensors();
    // delay(10);
    // }}
    // stopMotors();
    // delay(50);
    // leftCount = 0;
    // rightCount = 0;
    
    digitalWrite(LEFT_MOTOR_A, HIGH);
    digitalWrite(LEFT_MOTOR_B, LOW);
    digitalWrite(RIGHT_MOTOR_A, LOW);
    digitalWrite(RIGHT_MOTOR_B, HIGH);
    ledcWrite(LEFT_PWM_CHANNEL, TURN_SPEED);
    ledcWrite(RIGHT_PWM_CHANNEL, TURN_SPEED);
    
    while(leftCount < TURN_90_DEGREES_R || abs(rightCount) < TURN_90_DEGREES_R) {
        delay(5);
    }
    
    stopMotors();
    delay(200);
    backwardOneCell_turn();
    delay(50);
    currDirection = (currDirection + 1) % 4;
    Serial.print("Turn complete. New direction: ");
    Serial.println(currDirection);
}

void turn90DegreesLeft() {
    Serial.println(">>> Turning LEFT");
    leftCount = 0;
    rightCount = 0;
    leftMovingForward = false;
    rightMovingForward = true;
    //  readSensors();
    // if(frontDistance < 150){
    // while(frontDistance > 30){
    // digitalWrite(LEFT_MOTOR_A, HIGH);
    // digitalWrite(LEFT_MOTOR_B, LOW);
    // digitalWrite(RIGHT_MOTOR_A, HIGH);
    // digitalWrite(RIGHT_MOTOR_B, LOW);
    // ledcWrite(LEFT_PWM_CHANNEL, BASE_SPEED_INIT);
    // ledcWrite(RIGHT_PWM_CHANNEL, BASE_SPEED_INIT);
    // readSensors();
    // delay(10);
    // }}
    // stopMotors();
    // delay(50);
    // leftCount = 0;
    // rightCount = 0;
    
    digitalWrite(LEFT_MOTOR_A, LOW);
    digitalWrite(LEFT_MOTOR_B, HIGH);
    digitalWrite(RIGHT_MOTOR_A, HIGH);
    digitalWrite(RIGHT_MOTOR_B, LOW);
    ledcWrite(LEFT_PWM_CHANNEL, TURN_SPEED);
    ledcWrite(RIGHT_PWM_CHANNEL, TURN_SPEED);
    
    while(abs(leftCount) < TURN_90_DEGREES || rightCount < TURN_90_DEGREES) {
        delay(5);
    }
    
    stopMotors();
    delay(200);
    backwardOneCell_turn();
    delay(50);
    currDirection = (currDirection + 3) % 4;
    Serial.print("Turn complete. New direction: ");
    Serial.println(currDirection);
}

void turn180Degrees() {
    leftCount = 0;
    rightCount = 0;
    leftMovingForward = false;
    rightMovingForward = true;
    
    unsigned long startTime = millis();
    readSensors();
    // while(frontDistance > 35){
    //   digitalWrite(LEFT_MOTOR_A, HIGH);
    //   digitalWrite(LEFT_MOTOR_B, LOW);
    //   digitalWrite(RIGHT_MOTOR_A, HIGH);
    //   digitalWrite(RIGHT_MOTOR_B, LOW);
    //   ledcWrite(LEFT_PWM_CHANNEL, 70);
    //   ledcWrite(RIGHT_PWM_CHANNEL, 70);
    //   readSensors();
    //   delay(10);

    // }
    // stopMotors();
    // delay(50);
    // while(frontDistance < 20){
    //   digitalWrite(LEFT_MOTOR_A, LOW);
    //   digitalWrite(LEFT_MOTOR_B, HIGH);
    //   digitalWrite(RIGHT_MOTOR_A, LOW);
    //   digitalWrite(RIGHT_MOTOR_B, HIGH);
    //   ledcWrite(LEFT_PWM_CHANNEL, 70);
    //   ledcWrite(RIGHT_PWM_CHANNEL, 70);
    //   readSensors();
    //   delay(10);

    // }
    stopMotors();
    delay(50);


    // Start turning
    if(leftDistance >= rightDistance){
    digitalWrite(LEFT_MOTOR_A, LOW);
    digitalWrite(LEFT_MOTOR_B, HIGH);
    digitalWrite(RIGHT_MOTOR_A, HIGH);
    digitalWrite(RIGHT_MOTOR_B, LOW);
    ledcWrite(LEFT_PWM_CHANNEL, TURN_SPEED);
    ledcWrite(RIGHT_PWM_CHANNEL, TURN_SPEED);
    }else{
    digitalWrite(LEFT_MOTOR_A, HIGH);
    digitalWrite(LEFT_MOTOR_B, LOW);
    digitalWrite(RIGHT_MOTOR_A, LOW);
    digitalWrite(RIGHT_MOTOR_B, HIGH);
    ledcWrite(LEFT_PWM_CHANNEL, TURN_SPEED);
    ledcWrite(RIGHT_PWM_CHANNEL, TURN_SPEED);

    }
    Serial.println("Turning 180°...");
    
    // Turn until target reached
    while(abs(leftCount) < TURN_180_DEGREES || rightCount < TURN_180_DEGREES) {
        delay(5);
        
        // Safety timeout
        if (millis() - startTime > 3000) {
            Serial.println("Turn timeout!");
            break;
        }
    }
    
    unsigned long turnTime = millis() - startTime;
    
    stopMotors();
    delay(50);
    backwardOneCell();
    delay(50);
    // Print turn statistics
    Serial.println("  ✓ Turn complete!");
    Serial.print("  Time taken: "); Serial.print(turnTime); Serial.println("ms");
    Serial.print("  Encoder counts - L:"); Serial.print(abs(leftCount));
    Serial.print(" R:"); Serial.println(rightCount);
    Serial.print("  Average: "); Serial.println((abs(leftCount) + rightCount) / 2);
    Serial.print("  Difference: "); Serial.println(abs(abs(leftCount) - rightCount));
}


// void turn180Degrees() {
//     Serial.println(">>> Turning 180");
//     leftCount = 0;
//     rightCount = 0;
//     leftMovingForward = false;
//     rightMovingForward = true;
    
//     digitalWrite(LEFT_MOTOR_A, LOW);
//     digitalWrite(LEFT_MOTOR_B, HIGH);
//     digitalWrite(RIGHT_MOTOR_A, HIGH);
//     digitalWrite(RIGHT_MOTOR_B, LOW);
//     ledcWrite(LEFT_PWM_CHANNEL, TURN_SPEED);
//     ledcWrite(RIGHT_PWM_CHANNEL, TURN_SPEED);
    
//     while(abs(leftCount) < 350 || rightCount < 350) {
//         delay(5);
//     }
    
//     stopMotors();
//     delay(200);
    
//     currDirection = (currDirection + 2) % 4;
//     Serial.print("Turn complete. New direction: ");
//     Serial.println(currDirection);
// }

void stopMotors() {
    digitalWrite(LEFT_MOTOR_A, LOW);
    digitalWrite(LEFT_MOTOR_B, LOW);
    digitalWrite(RIGHT_MOTOR_A, LOW);
    digitalWrite(RIGHT_MOTOR_B, LOW);
    ledcWrite(LEFT_PWM_CHANNEL, 0);
    ledcWrite(RIGHT_PWM_CHANNEL, 0);
}

void initializeMaze() {
    for (int i = 0; i < MAZE_SIZE; i++) {
        for (int j = 0; j < MAZE_SIZE; j++) {
            maze[i][j].x = i;
            maze[i][j].y = j;
            maze[i][j].northWall = (i == 0);
            maze[i][j].eastWall = (j == MAZE_SIZE - 1);
            maze[i][j].southWall = (i == MAZE_SIZE - 1);
            maze[i][j].westWall = (j == 0);
            maze[i][j].floodValue = -1;
        }
    }
}

void resetMaze() {
    for (int i = 0; i < MAZE_SIZE; i++) {
        for (int j = 0; j < MAZE_SIZE; j++) {
            maze[i][j].floodValue = -1;
        }
    }
    setDestination();
}

void setDestination() {
    for (int i = 0; i <= 1; i++) {
        for (int j = 0; j <= 1; j++) {
            if (i >= 0 && i < MAZE_SIZE && j >= 0 && j < MAZE_SIZE) {
                maze[i][j].floodValue = 0;
            }
        }
    }
}

bool isValidNeighbor(int x, int y) {
    return (x >= 0 && x < MAZE_SIZE && y >= 0 && y < MAZE_SIZE);
}

void floodFill() {
    std::queue<Cell> cellQueue;
    
    for (int i = 0; i <= 1; i++) {
        for (int j = 0; j <= 1; j++) {
            if (i >= 0 && i < MAZE_SIZE && j >= 0 && j < MAZE_SIZE) {
                cellQueue.push(maze[i][j]);
            }
        }
    }

    int dx[] = {-1, 0, 1, 0};
    int dy[] = {0, 1, 0, -1};

    while (!cellQueue.empty()) {
        Cell current = cellQueue.front();
        cellQueue.pop();

        for (int i = 0; i < 4; i++) {
            int neighborX = current.x + dx[i];
            int neighborY = current.y + dy[i];

            if (isValidNeighbor(neighborX, neighborY)) {
                Cell& neighbor = maze[neighborX][neighborY];

                if (!current.northWall && i == 0 && neighbor.floodValue == -1) {
                    neighbor.floodValue = current.floodValue + 1;
                    cellQueue.push(neighbor);
                } else if (!current.eastWall && i == 1 && neighbor.floodValue == -1) {
                    neighbor.floodValue = current.floodValue + 1;
                    cellQueue.push(neighbor);
                } else if (!current.southWall && i == 2 && neighbor.floodValue == -1) {
                    neighbor.floodValue = current.floodValue + 1;
                    cellQueue.push(neighbor);
                } else if (!current.westWall && i == 3 && neighbor.floodValue == -1) {
                    neighbor.floodValue = current.floodValue + 1;
                    cellQueue.push(neighbor);
                }
            }
        }
    }
}


// #include <climits> // for INT_MAX

// void findNextCell() {
//     Cell& current = maze[currX][currY];

//     // ensure nextX/nextY start as "not chosen"
//     nextX = currX;
//     nextY = currY;

//     int dx[] = {-1, 0, 1, 0};
//     int dy[] = {0, 1, 0, -1};

//     int directions[4];
//     if (currDirection == 0) {
//         directions[0] = 0; directions[1] = 1; directions[2] = 3; directions[3] = 2;
//     } else if (currDirection == 1) {
//         directions[0] = 1; directions[1] = 0; directions[2] = 2; directions[3] = 3;
//     } else if (currDirection == 2) {
//         directions[0] = 2; directions[1] = 3; directions[2] = 1; directions[3] = 0;
//     } else {
//         directions[0] = 3; directions[1] = 2; directions[2] = 0; directions[3] = 1;
//     }

//     // helper to check if the wall is blocking in 'dir'
//     auto isBlocked = [&](int dir)->bool {
//         if (dir == 0) return current.northWall;
//         if (dir == 1) return current.eastWall;
//         if (dir == 2) return current.southWall;
//         return current.westWall; // dir == 3
//     };

//     int targetFlood = current.floodValue - 1;

//     // 1) Prefer neighbor with flood = current - 1
//     for (int i = 0; i < 4; ++i) {
//         int dir = directions[i];
//         int nx = currX + dx[dir];
//         int ny = currY + dy[dir];

//         if (!isValidNeighbor(nx, ny)) continue;
//         if (isBlocked(dir)) continue;

//         if (maze[nx][ny].floodValue == targetFlood) {
//             nextX = nx; nextY = ny;
//             return;
//         }
//     }

//     // 2) Otherwise choose the open neighbor with the minimum floodValue (>= 0)
//     int bestFlood = INT_MAX;
//     for (int i = 0; i < 4; ++i) {
//         int dir = directions[i];
//         int nx = currX + dx[dir];
//         int ny = currY + dy[dir];

//         if (!isValidNeighbor(nx, ny)) continue;
//         if (isBlocked(dir)) continue;

//         int fv = maze[nx][ny].floodValue;
//         // skip obviously invalid/uninitialized values (adjust if you use a different sentinel)
//         if (fv >= 0 && fv < bestFlood) {
//             bestFlood = fv;
//             nextX = nx; nextY = ny;
//         }
//     }
//     if (nextX != currX || nextY != currY) return;

//     // 3) If no valid flood-number neighbors, prefer unexplored neighbors (e.g., floodValue < 0)
//     for (int i = 0; i < 4; ++i) {
//         int dir = directions[i];
//         int nx = currX + dx[dir];
//         int ny = currY + dy[dir];

//         if (!isValidNeighbor(nx, ny)) continue;
//         if (isBlocked(dir)) continue;

//         int fv = maze[nx][ny].floodValue;
//         if (fv < 0) { // treat negative as unvisited; change condition to your sentinel if needed
//             nextX = nx; nextY = ny;
//             return;
//         }
//     }

//     // 4) Final fallback: take the first open neighbor (keeps robot moving)
//     for (int i = 0; i < 4; ++i) {
//         int dir = directions[i];
//         int nx = currX + dx[dir];
//         int ny = currY + dy[dir];

//         if (!isValidNeighbor(nx, ny)) continue;
//         if (isBlocked(dir)) continue;

//         nextX = nx; nextY = ny;
//         return;
//     }

//     // if we reach here, no move possible (fully enclosed) — nextX/nextY remain currX/currY
// }



// void findNextCell() {
//     Cell current = maze[currX][currY];
    
//     int dx[] = {-1, 0, 1, 0};
//     int dy[] = {0, 1, 0, -1};
    
//     // Set direction priority based on current facing direction
//     int directions[4];
//     if (currDirection == 0) {
//         directions[0] = 0; directions[1] = 1; directions[2] = 3; directions[3] = 2;
//     } else if (currDirection == 1) {
//         directions[0] = 1; directions[1] = 0; directions[2] = 2; directions[3] = 3;
//     } else if (currDirection == 2) {
//         directions[0] = 2; directions[1] = 3; directions[2] = 1; directions[3] = 0;
//     } else {
//         directions[0] = 3; directions[1] = 2; directions[2] = 0; directions[3] = 1;
//     }
    
//     // Helper function to check if wall blocks a direction
//     auto hasWall = [&](int dir) -> bool {
//         if (dir == 0) return current.northWall;
//         if (dir == 1) return current.eastWall;
//         if (dir == 2) return current.southWall;
//         return current.westWall;
//     };
    
//     // STEP 1: Try to find ideal neighbor (floodValue = current - 1)
//     int targetFlood = current.floodValue - 1;
    
//     for (int i = 0; i < 4; i++) {
//         int dir = directions[i];
//         int neighborX = currX + dx[dir];
//         int neighborY = currY + dy[dir];

//         if (isValidNeighbor(neighborX, neighborY) && !hasWall(dir)) {
//             if (maze[neighborX][neighborY].floodValue == targetFlood) {
//                 nextX = neighborX;
//                 nextY = neighborY;
//                 return;  // Found ideal path!
//             }
//         }
//     }
    
//     // STEP 2: No ideal path found, pick neighbor with MINIMUM flood value
//     int bestFlood = 999;
    
//     for (int i = 0; i < 4; i++) {
//         int dir = directions[i];
//         int neighborX = currX + dx[dir];
//         int neighborY = currY + dy[dir];

//         if (isValidNeighbor(neighborX, neighborY) && !hasWall(dir)) {
//             int fv = maze[neighborX][neighborY].floodValue;
            
//             if (fv >= 0 && fv < bestFlood) {
//                 bestFlood = fv;
//                 nextX = neighborX;
//                 nextY = neighborY;
//             }
//         }
//     }
    
//     // STEP 3: Emergency fallback - if still no cell found (shouldn't happen)
//     if (nextX == currX && nextY == currY) {
//         Serial.println("ERROR: No accessible neighbor found!");
//         // Robot is completely stuck - handle error
//     }
// }
void findNextCell() {
    int minFloodValue = maze[currX][currY].floodValue - 1;
    Cell current = maze[currX][currY];
    nextX = currX;
    nextY = currY;

    int dx[] = {-1, 0, 1, 0};
    int dy[] = {0, 1, 0, -1};

    int directions[4];
    if (currDirection == 0) {
        directions[0] = 0; directions[1] = 1; directions[2] = 3; directions[3] = 2;
    } else if (currDirection == 1) {
        directions[0] = 1; directions[1] = 0; directions[2] = 2; directions[3] = 3;
    } else if (currDirection == 2) {
        directions[0] = 2; directions[1] = 3; directions[2] = 1; directions[3] = 0;
    } else {
        directions[0] = 3; directions[1] = 2; directions[2] = 0; directions[3] = 1;
    }

    for (int i = 0; i < 4; i++) {
        int dir = directions[i];
        int neighborX = currX + dx[dir];
        int neighborY = currY + dy[dir];

        if (isValidNeighbor(neighborX, neighborY)) {
            Cell& neighbor = maze[neighborX][neighborY];
            
            if (!current.northWall && dir == 0 && neighbor.floodValue == minFloodValue) {
                nextX = neighborX; nextY = neighborY; return;
            } else if (!current.eastWall && dir == 1 && neighbor.floodValue == minFloodValue) {
                nextX = neighborX; nextY = neighborY; return;
            } else if (!current.southWall && dir == 2 && neighbor.floodValue == minFloodValue) {
                nextX = neighborX; nextY = neighborY; return;
            } else if (!current.westWall && dir == 3 && neighbor.floodValue == minFloodValue) {
                nextX = neighborX; nextY = neighborY; return;
            } 
        }
    }
    if ( nextX == currX && nextY == currY){
        readSensors();
        if(leftDistance > 130){
            if(currDirection == 0){
            nextX = currX;
            nextY = currY - 1;
            return; 
            } else if(currDirection == 1){
            nextX = currX - 1;
            nextY = currY;
            return;  
            }else if(currDirection == 2){
            nextX = currX;
            nextY = currY + 1;
            return;  
            }else if(currDirection == 3){
            nextX = currX + 1;
            nextY = currY;
            return;  
            }
            
        }

        if(rightDistance > 130){
            if(currDirection == 0){
            nextX = currX;
            nextY = currY + 1;
            return; 
            } else if(currDirection == 1){
            nextX = currX + 1;
            nextY = currY;
            return;  
            }else if(currDirection == 2){
            nextX = currX;
            nextY = currY - 1;
            return;  
            }else if(currDirection == 3){
            nextX = currX - 1;
            nextY = currY;
            return;  
            }
            
        }


    }
    // for (int i = 0; i < 4; i++) {
    //     int dir = directions[i];
    //     int neighborX = currX + dx[dir];
    //     int neighborY = currY + dy[dir];

    //     if (isValidNeighbor(neighborX, neighborY)) {
    //         Cell& neighbor = maze[neighborX][neighborY];
            
    //         if (!current.northWall && dir == 0) {
    //             nextX = neighborX; nextY = neighborY; return;
    //         } else if (!current.eastWall && dir == 1) {
    //             nextX = neighborX; nextY = neighborY; return;
    //         } else if (!current.southWall && dir == 2) {
    //             nextX = neighborX; nextY = neighborY; return;
    //         } else if (!current.westWall && dir == 3) {
    //             nextX = neighborX; nextY = neighborY; return;
    //         } 
    //     }
    // }

    // if(nextX == currX && nextY == currY){
    // for (int i = 0; i < 4; i++) {
    //     int dir = directions[i];
    //     int neighborX = currX + dx[dir];
    //     int neighborY = currY + dy[dir];

    //     if (isValidNeighbor(neighborX, neighborY)) {
    //         Cell& neighbor = maze[neighborX][neighborY];
            
    //         if (!current.northWall && dir == 0) {
    //             nextX = neighborX; nextY = neighborY; return;
    //         } else if (!current.eastWall && dir == 1) {
    //             nextX = neighborX; nextY = neighborY; return;
    //         } else if (!current.southWall && dir == 2) {
    //             nextX = neighborX; nextY = neighborY; return;
    //         } else if (!current.westWall && dir == 3) {
    //             nextX = neighborX; nextY = neighborY; return;
    //         } 
    //     }
    // }
    // }
    // resetMaze();  // Reset flood values
    // floodFill();
    // // After re-flood fill, try again with new flood values
    // minFloodValue = maze[currX][currY].floodValue - 1;
    // Serial.print("  After re-flood fill, new target flood value: ");
    // Serial.println(minFloodValue);
    
    // // Try again with updated flood values
    // for (int i = 0; i < 4; i++) {
    //     int dir = directions[i];
    //     int neighborX = currX + dx[dir];
    //     int neighborY = currY + dy[dir];

    //     if (isValidNeighbor(neighborX, neighborY)) {
    //         Cell& neighbor = maze[neighborX][neighborY];

    //         if (!current.northWall && dir == 0 && neighbor.floodValue == minFloodValue) {
    //             nextX = neighborX; nextY = neighborY;
    //             Serial.println("  → REFLOOD: Selected NORTH");
    //             return;
    //         } else if (!current.eastWall && dir == 1 && neighbor.floodValue == minFloodValue) {
    //             nextX = neighborX; nextY = neighborY;
    //             Serial.println("  → REFLOOD: Selected EAST");
    //             return;
    //         } else if (!current.southWall && dir == 2 && neighbor.floodValue == minFloodValue) {
    //             nextX = neighborX; nextY = neighborY;
    //             Serial.println("  → REFLOOD: Selected SOUTH");
    //             return;
    //         } else if (!current.westWall && dir == 3 && neighbor.floodValue == minFloodValue) {
    //             nextX = neighborX; nextY = neighborY;
    //             Serial.println("  → REFLOOD: Selected WEST");
    //             return;
    //         }
    //     }
    // }
    //  int bestFloodValue = 999;
    // int bestDir = -1;
    // int bestX = -1, bestY = -1;
    
    // for (int i = 0; i < 4; i++) {
    //     int dir = directions[i];
    //     int neighborX = currX + dx[dir];
    //     int neighborY = currY + dy[dir];

    //     bool hasWall = false;
    //     if (dir == 0) hasWall = current.northWall;
    //     else if (dir == 1) hasWall = current.eastWall;
    //     else if (dir == 2) hasWall = current.southWall;
    //     else if (dir == 3) hasWall = current.westWall;

    //     if (isValidNeighbor(neighborX, neighborY) && !hasWall) {
    //         Cell& neighbor = maze[neighborX][neighborY];
            
    //         // Find neighbor with lowest flood value
    //         if (neighbor.floodValue >= 0 && neighbor.floodValue < bestFloodValue) {
    //             bestFloodValue = neighbor.floodValue;
    //             bestDir = dir;
    //             bestX = neighborX;
    //             bestY = neighborY;
    //         }
    //     }
    // }

    // if (bestDir != -1) {
    //     nextX = bestX;
    //     nextY = bestY;
    //     // Serial.print("  → FALLBACK: Selected ");
    //     // Serial.print(dirNames[bestDir]);
    //     // Serial.print(" (");
    //     // Serial.print(bestX);
    //     // Serial.print(",");
    //     // Serial.print(bestY);
    //     // Serial.print(") FloodVal: ");
    //     // Serial.println(bestFloodValue);
    //     return;
    // }

    // // EMERGENCY: No accessible neighbor found - STUCK!
    // Serial.println("  ❌ ERROR: NO ACCESSIBLE NEIGHBORS FOUND!");
    // Serial.println("  Robot is stuck! Printing maze for debugging...");
    // // printMazeDetailed();
    
    // stopMotors();
    // while(1) {
    //     delay(1000);
    //     Serial.println("STUCK - Check maze configuration!");
    // }
}

void setWall(int x, int y, int dir) {
    if (dir == 0) {
        maze[x][y].northWall = true;
        if (x - 1 >= 0) maze[x - 1][y].southWall = true;
    } else if (dir == 1) {
        maze[x][y].eastWall = true;
        if (y + 1 < MAZE_SIZE) maze[x][y + 1].westWall = true;
    } else if (dir == 2) {
        maze[x][y].southWall = true;
        if (x + 1 < MAZE_SIZE) maze[x + 1][y].northWall = true;
    } else if (dir == 3) {
        maze[x][y].westWall = true;
        if (y - 1 >= 0) maze[x][y - 1].eastWall = true;
    }
}

bool isTopWall() {
    return (frontDistance < WALL_THRESHOLD_FRONT);
}

bool isLeftWall() {
    return (leftDistance < WALL_THRESHOLD_SIDE);
}

bool isRightWall() {
    return (rightDistance < WALL_THRESHOLD_SIDE);
}

void setWalls() {
    if (currDirection == 0) {
        if (isTopWall()) setWall(currX, currY, 0);
        if (isLeftWall()) setWall(currX, currY, 3);
        if (isRightWall()) setWall(currX, currY, 1);
    } else if (currDirection == 1) {
        if (isTopWall()) setWall(currX, currY, 1);
        if (isLeftWall()) setWall(currX, currY, 0);
        if (isRightWall()) setWall(currX, currY, 2);
    } else if (currDirection == 2) {
        if (isTopWall()) setWall(currX, currY, 2);
        if (isLeftWall()) setWall(currX, currY, 1);
        if (isRightWall()) setWall(currX, currY, 3);
    } else if (currDirection == 3) {
        if (isTopWall()) setWall(currX, currY, 3);
        if (isLeftWall()) setWall(currX, currY, 2);
        if (isRightWall()) setWall(currX, currY, 0);
    }
}

void nextTurn() {
    int dx = nextX - currX;
    int dy = nextY - currY;
    
    Serial.print("Planning turn - dx:");
    Serial.print(dx);
    Serial.print(" dy:");
    Serial.println(dy);

    if (currDirection == 0) {
    } else if (currDirection == 1) {
        if (dx == -1) turn90DegreesLeft();
        else if (dx == 1) turn90DegreesRight();
        else if (dy == 1) { Serial.println("Going straight"); }
        else if (dy == -1) turn180Degrees();
    } else if (currDirection == 2) {
        if (dx == -1) turn180Degrees();
        else if (dx == 1) { Serial.println("Going straight"); }
        else if (dy == 1) turn90DegreesLeft();
        else if (dy == -1) turn90DegreesRight();
    } else if (currDirection == 3) {
        if (dx == -1) turn90DegreesRight();
        else if (dx == 1) turn90DegreesLeft(); 
        else if (dy == 1) turn180Degrees();
        else if (dy == -1) { Serial.println("Going straight"); }
    }
    forwardOneCell();
    switch(currDirection) {
        case 0: currX--; break;
        case 1: currY++; break;
        case 2: currX++; break;
        case 3: currY--; break;
    }
    
    Serial.print("New Position: (");
    Serial.print(currX);
    Serial.print(", ");
    Serial.print(currY);
    Serial.print(") Direction: ");
    Serial.println(currDirection);
    
    // Read walls and update maze
    readSensors();
    printDebugInfo();
    setWalls();
    
    // Re-flood fill and find next move
    resetMaze();
    floodFill();
    findNextCell();
    Serial.print("Next target: (");
    Serial.print(nextX);
    Serial.print(",");
    Serial.print(nextY);
    Serial.println(")");
    
}

void printMaze() {
    Serial.println("\n=== Flood Fill Values ===");
    for (int i = 0; i < MAZE_SIZE; i++) {
        for (int j = 0; j < MAZE_SIZE; j++) {
            if (maze[i][j].floodValue < 10) Serial.print(" ");
            Serial.print(maze[i][j].floodValue);
            Serial.print(" ");
        }
        Serial.println();
    }
    Serial.println();
}