#include <Wire.h>
#include <MPU6050.h>
#include <LiquidCrystal_I2C.h>

// I2C Multiplexer & LCD
#define PCA9548A_ADDRESS 0x70
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Motor control pins
#define MOTOR1_PWM 6
#define MOTOR1_DIR 7
#define MOTOR2_PWM 8
#define MOTOR2_DIR 9
#define MOTOR3_PWM 10
#define MOTOR3_DIR 11
#define MOTOR4_PWM 12
#define MOTOR4_DIR 13

// Potentiometer pins
#define POT1 A0
#define POT2 A1
#define POT3 A2
#define POT4 A3

// Button pins
#define RESET_BUTTON 2
#define PAGE_BUTTON 3
#define SPEED_UP_BUTTON 4
#define SPEED_DOWN_BUTTON 5

#define POT_HYSTERESIS 5

float potResetAngles[4] = {170.0, 170.0, 170.0, 170.0};
float potEndHigh[4] = {280.0, 250.0, 280.0, 250.0};
float potEndLow[4] = {170.0, 145.0, 170.0, 145.0};

// MPU Thresholds
float mpuThresholds[4] = {6.0, 6.0, 6.0, 6.0};

#define MPU1_CHANNEL 0
#define MPU2_CHANNEL 1
#define MPU3_CHANNEL 2
#define MPU4_CHANNEL 3

MPU6050 mpu1, mpu2, mpu3, mpu4;

bool motorRunning[4] = {false, false, false, false};
bool reachedHigh[4] = {false, false, false, false};
bool initialForwardDirection[4] = {true, false, true, false}; // M1 & M3: Forward, M2 & M4: Reverse

int pwmPins[4] = {MOTOR1_PWM, MOTOR2_PWM, MOTOR3_PWM, MOTOR4_PWM};
int dirPins[4] = {MOTOR1_DIR, MOTOR2_DIR, MOTOR3_DIR, MOTOR4_DIR};
int potPins[4] = {POT1, POT2, POT3, POT4};
float potAngles[4] = {0, 0, 0, 0};
float accelY[4] = {0, 0, 0, 0};

byte currentPage = 0;
unsigned long lastButtonPress = 0;
const unsigned long DEBOUNCE_DELAY = 50;
int MOTOR_SPEED = 30;
const int SPEED_INCREMENT = 1;
const int MIN_SPEED = 0;
const int MAX_SPEED = 255;

enum State { WAIT_FORWARD, FORWARD, WAIT_REVERSE, REVERSE };
State currentState[4] = {WAIT_FORWARD, WAIT_FORWARD, WAIT_FORWARD, WAIT_FORWARD};

void selectI2CChannel(uint8_t channel) {
  Wire.beginTransmission(PCA9548A_ADDRESS);
  Wire.write(1 << channel);
  Wire.endTransmission();
  delay(10);
}

void setupMPU(MPU6050 &mpu, uint8_t channel) {
  selectI2CChannel(channel);
  mpu.initialize();
  if (mpu.testConnection()) {
    Serial.print("MPU ");
    Serial.print(channel);
    Serial.println(" connected.");
  } else {
    Serial.print("MPU ");
    Serial.print(channel);
    Serial.println(" FAILED!");
  }
}

float getAccelY(MPU6050 &mpu, uint8_t channel) {
  selectI2CChannel(channel);
  int16_t ax, ay, az;
  mpu.getMotion6(&ax, &ay, &az, nullptr, nullptr, nullptr);
  return (float)ay / 16384.0 * 9.81;
}

float readPotentiometerAngle(int pin) {
  int val = analogRead(pin);
  return map(val, 0, 1023, 0, 300);
}

void startMotor(int index, bool forward) {
  digitalWrite(dirPins[index], forward ? HIGH : LOW);
  analogWrite(pwmPins[index], MOTOR_SPEED);
  motorRunning[index] = true;
  Serial.print("Motor ");
  Serial.print(index + 1);
  Serial.println(forward ? " STARTED FORWARD" : " STARTED REVERSE");
}

void stopMotor(int index) {
  analogWrite(pwmPins[index], 0);
  motorRunning[index] = false;
  Serial.print("Motor ");
  Serial.print(index + 1);
  Serial.println(" STOPPED");
}

void displayLCD() {
  lcd.clear();
  switch (currentPage) {
    case 0:
      lcd.setCursor(0, 0);
      lcd.print("P1:");
      lcd.print((int)potAngles[0]);
      lcd.print(" M1:");
      lcd.print(accelY[0], 1);
      lcd.setCursor(0, 1);
      lcd.print("P2:");
      lcd.print((int)potAngles[1]);
      lcd.print(" M2:");
      lcd.print(accelY[1], 1);
      break;
    case 1:
      lcd.setCursor(0, 0);
      lcd.print("P3:");
      lcd.print((int)potAngles[2]);
      lcd.print(" M3:");
      lcd.print(accelY[2], 1);
      lcd.setCursor(0, 1);
      lcd.print("P4:");
      lcd.print((int)potAngles[3]);
      lcd.print(" M4:");
      lcd.print(accelY[3], 1);
      break;
    case 2:
      lcd.setCursor(0, 0);
      lcd.print("Speed:");
      lcd.print(MOTOR_SPEED);
      break;
  }
}

void resetToAngle() {
  for (int i = 0; i < 4; i++) {
    float targetAngle = potResetAngles[i];
    float currentAngle = potAngles[i];

    // Determine direction based on motor position and target
    bool forward;
    if (i == 0 || i == 2) { // M1 and M3
      forward = (currentAngle < targetAngle);
    } else { // M2 and M4
      forward = (currentAngle > targetAngle);
    }

    // Start the motor
    startMotor(i, forward);
    motorRunning[i] = true; // Set the motorRunning flag

    // Set the state to FORWARD or REVERSE depending on the direction
    currentState[i] = forward ? FORWARD : REVERSE;
  }
  // Add this section to stop the motors
  bool allStopped = false;
  while (!allStopped) {
    allStopped = true; // Assume all are stopped until proven otherwise
    for (int i = 0; i < 4; i++) {
      float currentAngle = readPotentiometerAngle(potPins[i]); // Re-read pot angle
      if (motorRunning[i]) { //if motor is still running
        if (abs(currentAngle - potResetAngles[i]) <= 5) { // Within 5 degrees
          stopMotor(i);
        }
        else{
          allStopped = false;
        }
      }
    }
    delay(10); // Short delay to prevent overwhelming the processor
  }
  for(int i=0; i<4; i++){
    currentState[i] = WAIT_FORWARD;
  }
}


void handlePageButton() {
  if (digitalRead(PAGE_BUTTON) == LOW && (millis() - lastButtonPress > DEBOUNCE_DELAY)) {
    currentPage++;
    if (currentPage > 2) {
      currentPage = 0;
    }
    displayLCD();
    lastButtonPress = millis();
  }
}

void adjustSpeed(int change) {
  if (millis() - lastButtonPress > DEBOUNCE_DELAY) {
    MOTOR_SPEED += change;
    if (MOTOR_SPEED > MAX_SPEED) {
      MOTOR_SPEED = MAX_SPEED;
    } else if (MOTOR_SPEED < MIN_SPEED) {
      MOTOR_SPEED = MIN_SPEED;
    }
    lastButtonPress = millis();

    // Update the speed of any currently running motors
    for (int i = 0; i < 4; i++) {
      if (motorRunning[i]) {
        analogWrite(pwmPins[i], MOTOR_SPEED);
      }
    }
    displayLCD();
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Initializing...");

  pinMode(RESET_BUTTON, INPUT_PULLUP);
  pinMode(PAGE_BUTTON, INPUT_PULLUP);
  pinMode(SPEED_UP_BUTTON, INPUT_PULLUP);
  pinMode(SPEED_DOWN_BUTTON, INPUT_PULLUP);

  for (int i = 0; i < 4; i++) {
    pinMode(pwmPins[i], OUTPUT);
    pinMode(dirPins[i], OUTPUT);
  }

  setupMPU(mpu1, MPU1_CHANNEL);
  setupMPU(mpu2, MPU2_CHANNEL);
  setupMPU(mpu3, MPU3_CHANNEL);
  setupMPU(mpu4, MPU4_CHANNEL);

  lcd.clear();
  lcd.print("System Ready");
  delay(1000);
}

void loop() {
  MPU6050* mpus[4] = {&mpu1, &mpu2, &mpu3, &mpu4};
  uint8_t channels[4] = {MPU1_CHANNEL, MPU2_CHANNEL, MPU3_CHANNEL, MPU4_CHANNEL};

  for (int i = 0; i < 4; i++) {
    potAngles[i] = readPotentiometerAngle(potPins[i]);
    accelY[i] = getAccelY(*mpus[i], channels[i]);
  }

  if (digitalRead(RESET_BUTTON) == LOW) {
    resetToAngle();
    delay(300);
    return;
  }

  handlePageButton();
  if (digitalRead(SPEED_UP_BUTTON) == LOW) {
    adjustSpeed(SPEED_INCREMENT);
  }
  if (digitalRead(SPEED_DOWN_BUTTON) == LOW) {
    adjustSpeed(-SPEED_INCREMENT);
  }

  // Check MPU pairs
  bool m1m2Ready = (accelY[0] > mpuThresholds[0] && accelY[1] < -mpuThresholds[1]);
  bool m3m4Ready = (accelY[2] > mpuThresholds[2] && accelY[3] < -mpuThresholds[3]);
  bool m1m2ReadyReverse = (accelY[0] < -mpuThresholds[0] && accelY[1] > mpuThresholds[1]);
  bool m3m4ReadyReverse = (accelY[2] < -mpuThresholds[2] && accelY[3] > mpuThresholds[2]);


  for (int i = 0; i < 4; i++) {
    float currentThreshold = reachedHigh[i] ? mpuThresholds[i] : mpuThresholds[i];

    switch (currentState[i]) {
      case WAIT_FORWARD:
        if ((i == 0 || i == 1) && m1m2Ready) {
          startMotor(i, initialForwardDirection[i]);
          currentState[i] = FORWARD;
        } else if ((i == 2 || i == 3) && m3m4Ready) {
          startMotor(i, initialForwardDirection[i]);
          currentState[i] = FORWARD;
        }
        break;
      case FORWARD:
        if (potAngles[i] >= potEndHigh[i] - POT_HYSTERESIS) {
          stopMotor(i);
          currentState[i] = WAIT_REVERSE;
          reachedHigh[i] = true;
        }
        break;
      case WAIT_REVERSE:
        if ((i == 0 || i == 1) && m1m2ReadyReverse) {
          startMotor(i, !initialForwardDirection[i]);
          currentState[i] = REVERSE;
        } else if ((i == 2 || i == 3) && m3m4ReadyReverse) {
          startMotor(i, !initialForwardDirection[i]);
          currentState[i] = REVERSE;
        }
        break;
      case REVERSE:
        if (potAngles[i] <= potEndLow[i] + POT_HYSTERESIS) {
          stopMotor(i);
          currentState[i] = WAIT_FORWARD;
        }
        break;
        delay(53);
    }
  }

  displayLCD();
  delay(300);
}
