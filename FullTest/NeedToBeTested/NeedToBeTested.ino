#include <Wire.h>
#include <MPU6050.h>

#define PCA9548A_ADDRESS 0x70   // I2C address of PCA9548A
#define ACCEL_THRESHOLD 500     // Threshold in raw accel units
#define POT_PIN A0              // Analog pin for potentiometer

// Motor control pins for 4 motors
#define MOTOR1_PWM 5
#define MOTOR1_DIR 4
#define MOTOR2_PWM 6
#define MOTOR2_DIR 7
#define MOTOR3_PWM 9
#define MOTOR3_DIR 8
#define MOTOR4_PWM 10
#define MOTOR4_DIR 11

// Channels for each MPU
#define MPU1_CHANNEL 0
#define MPU2_CHANNEL 1
#define MPU3_CHANNEL 2
#define MPU4_CHANNEL 3

// MPU instances
MPU6050 mpu1;
MPU6050 mpu2;
MPU6050 mpu3;
MPU6050 mpu4;

// Motor state
bool motorRunning[4] = {false, false, false, false};
bool motorDirectionForward[4] = {true, true, true, true};
const float POT_RESET_ANGLE = 200.0; // Angle to stop motor

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
    Serial.print("MPU6050 on channel ");
    Serial.print(channel);
    Serial.println(" connected successfully.");
  } else {
    Serial.print("MPU6050 on channel ");
    Serial.print(channel);
    Serial.println(" connection failed!");
  }
}

float getAccelX(MPU6050 &mpu, uint8_t channel) {
  selectI2CChannel(channel);
  int16_t ax, ay, az;
  mpu.getMotion6(&ax, &ay, &az, nullptr, nullptr, nullptr);
  return (float)ax / 16384.0 * 9.81;
}

float readPotentiometerAngle() {
  int potValue = analogRead(POT_PIN);
  float angle = map(potValue, 0, 1023, 0, 300);
  return angle;
}

void startMotor(int index, int pwmPin, int dirPin, bool forward) {
  digitalWrite(dirPin, forward ? HIGH : LOW);
  analogWrite(pwmPin, 200);
  motorRunning[index] = true;
  motorDirectionForward[index] = forward;
  Serial.print("Motor "); Serial.print(index + 1);
  Serial.println(forward ? " STARTED FORWARD." : " STARTED REVERSE.");
}

void stopMotor(int index, int pwmPin) {
  analogWrite(pwmPin, 0);
  motorRunning[index] = false;
  Serial.print("Motor "); Serial.print(index + 1);
  Serial.println(" STOPPED by potentiometer.");
}

void checkThresholds() {
  float accelX[4];
  MPU6050* mpus[4] = {&mpu1, &mpu2, &mpu3, &mpu4};
  uint8_t channels[4] = {MPU1_CHANNEL, MPU2_CHANNEL, MPU3_CHANNEL, MPU4_CHANNEL};
  int pwmPins[4] = {MOTOR1_PWM, MOTOR2_PWM, MOTOR3_PWM, MOTOR4_PWM};
  int dirPins[4] = {MOTOR1_DIR, MOTOR2_DIR, MOTOR3_DIR, MOTOR4_DIR};

  for (int i = 0; i < 4; i++) {
    accelX[i] = getAccelX(*mpus[i], channels[i]);
    Serial.print("MPU"); Serial.print(i + 1); Serial.print(" Accel X: "); Serial.print(accelX[i]); Serial.print(" | ");
  }
  Serial.println();

  float scaled_threshold = ACCEL_THRESHOLD / 100.0;
  float final_threshold = 8.0;

  // Pair logic: MPU1 & MPU2, MPU3 & MPU4
  float avgAccelGroup1 = (accelX[0] + accelX[1]) / 2.0;
  float avgAccelGroup2 = (accelX[2] + accelX[3]) / 2.0;

  for (int i = 0; i < 2; i++) {
    if (!motorRunning[i]) {
      if (avgAccelGroup1 > final_threshold || abs(avgAccelGroup1) > scaled_threshold) {
        startMotor(i, pwmPins[i], dirPins[i], true);
      } else if (avgAccelGroup1 < -2.0) {
        startMotor(i, pwmPins[i], dirPins[i], false);
      }
    }
  }

  for (int i = 2; i < 4; i++) {
    if (!motorRunning[i]) {
      if (avgAccelGroup2 > final_threshold || abs(avgAccelGroup2) > scaled_threshold) {
        startMotor(i, pwmPins[i], dirPins[i], true);
      } else if (avgAccelGroup2 < -2.0) {
        startMotor(i, pwmPins[i], dirPins[i], false);
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  pinMode(MOTOR1_PWM, OUTPUT);
  pinMode(MOTOR1_DIR, OUTPUT);
  pinMode(MOTOR2_PWM, OUTPUT);
  pinMode(MOTOR2_DIR, OUTPUT);
  pinMode(MOTOR3_PWM, OUTPUT);
  pinMode(MOTOR3_DIR, OUTPUT);
  pinMode(MOTOR4_PWM, OUTPUT);
  pinMode(MOTOR4_DIR, OUTPUT);

  Serial.println("Initializing MPU6050 sensors...");
  setupMPU(mpu1, MPU1_CHANNEL);
  setupMPU(mpu2, MPU2_CHANNEL);
  setupMPU(mpu3, MPU3_CHANNEL);
  setupMPU(mpu4, MPU4_CHANNEL);
  Serial.println("Sensors initialized.");
}

void loop() {
  float potAngle = readPotentiometerAngle();
  Serial.print("Potentiometer Angle: ");
  Serial.println(potAngle);

  checkThresholds();

  int pwmPins[4] = {MOTOR1_PWM, MOTOR2_PWM, MOTOR3_PWM, MOTOR4_PWM};

  for (int i = 0; i < 4; i++) {
    if (motorRunning[i] && potAngle >= POT_RESET_ANGLE) {
      stopMotor(i, pwmPins[i]);
    }
  }

  delay(2000);
}
