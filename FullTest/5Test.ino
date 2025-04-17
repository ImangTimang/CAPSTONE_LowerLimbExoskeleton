#include <Wire.h>
#include <MPU6050.h>
#include <IMPU.h>
#include <WMPU.h>

#define PCA9548A_ADDRESS 0x70  // Default I2C address of PCA9548A
#define MPU1_CHANNEL 0         // Channel for first MPU6050 
#define MPU2_CHANNEL 1         // Channel for second MPU6050 
#define MPU3_CHANNEL 2         // Channel for third MPU6050
#define MPU4_CHANNEL 3         // Channel for fourth MPU6050
#define MPU5_CHANNEL 4        // Channel for fifth MPU6050
#define MPU6_CHANNEL 5         // Channel for sixth MPU6050
#define LED 13        // Pin connected to the LED
#define ACCEL_THRESHOLD 500  // Change this value based on your sensitivity needs

MPU6050 mpu1;
MPU6050 mpu2;
MPU6050 mpu3;
MPU6050 mpu4;
MPU6050 mpu5;
MPU6050 mpu6;

void selectI2CChannel(uint8_t channel) {
    Wire.beginTransmission(PCA9548A_ADDRESS);
    Wire.write(1 << channel);  // Enable the selected channel
    Wire.endTransmission();
    delay(10);  // Allow some time for switching
}

void setupMPU(MPU6050 &mpu, uint8_t channel) {
    selectI2CChannel(channel);
    mpu.initialize();
    if (mpu.testConnection()) {
        Serial.print("MPU6050 on channel ");
        Serial.print(channel);
        Serial.println(" connected successfully!");
    } else {
        Serial.print("MPU6050 on channel ");
        Serial.print(channel);
        Serial.println(" connection failed!");
    }
}

void setup() {
    Serial.begin(115200);
    Wire.begin();
    pinMode(LED, OUTPUT);
    Serial.println("Initializing MPU6050 sensors...");
    setupMPU(mpu1, MPU1_CHANNEL);
    setupMPU(mpu2, MPU2_CHANNEL);
    setupMPU(mpu3, MPU3_CHANNEL);
    setupMPU(mpu4, MPU4_CHANNEL);
    setupMPU(mpu5, MPU5_CHANNEL);
    setupMPU(mpu6, MPU6_CHANNEL);
}

float getAccelX(MPU6050 &mpu, uint8_t channel) {
    selectI2CChannel(channel);
    int16_t ax, ay, az;
    mpu.getMotion6(&ax, &ay, &az, nullptr, nullptr, nullptr);
    return (float)ax / 16384.0 * 9.81;
}

void checkThreshold(MPU6050 &mpu, uint8_t channel) {
    selectI2CChannel(channel);
    int16_t ax, ay, az;
    mpu.getMotion6(&ax, &ay, &az, nullptr, nullptr, nullptr);

    float accel_x = (float)ax / 16384.0 * 9.81; // Convert to m/s^2 (approximate)

    float scaled_threshold = ACCEL_THRESHOLD / 100.0; // Example adjustment
    float accelX_mpu1 = getAccelX(mpu1, MPU1_CHANNEL);
    float accelX_mpu2 = getAccelX(mpu2, MPU2_CHANNEL);
    float accelX_mpu3 = getAccelX(mpu3, MPU3_CHANNEL);
    float accelX_mpu4 = getAccelX(mpu4, MPU4_CHANNEL);
    float accelX_mpu5 = getAccelX(mpu5, MPU5_CHANNEL);
    float final_threshold = 8; // FINAL Threshold

    Serial.print("MPU1 Accel X: ");
    Serial.print(accelX_mpu1);
    Serial.print(" | MPU2 Accel X: ");
    Serial.println(accelX_mpu2);
    Serial.print("MPU3 Accel X: ");
    Serial.print(accelX_mpu3);
    Serial.print(" | MPU4 Accel X: ");
    Serial.println(accelX_mpu4);
    Serial.print("MPU5 Accel X: ");
    Serial.print(accelX_mpu5);
    if (abs(accelX_mpu1) > scaled_threshold && abs(accelX_mpu2) > scaled_threshold) {
        digitalWrite(LED, HIGH);
    } else {
        digitalWrite(LED, LOW);
    }
    if ((abs(accelX_mpu1) > final_threshold && abs(accelX_mpu2) > final_threshold)) {
        digitalWrite(LED,LOW);
    }
}

void loop() {
    checkThreshold(mpu1, MPU1_CHANNEL);
    delay(2000);
    checkThreshold(mpu2, MPU2_CHANNEL);
    delay(2000);
    checkThreshold(mpu3, MPU3_CHANNEL);
    delay(2000);
    checkThreshold(mpu4, MPU4_CHANNEL);
    delay(2000);
    checkThreshold(mpu5, MPU5_CHANNEL);
    delay(2000);
} 

