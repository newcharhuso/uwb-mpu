#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

int firstreading = 1;
float displacementX = 0, displacementY = 0, displacementZ = 0;
unsigned long lastUpdateTime = 0; // Track the last update time
const float accelThreshold = 0.05;
float offsetX = -0.725, offsetY = -1.415, offsetZ = 0;
float accelX, accelY,accelZ, gyroXangle, gyroYangle, x, y, z, accelXangle, accelYangle, roll, pitch, yaw;
float velocityX = 0, velocityY = 0, velocityZ = 0;
float prevAccelX = 0, prevAccelY = 0, prevAccelZ = 0;
// Gravity vector
float gravityX = 0;
float gravityY = 0;
float gravityZ = 0;
float gravity = 9.81; // m/s^2
const float alpha = 0.90; // Complementary filter coefficient

#include <Wire.h>
float RateRoll, RatePitch, RateYaw;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;

void gyro_signals(void) {
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(); 
  Wire.requestFrom(0x68,6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();

  Wire.beginTransmission(0x68);
  Wire.write(0x1B); 
  Wire.write(0x8);
  Wire.endTransmission();                                                   
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);

  int16_t GyroX=Wire.read()<<8 | Wire.read();
  int16_t GyroY=Wire.read()<<8 | Wire.read();
  int16_t GyroZ=Wire.read()<<8 | Wire.read();

  RateRoll=(float)GyroX/65.5;
  RatePitch=(float)GyroY/65.5;
  RateYaw=(float)GyroZ/65.5;
  AccX=(float)AccXLSB/4096 - 0.02;
  AccY=(float)AccYLSB/4096 + 0.02;
  AccZ=(float)AccZLSB/4096 - 0.04;

}
void setup() {
  Serial.begin(115200 );
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68); 
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
}

void loop() {
  gyro_signals();

  unsigned long currentTime = millis();
  float timeDiff = (currentTime - lastUpdateTime) / 1000.0;

  accelX = AccX * gravity;
  accelY = AccY * gravity;
  accelZ = AccZ * gravity;

  // Apply high-pass filter to remove gravity component
  if (firstreading == 1){
	gravityX = accelX;
	gravityY = accelY;
	gravityZ = accelZ;
	firstreading = 0;
  }
  else{
  gravityX = alpha * gravityX + (1 - alpha) * accelX;
  gravityY = alpha * gravityY + (1 - alpha) * accelY;
  gravityZ = alpha * gravityZ + (1 - alpha) * accelZ;
  }
  // Subtract gravity component from accelerometer readings
  accelX -= gravityX;
  accelY -= gravityY;
  accelZ -= gravityZ;

  pitch = atan2(-AccX, sqrt(AccY * AccY + AccZ * AccZ)) * 180 / PI;
  roll = atan2(AccY, AccZ) * 180 / PI;
  
  if (abs(accelX) < accelThreshold) accelX = 0;
  if (abs(accelY) < accelThreshold) accelY = 0;
  if (abs(accelZ) < accelThreshold) accelZ = 0;

  displacementX += (prevAccelX + accelX) * timeDiff / 2;
  displacementY += (prevAccelY + accelY) * timeDiff / 2;
  displacementZ += (prevAccelZ + accelZ) * timeDiff / 2;

  velocityX = (prevAccelX + accelX) / 2;
  velocityY = (prevAccelX + accelX) / 2; 
  // Update previous acceleration values
  prevAccelX = accelX;
  prevAccelY = accelY;

  lastUpdateTime = currentTime;
  Serial.print("VelocityX:"); Serial.print(accelX); Serial.print(", ");
  Serial.print("velocityY:"); Serial.print(velocityY); Serial.println(", ");

  Serial.print("Coordinates : "); Serial.print("X: "); Serial.print(displacementX); Serial.print(" Y: "); Serial.print(displacementY); Serial.print(" Z: "); Serial.println(displacementZ);

  Serial.println("  ");
  Serial.println("  ");
  Serial.println("  ");
  Serial.println("  ");
  delay(100);
}
