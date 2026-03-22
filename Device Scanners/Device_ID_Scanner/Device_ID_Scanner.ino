#include <Adafruit_I2CDevice.h>

Adafruit_I2CDevice i2c_dev = Adafruit_I2CDevice(0x69);
#include <Wire.h>
#define MPU6050_DEVICE_ID 0x98 ///< The correct MPU6050_WHO_AM_I value
#define SDA_PIN 12
#define SCL_PIN 11
void setup() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Serial.begin(115200);
}

void loop() {
   while (!Serial) {
    delay(10);
  }
  
  Serial.println("I2C address detection test");

  if (!i2c_dev.begin()) {
    Serial.print("Did not find device at 0x");
    Serial.println(i2c_dev.address(), HEX);
    while (1)
      ;
  }
  Serial.print("Device found on address 0x");
  Serial.println(i2c_dev.address(), HEX);
  }
