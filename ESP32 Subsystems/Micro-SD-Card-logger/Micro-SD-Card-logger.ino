#include <SPI.h>
#include <SD.h>


// SD card pin definitions
#define SD_SCK   1
#define SD_MISO  2
#define SD_MOSI  3
#define SD_CS    4

SPIClass sdSPI(FSPI);
File logFile;

const char *fileName = "/imu_log.csv";



bool initSDLogger() {
  sdSPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);

  if (!SD.begin(SD_CS, sdSPI)) {
    Serial.println("ERROR: SD card initialization failed.");
    return false;
  }

  // Create file and header only if file does not already exist
  if (!SD.exists(fileName)) {
    logFile = SD.open(fileName, FILE_WRITE);
    if (!logFile) {
      Serial.println("ERROR: Could not create CSV file.");
      return false;
    }

    logFile.println("time_ms,ax,ay,az,gx,gy,gz,temp");
    logFile.close();
  }

  Serial.println("SD logger ready.");
  return true;
}

void logIMUToCSV(unsigned long time_ms,
                 float ax,
                 float ay,
                 float az,
                 float gx,
                 float gy,
                 float gz,
                 float temp) {
  logFile = SD.open(fileName, FILE_APPEND);

  if (logFile) {
    logFile.print(time_ms);
    logFile.print(",");

    logFile.print(ax, 6);
    logFile.print(",");
    logFile.print(ay, 6);
    logFile.print(",");
    logFile.print(az, 6);
    logFile.print(",");

    logFile.print(gx, 6);
    logFile.print(",");
    logFile.print(gy, 6);
    logFile.print(",");
    logFile.print(gz, 6);
    logFile.print(",");

    logFile.println(temp, 6);

    logFile.close();
  } else {
    Serial.println("ERROR: Failed to open CSV file for writing.");
  }
}





/*


//In Setup add this
if (!initSDLogger()) {
  while (1) {
    delay(100);
  }
}


//Add this after IMU data is gathered
logIMUToCSV(millis(), ax, ay, az, gx, gy, gz, tempC);


*/





