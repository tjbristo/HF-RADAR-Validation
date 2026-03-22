

/*  READ ME READ ME READ ME!!!!!!!!!!!!!
Some basic information for the file below and how it works, lteconencted() - Tells the walter modem to turn the cellular radio on,
lteConnect() - Tells it to connect to a tower, lteSleep() - Tells the modem to turn off to save power.
mqttConnectThingspeak() - Logs into thingspeak with the given username and password
publishTS() - This takes a string of data and actually pushes it out over the internet to the dedicated thingspeak channel
gnssEventHandler() - Its a callback function that checks when the modem hardware is finally locked on to satellites, it will 
interrupt the code to run, saving latitude and longitude into the latestGnssFix variable.
syncClockAtBootIfNeeded() - GNSS needs a rough idea of time for it to find satellites, this checks the modem to see if it knows the
time, if not, it will reach out to a LTE tower to figure out the time.
isValidFix() - Quality control check, it makes sure the GNSS data has enough satellites for accurate reporting.
*/






#include <WalterModem.h>
#include <inttypes.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <Adafruit_MPU6050_0x98.h> // patched library

// ---- SD Card Libraries ----
#include <SPI.h>
#include <SD.h>

// ===================== USER SETTINGS =====================
#define SDA_PIN 12
#define SCL_PIN 11

// SPI Chip Select Pin for SD Card
#define SD_CS_PIN 5 


const uint32_t UPLOAD_INTERVAL_MS = 120000; // 120s cadence
const uint32_t GNSS_WAIT_MS       = 90000;  // up to 90s GNSS attempt per cycle
const uint32_t IMU_SAMPLE_MS      = 20;     // 50 Hz
const uint32_t SD_LOG_INTERVAL_MS = 15000;  // 15s SD logging cadence

// ---- IMU configuration (range + bandwidth) ----
const mpu6050_accel_range_t IMU_ACCEL_RANGE = MPU6050_RANGE_8_G;
const mpu6050_gyro_range_t  IMU_GYRO_RANGE  = MPU6050_RANGE_500_DEG;
const mpu6050_bandwidth_t   IMU_BANDWIDTH   = MPU6050_BAND_21_HZ;
// ---------------------------------------------

// GNSS confidence filter
#define MAX_GNSS_CONFIDENCE 100.0

// ThingSpeak MQTT
#define THINGSPEAK_MQTT_USERNAME  "MzcYMQcDEg0eMTYfMRMBKyI"
#define THINGSPEAK_MQTT_CLIENT_ID "MzcYMQcDEg0eMTYfMRMBKyI"
#define THINGSPEAK_MQTT_PASSWORD  "jnV0bx3kzd8ysQElnuwh9LWy"
#define THINGSPEAK_CHANNEL_ID "3230022"
#define THINGSPEAK_TOPIC "channels/" THINGSPEAK_CHANNEL_ID "/publish"
// ==========================================================

WalterModem modem;
Adafruit_MPU6050 mpu;

volatile bool gnssFixRcvd = false;
WalterModemGNSSFix latestGnssFix = {};

bool mqttConnected = false;
bool sdInitialized = false;

uint32_t nextUploadMs = 0;
uint32_t lastImuSampleMs = 0;
uint32_t lastSdLogMs = 0;

// ===================== IMU DATA =====================
struct ImuSample {
  uint32_t t_ms = 0;
  float ax = 0.0f;  // m/s^2
  float ay = 0.0f;
  float az = 0.0f;
  float gx = 0.0f;
  float gy = 0.0f;
  float gz = 0.0f;
};
ImuSample latestImu;

// ===================== LTE / MQTT HELPERS =====================
static bool lteConnected() {
  WalterModemNetworkRegState regState = modem.getNetworkRegState();
  return (regState == WALTER_MODEM_NETWORK_REG_REGISTERED_HOME ||
          regState == WALTER_MODEM_NETWORK_REG_REGISTERED_ROAMING);
}

static bool lteConnect() {
  if (!modem.setOpState(WALTER_MODEM_OPSTATE_NO_RF)) return false;
  if (!modem.definePDPContext()) return false;
  if (!modem.setOpState(WALTER_MODEM_OPSTATE_FULL)) return false;
  if (!modem.setNetworkSelectionMode(WALTER_MODEM_NETWORK_SEL_MODE_AUTOMATIC)) return false;

  int timeout = 0;
  while (!lteConnected()) {
    delay(1000);
    timeout++;
    if (timeout > 300) {
      ESP.restart();
      return false;
    }
  }
  Serial.println("Connected to LTE network");
  return true;
}

static bool lteSleep() {
  mqttConnected = false;
  if (!modem.setOpState(WALTER_MODEM_OPSTATE_MINIMUM)) {
    Serial.println("Error: Could not set op state MINIMUM");
    return false;
  }
  WalterModemNetworkRegState regState = modem.getNetworkRegState();
  while (regState != WALTER_MODEM_NETWORK_REG_NOT_SEARCHING) {
    delay(100);
    regState = modem.getNetworkRegState();
  }
  Serial.println("LTE set to MINIMUM (sleep)");
  return true;
}

static bool mqttConnectThingspeak() {
  if (!modem.mqttConfig(THINGSPEAK_MQTT_CLIENT_ID, THINGSPEAK_MQTT_USERNAME, THINGSPEAK_MQTT_PASSWORD)) {
    Serial.println("Error: MQTT config failed");
    return false;
  }
  mqttConnected = modem.mqttConnect("mqtt3.thingspeak.com", 1883);
  if (!mqttConnected) {
    Serial.println("Error: MQTT connect failed");
    return false;
  }
  Serial.println("MQTT connected");
  return true;
}

static bool publishTS(const char* payload) {
  Serial.printf("Publishing: %s\n", payload);
  bool ok = modem.mqttPublish(THINGSPEAK_TOPIC, (uint8_t*)payload, strlen(payload), 0);
  if (!ok) {
    Serial.println("Publish failed");
    mqttConnected = false;
  }
  return ok;
}

// ===================== GNSS HELPERS =====================
static void gnssEventHandler(const WalterModemGNSSFix* fix, void* args) {
  (void)args;
  memcpy(&latestGnssFix, fix, sizeof(WalterModemGNSSFix));
  gnssFixRcvd = true;
  Serial.printf("GNSS fix received: Lat %.6f, Lon %.6f, Conf %.2f, Sats %d\n",
                latestGnssFix.latitude, latestGnssFix.longitude,
                latestGnssFix.estimatedConfidence, latestGnssFix.satCount);
}

static bool gnssClockValid() {
  WalterModemRsp rspLocal = {};
  modem.gnssGetUTCTime(&rspLocal);
  return (rspLocal.data.clock.epochTime > 4);
}

static bool syncClockAtBootIfNeeded() {
  if (gnssClockValid()) {
    Serial.println("GNSS clock already valid");
    return true;
  }
  Serial.println("GNSS clock invalid at boot -> syncing via LTE briefly...");
  if (!lteConnect()) return false;

  WalterModemRsp rspLocal = {};
  for (int i = 0; i < 5; ++i) {
    modem.gnssGetUTCTime(&rspLocal);
    if (rspLocal.data.clock.epochTime > 4) {
      Serial.printf("Clock synced: %" PRIi64 "\n", rspLocal.data.clock.epochTime);
      return true;
    }
    delay(2000);
  }
  Serial.println("Clock sync failed (continuing anyway)");
  return false;
}

static bool isValidFix(const WalterModemGNSSFix& f) {
  if (f.estimatedConfidence > MAX_GNSS_CONFIDENCE) return false;
  if (f.satCount < 4) return false;
  if (f.latitude == 0.0 && f.longitude == 0.0) return false;
  if (f.timestamp <= 0) return false;
  return true;
}

static void gnssStopWithNiceMessage(bool fixWasReceived) {
  WalterModemRsp rspLocal = {};
  bool ok = modem.gnssPerformAction(WALTER_MODEM_GNSS_ACTION_CANCEL, &rspLocal);
  if (ok) {
    Serial.println("GNSS stop: OK");
    return;
  }
  if (fixWasReceived) {
    Serial.println("GNSS stop: not needed (already stopped by modem)");
  } else {
    Serial.println("GNSS stop: ERROR (stop command failed)");
  }
}

// ===================== MULTI-TASKING SYSTEM =====================
void processFastTasks(uint32_t now) { //Very important, the IMU if statement asks to check the clock, if 20 ms has passed, it will give I2C commands out
  // 1) Fast IMU Sampling (every 20ms)
  if (now - lastImuSampleMs >= IMU_SAMPLE_MS) {
    lastImuSampleMs = now;
    sensors_event_t a, g;
    mpu.getEvent(&a, &g);

    latestImu.t_ms = now;
    latestImu.ax = a.acceleration.x;
    latestImu.ay = a.acceleration.y;
    latestImu.az = a.acceleration.z;
    latestImu.gx = g.gyro.x;
    latestImu.gy = g.gyro.y;
    latestImu.gz = g.gyro.z;
  }

  // 2) SD Card Logging (every 15s)
  /*
  This part is for the SD card, it checks the clock again and makes sure 15 seconds has passed, if so it will open the SD card
  Over SPI commands and format a CSV string using the latestGnssfix and latestIMU data, write them in it, and close the file
  */
  if (now - lastSdLogMs >= SD_LOG_INTERVAL_MS) { 
    lastSdLogMs = now;
    
    if (sdInitialized) {
      File dataFile = SD.open("/datalog.csv", FILE_APPEND);
      if (dataFile) {
        // Log Format: Timestamp, Lat, Lon, Ax, Ay, Az, Gx, Gy, Gz
        dataFile.printf("%lu,%.6f,%.6f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
                        now,
                        latestGnssFix.latitude,
                        latestGnssFix.longitude,
                        latestImu.ax, latestImu.ay, latestImu.az,
                        latestImu.gx, latestImu.gy, latestImu.gz);
        dataFile.close();
        Serial.println("Data saved to SD card");
      } else {
        Serial.println("Error opening datalog.csv");
      }
    }
  }
}

static bool getGnssFixThisCycle(bool& fixEventReceivedOut) {
  fixEventReceivedOut = false;
  if (!gnssClockValid()) {
    Serial.println("GNSS clock invalid -> skipping GNSS this cycle");
    return false;
  }

  gnssFixRcvd = false;
  WalterModemRsp rspLocal = {};
  if (!modem.gnssPerformAction(WALTER_MODEM_GNSS_ACTION_GET_SINGLE_FIX, &rspLocal)) {
    Serial.println("Error: Could not request GNSS fix");
    return false;
  }
  Serial.println("GNSS fix requested");

  uint32_t start = millis();
  while (!gnssFixRcvd && (millis() - start) < GNSS_WAIT_MS) {
    processFastTasks(millis());
    delay(5); 
  }

  if (!gnssFixRcvd) {
    Serial.println("GNSS timed out (no fix event)");
    return false;
  }

  fixEventReceivedOut = true;
  if (isValidFix(latestGnssFix)) {
    Serial.println("GNSS fix accepted");
    return true;
  }
  Serial.println("GNSS fix rejected (did not pass validity checks)");
  return false;
}

// ===================== SETUP / LOOP =====================
void setup() { // Starts serial monitor, turns on modem.
  Serial.begin(115200);
  delay(2000);
  Serial.println("Walter GNSS+IMU upload (ThingSpeak GNSS + SD Card Logging)");

  modem.gnssSetEventHandler(gnssEventHandler, NULL);
  if (!WalterModem::begin(&Serial2)) {
    Serial.println("Modem initialization ERROR");
    while (1) delay(100);
  }
  Serial.println("Modem initialization OK");
  
  lteSleep();
  (void)syncClockAtBootIfNeeded();
  lteSleep();

  // IMU init
  Wire.begin(SDA_PIN, SCL_PIN); // Starts the I2C interface for the IMU
  Wire.setClock(100000);
  if (!mpu.begin(0x68, &Wire)) {
    Serial.println("Failed to find MPU6050");
    while (1) delay(100);
  }
  Serial.println("IMU OK");

  mpu.setAccelerometerRange(IMU_ACCEL_RANGE);
  mpu.setGyroRange(IMU_GYRO_RANGE);
  mpu.setFilterBandwidth(IMU_BANDWIDTH);

  // SD Card init
  if (!SD.begin(SD_CS_PIN)) { //Boots up the SPI bus for the SD card, datalog.csv is the file created from the values reported
    Serial.println("SD Card initialization failed! Check wiring.");
  } else {
    Serial.println("SD Card OK");
    sdInitialized = true;
    if (!SD.exists("/datalog.csv")) {
      File dataFile = SD.open("/datalog.csv", FILE_WRITE);
      if (dataFile) {
        dataFile.println("Time_ms,Latitude,Longitude,Accel_X,Accel_Y,Accel_Z,Gyro_X,Gyro_Y,Gyro_Z");
        dataFile.close();
      }
    }
  }

  uint32_t now = millis();
  nextUploadMs = now;
  lastImuSampleMs = now;
  lastSdLogMs = now;
}

void loop() {
  uint32_t now = millis();
  
  // 1) Run fast tasks (IMU + SD card)
  processFastTasks(now);

  // 2) Fixed-cadence cycle tick (GNSS + ThingSpeak upload)
  if ((int32_t)(now - nextUploadMs) >= 0) {
    nextUploadMs += UPLOAD_INTERVAL_MS;
    if ((int32_t)(now - nextUploadMs) >= 0) {
      nextUploadMs = now + UPLOAD_INTERVAL_MS;
    }

    // ---- GNSS window (LTE OFF) ----
    if (lteConnected()) lteSleep();
    bool fixEventReceived = false;
    bool gotValidFix = getGnssFixThisCycle(fixEventReceived);

    if (!fixEventReceived) {
      gnssStopWithNiceMessage(false);
    } else {
      gnssStopWithNiceMessage(true);
    }

    // ---- LTE/MQTT upload window ----
    if (gotValidFix) {
      if (!lteConnect()) {
        Serial.println("Error: LTE connect failed, skipping upload");
        lteSleep();
        return;
      }

      if (!mqttConnectThingspeak()) {
        Serial.println("Error: MQTT connect failed, skipping upload");
        lteSleep();
        return;
      }

      static char msg[128];
      snprintf(msg, sizeof(msg),
               "field1=%.6f&field2=%.6f",
               latestGnssFix.latitude, latestGnssFix.longitude);

      publishTS(msg); //This publishes the latitude and longitude values to Thingspeak
      lteSleep();
    } else {
      Serial.println("No valid GNSS fix this cycle. Skipping ThingSpeak upload.");
    }
  }

  delay(5);
}