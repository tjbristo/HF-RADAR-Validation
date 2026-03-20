#include <WalterModem.h>
#include <inttypes.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <Adafruit_MPU6050_0x98.h> //patched library

// ===================== USER SETTINGS =====================
#define SDA_PIN 12
#define SCL_PIN 11

const uint32_t UPLOAD_INTERVAL_MS = 120000;   // 60s cadence
const uint32_t GNSS_WAIT_MS       = 90000;   // up to 30s GNSS attempt per cycle

const uint32_t IMU_SAMPLE_MS      = 20  ;      // 50 Hz

// ---- IMU configuration (range + bandwidth) ----
const mpu6050_accel_range_t IMU_ACCEL_RANGE = MPU6050_RANGE_8_G;
const mpu6050_gyro_range_t  IMU_GYRO_RANGE  = MPU6050_RANGE_500_DEG;
const mpu6050_bandwidth_t   IMU_BANDWIDTH   = MPU6050_BAND_21_HZ;
// ---------------------------------------------

// GNSS confidence filter (you said you’re fine with lowering this)
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

uint32_t nextUploadMs = 0;          // fixed cadence schedule
uint32_t lastImuSampleMs = 0;

// ===================== IMU DATA (RAW ONLY, NOT UPLOADED) =====================
struct ImuSample {
  uint32_t t_ms = 0;
  float ax = 0.0f;  // m/s^2
  float ay = 0.0f;
  float az = 0.0f;
  float gx = 0.0f;  // rad/s
  float gy = 0.0f;
  float gz = 0.0f;
};
ImuSample latestImu;

// ===================== WAVE METRICS PLACEHOLDERS (UPLOADED) =====================
// Only these are uploaded to ThingSpeak.
struct WaveMetrics {
  float height_m = NAN;   // TODO compute
  float period_s = NAN;   // TODO compute
};
WaveMetrics wave;

// Put whatever you need here later (buffers, filters, peak detection state, etc.)
struct WaveEstimatorState {
  // TODO: add estimator state
};
WaveEstimatorState waveState;

// Called every IMU sample. Leave math blank for now.
static void updateWaveEstimates(const ImuSample& s, WaveEstimatorState& st, WaveMetrics& out) {
  (void)s;
  (void)st;
  (void)out;
  // TODO:
  // - derive heave / vertical displacement proxy
  // - detect peaks/troughs
  // - update out.height_m and out.period_s
}

// Optional: called once per 60s upload cycle.
static void finalizeWaveWindow(WaveEstimatorState& st, WaveMetrics& out) {
  (void)st;
  (void)out;
  // TODO:
  // - compute 60s summary (e.g., significant wave height, dominant period)
  // - reset any windowed state
}

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
  if (!modem.mqttConfig(THINGSPEAK_MQTT_CLIENT_ID,
                        THINGSPEAK_MQTT_USERNAME,
                        THINGSPEAK_MQTT_PASSWORD)) {
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
  bool ok = modem.mqttPublish(THINGSPEAK_TOPIC,
                             (uint8_t*)payload,
                             strlen(payload),
                             0);
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
                latestGnssFix.latitude,
                latestGnssFix.longitude,
                latestGnssFix.estimatedConfidence,
                latestGnssFix.satCount);
}

static bool gnssClockValid() {
  WalterModemRsp rspLocal = {};
  modem.gnssGetUTCTime(&rspLocal);
  return (rspLocal.data.clock.epochTime > 4);
}

// One-time clock sync at boot (before starting the cadence).
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

// Valid fix gating: avoids accepting the default-ish 0,0 / huge confidence “fix”.
static bool isValidFix(const WalterModemGNSSFix& f) {
  if (f.estimatedConfidence > MAX_GNSS_CONFIDENCE) return false;  // confidence must be <= 100
  if (f.satCount < 4) return false;

  // Reject the common “not really fixed yet” placeholder (0,0).
  // (If you ever *actually* need (0,0), remove this.)
  if (f.latitude == 0.0 && f.longitude == 0.0) return false;

  // Timestamp should be non-zero in a real fix (extra sanity)
  if (f.timestamp <= 0) return false;

  return true;
}

// Stop GNSS “fix program”. IMPORTANT:
// - After a successful fix, the modem often auto-stops the fix program.
// - Sending "stop" after that can return ERROR -> your old code printed a scary warning.
// Here we still *check* stop behavior, but we don’t print a “failed” warning on the normal case.
static void gnssStopWithNiceMessage(bool fixWasReceived) {
  WalterModemRsp rspLocal = {};
  bool ok = modem.gnssPerformAction(WALTER_MODEM_GNSS_ACTION_CANCEL, &rspLocal);

  if (ok) {
    Serial.println("GNSS stop: OK");
    return;
  }

  if (fixWasReceived) {
    // This is the common/expected case: fix program already ended, so stop may error.
    Serial.println("GNSS stop: not needed (already stopped by modem)");
  } else {
    // This one is worth knowing about because you timed out and wanted to stop GNSS.
    Serial.println("GNSS stop: ERROR (stop command failed)");
  }
}

// Attempt a single GNSS fix with a 30s budget (LTE OFF).
static bool getGnssFixThisCycle(bool& fixEventReceivedOut) {
  fixEventReceivedOut = false;

  // If time isn't valid, skip (we only LTE-sync at boot).
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
    delay(200);
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
void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("Walter GNSS+IMU upload (wave metrics only)");

  modem.gnssSetEventHandler(gnssEventHandler, NULL);

  if (!WalterModem::begin(&Serial2)) {
    Serial.println("Modem initialization ERROR");
    while (1) delay(100);
  }
  Serial.println("Modem initialization OK");

  // Start asleep
  lteSleep();

  // ---- ONE-TIME CLOCK SYNC BEFORE STARTING CADENCE ----
  (void)syncClockAtBootIfNeeded();
  lteSleep(); // ensure we go back to MINIMUM after sync attempt

  // IMU init
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  if (!mpu.begin(0x68, &Wire)) {
    Serial.println("Failed to find MPU6050");
    while (1) delay(100);
  }
  Serial.println("IMU OK");

  mpu.setAccelerometerRange(IMU_ACCEL_RANGE);
  mpu.setGyroRange(IMU_GYRO_RANGE);
  mpu.setFilterBandwidth(IMU_BANDWIDTH);

  // IMPORTANT: run first cycle immediately AFTER boot sync + checks
  uint32_t now = millis();
  nextUploadMs = now;
  lastImuSampleMs = now;

  wave.height_m = NAN;
  wave.period_s = NAN;
}

void loop() {
  uint32_t now = millis();

  // 1) IMU sampling (raw only) + estimator hook
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

    updateWaveEstimates(latestImu, waveState, wave);
  }

  // 2) Fixed-cadence cycle tick
  if ((int32_t)(now - nextUploadMs) >= 0) {
    // maintain cadence
    nextUploadMs += UPLOAD_INTERVAL_MS;
    if ((int32_t)(now - nextUploadMs) >= 0) {
      nextUploadMs = now + UPLOAD_INTERVAL_MS;
    }

    finalizeWaveWindow(waveState, wave);

    // ---- GNSS window (LTE OFF) ----
    if (lteConnected()) lteSleep();

    bool fixEventReceived = false;
    bool gotValidFix = getGnssFixThisCycle(fixEventReceived);

    // Only attempt STOP in a way that won't spam scary errors:
    // - If we timed out (no event), stop is meaningful and failure is worth printing.
    // - If we did get an event, stop is usually not needed; we'll print a friendly message if it errors.
    if (!fixEventReceived) {
      gnssStopWithNiceMessage(false);
    } else {
      gnssStopWithNiceMessage(true);
    }

    // ---- LTE/MQTT upload window (wave only) ----
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

    // field1 lat, field2 lon (if valid fix)
    // field3 wave height (m), field4 wave period (s)
    static char msg[256];
    if (gotValidFix) {
      snprintf(msg, sizeof(msg),
               "field1=%.6f&field2=%.6f&field3=%.4f&field4=%.4f",
               latestGnssFix.latitude,
               latestGnssFix.longitude,
               wave.height_m,
               wave.period_s);
    } else {
      snprintf(msg, sizeof(msg),
               "field3=%.4f&field4=%.4f",
               wave.height_m,
               wave.period_s);
    }

    publishTS(msg);

    // Back to sleep until next tick
    lteSleep();
  }

  delay(5);
}