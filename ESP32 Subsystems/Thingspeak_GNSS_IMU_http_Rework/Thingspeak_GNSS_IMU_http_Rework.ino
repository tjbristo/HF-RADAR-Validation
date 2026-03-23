#include <WalterModem.h>
#include <inttypes.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <Adafruit_MPU6050_0x98.h>
#include <math.h>
#include <string.h>

#define SDA_PIN 12
#define SCL_PIN 11

const uint32_t GNSS_FIX_PERIOD_MS   = 30000;
const uint32_t GNSS_FIX_TIMEOUT_MS  = 90000;
const uint32_t THINGSPEAK_UPLOAD_MS = 600000;
const size_t   THINGSPEAK_UPLOAD_BATCH_SIZE = 20;
const uint32_t IMU_SAMPLE_MS        = 20;

const size_t GNSS_BUFFER_CAPACITY   = 120;
const size_t IMU_BUFFER_CAPACITY    = 600;
const size_t GNSS_HTTP_BATCH_LIMIT  = 20;
const size_t HTTP_BODY_CAPACITY     = 4096;
const uint32_t HTTP_WAIT_MS         = 30000;

const mpu6050_accel_range_t IMU_ACCEL_RANGE = MPU6050_RANGE_8_G;
const mpu6050_gyro_range_t  IMU_GYRO_RANGE  = MPU6050_RANGE_500_DEG;
const mpu6050_bandwidth_t   IMU_BANDWIDTH   = MPU6050_BAND_21_HZ;

#define MAX_GNSS_CONFIDENCE 100.0

#define THINGSPEAK_CHANNEL_ID        "3230022"
#define THINGSPEAK_WRITE_API_KEY     "YOUR_WRITE_API_KEY"
#define THINGSPEAK_HTTP_HOST         "api.thingspeak.com"
#define THINGSPEAK_HTTP_PORT         80
#define THINGSPEAK_HTTP_PROFILE      1
#define THINGSPEAK_BULK_ENDPOINT     "/channels/" THINGSPEAK_CHANNEL_ID "/bulk_update.json"

WalterModem modem;
Adafruit_MPU6050 mpu;

volatile bool gnssFixRcvd = false;
volatile bool assistanceUpdateRcvd = false;
WMGNSSFixEvent latestGnssFix = {};

bool gnssRequestInProgress = false;
uint32_t gnssRequestStartMs = 0;
uint32_t nextGnssRequestMs = 0;
uint32_t nextThingSpeakUploadMs = 0;
uint32_t lastImuSampleMs = 0;

volatile bool httpResponseReady = false;
volatile int httpLastStatus = -1;
volatile uint16_t httpLastDataLen = 0;
char httpRxBuf[512] = {0};

struct GNSSSample {
  uint32_t t_ms = 0;
  int64_t unixTime = 0;
  double latitude = 0.0;
  double longitude = 0.0;
  float confidence = 0.0f;
  uint8_t satCount = 0;
};

struct ImuSample {
  uint32_t t_ms = 0;
  float ax = 0.0f;
  float ay = 0.0f;
  float az = 0.0f;
  float gx = 0.0f;
  float gy = 0.0f;
  float gz = 0.0f;
};

GNSSSample gnssBuffer[GNSS_BUFFER_CAPACITY];
size_t gnssHead = 0;
size_t gnssCount = 0;
uint32_t gnssDroppedSamples = 0;

ImuSample imuBuffer[IMU_BUFFER_CAPACITY];
size_t imuHead = 0;
size_t imuCount = 0;
uint32_t imuDroppedSamples = 0;

static void pushGNSSSample(const GNSSSample& s) {
  if (gnssCount < GNSS_BUFFER_CAPACITY) {
    size_t idx = (gnssHead + gnssCount) % GNSS_BUFFER_CAPACITY;
    gnssBuffer[idx] = s;
    gnssCount++;
  } else {
    gnssBuffer[gnssHead] = s;
    gnssHead = (gnssHead + 1) % GNSS_BUFFER_CAPACITY;
    gnssDroppedSamples++;
  }
}

static void pushIMUSample(const ImuSample& s) {
  if (imuCount < IMU_BUFFER_CAPACITY) {
    size_t idx = (imuHead + imuCount) % IMU_BUFFER_CAPACITY;
    imuBuffer[idx] = s;
    imuCount++;
  } else {
    imuBuffer[imuHead] = s;
    imuHead = (imuHead + 1) % IMU_BUFFER_CAPACITY;
    imuDroppedSamples++;
  }
}

static const GNSSSample* latestGNSSBufferedSample() {
  if (gnssCount == 0) return nullptr;
  size_t idx = (gnssHead + gnssCount - 1) % GNSS_BUFFER_CAPACITY;
  return &gnssBuffer[idx];
}

static void popGNSSSamples(size_t n) {
  if (n > gnssCount) n = gnssCount;
  gnssHead = (gnssHead + n) % GNSS_BUFFER_CAPACITY;
  gnssCount -= n;
  if (gnssCount == 0) gnssHead = 0;
}

static const ImuSample* latestIMUBufferedSample() {
  if (imuCount == 0) return nullptr;
  size_t idx = (imuHead + imuCount - 1) % IMU_BUFFER_CAPACITY;
  return &imuBuffer[idx];
}

static void clearIMUBuffer() {
  imuHead = 0;
  imuCount = 0;
}

static void flushIMUBufferToFutureSDHook() {
}

static bool lteConnected() {
  WalterModemNetworkRegState regState = modem.getNetworkRegState();
  return (regState == WALTER_MODEM_NETWORK_REG_REGISTERED_HOME ||
          regState == WALTER_MODEM_NETWORK_REG_REGISTERED_ROAMING);
}

static bool lteConnect() {
  if (lteConnected()) return true;

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
  if (!modem.setOpState(WALTER_MODEM_OPSTATE_MINIMUM)) {
    Serial.println("Error: Could not set op state MINIMUM");
    return false;
  }

  WalterModemNetworkRegState regState = modem.getNetworkRegState();
  while (regState != WALTER_MODEM_NETWORK_REG_NOT_SEARCHING) {
    delay(100);
    regState = modem.getNetworkRegState();
  }

  Serial.println("LTE set to MINIMUM");
  return true;
}

static void gnssEventHandler(WMGNSSEventType type, const WMGNSSEventData* data, void* args) {
  (void)args;

  switch (type) {
    case WALTER_MODEM_GNSS_EVENT_FIX:
      memcpy(&latestGnssFix, &data->gnssfix, sizeof(WMGNSSFixEvent));
      gnssFixRcvd = true;
      Serial.printf("GNSS fix received: Lat %.6f, Lon %.6f, Conf %.2f, Sats %d",
                    latestGnssFix.latitude,
                    latestGnssFix.longitude,
                    latestGnssFix.estimatedConfidence,
                    latestGnssFix.satCount);
      break;

    case WALTER_MODEM_GNSS_EVENT_ASSISTANCE:
      assistanceUpdateRcvd = true;
      if (data->assistance == WALTER_MODEM_GNSS_ASSISTANCE_TYPE_ALMANAC) {
        Serial.println("GNSS assistance: Almanac updated");
      } else if (data->assistance == WALTER_MODEM_GNSS_ASSISTANCE_TYPE_REALTIME_EPHEMERIS) {
        Serial.println("GNSS assistance: Ephemeris updated");
      } else if (data->assistance == WALTER_MODEM_GNSS_ASSISTANCE_TYPE_PREDICTED_EPHEMERIS) {
        Serial.println("GNSS assistance: Predicted ephemeris updated");
      }
      break;

    default:
      break;
  }
}

static void httpEventHandler(WMHTTPEventType event, const WMHTTPEventData* data, void* args) {
  (void)args;

  switch (event) {
    case WALTER_MODEM_HTTP_EVENT_CONNECTED:
      if (data->rc == 0) {
        Serial.printf("HTTP connected (profile %d)\n", data->profile_id);
      } else {
        Serial.printf("HTTP connect failed, CURL %d\n", data->rc);
      }
      break;

    case WALTER_MODEM_HTTP_EVENT_DISCONNECTED:
      Serial.printf("HTTP disconnected (profile %d)\n", data->profile_id);
      break;

    case WALTER_MODEM_HTTP_EVENT_CONNECTION_CLOSED:
      Serial.printf("HTTP closed, CURL %d\n", data->rc);
      break;

    case WALTER_MODEM_HTTP_EVENT_RING: {
      httpLastStatus = data->status;
      httpLastDataLen = data->data_len;

      size_t rxLen = data->data_len;
      if (rxLen >= sizeof(httpRxBuf)) rxLen = sizeof(httpRxBuf) - 1;
      memset(httpRxBuf, 0, sizeof(httpRxBuf));

      if (modem.httpReceive(data->profile_id, (uint8_t*)httpRxBuf, rxLen)) {
        httpRxBuf[rxLen] = '\0';
        Serial.printf("HTTP status %d, %u bytes\n", data->status, data->data_len);
        Serial.printf("HTTP body: %s\n", httpRxBuf);
      } else {
        Serial.println("HTTP receive failed");
        httpRxBuf[0] = '\0';
      }

      httpResponseReady = true;
      break;
    }
  }
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

  Serial.println("GNSS clock invalid at boot");
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

  Serial.println("Clock sync failed");
  return false;
}

static bool checkAssistanceStatus(WalterModemRsp* rsp, bool* updateAlmanac, bool* updateEphemeris) {
  if (updateAlmanac) *updateAlmanac = false;
  if (updateEphemeris) *updateEphemeris = false;

  if (!modem.gnssGetAssistanceStatus(rsp) ||
      rsp->type != WALTER_MODEM_RSP_DATA_TYPE_GNSS_ASSISTANCE_DATA) {
    Serial.println("Could not get GNSS assistance status");
    return false;
  }

  const WMGNSSAssistance& almanac =
      rsp->data.gnssAssistance[WALTER_MODEM_GNSS_ASSISTANCE_TYPE_ALMANAC];
  const WMGNSSAssistance& eph =
      rsp->data.gnssAssistance[WALTER_MODEM_GNSS_ASSISTANCE_TYPE_REALTIME_EPHEMERIS];

  Serial.printf("Almanac: %s, ttu=%ld\n", almanac.available ? "ok" : "missing", almanac.timeToUpdate);
  Serial.printf("Ephemeris: %s, ttu=%ld\n", eph.available ? "ok" : "missing", eph.timeToUpdate);

  if (updateAlmanac) *updateAlmanac = (!almanac.available || almanac.timeToUpdate <= 0);
  if (updateEphemeris) *updateEphemeris = (!eph.available || eph.timeToUpdate <= 0);
  return true;
}

static bool waitForAssistanceEvent(uint32_t timeoutMs) {
  uint32_t start = millis();
  while ((millis() - start) < timeoutMs) {
    if (assistanceUpdateRcvd) return true;
    delay(50);
  }
  return false;
}

static bool updateGNSSAssistance() {
  WalterModemRsp rspLocal = {};
  bool updateAlmanac = false;
  bool updateEphemeris = false;

  if (!checkAssistanceStatus(&rspLocal, &updateAlmanac, &updateEphemeris)) {
    return false;
  }

  if (!updateAlmanac && !updateEphemeris) {
    Serial.println("GNSS assistance already valid");
    return true;
  }

  if (!lteConnect()) {
    Serial.println("LTE connect failed for assistance");
    return false;
  }

  if (updateAlmanac) {
    assistanceUpdateRcvd = false;
    if (!modem.gnssUpdateAssistance(WALTER_MODEM_GNSS_ASSISTANCE_TYPE_ALMANAC)) {
      Serial.println("Almanac update failed");
      return false;
    }
    if (!waitForAssistanceEvent(60000)) {
      Serial.println("Almanac update timed out");
      return false;
    }
  }

  if (updateEphemeris) {
    assistanceUpdateRcvd = false;
    if (!modem.gnssUpdateAssistance(WALTER_MODEM_GNSS_ASSISTANCE_TYPE_REALTIME_EPHEMERIS)) {
      Serial.println("Ephemeris update failed");
      return false;
    }
    if (!waitForAssistanceEvent(60000)) {
      Serial.println("Ephemeris update timed out");
      return false;
    }
  }

  return checkAssistanceStatus(&rspLocal, nullptr, nullptr);
}

static bool isValidFix(const WMGNSSFixEvent& f) {
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
    Serial.println("GNSS stop: not needed");
  } else {
    Serial.println("GNSS stop: ERROR");
  }
}

static bool startGnssFixRequest() {
  if (gnssRequestInProgress) return true;

  if (!syncClockAtBootIfNeeded()) {
    Serial.println("GNSS clock invalid");
    return false;
  }

  if (!updateGNSSAssistance()) {
    Serial.println("GNSS assistance update skipped");
  }

  if (lteConnected()) lteSleep();

  gnssFixRcvd = false;
  WalterModemRsp rspLocal = {};
  if (!modem.gnssPerformAction(WALTER_MODEM_GNSS_ACTION_GET_SINGLE_FIX, &rspLocal)) {
    Serial.println("Error: Could not request GNSS fix");
    return false;
  }

  gnssRequestInProgress = true;
  gnssRequestStartMs = millis();
  Serial.println("GNSS fix requested");
  return true;
}

static void storeAcceptedFix(const WMGNSSFixEvent& f, uint32_t nowMs) {
  GNSSSample s;
  s.t_ms = nowMs;
  s.unixTime = f.timestamp;
  s.latitude = f.latitude;
  s.longitude = f.longitude;
  s.confidence = f.estimatedConfidence;
  s.satCount = f.satCount;
  pushGNSSSample(s);
}

static void serviceGnss(uint32_t now) {
  if (!gnssRequestInProgress) {
    if ((int32_t)(now - nextGnssRequestMs) >= 0) {
      startGnssFixRequest();
      nextGnssRequestMs = now + GNSS_FIX_PERIOD_MS;
    }
    return;
  }

  if (gnssFixRcvd) {
    gnssFixRcvd = false;
    gnssRequestInProgress = false;

    if (isValidFix(latestGnssFix)) {
      Serial.println("GNSS fix accepted");
      storeAcceptedFix(latestGnssFix, now);
    } else {
      Serial.println("GNSS fix rejected");
    }

    gnssStopWithNiceMessage(true);
    return;
  }

  if ((now - gnssRequestStartMs) >= GNSS_FIX_TIMEOUT_MS) {
    Serial.println("GNSS timed out");
    gnssRequestInProgress = false;
    gnssStopWithNiceMessage(false);
  }
}

static void sampleAndStoreIMU(uint32_t now) {
  sensors_event_t a, g;
  mpu.getEvent(&a, &g);

  ImuSample s;
  s.t_ms = now;
  s.ax = a.acceleration.x;
  s.ay = a.acceleration.y;
  s.az = a.acceleration.z;
  s.gx = g.gyro.x;
  s.gy = g.gyro.y;
  s.gz = g.gyro.z;

  pushIMUSample(s);
}

static size_t buildBulkJSON(char* out, size_t outSize, size_t sampleCount, unsigned packetBytesField) {
  if (outSize == 0 || sampleCount == 0) return 0;

  size_t used = 0;
  int n = snprintf(out + used, outSize - used,
                   "{\"write_api_key\":\"%s\",\"updates\":[",
                   THINGSPEAK_WRITE_API_KEY);
  if (n < 0 || (size_t)n >= (outSize - used)) return 0;
  used += (size_t)n;

  size_t lastIdx = (gnssHead + sampleCount - 1) % GNSS_BUFFER_CAPACITY;
  int64_t newestTime = gnssBuffer[lastIdx].unixTime;

  for (size_t i = 0; i < sampleCount; ++i) {
    size_t idx = (gnssHead + i) % GNSS_BUFFER_CAPACITY;
    const GNSSSample& s = gnssBuffer[idx];
    bool isLast = (i == sampleCount - 1);
    unsigned packetField = isLast ? packetBytesField : 0u;
    int64_t delta_t = s.unixTime - newestTime;

    n = snprintf(out + used, outSize - used,
                 "%s{\"delta_t\":%" PRIi64 ",\"field1\":%.6f,\"field2\":%.6f,\"field3\":%.2f,\"field4\":%u,\"field5\":%u}",
                 (i == 0 ? "" : ","),
                 delta_t,
                 s.latitude,
                 s.longitude,
                 s.confidence,
                 (unsigned)s.satCount,
                 packetField);

    if (n < 0 || (size_t)n >= (outSize - used)) return 0;
    used += (size_t)n;
  }

  n = snprintf(out + used, outSize - used, "]}");
  if (n < 0 || (size_t)n >= (outSize - used)) return 0;
  used += (size_t)n;

  return used;
}

static bool waitForHTTPResponse(uint32_t timeoutMs) {
  uint32_t start = millis();
  while ((millis() - start) < timeoutMs) {
    if (httpResponseReady) return true;
    delay(10);
  }
  return false;
}

static bool uploadBufferedGNSSBulk() {
  if (gnssCount == 0) {
    Serial.println("No GNSS samples to upload");
    return true;
  }

  size_t sampleCount = gnssCount;
  if (sampleCount > GNSS_HTTP_BATCH_LIMIT) sampleCount = GNSS_HTTP_BATCH_LIMIT;

  static char body[HTTP_BODY_CAPACITY];
  size_t len1 = buildBulkJSON(body, sizeof(body), sampleCount, 0u);
  if (len1 == 0) {
    Serial.println("JSON build failed");
    return false;
  }

  size_t len2 = buildBulkJSON(body, sizeof(body), sampleCount, (unsigned)len1);
  if (len2 == 0) {
    Serial.println("JSON rebuild failed");
    return false;
  }

  size_t len3 = buildBulkJSON(body, sizeof(body), sampleCount, (unsigned)len2);
  if (len3 == 0) {
    Serial.println("JSON final build failed");
    return false;
  }

  if (!lteConnect()) {
    Serial.println("LTE connect failed");
    lteSleep();
    return false;
  }

  httpResponseReady = false;
  httpLastStatus = -1;
  httpLastDataLen = 0;
  httpRxBuf[0] = '\0';

  Serial.printf("HTTP bulk upload: %u samples, %u bytes\n", (unsigned)sampleCount, (unsigned)len3);

  char ctBuf[32] = {0};
  bool queued = modem.httpSend(THINGSPEAK_HTTP_PROFILE,
                               THINGSPEAK_BULK_ENDPOINT,
                               (uint8_t*)body,
                               (uint16_t)len3,
                               WALTER_MODEM_HTTP_SEND_CMD_POST,
                               WALTER_MODEM_HTTP_POST_PARAM_JSON,
                               ctBuf,
                               sizeof(ctBuf));

  if (!queued) {
    Serial.println("HTTP POST failed to start");
    lteSleep();
    return false;
  }

  if (!waitForHTTPResponse(HTTP_WAIT_MS)) {
    Serial.println("HTTP response timeout");
    lteSleep();
    return false;
  }

  bool ok = (httpLastStatus == 200) && (strstr(httpRxBuf, "success") != nullptr);
  if (ok) {
    popGNSSSamples(sampleCount);
    Serial.printf("ThingSpeak bulk upload OK, sent %u samples\n", (unsigned)sampleCount);
  } else {
    Serial.printf("ThingSpeak bulk upload failed, status %d\n", httpLastStatus);
  }

  lteSleep();
  return ok;
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("Walter GNSS + IMU + ThingSpeak bulk HTTP");

  modem.gnssSetEventHandler(gnssEventHandler, NULL);

  if (!WalterModem::begin(&Serial2)) {
    Serial.println("Modem initialization ERROR");
    while (1) delay(100);
  }
  Serial.println("Modem initialization OK");

  modem.setHTTPEventHandler(httpEventHandler, NULL);
  if (!modem.httpConfigProfile(THINGSPEAK_HTTP_PROFILE,
                               THINGSPEAK_HTTP_HOST,
                               THINGSPEAK_HTTP_PORT)) {
    Serial.println("HTTP profile config failed");
    while (1) delay(100);
  }
  Serial.println("HTTP profile OK");

  lteSleep();
  (void)syncClockAtBootIfNeeded();
  lteSleep();

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

  uint32_t now = millis();
  nextGnssRequestMs = now;
  nextThingSpeakUploadMs = now + THINGSPEAK_UPLOAD_MS;
  lastImuSampleMs = now;
}

void loop() {
  uint32_t now = millis();

  if (now - lastImuSampleMs >= IMU_SAMPLE_MS) {
    lastImuSampleMs += IMU_SAMPLE_MS;
    sampleAndStoreIMU(now);
  }

  serviceGnss(now);

  bool timeToUpload = ((int32_t)(now - nextThingSpeakUploadMs) >= 0);
  bool bufferFull = (gnssCount >= THINGSPEAK_UPLOAD_BATCH_SIZE);

  if (timeToUpload || bufferFull) {
    if (timeToUpload) {
      nextThingSpeakUploadMs += THINGSPEAK_UPLOAD_MS;
      if ((int32_t)(now - nextThingSpeakUploadMs) >= 0) {
        nextThingSpeakUploadMs = now + THINGSPEAK_UPLOAD_MS;
      }
    } else {
      nextThingSpeakUploadMs = now + THINGSPEAK_UPLOAD_MS;
    }

    uploadBufferedGNSSBulk();
  }

  delay(5);
}
