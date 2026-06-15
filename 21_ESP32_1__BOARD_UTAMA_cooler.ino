//  PROGRAM : SISTEM CERDAS BERBASIS FUZZY LOGIC UNTUK SMART MONITOR DAN PENGENDALIAN KONDISI GUDANG PADI
//  PENULIS : [M. FAZARAHUL SOFUAN]

/*
CATATAN.
1. Board ESP32 V. 2.0.17 Core
2. Dimmer 5V
*/


// 🟪 A. INISIALISASI
// ============================================================================
// 1. LIBRARY
// ============================================================================
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <RBDdimmer.h>


// ============================================================================
// 2. PIN & LOGIN INFO
// ============================================================================
// --- UART2 (ESP32-1 → ESP32-2) ---
#define UART2_TX 17
#define UART2_RX 16

// --- BME280 ---
// SDA : 21, SCL : 22

// --- Sensor kelembapan tanah ---
#define soilMoisturePin1 34  //  1
#define soilMoisturePin2 35  //  2
#define soilMoisturePin3 32  //  3

// --- Dimmer (kipas angin AC) ---
#define DIMMER_OUT 5
#define ZEROCROSS 23

// --- Relay ---
#define humidifierPin 13    // R4
#define dehumidifierPin 12  // R3
#define RELAY_R1 19         // Relay bypass PLN      // R2
#define RELAY_R2 18         // Relay seri dimmer     // R1

// --- LED RGB ---
#define redPin 27
#define greenPin 26
#define bluePin 25

// --- Buzzer ---
#define buzzerPin 14

// --- Kipas Pendingin Komponen ---
#define RELAY_COOLER_FAN 33

// --- WiFi ---
const char* ssid = "ESP32";
const char* password = "ARDUINOMEGA";

// --- Firebase ---
String firebaseHost = "https://projek-gudang-padi-default-rtdb.asia-southeast1.firebasedatabase.app";
String firebasePath = "/DATA_GUDANG";
String firebaseKey = "---------------";


// ============================================================================
// 3. INISIALISASI OBJEK SENSOR
// ============================================================================
Adafruit_BME280 bme;                       // BME280
dimmerLamp dimmer(DIMMER_OUT, ZEROCROSS);  // Zero Crossing


// ============================================================================
// 4. STATE MACHINE
// ============================================================================
// --- PROTEKSI ---
enum ModeSistem {
  MODE_STARTUP,
  MODE_RUN,
  MODE_ERROR_TOTAL
};

ModeSistem modeSistem = MODE_STARTUP;

// --- KENDALI KIPAS ---
enum FanState {
  FAN_OFF,             // kipas mati, ZC aktif, R1 R2 OFF
  FAN_STARTING,        // arus start awal via bypass
  FAN_RUNNING_DIMMER,  // operasi normal via dimmer
};

FanState fanState = FAN_OFF;


// ============================================================================
// 5. DEFINE KONSTAN & VARIABEL GLOBAL
// ============================================================================
// --- BME 280 ---
#define SEALEVELPRESSURE_HPA (1013.25)
#define BME_I2C_ADDR 0x76
#define BME_I2C_CHECK_INTERVAL 5000UL  // 5 detik (TESTING, nanti bebas kamu ubah)
#define BME_FREEZE_THRESHOLD 0.01
#define BME_FREEZE_TIMEOUT 120000UL  // 2 MENIT

float suhuGlobal = 0.0;
float kelembapanGlobal = 0.0;
float lastSuhuBME = NAN;
float lastKelembapanBME = NAN;

bool bmeEverValid = false;
bool bmeFreezeLatched = false;

unsigned long bmeFreezeStart = 0;
unsigned long lastBmeValidRead = 0;

// --- SOIL SENSOR ---
#define ADC_VALID_MIN 800
#define ADC_SPIKE_HIGH 4090
#define ADC_RANGE_MAX 150
#define SOIL_STABLE_REQUIRED 5     // stabilisasi ADC
#define SOIL_ADC_MIN_FLOATING 200  // batas ADC floating startup
#define SOIL_ADC_MAX_INVALID 4100  // batas ADC Over-Range startup

int soilMoistureValue1 = 0, soilMoistureValue2 = 0, soilMoistureValue3 = 0;
int correctedValue1 = 0, correctedValue2 = 0, correctedValue3 = 0;

int lastAdcRange1 = 0;
int lastAdcRange2 = 0;
int lastAdcRange3 = 0;

const float SOIL_ALPHA = 0.08;     // smoothing (0.08 – 0.12 aman)
const float SOIL_DEADBAND = 0.5;  // toleransi 0.5%

float soilFiltered1 = 0;
float soilFiltered2 = 0;
float soilFiltered3 = 0;

float soilMoisturePercentage1 = 0;
float soilMoisturePercentage2 = 0;
float soilMoisturePercentage3 = 0;

bool soilFilterInit1 = false;
bool soilFilterInit2 = false;
bool soilFilterInit3 = false;

bool soilEverValid1 = false;
bool soilEverValid2 = false;
bool soilEverValid3 = false;

uint8_t soilStableCount1 = 0;
uint8_t soilStableCount2 = 0;
uint8_t soilStableCount3 = 0;

// --- KENDALI KIPAS ---
#define START_TIME 3000  // ms, bypass start
#define OFF_DELAY 5000   // ms, delay mati

float crispFanGlobal = 0.0;
float tonFanGlobal = 0.0;
float toffFanGlobal = 0.0;

unsigned long stateTimer = 0;
unsigned long offTimer = 0;

// --- HUMIDIFIER ---
#define HUMIDIFIER_PULSE_MS 700

unsigned long prevMillisHum = 0;
bool humidifierState = false;

float tonGlobal = 0.0;
float toffGlobal = 0.0;

// --- DEHUMIDIFIER ---
#define DEHUMIDIFIER_PULSE_MS 700
bool dehumidifierState = false;

// --- LED & BUZZER ---
bool buzzerState = false;

// --- KIPAS PENDINGIN KOMPONEN ---
bool kipasKomponenState = false;

// --- CHECKING AKTUATOR ---
#define CHECKING_TIMEOUT_MS 1800000UL  // 30 menit, ubah sesuai kebutuhan
unsigned long nonAmanStartTime = 0;
bool checkingAktif = false;

// --- FUZZY LOGIC ---
String kondisiSuhu = "";
String kondisiKelembapan = "";
String kondisiKadarAir = "";
String kondisiGudang = "";

float suhuDingin = 0.0, suhuIdeal = 0.0, suhuPanas = 0.0;
float kelembapanKering = 0.0, kelembapanIdeal = 0.0, kelembapanLembab = 0.0;

float rendah1 = 0.0, normal1 = 0.0, tinggi1 = 0.0;
float rendah2 = 0.0, normal2 = 0.0, tinggi2 = 0.0;
float rendah3 = 0.0, normal3 = 0.0, tinggi3 = 0.0;

float hfsRendah = 0.0, hfsNormal = 0.0, hfsTinggi = 0.0;

// --- NILAI Zi FUZZY SUGENO ---
const float ziAman = 12.5;
const float ziWaspada = 37.5;
const float ziPeringatan = 62.5;
const float ziBerbahaya = 87.5;

// --- PROTEKSI SENSOR ---
unsigned long startupStartTime = 0;
unsigned long systemStartTime = 0;

bool sistemPernahRun = false;
bool errorFirebaseSudahTerkirim = false;
bool errorPerluDikirim = false;
bool bmeError = false;
bool soil1Error = false;
bool soil2Error = false;
bool soil3Error = false;
bool anySoilError = false;
bool allSoilError = false;
bool anySensorError = false;
bool sistemErrorAktif = false;
bool stopFirebaseKirim = false;

String errorReason = "";

// --- IoT ---
bool firebaseConnected = false;
bool errorButuhKirimRealtime = false;
bool errorButuhKirimRiwayat = false;

// --- WIFI ---
bool wifiConnected = false;
bool firebaseOK = false;

int wifiRSSI = 0;

// --- NTP / TIME ---
bool ntpSudahInit = false;

// --- MEMORI ESP32 1 ---
int heapFree = 0;
int heapTotal = 0;
int heapUsed = 0;

float heapPercent = 0.0;


// ============================================================================
// 6. KALIBRASI (OFFSET SENSOR)
// ============================================================================
// --- BME 280 ---
float suhuOffset = 0;
float kelembapanOffset = 0;

// --- ADC SOIL ---
const int offsetADC1 = 0;
const int offsetADC2 = 0;
const int offsetADC3 = 0;

// --- % SOIL ---
float offset1 = 0;  
float offset2 = 0;  
float offset3 = 0;  

// --- KALIBRASI SOIL FINAL (AR991) ---
// Sensor 1
const int ADC_MIN_1 = 1245;               // ADC saat AR991 = MOIST_MAX_1 (gabah lembap)
const int ADC_MAX_1 = 2366;               // ADC saat AR991 = MOIST_MIN_1 (gabah kering)
const float MOIST_MIN_1 = 13,8;           // % AR991 terendah
const float MOIST_MAX_1 = 49.9;           // % AR991 tertinggi

// Sensor 2
const int ADC_MIN_2 = 1458;
const int ADC_MAX_2 = 2366;
const float MOIST_MIN_2 = 13,8;
const float MOIST_MAX_2 = 49.9;

// Sensor 3
const int ADC_MIN_3 = 1374;
const int ADC_MAX_3 = 2348;
const float MOIST_MIN_3 = 13,8;
const float MOIST_MAX_3 = 49.9;



// 🟪 B. PEMBACAAN SENSOR
// ============================================================================
// 1. VALIDASI SENSOR BME 280
// ============================================================================
// --- I2C hidup ---
bool cekI2CBME280() {
  Wire.beginTransmission(BME_I2C_ADDR);
  return (Wire.endTransmission() == 0);
}

//  --- NAN ---
bool cekBME_NaN(float t, float h) {
  return isnan(t) || isnan(h);
}

// --- Domain fisik ---
bool cekBME_Domain(float t, float h) {
  return (t < 10.0 || t > 39.0 || h < 15.0 || h > 95.0);
}

// --- Freeze ---
bool cekBME_Freeze(float t, float h) {
  unsigned long now = millis();
  if (!isnan(lastSuhuBME) && !isnan(lastKelembapanBME)) {
    bool freezeT = fabs(t - lastSuhuBME) < BME_FREEZE_THRESHOLD;
    bool freezeH = fabs(h - lastKelembapanBME) < BME_FREEZE_THRESHOLD;
    if (freezeT && freezeH) {
      if (bmeFreezeStart == 0) bmeFreezeStart = now;
      if (now - bmeFreezeStart > BME_FREEZE_TIMEOUT) return true;
    } else {
      bmeFreezeStart = 0;
    }
  }

  lastSuhuBME = t;
  lastKelembapanBME = h;
  return false;
}


// ============================================================================
// 2. VALIDASI SENSOR SOIL
// ============================================================================
int readADCStable(int pin, int& rangeOut) {
  const int N = 10;
  int samples[N];
  int minVal = 4095;
  int maxVal = 0;

  for (int i = 0; i < N; i++) {
    samples[i] = analogRead(pin);
    if (samples[i] < minVal) minVal = samples[i];
    if (samples[i] > maxVal) maxVal = samples[i];
    delayMicroseconds(200);
  }

  // Bubble sort
  for (int i = 0; i < N - 1; i++) {
    for (int j = 0; j < N - 1 - i; j++) {
      if (samples[j] > samples[j + 1]) {
        int tmp = samples[j];
        samples[j] = samples[j + 1];
        samples[j + 1] = tmp;
      }
    }
  }

  rangeOut = maxVal - minVal;
  return (samples[4] + samples[5]) / 2;  // median 10 sampel
}

// --- FINALISASI ADC ---
bool adcValidFinal(
  int adc,
  int range,
  uint8_t& stableCount,
  bool& everValid) {

  if (adc > ADC_SPIKE_HIGH) {
    stableCount = 0;
    return false;
  }

  if (range > ADC_RANGE_MAX) {
    stableCount = 0;
    return false;
  }

  if (adc < ADC_VALID_MIN) {
    stableCount = 0;
    return false;
  }

  stableCount++;
  if (stableCount >= SOIL_STABLE_REQUIRED) {
    everValid = true;
    return true;
  }

  return false;
}

// --- Normalisasi ADC ---
float hitungSoilIndex(int adc, int adcBasah, int adcKering) {
  if (adcKering == adcBasah) return 0.0;

  float idx = (float)(adcKering - adc) / (float)(adcKering - adcBasah);
  idx = constrain(idx, 0.0, 1.0);
  return idx;
}

// --- VALIDASI RINGAN KHUSUS STARTUP ---
bool adcValidStartup(int adc) {
  if (adc <= 0) return false;
  if (adc >= 4095) return false;
  if (adc < SOIL_ADC_MIN_FLOATING) return false;
  if (adc > SOIL_ADC_MAX_INVALID) return false;
  return true;
}



// 🟪 C. SISTEM PROTEKSI DAN VERIVIKASI SENSOR
// ============================================================================
// 1. PROTEKSI SENSOR
// ============================================================================
void Proteksi_BME280() {
  bool errI2C = !cekI2CBME280();
  bool errNaN = cekBME_NaN(suhuGlobal, kelembapanGlobal);
  bool errDomain = cekBME_Domain(suhuGlobal, kelembapanGlobal);
  bool errFreeze = cekBME_Freeze(suhuGlobal, kelembapanGlobal);

  if (errFreeze) {
    bmeFreezeLatched = true;
  }

  bmeError = errI2C || errNaN || errDomain || errFreeze;
}

void Proteksi_Soil1() {
  static uint8_t failCount1 = 0;
  if (soilMoisturePercentage1 == -1) {
    if (failCount1 < 25) failCount1++;
  } else {
    failCount1 = 0;
  }

  soil1Error = (failCount1 >= 25);
}

void Proteksi_Soil2() {
  static uint8_t failCount2 = 0;
  if (soilMoisturePercentage2 == -1) {
    if (failCount2 < 25) failCount2++;
  } else {
    failCount2 = 0;
  }

  soil2Error = (failCount2 >= 25);
}

void Proteksi_Soil3() {
  static uint8_t failCount3 = 0;
  if (soilMoisturePercentage3 == -1) {
    if (failCount3 < 25) failCount3++;
  } else {
    failCount3 = 0;
  }

  soil3Error = (failCount3 >= 25);
}

void Proteksi_EvaluasiSensor() {
  Proteksi_BME280();
  Proteksi_Soil1();
  Proteksi_Soil2();
  Proteksi_Soil3();

  anySoilError =
    soil1Error || soil2Error || soil3Error;
  allSoilError =
    soil1Error && soil2Error && soil3Error;
  anySensorError =
    bmeError || anySoilError;

  int errorCount = 0;
  if (bmeError) errorCount++;
  if (soil1Error) errorCount++;
  if (soil2Error) errorCount++;
  if (soil3Error) errorCount++;

  if (errorCount == 4) {
    errorReason = "ERROR SYSTEM";
    return;
  }

  errorReason = "";
  bool first = true;

  if (bmeError) {
    errorReason += "BME280";
    first = false;
  }
  if (soil1Error) {
    if (!first) errorReason += ", ";
    errorReason += "SOIL1";
    first = false;
  }
  if (soil2Error) {
    if (!first) errorReason += ", ";
    errorReason += "SOIL2";
    first = false;
  }
  if (soil3Error) {
    if (!first) errorReason += ", ";
    errorReason += "SOIL3";
  }
}


// ============================================================================
// 2. PROTEKSI SISTEM
// ============================================================================
void Proteksi_Sistem() {
  unsigned long now = millis();
  Proteksi_EvaluasiSensor();

  if (modeSistem == MODE_STARTUP) {

    if (now - startupStartTime >= 180000UL) {
      modeSistem = MODE_ERROR_TOTAL;
      sistemErrorAktif = true;
      errorReason = "STARTUP TIMEOUT";
      errorPerluDikirim = true;
      errorButuhKirimRealtime = true;
      errorButuhKirimRiwayat = true;
      return;
    }

    if (!bmeError && !soil1Error && !soil2Error && !soil3Error) {
      modeSistem = MODE_RUN;
      sistemPernahRun = true;
      systemStartTime = now;
      startupStartTime = 0;
      Serial.println("✅ STARTUP SELESAI → MODE RUN");
    }

    return;
  }

  if (modeSistem == MODE_RUN) {

    if (anySensorError) {
      modeSistem = MODE_ERROR_TOTAL;
      sistemErrorAktif = true;
      errorPerluDikirim = true;
      errorButuhKirimRealtime = true;
      errorButuhKirimRiwayat = true;
    }
  }

  if (modeSistem == MODE_ERROR_TOTAL) {
    matikanSemuaAktuator();
    indikatorErrorLED();
  }
}


// ============================================================================
// 3. FUNGSI PENDUKUNG PROTEKSI
// ============================================================================
void indikatorErrorLED() {
  static unsigned long lastBlink = 0;
  static bool ledState = false;

  if (millis() - lastBlink >= 200) {
    lastBlink = millis();
    ledState = !ledState;
    digitalWrite(redPin, ledState);
  }

  digitalWrite(greenPin, LOW);
  digitalWrite(bluePin, LOW);
}

void matikanSemuaAktuator() {
  if (humidifierState) {
    digitalWrite(humidifierPin, HIGH);
    delay(HUMIDIFIER_PULSE_MS);
    digitalWrite(humidifierPin, LOW);
    humidifierState = false;
  }

  if (dehumidifierState) {
    digitalWrite(dehumidifierPin, HIGH);
    delay(DEHUMIDIFIER_PULSE_MS);
    digitalWrite(dehumidifierPin, LOW);
    dehumidifierState = false;
  }

  digitalWrite(RELAY_R1, LOW);
  digitalWrite(RELAY_R2, LOW);
  fanState = FAN_OFF;
  digitalWrite(DIMMER_OUT, LOW);
  digitalWrite(buzzerPin, LOW);

  buzzerState = false;
  digitalWrite(redPin, LOW);
  digitalWrite(greenPin, LOW);
  digitalWrite(bluePin, LOW);
}



// 🟪 D. PROSES LOGIKA FUZZY
// ============================================================================
// 1. FUZZYFIKASI SUHU
// ============================================================================
void fuzzifikasiSuhu(float suhu) {
  suhuDingin = 0.0;
  suhuIdeal = 0.0;
  suhuPanas = 0.0;
  kondisiSuhu = "";

  if (suhu <= 15) {
    suhuDingin = 1.0;
  } else if (suhu > 15 && suhu <= 16) {
    suhuDingin = (16 - suhu) / (16 - 15);
  }

  if (suhu >= 15 && suhu <= 16) {
    suhuIdeal = (suhu - 15) / (16 - 15);
  } else if (suhu > 16 && suhu <= 25) {
    suhuIdeal = 1.0;
  } else if (suhu > 25 && suhu <= 30) {
    suhuIdeal = (30 - suhu) / (30 - 25);
  }

  if (suhu >= 25 && suhu <= 30) {
    suhuPanas = (suhu - 25) / (30 - 25);
  } else if (suhu > 30) {
    suhuPanas = 1.0;
  }

  if (suhuDingin >= suhuIdeal && suhuDingin >= suhuPanas)
    kondisiSuhu = "DINGIN";
  else if (suhuIdeal >= suhuDingin && suhuIdeal >= suhuPanas)
    kondisiSuhu = "IDEAL";
  else
    kondisiSuhu = "PANAS";
}


// ============================================================================
// 2. FUZZYFIKASI KELEMBAPAN
// ============================================================================
void fuzzifikasiKelembapan(float kelembapan) {
  kelembapanKering = 0.0;
  kelembapanIdeal = 0.0;
  kelembapanLembab = 0.0;
  kondisiKelembapan = "";

  if (kelembapan <= 30) {
    kelembapanKering = 1.0;
  } else if (kelembapan > 30 && kelembapan <= 60) {
    kelembapanKering = (60 - kelembapan) / (60 - 30);
  }

  if (kelembapan >= 30 && kelembapan <= 60) {
    kelembapanIdeal = (kelembapan - 30) / (60 - 30);
  } else if (kelembapan >= 60 && kelembapan <= 70) {
    kelembapanIdeal = 1.0;
  } else if (kelembapan > 70 && kelembapan <= 80) {
    kelembapanIdeal = (80 - kelembapan) / (80 - 70);
  }

  if (kelembapan >= 70 && kelembapan <= 80) {
    kelembapanLembab = (kelembapan - 70) / (80 - 70);
  } else if (kelembapan > 80) {
    kelembapanLembab = 1.0;
  }

  if (kelembapanKering >= kelembapanIdeal && kelembapanKering >= kelembapanLembab)
    kondisiKelembapan = "KERING";
  else if (kelembapanIdeal >= kelembapanKering && kelembapanIdeal >= kelembapanLembab)
    kondisiKelembapan = "IDEAL";
  else
    kondisiKelembapan = "LEMBAB";
}


// ============================================================================
// 3. FUZZYFIKASI SOIL MOISTURE (3 SENSOR)
// ============================================================================
float fuzzyMembershipRendah(float x) {
  if (x < 11) return 1.0;
  else if (x >= 11 && x <= 14) return (14 - x) / (14 - 11);
  return 0.0;
}

float fuzzyMembershipNormal(float x) {
  if (x >= 11 && x <= 14) return (x - 11) / (14 - 11);
  else if (x >= 14 && x <= 18) return (18 - x) / (18 - 14);
  return 0.0;
}

float fuzzyMembershipTinggi(float x) {
  if (x >= 14 && x <= 18) return (x - 14) / (18 - 14);
  else if (x > 18) return 1.0;
  return 0.0;
}


// ============================================================================
// 4. PENGGABUNGAN NILAI SOIL [MAX AGREGATION]
// ============================================================================
void hitungHFS() {
  hfsRendah = max(rendah1, max(rendah2, rendah3));
  hfsNormal = max(normal1, max(normal2, normal3));
  hfsTinggi = max(tinggi1, max(tinggi2, tinggi3));

  if (hfsRendah >= hfsNormal && hfsRendah >= hfsTinggi)
    kondisiKadarAir = "RENDAH";
  else if (hfsNormal >= hfsRendah && hfsNormal >= hfsTinggi)
    kondisiKadarAir = "IDEAL";
  else
    kondisiKadarAir = "TINGGI";
}


// ============================================================================
// 5. INFERENSI & DEFUZZYFIKASI FUZZY SUGENO (SISTEM MONITORING)
// ============================================================================
String Inferensi(
  float suhuDingin, float suhuIdeal, float suhuPanas,
  float kelembapanKering, float kelembapanIdeal, float kelembapanLembab,
  float hfsRendah, float hfsNormal, float hfsTinggi) {

  float sumWiZi = 0.0;
  float sumWi = 0.0;
  float wi = 0.0;

  wi = min(suhuIdeal, min(kelembapanIdeal, hfsNormal));
  sumWiZi += wi * ziAman; sumWi += wi;

  wi = min(suhuDingin, min(kelembapanIdeal, hfsNormal));
  sumWiZi += wi * ziWaspada; sumWi += wi;

  wi = min(suhuPanas, min(kelembapanIdeal, hfsNormal));
  sumWiZi += wi * ziWaspada; sumWi += wi;

  wi = min(suhuIdeal, min(kelembapanKering, hfsNormal));
  sumWiZi += wi * ziWaspada; sumWi += wi;

  wi = min(suhuIdeal, min(kelembapanLembab, hfsNormal));
  sumWiZi += wi * ziWaspada; sumWi += wi;

  wi = min(suhuDingin, min(kelembapanKering, hfsNormal));
  sumWiZi += wi * ziWaspada; sumWi += wi;

  wi = min(suhuDingin, min(kelembapanLembab, hfsNormal));
  sumWiZi += wi * ziWaspada; sumWi += wi;

  wi = min(suhuPanas, min(kelembapanKering, hfsNormal));
  sumWiZi += wi * ziWaspada; sumWi += wi;

  wi = min(suhuPanas, min(kelembapanLembab, hfsNormal));
  sumWiZi += wi * ziPeringatan; sumWi += wi;

  wi = min(suhuIdeal, min(kelembapanIdeal, hfsRendah));
  sumWiZi += wi * ziPeringatan; sumWi += wi;

  wi = min(suhuIdeal, min(kelembapanIdeal, hfsTinggi));
  sumWiZi += wi * ziPeringatan; sumWi += wi;

  wi = min(suhuDingin, min(kelembapanIdeal, hfsRendah));
  sumWiZi += wi * ziBerbahaya; sumWi += wi;

  wi = min(suhuDingin, min(kelembapanIdeal, hfsTinggi));
  sumWiZi += wi * ziBerbahaya; sumWi += wi;

  wi = min(suhuPanas, min(kelembapanIdeal, hfsRendah));
  sumWiZi += wi * ziBerbahaya; sumWi += wi;

  wi = min(suhuPanas, min(kelembapanIdeal, hfsTinggi));
  sumWiZi += wi * ziBerbahaya; sumWi += wi;

  wi = min(suhuIdeal, min(kelembapanKering, hfsRendah));
  sumWiZi += wi * ziBerbahaya; sumWi += wi;

  wi = min(suhuIdeal, min(kelembapanKering, hfsTinggi));
  sumWiZi += wi * ziBerbahaya; sumWi += wi;

  wi = min(suhuIdeal, min(kelembapanLembab, hfsRendah));
  sumWiZi += wi * ziBerbahaya; sumWi += wi;

  wi = min(suhuIdeal, min(kelembapanLembab, hfsTinggi));
  sumWiZi += wi * ziBerbahaya; sumWi += wi;

  wi = min(suhuDingin, min(kelembapanKering, hfsRendah));
  sumWiZi += wi * ziBerbahaya; sumWi += wi;

  wi = min(suhuDingin, min(kelembapanKering, hfsTinggi));
  sumWiZi += wi * ziBerbahaya; sumWi += wi;

  wi = min(suhuDingin, min(kelembapanLembab, hfsRendah));
  sumWiZi += wi * ziBerbahaya; sumWi += wi;

  wi = min(suhuDingin, min(kelembapanLembab, hfsTinggi));
  sumWiZi += wi * ziBerbahaya; sumWi += wi;

  wi = min(suhuPanas, min(kelembapanKering, hfsRendah));
  sumWiZi += wi * ziBerbahaya; sumWi += wi;

  wi = min(suhuPanas, min(kelembapanKering, hfsTinggi));
  sumWiZi += wi * ziBerbahaya; sumWi += wi;

  wi = min(suhuPanas, min(kelembapanLembab, hfsRendah));
  sumWiZi += wi * ziBerbahaya; sumWi += wi;

  wi = min(suhuPanas, min(kelembapanLembab, hfsTinggi));
  sumWiZi += wi * ziBerbahaya; sumWi += wi;

  if (sumWi == 0.0) return "TIDAK TERDETEKSI";
  float z = sumWiZi / sumWi;
  z = constrain(z, 0.0, 100.0);

  if (z < 25.0) return "AMAN";
  else if (z < 50.0) return "WASPADA";
  else if (z < 75.0) return "PERINGATAN";
  else return "BERBAHAYA";
}


// ============================================================================
// 6. INFERENSI & DEFUZZYFIKASI FUZZY MAMDANI (KENDALI KIPAS)
// ============================================================================
float inferensiMamdani(float suhuDingin, float suhuIdeal, float suhuPanas,
                       float kelembapanKering, float kelembapanIdeal, float kelembapanLembab) {

  float wOFF = max(min(suhuDingin, kelembapanKering),
                   max(min(suhuDingin, kelembapanIdeal),
                       max(min(suhuDingin, kelembapanLembab),
                           min(suhuIdeal, kelembapanIdeal))));

  float wLambat = max(min(suhuIdeal, kelembapanKering),
                      min(suhuIdeal, kelembapanLembab));

  float wSedang = min(suhuPanas, kelembapanIdeal);
  float wCepat = min(suhuPanas, kelembapanKering);
  float wMaksimum = min(suhuPanas, kelembapanLembab);

  float numerator = 0.0;
  float denominator = 0.0;

  for (int z = 0; z <= 100; z++) {

    float muOFF = 0.0;
    if (z <= 0) muOFF = 1.0;
    else if (z < 12) muOFF = (12.0 - z) / 12.0;

    float muLambat = 0.0;
    if (z >= 12 && z <= 24) muLambat = (z - 12.0) / 12.5;
    else if (z > 24 && z <= 37) muLambat = (37.0 - z) / 12.5;

    float muSedang = 0.0;
    if (z >= 37 && z <= 49) muSedang = (z - 37.0) / 12.5;
    else if (z > 49 && z <= 62) muSedang = (62.0 - z) / 12.5;

    float muCepat = 0.0;
    if (z >= 62 && z <= 74) muCepat = (z - 62.0) / 12.5;
    else if (z > 74 && z <= 87) muCepat = (87.0 - z) / 12.5;

    float muMaksimum = 0.0;
    if (z >= 87 && z <= 100) muMaksimum = (z - 87.0) / 13.0;
    else if (z > 100) muMaksimum = 1.0;

    muOFF = min(wOFF, muOFF);
    muLambat = min(wLambat, muLambat);
    muSedang = min(wSedang, muSedang);
    muCepat = min(wCepat, muCepat);
    muMaksimum = min(wMaksimum, muMaksimum);

    float muAgregasi = max(muOFF, max(muLambat, max(muSedang, max(muCepat, muMaksimum))));
    numerator += z * muAgregasi;
    denominator += muAgregasi;
  }

  float crisp = 0.0;
  if (denominator > 0.0)
    crisp = numerator / denominator;

  return constrain(crisp, 0.0, 100.0);
}


// ============================================================================
// 7. VERIVIKASI AKTUATOR
// ============================================================================
void checking() {
  if (kondisiGudang != "AMAN") {
    if (nonAmanStartTime == 0) {
      nonAmanStartTime = millis();
    }
    if (millis() - nonAmanStartTime >= CHECKING_TIMEOUT_MS) {
      checkingAktif = true;
    }
  } else {
    nonAmanStartTime = 0;
    checkingAktif = false;
  }
}


// ============================================================================
// 8. PROSES FUZZY UTAMA
// ============================================================================
void Fuzzy(float suhu, float kelembapan) {

  fuzzifikasiSuhu(suhu);
  fuzzifikasiKelembapan(kelembapan);

  rendah1 = fuzzyMembershipRendah(soilMoisturePercentage1);
  normal1 = fuzzyMembershipNormal(soilMoisturePercentage1);
  tinggi1 = fuzzyMembershipTinggi(soilMoisturePercentage1);

  rendah2 = fuzzyMembershipRendah(soilMoisturePercentage2);
  normal2 = fuzzyMembershipNormal(soilMoisturePercentage2);
  tinggi2 = fuzzyMembershipTinggi(soilMoisturePercentage2);

  rendah3 = fuzzyMembershipRendah(soilMoisturePercentage3);
  normal3 = fuzzyMembershipNormal(soilMoisturePercentage3);
  tinggi3 = fuzzyMembershipTinggi(soilMoisturePercentage3);

  hitungHFS();

  kondisiGudang = Inferensi(suhuDingin, suhuIdeal, suhuPanas, kelembapanKering, kelembapanIdeal, kelembapanLembab, hfsRendah, hfsNormal, hfsTinggi);

  checking();

  kontrolKipas(suhuDingin, suhuIdeal, suhuPanas, kelembapanKering, kelembapanIdeal, kelembapanLembab);
  kontrolHumidifier(kelembapan);
  kontrolDehumidifier(kelembapan);
  kontrolLED(kondisiGudang);
  kontrolBuzzer(kondisiGudang);
}



// 🟪 E. KENDALI AKTUATOR
// ============================================================================
// 1. KENDALI KIPAS AC
// ============================================================================
void kontrolKipas(float suhuDingin, float suhuIdeal, float suhuPanas,
                  float kelembapanKering, float kelembapanIdeal, float kelembapanLembab) {

  float crisp = inferensiMamdani(suhuDingin, suhuIdeal, suhuPanas,
                                 kelembapanKering, kelembapanIdeal, kelembapanLembab);
  crispFanGlobal = crisp;

  int dimmerPowerPercent = 0;

  if (crisp < 12.0) dimmerPowerPercent = 0;
  else if (crisp < 37.0) dimmerPowerPercent = 20;
  else if (crisp < 62.0) dimmerPowerPercent = 40;
  else if (crisp < 87.0) dimmerPowerPercent = 85;
  else dimmerPowerPercent = 95;

  toffFanGlobal = dimmerPowerPercent;

  if (toffFanGlobal >= 20 && toffFanGlobal <= 95) {
    tonFanGlobal = ((float)(toffFanGlobal - 20) / (95.0 - 20.0)) * 100.0;
  } else {
    tonFanGlobal = 0.0;
  }
}

void stateFan() {
  unsigned long now = millis();
  int cmd = toffFanGlobal;
  switch (fanState) {

    case FAN_OFF:
      digitalWrite(RELAY_R1, LOW);
      digitalWrite(RELAY_R2, LOW);
      dimmer.setPower(0);

      if (cmd >= 20 && cmd <= 95) {
        fanState = FAN_STARTING;
        stateTimer = now;
        digitalWrite(RELAY_R1, HIGH);
        digitalWrite(RELAY_R2, LOW);
      }
      break;

    case FAN_STARTING:
      digitalWrite(RELAY_R1, HIGH);
      digitalWrite(RELAY_R2, LOW);
      dimmer.setPower(0);

      if (cmd < 20 || cmd > 95) {
        fanState = FAN_OFF;
        break;
      }

      if (now - stateTimer >= START_TIME) {
        fanState = FAN_RUNNING_DIMMER;
        digitalWrite(RELAY_R1, LOW);
        digitalWrite(RELAY_R2, HIGH);
      }
      break;

    case FAN_RUNNING_DIMMER:
      digitalWrite(RELAY_R1, LOW);
      digitalWrite(RELAY_R2, HIGH);

      dimmer.setPower(cmd);

      if (cmd < 20 || cmd > 95) {
        if (offTimer == 0) offTimer = now;
        if (now - offTimer >= OFF_DELAY) {
          fanState = FAN_OFF;
          offTimer = 0;
        }
      } else {
        offTimer = 0;
      }
      break;
  }
}


// ============================================================================
// 2. KENDALI HUMIDIFIER
// ============================================================================
void kontrolHumidifier(float kelembapan) {
  unsigned long intervalSiklus = 0;
  unsigned long durasiON = 0;
  bool humidifierRequest = true;

  if (kelembapan <= 30.0) {
    durasiON = 3000; intervalSiklus = 60000;
  } else if (kelembapan <= 35.0) {
    durasiON = 3000; intervalSiklus = 120000;
  } else if (kelembapan <= 40.0) {
    durasiON = 2000; intervalSiklus = 120000;
  } else if (kelembapan <= 45.0) {
    durasiON = 2000; intervalSiklus = 180000;
  } else if (kelembapan <= 50.0) {
    durasiON = 1000; intervalSiklus = 120000;
  } else if (kelembapan <= 55.0) {
    durasiON = 1000; intervalSiklus = 180000;
  } else if (kelembapan <= 60.0) {
    durasiON = 1000; intervalSiklus = 300000;
  } else
    humidifierRequest = false;

  unsigned long currentMillis = millis();

  static bool cycleActive = false;
  static bool faseON = false;
  static unsigned long waktuMulaiSiklus = 0;
  static unsigned long waktuMulaiON = 0;
  static unsigned long intervalAktif = 0;
  static bool requestTertunda = false;

  if (!humidifierRequest && cycleActive) {
    requestTertunda = true;
  }

  if (!cycleActive) {
    if (humidifierRequest) {
      cycleActive = true;
      faseON = true;
      intervalAktif = intervalSiklus;
      waktuMulaiSiklus = currentMillis;
      waktuMulaiON = currentMillis;
      humidifierState = true;
      digitalWrite(humidifierPin, HIGH);
      delay(HUMIDIFIER_PULSE_MS);
      digitalWrite(humidifierPin, LOW);
      prevMillisHum = currentMillis;
    } else {
      humidifierState = false;
    }
  } else {
    if (faseON && (currentMillis - waktuMulaiON >= durasiON)) {
      faseON = false;
      if (humidifierState) {
        digitalWrite(humidifierPin, HIGH);
        delay(HUMIDIFIER_PULSE_MS);
        digitalWrite(humidifierPin, LOW);
      }
      humidifierState = false;
      prevMillisHum = currentMillis;
    }

    if (currentMillis - waktuMulaiSiklus >= intervalAktif) {
      cycleActive = false;
      if (requestTertunda) {
        requestTertunda = false;
        if (humidifierState) {
          digitalWrite(humidifierPin, HIGH);
          delay(HUMIDIFIER_PULSE_MS);
          digitalWrite(humidifierPin, LOW);
        }
        humidifierState = false;
      }
    }
  }

  tonGlobal = humidifierRequest ? 1.0 : 0.0;
  toffGlobal = humidifierRequest ? intervalSiklus / 1000.0 : 0.0;
}


// ============================================================================
// 3. KENDALI DEHUMIDIFIER
// ============================================================================
void kontrolDehumidifier(float kelembapan) {
  static bool dehumPrevRequest = false;
  static bool dehumActive = false;

  bool dehumRequest;
  if (!dehumActive) {
    dehumRequest = (kelembapan > 70.0);
  } else {
    dehumRequest = (kelembapan >= 67.0);
  }
  dehumActive = dehumRequest;

  if (dehumRequest && !dehumPrevRequest) {
    digitalWrite(dehumidifierPin, HIGH);
    delay(DEHUMIDIFIER_PULSE_MS);
    digitalWrite(dehumidifierPin, LOW);
    dehumidifierState = true;
  }

  if (!dehumRequest && dehumPrevRequest) {
    if (dehumidifierState) {
      digitalWrite(dehumidifierPin, HIGH);
      delay(DEHUMIDIFIER_PULSE_MS);
      digitalWrite(dehumidifierPin, LOW);
    }
    dehumidifierState = false;
  }

  dehumPrevRequest = dehumRequest;
}


// ============================================================================
// 4. KENDALI LED RGB
// ============================================================================
void kontrolLED(String kondisiGudang) {
  if (modeSistem != MODE_RUN) return;

  digitalWrite(redPin, LOW);
  digitalWrite(greenPin, LOW);
  digitalWrite(bluePin, LOW);

  if (kondisiGudang == "AMAN") {
    digitalWrite(greenPin, HIGH);
  } else if (kondisiGudang == "WASPADA") {
    digitalWrite(bluePin, HIGH);
  } else if (kondisiGudang == "PERINGATAN") {
    digitalWrite(redPin, HIGH);
  } else if (kondisiGudang == "BERBAHAYA") {
    digitalWrite(redPin, buzzerState ? HIGH : LOW);
  }
}


// ============================================================================
// 5. KENDALI BUZZER
// ============================================================================
void kontrolBuzzer(String kondisiGudang) {
  static unsigned long prevMillisBuzz = 0;
  static unsigned long nonAmanStartTime = 0;
  static bool checkingAktif = false;
  unsigned long currentMillis = millis();

  if (kondisiGudang == "AMAN") {
    nonAmanStartTime = 0;
    checkingAktif = false;
    buzzerState = false;
    digitalWrite(buzzerPin, LOW);
    return;
  }

  if (nonAmanStartTime == 0) nonAmanStartTime = currentMillis;

  if (!checkingAktif && (currentMillis - nonAmanStartTime >= 1800000UL)) {
    checkingAktif = true;
  }

  if (checkingAktif) {
    buzzerState = true;
    digitalWrite(buzzerPin, HIGH);

  } else if (kondisiGudang == "BERBAHAYA") {
    if (buzzerState && (currentMillis - prevMillisBuzz >= 3000)) {
      buzzerState = false;
      prevMillisBuzz = currentMillis;
      digitalWrite(buzzerPin, LOW);
    } else if (!buzzerState && (currentMillis - prevMillisBuzz >= 1000)) {
      buzzerState = true;
      prevMillisBuzz = currentMillis;
      digitalWrite(buzzerPin, HIGH);
    }

  } else {
    buzzerState = false;
    digitalWrite(buzzerPin, LOW);
  }
}


// ============================================================================
// 6. KENDALI COOLER FAN
// ============================================================================
//   - ON  : suhu ruangan > 33°C
//   - OFF : suhu ruangan < 31°C (hysteresis)
//   - aktif di semua mode (RUN, STARTUP, ERROR)
void CoolerFan() {
  if (isnan(suhuGlobal)) return;

  if (!kipasKomponenState && suhuGlobal > 33.0) {
    kipasKomponenState = true;
    digitalWrite(RELAY_COOLER_FAN, HIGH);
  } else if (kipasKomponenState && suhuGlobal < 31.0) {
    kipasKomponenState = false;
    digitalWrite(RELAY_COOLER_FAN, LOW);
  }
}



// 🟪 F. SISTEM IOT
// ============================================================================
// 1. WIFI HANDLER
// ============================================================================
void koneksiWiFi() {
  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    wifiRSSI = WiFi.RSSI();
    return;
  }

  wifiConnected = false;
  static unsigned long lastAttempt = 0;
  if (millis() - lastAttempt < 3000) return;
  lastAttempt = millis();

  WiFi.begin(ssid, password);
}


// ============================================================================
// 2. CEK KONEKSI KE INTERNET
// ============================================================================
bool cekInternet() {
  if (!wifiConnected) return false;

  HTTPClient http;
  http.begin("http://clients3.google.com/generate_204");
  int httpCode = http.GET();
  http.end();

  if (httpCode == 204) return true;
  else return false;
}


// ============================================================================
// 3. WAKTU (NTP)
// ============================================================================
String waktuRealtime() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) return "Unknown";
  char buf[25];
  strftime(buf, sizeof(buf), "%H:%M:%S_%d-%m-%Y", &timeinfo);
  return String(buf);
}

String waktuRiwayat() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) return "Unknown/Unknown";
  char tanggal[20];
  char jam[20];

  strftime(tanggal, sizeof(tanggal), "%d-%m-%Y", &timeinfo);
  strftime(jam, sizeof(jam), "%H:%M:%S", &timeinfo);

  return String(tanggal) + "/" + String(jam);
}


// ============================================================================
// 4. CEK MEMORI (HEAP MONITOR)
// ============================================================================
void cekMemori() {
  heapFree = ESP.getFreeHeap();
  heapTotal = ESP.getHeapSize();
  heapUsed = heapTotal - heapFree;

  heapPercent = ((float)heapUsed / (float)heapTotal) * 100.0;
}


// ============================================================================
// 5. KIRIM DATA KE FIREBASE (REALTIME & RIWAYAT)
// ============================================================================
void kirimRealtime() {
  if (stopFirebaseKirim && errorFirebaseSudahTerkirim) return;
  if (!firebaseOK || !wifiConnected) return;

  String waktu = waktuRealtime();
  String url = firebaseHost + "/REALTIME.json?auth=" + firebaseKey;

  HTTPClient http;
  http.begin(url);
  http.addHeader("Content-Type", "application/json");

  String jsonData = "{";

  if (modeSistem == MODE_ERROR_TOTAL)
    jsonData += "\"suhu\":\"ERROR\",";
  else
    jsonData += "\"suhu\":" + String(suhuGlobal, 2) + ",";

  if (modeSistem == MODE_ERROR_TOTAL)
    jsonData += "\"kelembapan\":\"ERROR\",";
  else
    jsonData += "\"kelembapan\":" + String(kelembapanGlobal, 2) + ",";

  if (modeSistem == MODE_ERROR_TOTAL)
    jsonData += "\"kadarAir1\":\"ERROR\",";
  else
    jsonData += "\"kadarAir1\":" + String(soilMoisturePercentage1, 2) + ",";

  if (modeSistem == MODE_ERROR_TOTAL)
    jsonData += "\"kadarAir2\":\"ERROR\",";
  else
    jsonData += "\"kadarAir2\":" + String(soilMoisturePercentage2, 2) + ",";

  if (modeSistem == MODE_ERROR_TOTAL)
    jsonData += "\"kadarAir3\":\"ERROR\",";
  else
    jsonData += "\"kadarAir3\":" + String(soilMoisturePercentage3, 2) + ",";

  if (modeSistem == MODE_ERROR_TOTAL)
    jsonData += "\"kondisiGudang\":\"" + errorReason + "\",";
  else
    jsonData += "\"kondisiGudang\":\"" + kondisiGudang + "\",";

  jsonData += "\"waktu\":\"" + waktu + "\"";
  jsonData += "}";

  int httpCode = http.PUT(jsonData);
  http.end();

  if (httpCode == 200 || httpCode == 201) {
    firebaseConnected = true;
    if (modeSistem == MODE_ERROR_TOTAL && errorButuhKirimRealtime) {
      errorButuhKirimRealtime = false;
    }
  } else {
    firebaseConnected = false;
  }
}

void kirimRiwayat() {
  Serial.println(">>> RIWAYAT TERPANGGIL");

  if (stopFirebaseKirim && errorFirebaseSudahTerkirim) return;
  if (!firebaseOK || !wifiConnected) return;

  String waktu = waktuRiwayat();
  if (waktu == "Unknown") return;

  String pathWaktu = waktu;
  pathWaktu.replace("_", "/");

  String url = firebaseHost + "/RIWAYAT/" + pathWaktu + ".json?auth=" + firebaseKey;

  HTTPClient http;
  http.begin(url);
  http.addHeader("Content-Type", "application/json");

  String jsonData = "{";

  if (modeSistem == MODE_ERROR_TOTAL)
    jsonData += "\"suhu\":\"ERROR\",";
  else
    jsonData += "\"suhu\":" + String(suhuGlobal, 2) + ",";

  if (modeSistem == MODE_ERROR_TOTAL)
    jsonData += "\"kelembapan\":\"ERROR\",";
  else
    jsonData += "\"kelembapan\":" + String(kelembapanGlobal, 2) + ",";

  if (modeSistem == MODE_ERROR_TOTAL)
    jsonData += "\"kadarAir1\":\"ERROR\",";
  else
    jsonData += "\"kadarAir1\":" + String(soilMoisturePercentage1, 2) + ",";

  if (modeSistem == MODE_ERROR_TOTAL)
    jsonData += "\"kadarAir2\":\"ERROR\",";
  else
    jsonData += "\"kadarAir2\":" + String(soilMoisturePercentage2, 2) + ",";

  if (modeSistem == MODE_ERROR_TOTAL)
    jsonData += "\"kadarAir3\":\"ERROR\",";
  else
    jsonData += "\"kadarAir3\":" + String(soilMoisturePercentage3, 2) + ",";

  if (modeSistem == MODE_ERROR_TOTAL)
    jsonData += "\"kondisiGudang\":\"" + errorReason + "\"";
  else
    jsonData += "\"kondisiGudang\":\"" + kondisiGudang + "\"";

  jsonData += "}";

  int httpCode = http.PUT(jsonData);
  http.end();

  if (httpCode == 200) {
    firebaseConnected = true;
    if (modeSistem == MODE_ERROR_TOTAL && errorButuhKirimRiwayat) {
      errorButuhKirimRiwayat = false;
    }
  } else {
    firebaseConnected = false;
  }
}


// ============================================================================
// 6. UPDATE SCHEDULE IOT (REALTIME & RIWAYAT)
// ============================================================================
void updateFirebaseMinute() {
  static unsigned long lastRealtime = 0;
  static int lastHistoryDay = -1;
  static int lastHistoryHour = -1;
  static bool ntpReady = false;

  unsigned long now = millis();

  struct tm timeinfo;
  if (!ntpReady) {
    if (getLocalTime(&timeinfo)) {
      ntpReady = true;
      lastHistoryDay = timeinfo.tm_mday;
      lastHistoryHour = timeinfo.tm_hour;
    }
  }

  if (now - lastRealtime >= 5000) {
    lastRealtime = now;
    kirimRealtime();
  }

  if (modeSistem == MODE_ERROR_TOTAL && errorButuhKirimRiwayat && ntpReady) {
    kirimRealtime();
    kirimRiwayat();
    errorFirebaseSudahTerkirim = true;
    stopFirebaseKirim = true;
    return;
  }

  if (ntpReady && getLocalTime(&timeinfo)) {
    if (timeinfo.tm_mday != lastHistoryDay || timeinfo.tm_hour != lastHistoryHour) {
      lastHistoryDay = timeinfo.tm_mday;
      lastHistoryHour = timeinfo.tm_hour;
      kirimRiwayat();
    }
  }

  if (stopFirebaseKirim && errorFirebaseSudahTerkirim) {
    return;
  }
}



// 🟪 G. KONEKSI & KIRIM DATA KE BOARD 2
// ============================================================================
// 1. KIRIM DATA KE ESP32_2
// ============================================================================
void ToEsp32() {
  if (modeSistem == MODE_STARTUP) {
    String json = "{";
    json += "\"mode\":\"STARTUP\",";
    json += "\"pesan\":\"INITIZLIZING SISTEM.....\",";
    json += "\"statusJSON\":\"OK\"";
    json += "}";
    Serial2.println(json);
    return;
  }

  if (modeSistem == MODE_ERROR_TOTAL) {
    String pesan;
    if (!sistemPernahRun) {
      pesan = "ERROR STARTUP - SENSOR: " + errorReason;
    } else {
      pesan = "SISTEM ERROR - SENSOR: " + errorReason;
    }

    String json = "{";
    json += "\"mode\":\"ERROR\",";
    json += "\"pesan\":\"" + pesan + "\",";
    json += "\"led\":\"MERAH_KEDIP\",";
    json += "\"statusJSON\":\"OK\"";
    json += "}";

    Serial2.println(json);
    return;
  }

  static unsigned long lastSend = 0;
  unsigned long now = millis();
  if (now - lastSend < 2000) return;
  lastSend = now;

  String statusWiFi;
  String kekuatanSinyal;
  String kecepatanWiFi;

  statusWiFi = wifiConnected ? "TERHUBUNG" : "TIDAK TERHUBUNG";

  if (wifiConnected) {
    kekuatanSinyal = String(wifiRSSI) + " dBm";
    if (wifiRSSI >= -50) kecepatanWiFi = "SANGAT CEPAT";
    else if (wifiRSSI >= -60) kecepatanWiFi = "CEPAT";
    else if (wifiRSSI >= -70) kecepatanWiFi = "SEDANG";
    else if (wifiRSSI >= -80) kecepatanWiFi = "LAMBAT";
    else if (wifiRSSI >= -90) kecepatanWiFi = "SANGAT LAMBAT";
    else kecepatanWiFi = "HAMPIR PUTUS";
  } else {
    kekuatanSinyal = "-";
    kecepatanWiFi = "TIDAK TERHUBUNG";
  }

  String warnaLED;
  if (kondisiGudang == "AMAN") warnaLED = "HIJAU";
  else if (kondisiGudang == "WASPADA") warnaLED = "BIRU";
  else if (kondisiGudang == "PERINGATAN") warnaLED = "MERAH";
  else if (kondisiGudang == "BERBAHAYA") warnaLED = "MERAH KEDIP";
  else warnaLED = kondisiGudang;

  String json = "{";
  json += "\"mode\":\"NORMAL\",";
  json += "\"suhu\":" + String(suhuGlobal, 1) + ",";
  json += "\"kelembapan\":" + String(kelembapanGlobal, 1) + ",";
  json += "\"air1\":" + String(soilMoisturePercentage1, 2) + ",";
  json += "\"air2\":" + String(soilMoisturePercentage2, 2) + ",";
  json += "\"air3\":" + String(soilMoisturePercentage3, 2) + ",";
  json += "\"statusSuhu\":\"" + kondisiSuhu + "\",";
  json += "\"statusKelembapan\":\"" + kondisiKelembapan + "\",";
  json += "\"statusKadarAir\":\"" + kondisiKadarAir + "\",";
  json += "\"kondisiGudang\":\"" + kondisiGudang + "\",";
  json += "\"kipas\":" + String(crispFanGlobal, 1) + ",";
  json += "\"humidifier\":\"" + String(humidifierState ? "BEKERJA" : "TIDAK BEKERJA") + "\",";
  json += "\"dehumidifier\":\"" + String(dehumidifierState ? "BEKERJA" : "TIDAK BEKERJA") + "\",";
  json += "\"buzzer\":\"" + String(buzzerState ? "BEKERJA" : "TIDAK BEKERJA") + "\",";
  json += "\"led\":\"" + warnaLED + "\",";
  json += "\"wifiStatus\":\"" + statusWiFi + "\",";
  json += "\"wifiSignal\":\"" + kekuatanSinyal + "\",";
  json += "\"wifiSpeed\":\"" + kecepatanWiFi + "\",";
  json += "\"internet\":\"" + String(firebaseOK ? "TERHUBUNG" : "TIDAK TERHUBUNG") + "\",";
  json += "\"ramUsed\":" + String(heapPercent, 1) + ",";
  json += "\"statusJSON\":\"OK\"";
  json += "}";

  Serial2.println(json);
}



// 🟪 H. VOID SETUP + LOOP
// ============================================================================
// 1. VOID SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("SISTEM MONITORING GUDANG PADI - STARTING...");

  Serial2.begin(115200, SERIAL_8N1, UART2_RX, UART2_TX);

  Wire.begin(21, 22);
  bme.begin(0x76);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  analogSetPinAttenuation(soilMoisturePin1, ADC_11db);
  analogSetPinAttenuation(soilMoisturePin2, ADC_11db);
  analogSetPinAttenuation(soilMoisturePin3, ADC_11db);

  WiFi.mode(WIFI_OFF);
  WiFi.disconnect(true);

  delay(3000);

  // === Inisialisasi pin kendali kipas ===
  pinMode(RELAY_R1, OUTPUT);
  pinMode(RELAY_R2, OUTPUT);
  digitalWrite(RELAY_R1, LOW);
  digitalWrite(RELAY_R2, LOW);

  dimmer.begin(NORMAL_MODE, ON);
  dimmer.setPower(0);
  fanState = FAN_OFF;

  // === Inisialisasi pin humidifier ===
  pinMode(humidifierPin, OUTPUT);
  digitalWrite(humidifierPin, LOW);

  // === Inisialisasi pin dehumidifier ===
  pinMode(dehumidifierPin, OUTPUT);
  digitalWrite(dehumidifierPin, LOW);

  // === Inisialisasi pin LED RGB ===
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  digitalWrite(redPin, LOW);
  digitalWrite(greenPin, LOW);
  digitalWrite(bluePin, LOW);

  // === Inisialisasi pin BUZZER ===
  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, LOW);

  // === Inisialisasi pin COOLER FAN ===
  pinMode(RELAY_COOLER_FAN, OUTPUT);
  digitalWrite(RELAY_COOLER_FAN, LOW);

  stopFirebaseKirim = false;
  errorFirebaseSudahTerkirim = false;

  modeSistem = MODE_STARTUP;
  startupStartTime = millis();

  static const uint32_t iotIntervalMs = 1000UL;
  static const uint32_t firebaseIntervalMs = 1000UL;
  static const uint32_t monitorIntervalMs = 2000UL;

  // === TaskMainLoop (PROSES UTAMA) ===
  xTaskCreatePinnedToCore(
    [](void* param) -> void {
      (void)param;
      for (;;) {

        float suhu = NAN;
        float kelembapan = NAN;

        if (cekI2CBME280()) {
          suhu = bme.readTemperature();
          kelembapan = bme.readHumidity();

          if (!isnan(suhu) && !isnan(kelembapan)) {
            suhu += suhuOffset;
            kelembapan += kelembapanOffset;
            lastBmeValidRead = millis();
            bmeEverValid = true;
          }
        }

        suhuGlobal = suhu;
        kelembapanGlobal = kelembapan;

        if (modeSistem == MODE_STARTUP) {
          matikanSemuaAktuator();
        }

        soilMoistureValue1 = readADCStable(soilMoisturePin1, lastAdcRange1);
        soilMoistureValue2 = readADCStable(soilMoisturePin2, lastAdcRange2);
        soilMoistureValue3 = readADCStable(soilMoisturePin3, lastAdcRange3);

        bool sensorOK1 = adcValidFinal(soilMoistureValue1, lastAdcRange1, soilStableCount1, soilEverValid1);
        if (modeSistem == MODE_STARTUP && !soilEverValid1) {
          if (adcValidStartup(soilMoistureValue1)) soilEverValid1 = true;
        }

        bool sensorOK2 = adcValidFinal(soilMoistureValue2, lastAdcRange2, soilStableCount2, soilEverValid2);
        if (modeSistem == MODE_STARTUP && !soilEverValid2) {
          if (adcValidStartup(soilMoistureValue2)) soilEverValid2 = true;
        }

        bool sensorOK3 = adcValidFinal(soilMoistureValue3, lastAdcRange3, soilStableCount3, soilEverValid3);
        if (modeSistem == MODE_STARTUP && !soilEverValid3) {
          if (adcValidStartup(soilMoistureValue3)) soilEverValid3 = true;
        }

        correctedValue1 = sensorOK1 ? soilMoistureValue1 : -1;
        correctedValue2 = sensorOK2 ? soilMoistureValue2 : -1;
        correctedValue3 = sensorOK3 ? soilMoistureValue3 : -1;

        if (correctedValue1 != -1) correctedValue1 += offsetADC1;
        if (correctedValue2 != -1) correctedValue2 += offsetADC2;
        if (correctedValue3 != -1) correctedValue3 += offsetADC3;

        if (correctedValue1 == -1 || ADC_MAX_1 == ADC_MIN_1)
          soilMoisturePercentage1 = -1;
        else {
          float idx = hitungSoilIndex(correctedValue1, ADC_MIN_1, ADC_MAX_1);
          soilMoisturePercentage1 = idx * (MOIST_MAX_1 - MOIST_MIN_1) + MOIST_MIN_1;
        }

        if (correctedValue2 == -1 || ADC_MAX_2 == ADC_MIN_2)
          soilMoisturePercentage2 = -1;
        else {
          float idx = hitungSoilIndex(correctedValue2, ADC_MIN_2, ADC_MAX_2);
          soilMoisturePercentage2 = idx * (MOIST_MAX_2 - MOIST_MIN_2) + MOIST_MIN_2;
        }

        if (correctedValue3 == -1 || ADC_MAX_3 == ADC_MIN_3)
          soilMoisturePercentage3 = -1;
        else {
          float idx = hitungSoilIndex(correctedValue3, ADC_MIN_3, ADC_MAX_3);
          soilMoisturePercentage3 = idx * (MOIST_MAX_3 - MOIST_MIN_3) + MOIST_MIN_3;
        }

        // ================= EMA SENSOR 1 =================
        if (soilMoisturePercentage1 != -1) {
          if (!soilFilterInit1) {
            soilFiltered1 = soilMoisturePercentage1;
            soilFilterInit1 = true;
          } else {
            soilFiltered1 = SOIL_ALPHA * soilMoisturePercentage1 + (1.0 - SOIL_ALPHA) * soilFiltered1;
          }
          soilMoisturePercentage1 = soilFiltered1;
        }

        // ================= EMA SENSOR 2 =================
        if (soilMoisturePercentage2 != -1) {
          if (!soilFilterInit2) {
            soilFiltered2 = soilMoisturePercentage2;
            soilFilterInit2 = true;
          } else {
            soilFiltered2 = SOIL_ALPHA * soilMoisturePercentage2 + (1.0 - SOIL_ALPHA) * soilFiltered2;
          }
          soilMoisturePercentage2 = soilFiltered2;
        }

        // ================= EMA SENSOR 3 =================
        if (soilMoisturePercentage3 != -1) {
          if (!soilFilterInit3) {
            soilFiltered3 = soilMoisturePercentage3;
            soilFilterInit3 = true;
          } else {
            soilFiltered3 = SOIL_ALPHA * soilMoisturePercentage3 + (1.0 - SOIL_ALPHA) * soilFiltered3;
          }
          soilMoisturePercentage3 = soilFiltered3;
        }

        if (soilMoisturePercentage1 != -1) {
          soilMoisturePercentage1 += offset1;
          soilMoisturePercentage1 = constrain(soilMoisturePercentage1, 0, 100);
        }

        if (soilMoisturePercentage2 != -1) {
          soilMoisturePercentage2 += offset2;
          soilMoisturePercentage2 = constrain(soilMoisturePercentage2, 0, 100);
        }

        if (soilMoisturePercentage3 != -1) {
          soilMoisturePercentage3 += offset3;
          soilMoisturePercentage3 = constrain(soilMoisturePercentage3, 0, 100);
        }

        if (!soilEverValid1 && !soilEverValid2 && !soilEverValid3) {
          correctedValue1 = -1;
          correctedValue2 = -1;
          correctedValue3 = -1;
        }

        // --- PROTEKSI GLOBAL ---
        Proteksi_Sistem();
        if (modeSistem >= MODE_ERROR_TOTAL) {
          vTaskDelay(pdMS_TO_TICKS(1000));
          continue;
        }

        // --- SISTEM UTAMA => FUZZY ---
        if (modeSistem != MODE_RUN) {
          vTaskDelay(pdMS_TO_TICKS(500));
          continue;
        }

        Fuzzy(suhu, kelembapan);

        // --- COOLER FAN (independen) ---
        CoolerFan();

        vTaskDelay(pdMS_TO_TICKS(1000));
      }
      vTaskDelete(NULL);
    },
    "TaskMainLoop",
    4096,
    nullptr,
    2,
    NULL,
    1
  );

  // === TaskIoT (Jaringan) ===
  xTaskCreatePinnedToCore(
    [](void* param) -> void {
      (void)param;
      const TickType_t intervalTicks = pdMS_TO_TICKS(iotIntervalMs);
      for (;;) {

        if (modeSistem == MODE_ERROR_TOTAL) {
          vTaskDelay(intervalTicks);
          continue;
        }

        koneksiWiFi();
        wifiConnected = (WiFi.status() == WL_CONNECTED);
        if (wifiConnected) wifiRSSI = WiFi.RSSI();

        if (modeSistem == MODE_RUN && wifiConnected && !ntpSudahInit) {
          configTime(7 * 3600, 0, "pool.ntp.org", "time.google.com", "time.nist.gov");
          ntpSudahInit = true;
        }

        firebaseOK = cekInternet();
        cekMemori();

        vTaskDelay(intervalTicks);
      }
      vTaskDelete(NULL);
    },
    "TaskIoT",
    4096,
    nullptr,
    1,
    NULL,
    0);

  // === Task FireBase ===
  xTaskCreatePinnedToCore(
    [](void* param) -> void {
      (void)param;
      const TickType_t intervalTicks = pdMS_TO_TICKS(firebaseIntervalMs);
      for (;;) {
        if (stopFirebaseKirim && errorFirebaseSudahTerkirim) {
          vTaskDelay(intervalTicks);
          continue;
        }

        if (!wifiConnected || !firebaseOK) {
          vTaskDelay(intervalTicks);
          continue;
        }

        updateFirebaseMinute();

        vTaskDelay(intervalTicks);
      }
      vTaskDelete(NULL);
    },
    "TaskFirebase",
    4096,
    nullptr,
    1,
    NULL,
    0);

  // === TaskMonitor ===
  xTaskCreatePinnedToCore(
    [](void* param) -> void {
      (void)param;
      const TickType_t intervalTicks = pdMS_TO_TICKS(monitorIntervalMs);
      for (;;) {
        Monitor(
          suhuGlobal, kelembapanGlobal, kondisiSuhu, kondisiKelembapan, kondisiKadarAir,
          suhuDingin, suhuIdeal, suhuPanas,
          kelembapanKering, kelembapanIdeal, kelembapanLembab,
          rendah1, normal1, tinggi1,
          rendah2, normal2, tinggi2,
          rendah3, normal3, tinggi3,
          hfsRendah, hfsNormal, hfsTinggi,
          kondisiGudang);

        ToEsp32();

        vTaskDelay(intervalTicks);
      }
      vTaskDelete(NULL);
    },
    "TaskMonitor",
    4096,
    nullptr,
    1,
    NULL,
    1);

  // === Task LOAD (Beban Aktuator) ===
  xTaskCreatePinnedToCore(
    [](void* param) -> void {
      (void)param;
      const TickType_t interval = pdMS_TO_TICKS(100);
      for (;;) {

        if (modeSistem != MODE_RUN) {
          digitalWrite(RELAY_R1, LOW);
          digitalWrite(RELAY_R2, LOW);
          dimmer.setPower(0);
          fanState = FAN_OFF;
          vTaskDelay(interval);
          continue;
        }

        stateFan();

        vTaskDelay(interval);
      }

      vTaskDelete(NULL);
    },
    "TaskLoad",
    4096,
    nullptr,
    3,
    NULL,
    1);
}


// ============================================================================
// 2. VOID LOOP
// ============================================================================
void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}



// 🟪 I. TAMPILAN SERIAL MONITOR
// ============================================================================
// 1. SERIAL MONITOR
// ============================================================================
void Monitor(float suhu, float kelembapan, String kondisiSuhu, String kondisiKelembapan, String kondisiKadarAir,
             float suhuDingin, float suhuIdeal, float suhuPanas,
             float kelembapanKering, float kelembapanIdeal, float kelembapanLembab,
             float rendah1, float normal1, float tinggi1,
             float rendah2, float normal2, float tinggi2,
             float rendah3, float normal3, float tinggi3,
             float hfsRendah, float hfsNormal, float hfsTinggi,
             String kondisiGudang) {

  if (modeSistem == MODE_STARTUP) {
    Serial.println("=========================================================================");
    Serial.println("                         INITIZLIZING SISTEM.....                         ");
    Serial.println("                     MENUNGGU SENSOR & SISTEM STABIL                      ");
    Serial.println("=========================================================================");
    return;
  }

  if (modeSistem == MODE_ERROR_TOTAL) {
    Serial.println("=========================================================================");
    Serial.println("                              SISTEM ERROR                               ");
    Serial.println("                        SISTEM DIHENTIKAN TOTAL                          ");
    Serial.print("                 SENSOR BERMASALAH : ");
    Serial.println(errorReason);
    Serial.println("=========================================================================");
    return;
  }

  Serial.println("============================ Pembacaan Sensor BME280 ============================");

  Serial.print("\t    Suhu: ");
  Serial.print(suhu);
  Serial.print("°C    | ");
  Serial.print(kondisiSuhu);
  Serial.print(" |  ");
  Serial.print("  Kelembapan: ");
  Serial.print(kelembapan);
  Serial.print("%  | ");
  Serial.print(kondisiKelembapan);
  Serial.println(" |");

  Serial.print("\t\t -> dingin : ");
  Serial.print(suhuDingin);
  Serial.print("       ");
  Serial.print("       -> kering : ");
  Serial.println(kelembapanKering);

  Serial.print("\t\t -> ideal  : ");
  Serial.print(suhuIdeal);
  Serial.print("       ");
  Serial.print("       -> ideal  : ");
  Serial.println(kelembapanIdeal);

  Serial.print("\t\t -> panas  : ");
  Serial.print(suhuPanas);
  Serial.print("       ");
  Serial.print("       -> lembab : ");
  Serial.println(kelembapanLembab);

  Serial.println("========================= Pembacaan Sensor Soil Moisture ========================");

  Serial.print("                         Sensor 1   : ");
  if (soilMoisturePercentage1 == -1) {
    Serial.println("ERROR : SENSOR KADAR AIR BERMASALAH");
  } else {
    Serial.print(soilMoistureValue1);
    Serial.print("\t→ ");
    Serial.print(soilMoisturePercentage1, 2);
    Serial.println(" %");
  }

  Serial.print("                         Sensor 2   : ");
  if (soilMoisturePercentage2 == -1) {
    Serial.println("ERROR : SENSOR KADAR AIR BERMASALAH");
  } else {
    Serial.print(soilMoistureValue2);
    Serial.print("\t→ ");
    Serial.print(soilMoisturePercentage2, 2);
    Serial.println(" %");
  }

  Serial.print("                         Sensor 3   : ");
  if (soilMoisturePercentage3 == -1) {
    Serial.println("ERROR : SENSOR KADAR AIR BERMASALAH");
  } else {
    Serial.print(soilMoistureValue3);
    Serial.print("\t→ ");
    Serial.print(soilMoisturePercentage3, 2);
    Serial.println(" %");
  }

  Serial.println("=== Derajat Keanggotaan Tiap Sensor ===  |  ===== Tingkat Kadar Air Gabah =======");
  Serial.print("    Kategori      S1  |  S2  |  S3                    |  ");
  Serial.print(kondisiKadarAir);
  Serial.println("  | ");

  Serial.print("    → Rendah   : ");
  Serial.print(rendah1, 2);
  Serial.print(" | ");
  Serial.print(rendah2, 2);
  Serial.print(" | ");
  Serial.print(rendah3, 2);
  Serial.println("\t\t   → Rendah\t: " + String(hfsRendah, 3));

  Serial.print("    → Normal   : ");
  Serial.print(normal1, 2);
  Serial.print(" | ");
  Serial.print(normal2, 2);
  Serial.print(" | ");
  Serial.print(normal3, 2);
  Serial.println("\t\t   → Ideal\t: " + String(hfsNormal, 3));

  Serial.print("    → Tinggi   : ");
  Serial.print(tinggi1, 2);
  Serial.print(" | ");
  Serial.print(tinggi2, 2);
  Serial.print(" | ");
  Serial.print(tinggi3, 2);
  Serial.println("\t\t   → Tinggi\t: " + String(hfsTinggi, 3));

  Serial.println("=================================================================================");
  Serial.print("                                   |  ");
  Serial.print(kondisiGudang);
  Serial.println("  |");
  Serial.println("=================================================================================");

  Serial.print("1. Kipas Angin \t\t: ");
  Serial.print(crispFanGlobal, 1);
  Serial.print("%");
  Serial.print(" \t → DIMMER : ");
  Serial.print((int)toffFanGlobal);
  Serial.print("%");
  Serial.print(" → Kecepatan : ");
  Serial.print((int)tonFanGlobal);
  Serial.print("%");
  Serial.print("\t → STATUS: ");
  if (tonFanGlobal > 0) Serial.println("BEKERJA");
  else Serial.println("MATI");

  Serial.print("2. Humidifier \t\t: ");
  if (humidifierState) Serial.print("BEKERJA");
  else Serial.print("MATI");
  Serial.print(" \t\t → T_OFF : ");
  Serial.print(toffGlobal, 1);
  Serial.println("s");

  Serial.print("3. Dehumidifier \t: ");
  if (dehumidifierState) {
    Serial.println("AKTIF \t → STATUS: BEKERJA");
  } else {
    Serial.println("NONAKTIF \t → STATUS: MATI");
  }

  Serial.print("4. Cooler Fan \t\t: ");
  if (kipasKomponenState) Serial.println("BEKERJA");
  else Serial.println("MATI");

  Serial.print("5. Warna LED RGB \t: ");
  if (modeSistem == MODE_ERROR_TOTAL) {
    Serial.println("MERAH KEDIP CEPAT (SYSTEM ERROR) | ");
    Serial.println("SENSOR : " + errorReason);
  } else {
    if (kondisiGudang == "AMAN") Serial.println("HIJAU");
    else if (kondisiGudang == "WASPADA") Serial.println("BIRU");
    else if (kondisiGudang == "PERINGATAN") Serial.println("MERAH");
    else if (kondisiGudang == "BERBAHAYA") Serial.println("MERAH KEDIP");
    else Serial.println("TIDAK TERDETEKSI");
  }

  Serial.print("6. Buzzer\t\t: ");
  if (buzzerState) Serial.println("BEKERJA");
  else Serial.println("TIDAK BEKERJA");

  Serial.println("================================= STATUS SISTEM =================================");
  Serial.println("============ STATUS WIFI ============================== STATUS MEMORI ===========");

  Serial.print("Status WiFi       : ");
  if (wifiConnected) Serial.print("TERHUBUNG");
  else Serial.print("TIDAK TERHUBUNG");
  Serial.print("\t\t\t Total Heap        : ");
  Serial.print(heapTotal);
  Serial.println(" byte ");

  Serial.print("Kekuatan Sinyal   : ");
  if (wifiConnected) Serial.print(wifiRSSI);
  else Serial.print("-");
  Serial.print(" dBm");
  Serial.print("\t\t\t Heap Terpakai     : ");
  Serial.print(heapUsed);
  Serial.println(" byte ");

  Serial.print("Kecepatan WiFi    : ");
  if (!wifiConnected) Serial.print("TIDAK TERHUBUNG");
  else {
    if (wifiRSSI >= -50) Serial.print("SANGAT CEPAT");
    else if (wifiRSSI >= -60) Serial.print("CEPAT");
    else if (wifiRSSI >= -70) Serial.print("SEDANG");
    else if (wifiRSSI >= -80) Serial.print("LAMBAT");
    else if (wifiRSSI >= -90) Serial.print("SANGAT LAMBAT");
    else Serial.print("HAMPIR PUTUS");
  }
  Serial.print("\t\t\t Heap Tersisa      : ");
  Serial.print(heapFree);
  Serial.println(" byte ");

  Serial.print("IP Address        : ");
  if (wifiConnected) Serial.print(WiFi.localIP());
  else Serial.print("-");
  Serial.print("\t\t\t Persentase RAM    : ");
  Serial.print(heapPercent, 2);
  Serial.println(" % ");

  Serial.print("Koneksi Internet  : ");
  if (firebaseOK) Serial.println("TERHUBUNG");
  else Serial.println("TIDAK TERHUBUNG");

  Serial.println("=================================================================================");

  Serial.print("📤 Data JSON ke ESP32-2: ");
  Serial.println((modeSistem == MODE_RUN) ? "BERHASIL" : "TIDAK TERKIRIM");
}
