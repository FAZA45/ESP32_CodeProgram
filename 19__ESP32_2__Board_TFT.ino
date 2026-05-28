// ============================================================================
//  PROGRAM : ESP32-2 PENERIMA DATA DARI ESP32-1
//  FUNGSI  : Menerima data JSON via UART dan menampilkan ke Serial Monitor
// ============================================================================

/*
 ==============================================================================
 FUNGSI :
 - Menerima data JSON dari ESP32-1 melalui UART2
 - Menampilkan data terstruktur ke Serial Monitor dan TFT
 - ESP32-2 berfungsi sebagai DISPLAY TERMINAL dan MONITORING UNIT.LA

 UART2 digunakan untuk komunikasi antar ESP32
 RX2 = GPIO16  (dari TX ESP32-1)
 TX2 = GPIO17  (ke RX ESP32-1, belum digunakan)
 ==============================================================================
*/

// ============================================================================
// A. LIBRARY
// ============================================================================
#include <Arduino.h>
#include <ArduinoJson.h>
#include <TFT_eSPI.h>
#include <Adafruit_GFX.h>
#include <WiFi.h>
#include "time.h"
#include "logoGudang_240x240_rgb565.h"


// ============================================================================
// B. KONFIGURASI UART2 + WIFI
// ============================================================================
#define UART2_RX 16  // RX ESP32-2  ← TX ESP32-1
#define UART2_TX 17  // TX ESP32-2  → RX ESP32-1 (belum digunakan)
#define UART_BAUDRATE 115200

const char* ssid = "ESP32";
const char* password = "ARDUINOMEGA";


// ============================================================================
// C. ENUM STATE
// ============================================================================
enum ModeSistem {
  MODE_BOOT = 0,
  MODE_STARTUP,
  MODE_NORMAL,
  MODE_ERROR_TOTAL
};


// ============================================================================
// D. STRUCT DATA RX (DATA DARI ESP32-1)
// ============================================================================
// ============================================================================
struct DataRx {
  // ================= MODE SISTEM =================
  String mode;        // "STARTUP" | "NORMAL" | "ERROR"
  String statusJSON;  // "OK"

  // ================= PESAN SISTEM =================
  // STARTUP  : "INITIZLIZING SISTEM....."
  // ERROR    : "ERROR STARTUP - SENSOR: xxx"
  String pesan;

  // ================= DATA SENSOR =================
  // HANYA VALID SAAT MODE == NORMAL
  float suhu;        // °C
  float kelembapan;  // %
  float air1;        // %
  float air2;        // %
  float air3;        // %

  // ================= STATUS FUZZY =================
  String statusSuhu;
  String statusKelembapan;
  String statusKadarAir;
  String kondisiGudang;

  // ================= AKTUATOR =================
  int kipas;            // % (0–100)
  String humidifier;    // "BEKERJA" | "TIDAK BEKERJA"
  String dehumidifier;  // "BEKERJA" | "TIDAK BEKERJA"
  String buzzer;        // "BEKERJA" | "TIDAK BEKERJA"

  // CATATAN:
  // MODE ERROR   → "MERAH_KEDIP_CEPAT"
  // MODE NORMAL  → "HIJAU/BIRU/KUNING/MERAH"
  String led;

  // ================= KONEKSI =================
  String wifiStatus;  // "TERHUBUNG" | "TIDAK TERHUBUNG"
  String wifiSignal;  // "-67 dBm"
  String wifiSpeed;   // "CEPAT", "LAMBAT", dll
  String internet;    // "TERHUBUNG" | "TIDAK TERHUBUNG"

  // ================= SISTEM =================
  float ramUsed;  // Heap ESP32-1 (%)
};


// ============================================================================
// E. VARIABEL GLOBAL
// ============================================================================
// ================= STATE =================
ModeSistem currentMode = MODE_BOOT;
ModeSistem lastMode = MODE_BOOT;

// ================= DATA RX =================
DataRx dataRx;
bool dataBaru = false;

// ================= JSON ===================
// Ukuran disesuaikan dengan JSON ESP32-1 (aman, tidak berlebihan)
StaticJsonDocument<1536> jsonDoc;

// ================= BUFFER UART =================
String uartLine = "";
bool frameReady = false;

// ================= TFT ====================
TFT_eSPI tft = TFT_eSPI();
bool tftInitialized = false;

// ================= STATUS SEBELUMNYA (OPTIMASI TFT) =================
static String prevWifiStatus = "";
static String prevWifiSpeed = "";
static String previnternet = "";
static String prevKondisi = "";
static String prevWaktu = "";
static int prevHeapUsed = -1;

// ================= WAKTU GLOBAL =================
String currentTimeStr = "--:-- - -- --- ----";
bool timeValid = false;


// ============================================================================
// F. PROTOTYPE FUNGSI
// ============================================================================
// ================= STATEGETCURRENT HANDLER =================
void handleBoot();        // MODE_BOOT      → logo saja
void handleStartup();     // MODE_STARTUP   → logo saja
void handleNormal();      // MODE_NORMAL    → data + waktu
void handleErrorTotal();  // MODE_ERROR     → error + waktu

// ================= UART & JSON ==================
void readUART();                   // baca buffer UART
bool parseJSON(const char* json);  // isi struct DataRx

// ================= TAMPILAN TFT =================
void tampilkanBootLogo();                     // khusus BOOT & STARTUP
void tampilkanTFT();                          // MODE_NORMAL saja
void tampilkanTFTError(const String& pesan);  // MODE_ERROR saja

// ================= SERIAL MONITOR ===============
void tampilkanSerialMonitor();  // MODE_NORMAL

// ================= RTOS TASK =================
void taskUART(void* pvParameters);
void taskWiFiNTP(void* pvParameters);
void taskDisplay(void* pvParameters);


// ============================================================================
// G. SETUP + LOOP
// ============================================================================
void setup() {
  // ================= SERIAL =================
  Serial.begin(115200);
  Serial.println();
  Serial.println("========================================");
  Serial.println(" ESP32-2 START ");
  Serial.println("========================================");

  // ================= TFT =================
  tft.init();
  tft.setRotation(3);
  tampilkanBootLogo();  // logo langsung muncul

  // ================= UART =================
  Serial2.begin(115200, SERIAL_8N1, UART2_RX, UART2_TX);

  // ================= WIFI =================
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);  // pastikan bersih
  delay(100);

  // ================= STATE AWAL =================
  currentMode = MODE_BOOT;
  lastMode = MODE_BOOT;

  // ===================================================
  // ================= RTOS TASK =================
  // ===================================================
  // ================= TASK UART =================
  xTaskCreatePinnedToCore(
    taskUART,
    "TaskUART",
    8192,
    NULL,
    3,  // PRIORITAS TERTINGGI
    NULL,
    1  // CORE 1
  );

  // ================= TASK TIME =================
  xTaskCreatePinnedToCore(
    taskWiFiNTP,
    "TaskWiFiNTP",
    8192,
    NULL,
    2,
    NULL,
    0  // CORE 0 (WiFi)
  );

  // ================= TASK DISPLAY =================
  xTaskCreatePinnedToCore(
    taskDisplay,
    "TaskDisplay",
    8192,
    NULL,
    1,
    NULL,
    1);
}

void loop() {
  vTaskDelay(portMAX_DELAY);
}


// ============================================================================
// H. TASK
// ============================================================================
// ================= TASK UART =================
void taskUART(void* pvParameters) {
  for (;;) {
    readUART();
    processFrame();
    vTaskDelay(1);  // yield saja
  }
}

// ================= TASK DIPLAY =================
void taskDisplay(void* pvParameters) {
  for (;;) {

    // ================= DETEKSI TRANSISI STATE =================
    if (currentMode != lastMode) {
      lastMode = currentMode;
      if (currentMode == MODE_NORMAL) {
        tftInitialized = false;
      }
    }

    // ================= STATE MACHINE =================
    switch (currentMode) {
      case MODE_BOOT:
        handleBoot();
        break;

      case MODE_STARTUP:
        handleStartup();
        break;

      case MODE_NORMAL:
        handleNormal();
        break;

      case MODE_ERROR_TOTAL:
        handleErrorTotal();
        break;

      default:
        currentMode = MODE_ERROR_TOTAL;
        break;
    }

    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}


// ============================================================================
// I. MODE BOOT
// ============================================================================
void handleBoot() {
  static bool bootShown = false;
  if (!bootShown) {
    tampilkanBootLogo();
    bootShown = true;
  }

  // BOOT hanya lewat, tidak boleh berhenti di sini
  currentMode = MODE_STARTUP;
}


// ============================================================================
// J. MODE STARTUP
// ============================================================================
void handleStartup() {

  // Keputusan mode SEPENUHNYA dari ESP32-1
  if (dataRx.mode == "NORMAL") {
    tftInitialized = false;
    currentMode = MODE_NORMAL;
    return;
  }

  if (dataRx.mode == "ERROR") {
    currentMode = MODE_ERROR_TOTAL;
    return;
  }

  // Selain itu: tetap di STARTUP (MENUNGGU)
}


// ============================================================================
// K. MODE NORMAL
// ============================================================================
void handleNormal() {
  // ================= TERIMA DATA =================
  if (!dataBaru) return;

  // ================= TRANSISI ERROR =================
  if (dataRx.mode == "ERROR") {
    currentMode = MODE_ERROR_TOTAL;
    dataBaru = false;
    return;
  }

  // ================= SALIN DATA KE VARIABEL LOKAL =================
  float suhu = dataRx.suhu;
  float kelembapan = dataRx.kelembapan;
  float air1 = dataRx.air1;
  float air2 = dataRx.air2;
  float air3 = dataRx.air3;

  String statusSuhu = dataRx.statusSuhu;
  String statusKelembapan = dataRx.statusKelembapan;
  String statusKadarAir = dataRx.statusKadarAir;
  String kondisiGudang = dataRx.kondisiGudang;

  int kipas = dataRx.kipas;
  String humidifier = dataRx.humidifier;
  String dehumid = dataRx.dehumidifier;
  String buzzer = dataRx.buzzer;
  String led = dataRx.led;

  String wifiStat = dataRx.wifiStatus;
  String wifiSig = dataRx.wifiSignal;
  String wifiSpd = dataRx.wifiSpeed;
  String internet = dataRx.internet;
  float ramUsed = dataRx.ramUsed;

  // ================= TAMPILAN SERIAL =================
  tampilkanSerialMonitor(
    suhu, kelembapan,
    air1, air2, air3,
    statusSuhu, statusKelembapan,
    statusKadarAir, kondisiGudang,
    kipas, humidifier,
    dehumid, buzzer, led,
    wifiStat, wifiSig,
    wifiSpd, internet,
    ramUsed);

  // ================= TAMPILAN TFT =================
  tampilkanTFT(
    suhu, kelembapan,
    air1, air2, air3,
    statusSuhu, statusKelembapan,
    statusKadarAir, kondisiGudang,
    kipas, humidifier,
    dehumid, buzzer, led,
    wifiStat, wifiSig,
    wifiSpd, internet,
    ramUsed);

  dataBaru = false;
}


// ============================================================================
// L. MODE ERROR TOTAL
// ============================================================================
void handleErrorTotal() {
  static bool errorDisplayed = false;

  // Tampilkan error HANYA SEKALI
  if (!errorDisplayed) {

    Serial.println();
    Serial.println("========================================");
    Serial.println(" SYSTEM ERROR TOTAL ");
    Serial.println("========================================");

    Serial.print("Pesan Error : ");
    Serial.println(dataRx.pesan);

    tampilkanTFTError(dataRx.pesan);

    errorDisplayed = true;
  }
}


// ============================================================================
// M. READ UART
// ============================================================================
void readUART() {
  while (Serial2.available()) {
    char c = Serial2.read();

    // delimiter resmi
    if (c == '\n') {
      frameReady = true;
      break;  // STOP: satu frame per loop
    }

    // Abaikan CR
    if (c == '\r') continue;

    // proteksi overflow
    if (uartLine.length() < 1500) {
      uartLine += c;
    } else {
      // frame rusak → buang total
      uartLine = "";
      frameReady = false;
    }
  }
}

void processFrame() {
  if (uartLine.length() < 10) {
    uartLine = "";
    return;
  }

  // FRAME BELUM SIAP → JANGAN SENTUH BUFFER
  if (!frameReady) return;

  // FRAME SIAP → AMANKAN
  frameReady = false;

  // copy & reset buffer (WAJIB)
  String json = uartLine;
  uartLine = "";

  // validasi kasar
  if (!json.startsWith("{") || !json.endsWith("}")) {
    Serial.println("❌ FRAME BUKAN JSON (DIBUANG)");
    return;
  }

  // Parse JSON
  DeserializationError err = deserializeJson(jsonDoc, json);
  if (err) {
    Serial.println("❌ JSON PARSE ERROR");
    jsonDoc.clear();
    return;
  }

  // ===== FIELD WAJIB =====
  dataRx.mode = jsonDoc["mode"] | "";
  dataRx.statusJSON = jsonDoc["statusJSON"] | "";

  // ===== MODE STARTUP =====
  if (dataRx.mode == "STARTUP") {
    dataRx.pesan = jsonDoc["pesan"] | "";
    dataBaru = true;
    return;
  }

  // ===== MODE ERROR =====
  if (dataRx.mode == "ERROR") {
    dataRx.pesan = jsonDoc["pesan"] | "";
    dataRx.led = jsonDoc["led"] | "";
    dataBaru = true;
    return;
  }

  // ===== MODE NORMAL =====
  if (dataRx.mode == "NORMAL") {

    dataRx.suhu = jsonDoc["suhu"] | 0.0;
    dataRx.kelembapan = jsonDoc["kelembapan"] | 0.0;
    dataRx.air1 = jsonDoc["air1"] | 0.0;
    dataRx.air2 = jsonDoc["air2"] | 0.0;
    dataRx.air3 = jsonDoc["air3"] | 0.0;

    dataRx.statusSuhu = jsonDoc["statusSuhu"] | "";
    dataRx.statusKelembapan = jsonDoc["statusKelembapan"] | "";
    dataRx.statusKadarAir = jsonDoc["statusKadarAir"] | "";
    dataRx.kondisiGudang = jsonDoc["kondisiGudang"] | "";
    dataRx.kipas = jsonDoc["kipas"] | 0.0;
    dataRx.humidifier = jsonDoc["humidifier"] | "";
    dataRx.dehumidifier = jsonDoc["dehumidifier"] | "";
    dataRx.buzzer = jsonDoc["buzzer"] | "";
    dataRx.led = jsonDoc["led"] | "";

    dataRx.wifiStatus = jsonDoc["wifiStatus"] | "";
    dataRx.wifiSignal = jsonDoc["wifiSignal"] | "";
    dataRx.wifiSpeed = jsonDoc["wifiSpeed"] | "";
    dataRx.internet = jsonDoc["internet"] | "";
    dataRx.ramUsed = jsonDoc["ramUsed"] | 0.0;

    dataBaru = true;
  }
}


// ============================================================================
// N. FUNGSI TAMPILAN SERIAL MONITOR
// ============================================================================
void tampilkanSerialMonitor(
  float suhu, float kelembapan,
  float air1, float air2, float air3,
  String statusSuhu, String statusKelembapan,
  String statusKadarAir, String kondisiGudang,
  int kipas, String humidifier,
  String dehumid, String buzzer, String led,
  String wifiStat, String wifiSig,
  String wifiSpd, String internet,
  float ramUsed) {

  Serial.println();
  Serial.println("========================================================================================");
  Serial.println("DATA DITERIMA DARI ESP32-1");
  Serial.println("========================================================================================");

  Serial.println("DATA SENSOR                    | AKTUATOR");
  Serial.printf("Suhu          : %.1f °C         | Kipas        : %d %%\n", suhu, kipas);
  Serial.printf("Kelembapan    : %.1f %%         | Humidifier   : %s\n", kelembapan, humidifier.c_str());
  Serial.printf("Kadar Air 1   : %.2f %%         | Dehumidifier : %s\n", air1, dehumid.c_str());
  Serial.printf("Kadar Air 2   : %.2f %%         | Buzzer       : %s\n", air2, buzzer.c_str());
  Serial.printf("Kadar Air 3   : %.2f %%         | LED          : %s\n", air3, led.c_str());

  Serial.println("----------------------------------------------------------------------------------------");
  Serial.println("STATUS FUZZY                   | KONEKSI");
  Serial.printf("Status Suhu       : %s     | WiFi Status  : %s\n", statusSuhu.c_str(), wifiStat.c_str());
  Serial.printf("Status Kelembapan : %s     | Signal       : %s\n", statusKelembapan.c_str(), wifiSig.c_str());
  Serial.printf("Status Kadar Air  : %s     | WiFi Speed   : %s\n", statusKadarAir.c_str(), wifiSpd.c_str());
  Serial.printf("Kondisi Gudang    : %s     | Internet     : %s\n", kondisiGudang.c_str(), internet.c_str());

  // ================= STATUS MEMORI =================
  int heapFree = ESP.getFreeHeap();
  int heapTotal = ESP.getHeapSize();
  int heapUsed = heapTotal - heapFree;
  float heapPct = ((float)heapUsed / heapTotal) * 100.0;

  Serial.println();
  Serial.println("========================================================================================");
  Serial.println("============ STATUS MEMORI ================================================");
  Serial.printf("Total Heap        : %d byte\n", heapTotal);
  Serial.printf("Heap Terpakai     : %d byte\n", heapUsed);
  Serial.printf("Heap Tersisa      : %d byte\n", heapFree);
  Serial.printf("ESP32 2 - RAM     : %.2f %%\n", heapPct);
  Serial.printf("ESP32 1 - RAM     : %.1f %%\n", ramUsed);
  Serial.println("========================================================================================");

  Serial.println("================ WAKTU ===================================================");
  String waktuSekarang = currentTimeStr;
  Serial.print("Waktu       : ");
  Serial.println(waktuSekarang);
  Serial.println("========================================================================================");
}


// ============================================================================
// O. FUNGSI TAMPILAN TFT (LANDSCAPE)
// ============================================================================
void tampilkanTFTError(const String& pesan) {

  tft.fillScreen(TFT_RED);
  tft.setTextColor(TFT_BLACK, TFT_RED);

  // ================= DATA =================
  String judul = "SYSTEM ERROR";
  String waktu = currentTimeStr;

  // Ambil hanya bagian sensor
  String sensorError = pesan;
  int idx = sensorError.lastIndexOf(':');
  if (idx >= 0) {
    sensorError = sensorError.substring(idx + 1);
  }
  sensorError.trim();


  // ================= JUDUL (FONT BESAR) =================
  tft.setTextSize(3);
  int wJudul = tft.textWidth(judul);
  int yJudul = 60;

  tft.setCursor((480 - wJudul) / 2, yJudul);
  tft.print(judul);

  // ================= LABEL SENSOR =================
  tft.setTextSize(2);
  String label = "SENSOR :";
  int wLabel = tft.textWidth(label);
  int yLabel = yJudul + 50;

  tft.setCursor((480 - wLabel) / 2, yLabel);
  tft.print(label);

  // ================= JENIS SENSOR ERROR =================
  int wSensor = tft.textWidth(sensorError);
  int ySensor = yLabel + 25;  // jarak rapat sesuai permintaan

  tft.setCursor((480 - wSensor) / 2, ySensor);
  tft.print(sensorError);

  // ================= WAKTU =================
  int wWaktu = tft.textWidth(waktu);
  int yWaktu = ySensor + 45;

  tft.setCursor((480 - wWaktu) / 2, yWaktu);
  tft.print(waktu);
}


void tampilkanTFT(
  float suhu, float kelembapan,
  float air1, float air2, float air3,
  String statusSuhu, String statusKelembapan,
  String statusKadarAir, String kondisiGudang,
  int kipas, String humidifier,
  String dehumid, String buzzer, String led,
  String wifiStat, String wifiSig,
  String wifiSpd, String internet,
  float ramUsed) {

  static float pSuhu = -999, pKelembapan = -999, pAir1 = -999, pAir2 = -999, pAir3 = -999;
  static String pStatusSuhu = "", pStatusKelembapan = "", pStatusKadarAir = "", pKondisiGudang = "";
  static int pKipas = -1;
  static String pHumidifier = "";
  static String pDehumid = "", pBuzzer = "", pLed = "";
  static String pWifiStat = "", pWifiSig = "", pWifiSpd = "", pinternet = "";
  static String pWaktu = "";
  static int pHeapUsed = -1;

  // ================= KONSTANTA LAYOUT =================
  const int M = 5;  // margin global
  const int W = 480;
  const int H = 320;

  // ================= INIT TFT =================
  if (!tftInitialized) {

    tft.fillScreen(TFT_WHITE);
    tft.setTextColor(TFT_BLACK, TFT_WHITE);

    // BLOK 1 HEADER
    tft.fillRect(M, M, W - 2 * M, 60, TFT_WHITE);
    tft.drawRect(M, M, W - 2 * M, 60, TFT_BLACK);
    tft.setTextSize(3);

    String title1 = "SMART MONITORING";
    int w1 = tft.textWidth(title1);
    tft.setCursor((W - w1) / 2, M + 5);
    tft.print(title1);

    String title2 = "GUDANG PADI";
    int w2 = tft.textWidth(title2);
    tft.setCursor((W - w2) / 2, M + 32);
    tft.print(title2);

    // BLOK 2 WAKTU
    tft.fillRect(M, 70, W - 2 * M, 20, TFT_WHITE);
    tft.drawRect(M, 70, W - 2 * M, 20, TFT_BLACK);

    // BLOK 3 KONDISI GUDANG
    tft.drawRect(M, 95, W - 2 * M, 45, TFT_BLACK);

    // BLOK 4 Judul Kolom
    tft.drawRect(M, 145, 145, 120, TFT_BLACK);
    tft.drawRect(M + 150, 145, 155, 120, TFT_BLACK);
    tft.drawRect(M + 312, 145, 158, 120, TFT_BLACK);

    tft.setTextSize(2);
    tft.setCursor(M + 5, 150);
    tft.print("DATA SENSOR");
    tft.setCursor(M + 155, 150);
    tft.print("PARAMETER");
    tft.setCursor(M + 316, 150);
    tft.print("AKTUATOR");

    // BLOK 7 FOOTER
    tft.fillRect(M, 280, W - 2 * M, 35, TFT_GREEN);

    tftInitialized = true;
  }

  // ================= LOGIKA TFT =================
  // UPDATE BLOK 2 WAKTU
  String waktuSekarang = currentTimeStr;
  if (waktuSekarang != pWaktu) {

    tft.fillRect(M + 1, 71, W - 2 * M - 2, 18, TFT_WHITE);
    tft.setTextSize(1);
    tft.setTextColor(TFT_BLACK, TFT_WHITE);

    int textW = tft.textWidth(waktuSekarang);
    int x = (W - textW) / 2;
    int y = 75;

    tft.setCursor(x, y);
    tft.print(waktuSekarang);

    pWaktu = waktuSekarang;
  }

  // UPDATE BLOK 3 KONDISI GUDANG
  if (kondisiGudang != pKondisiGudang) {

    uint16_t warna = TFT_GREEN;
    String k = kondisiGudang;
    k.toUpperCase();

    if (k == "AMAN") warna = TFT_GREEN;
    else if (k == "WASPADA") warna = TFT_YELLOW;
    else if (k == "PERINGATAN") warna = TFT_ORANGE;
    else if (k == "BERBAHAYA") warna = TFT_RED;

    tft.fillRect(M + 1, 96, W - 2 * M - 2, 43, warna);
    tft.setTextSize(3);
    tft.setTextColor(TFT_BLACK, warna);

    int textW = tft.textWidth(k);
    tft.setCursor((W - textW) / 2, 105);
    tft.print(k);

    pKondisiGudang = kondisiGudang;
  }

  // UPDATE BLOK 4 KOLOM UTAMA - SENSOR
  tft.setTextSize(1);
  tft.setTextColor(TFT_BLACK, TFT_WHITE);
  tft.fillRect(M + 1, 175, 143, 85, TFT_WHITE);

  tft.setCursor(M + 5, 175);
  tft.printf("Suhu        : %.1f C", suhu);
  tft.setCursor(M + 5, 190);
  tft.printf("Kelembapan  : %.1f %%", kelembapan);
  tft.setCursor(M + 5, 205);
  tft.printf("Kadar Air 1 : %.2f %%", air1);
  tft.setCursor(M + 5, 220);
  tft.printf("Kadar Air 2 : %.2f %%", air2);
  tft.setCursor(M + 5, 235);
  tft.printf("Kadar Air 3 : %.2f %%", air3);

  // STATUS FUZZY
  tft.fillRect(M + 151, 175, 150, 78, TFT_WHITE);

  tft.setCursor(M + 155, 180);
  tft.print("Suhu       : " + statusSuhu);
  tft.setCursor(M + 155, 200);
  tft.print("Kelembapan : " + statusKelembapan);
  tft.setCursor(M + 155, 220);
  tft.print("Kadar Air  : " + statusKadarAir);

  // AKTUATOR
  tft.fillRect(M + 313, 175, 155, 80, TFT_WHITE);

  tft.setCursor(M + 316, 175);
  tft.printf("Kipas      : %d %%", kipas);
  tft.setCursor(M + 316, 190);
  tft.print("Humidifier : " + humidifier);
  tft.setCursor(M + 316, 205);
  tft.print("Dehumid    : " + dehumid);
  tft.setCursor(M + 316, 220);
  tft.print("Buzzer     : " + buzzer);
  tft.setCursor(M + 316, 235);
  tft.print("LED        : " + led);

  // BLOK 7 FOOTER
  int heapUsed = ESP.getHeapSize() - ESP.getFreeHeap();
  if (wifiStat != pWifiStat || wifiSpd != pWifiSpd || internet != pinternet || heapUsed != pHeapUsed) {

    tft.fillRect(M, 280, W - 2 * M, 35, TFT_GREEN);
    tft.setTextColor(TFT_BLACK, TFT_GREEN);
    tft.setTextSize(1);

    // BARIS 1
    tft.setCursor(M + 5, 285);
    tft.print("WiFi    : " + wifiStat);

    tft.setCursor(M + 155, 285);
    tft.print("Signal: " + wifiSig);

    tft.setCursor(M + 316, 285);
    tft.print("Speed: " + wifiSpd);

    // BARIS 2
    tft.setCursor(M + 5, 298);
    tft.print("Internet: " + internet);

    tft.setCursor(M + 155, 298);
    tft.printf("RAM-2 : %.1f %%", ((float)heapUsed / ESP.getHeapSize()) * 100.0);

    tft.setCursor(M + 316, 298);
    tft.printf("RAM-1: %.1f %%", ramUsed);

    pWifiStat = wifiStat;
    pWifiSpd = wifiSpd;
    pinternet = internet;
    pHeapUsed = heapUsed;
  }
}


// ============================================================================
// P. BOOT LOGO
// ============================================================================
void tampilkanBootLogo() {
  tft.fillScreen(TFT_BLACK);
  tft.pushImage(
    120, 40,
    240, 240,
    logoGudang);

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(1);
  tft.setCursor(160, 300);
  tft.print("Initializing system...");
}


// ============================================================================
// Q. TIME
// ============================================================================
void taskWiFiNTP(void* pvParameters) {
  const char* ntpServer1 = "time.google.com";
  const char* ntpServer2 = "pool.ntp.org";
  const char* tzInfo = "WIB-7";

  for (;;) {
    // ================= WIFI =================
    while (WiFi.status() != WL_CONNECTED) {
      Serial.println("[WIFI] Connecting...");
      vTaskDelay(500 / portTICK_PERIOD_MS);
      WiFi.begin(ssid, password);

      // kasih waktu WiFi stack kerja
      for (int i = 0; i < 20; i++) {
        if (WiFi.status() == WL_CONNECTED) break;
        vTaskDelay(500 / portTICK_PERIOD_MS);
      }
    }

    Serial.print("[WIFI] Connected. IP: ");
    Serial.println(WiFi.localIP());

    // ================= NTP =================
    timeValid = false;
    currentTimeStr = "--:-- - -- --- ----";

    while (!timeValid) {
      Serial.println("[TIME] Init NTP");
      configTzTime(tzInfo, ntpServer1, ntpServer2);

      struct tm timeinfo;
      for (int i = 0; i < 20; i++) {  // tunggu max ~20 detik
        if (getLocalTime(&timeinfo)) {
          char buffer[30];
          strftime(buffer, sizeof(buffer),
                   "%H:%M - %d %b %Y", &timeinfo);
          currentTimeStr = buffer;
          timeValid = true;

          Serial.println("[TIME] NTP OK");
          Serial.println(currentTimeStr);
          break;
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
      }

      if (!timeValid) {
        Serial.println("[TIME] NTP FAIL, retry...");
        vTaskDelay(10000 / portTICK_PERIOD_MS);  // santai
      }
    }

    // ================= UPDATE WAKTU PERIODIK =================
    while (WiFi.status() == WL_CONNECTED) {
      struct tm timeinfo;
      if (getLocalTime(&timeinfo)) {
        char buffer[30];
        strftime(buffer, sizeof(buffer),
                 "%H:%M - %d %b %Y", &timeinfo);
        currentTimeStr = buffer;
      }

      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    // kalau WiFi putus, ulang dari awal
    Serial.println("[WIFI] Disconnected, restart WiFi+NTP");
    timeValid = false;
  }
}
