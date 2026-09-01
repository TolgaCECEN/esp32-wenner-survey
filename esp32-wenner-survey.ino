#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <EEPROM.h>
#include <Adafruit_ADS1115.h>
#include <Adafruit_INA219.h>
#include <RTClib.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
// =====================================================
// ESP32 - ÇOK SEVİYELİ WENNER / IP TARAMA
// =====================================================
#define VERSION "V4.07 (COKLU FREKANS + DINAMIK KALMAN)"
// =====================================================
// OLED
// =====================================================
#define OLED_WIDTH 128
#define OLED_HEIGHT 64
#define OLED_ADDR 0x3C
Adafruit_SSD1306 display(
  OLED_WIDTH,
  OLED_HEIGHT,
  &Wire,
  -1
);
// =====================================================
// I2C
// ADS1115 + INA219 + DS3231 + OLED
// =====================================================
#define I2C_SDA 25
#define I2C_SCL 27
Adafruit_ADS1115 ads1115(0x48);
Adafruit_INA219 ina219;
RTC_DS3231 rtc;
// =====================================================
// SD SPI
// =====================================================
#define SD_CS   5
#define SD_SCK  18
#define SD_MISO 19
#define SD_MOSI 23
bool sdOK = false;
// =====================================================
// AKIM KONTROL
// =====================================================
#define CURRENT_CTRL 26
// =====================================================
// BUTONLAR
// =====================================================
#define BUTTON_START 32
#define BUTTON_STOP  33
#define BUTTON_CAL   13
// =====================================================
// LED
// =====================================================
#define LED_MEASURE 4
#define LED_ERROR   16
#define LED_FAULT   17
#define LED_SD      21
#define LED_CAL     22
// =====================================================
// BATARYA
// =====================================================
#define BATTERY_PIN 34
#define LOW_BATTERY_WARNING 11.0
// =====================================================
// WENNER SEVİYELERİ (sabit - kod icinden degistirin)
// =====================================================
#define WENNER_LEVELS 3
float wennerSpacing[WENNER_LEVELS] = {
  5.0,
  10.0,
  20.0
};
// =====================================================
// ÖLÇÜM AYARLARI
// =====================================================
#define MEASUREMENTS_PER_POINT 5
#define TON_TIME     1500
#define REST_TIME    1500
#define ADC_SAMPLES  8
#define ADC_DELAY    2
#define MAX_STRING 32
// =====================================================
// ORTAM MODU
// =====================================================
enum Environment {
  ENV_LAB,
  ENV_FIELD
};
Environment currentEnv = ENV_FIELD;  // Varsayılan: SAHA

// =====================================================
// DINAMIK KALMAN PARAMETRELERI
// =====================================================
#define KALMAN_Q_LAB   0.01
#define KALMAN_R_LAB   0.10
#define KALMAN_Q_FIELD 0.05
#define KALMAN_R_FIELD 0.20
#define KALMAN_ERROR_THRESHOLD 0.05
// =====================================================
// PROB TEMAS DIRENCI KONTROLU
// =====================================================
#define CONTACT_TEST_DURATIONS 3
int contactTestDurations[CONTACT_TEST_DURATIONS] = {100, 500, 1500};
#define CONTACT_WARN_OHM   5000.0
#define CONTACT_POOR_OHM   10000.0
#define CONTACT_STABILIZE_DELAY 100
// =====================================================
// YÖN TEKRAR (tum seviyeler gecersizse)
// =====================================================
#define MAX_DIRECTION_RETRY 2
// =====================================================
// FİLTRE
// =====================================================
#define SPIKE_THRESHOLD 3.0
// =====================================================
// EEPROM
// =====================================================
#define EEPROM_SIZE 256
#define CALIBRATION_ADDR 0
// =====================================================
// UYARLANABILIR KALMAN
// =====================================================
struct AdaptiveKalman {
  float q;
  float r;
  float p;
  float k;
  float x;
  float error_variance;
  float q_base;
  float r_base;
};
// =====================================================
// KALİBRASYON
// =====================================================
struct CalData {
  float zero_offset_voltage;
  float zero_offset_current;
  bool calibrated;
  unsigned long last_cal;
};
// =====================================================
// TEK ÖLÇÜM
// =====================================================
struct SingleMeas {
  float V1;
  float V2;
  float I;
  float IP;
  float rho;
  bool reverse;
};
// =====================================================
// WENNER SONUCU
// =====================================================
struct WennerResult {
  float spacing;
  float rho;
  float ip;
  float rhoStd;
  float ipStd;
  float anomaly;
  bool valid;
  float contactR;
};
// =====================================================
// YÖN
// =====================================================
enum Direction {
  DIR_CENTER = 0,
  DIR_LEFT,
  DIR_RIGHT,
  DIR_UP,
  DIR_DOWN
};
const char* directionNames[] = {
  "MERKEZ",
  "SOL",
  "SAG",
  "YUKARI",
  "ASAGI"
};
#define DIRECTION_COUNT 5
// =====================================================
// GLOBAL
// =====================================================
CalData cal;
bool measuring = false;
bool calibrating = false;
WennerResult results[
  DIRECTION_COUNT
][
  WENNER_LEVELS
];
// =====================================================
// GEOMETRİK FAKTÖR
// WENNER ARRAY
//
// rho = 2 * PI * a * V / I
// =====================================================
float wennerK(float a) {
  return 2.0 * PI * a;
}
// =====================================================
// UYARLANABILIR KALMAN BASLAT
// =====================================================
AdaptiveKalman initAdaptiveKalman(float q_base, float r_base) {
  AdaptiveKalman ak;
  ak.q_base = q_base;
  ak.r_base = r_base;
  ak.q = q_base;
  ak.r = r_base;
  ak.p = 1.0;
  ak.k = 0.0;
  ak.x = 0.0;
  ak.error_variance = 0.0;
  return ak;
}
// =====================================================
// UYARLANABILIR KALMAN FİLTRE
// =====================================================
float adaptiveKalmanFilter(AdaptiveKalman* ak, float measurement) {
  ak->p += ak->q;
  ak->k = ak->p / (ak->p + ak->r);
  
  float error = measurement - ak->x;
  ak->x += ak->k * error;
  ak->p = (1.0 - ak->k) * ak->p;
  
  // Hataların varyansını izle (exponential moving average)
  ak->error_variance = ak->error_variance * 0.9 + error * error * 0.1;
  
  // Yüksek hata varsa parametreleri dinamik olarak artır
  if (ak->error_variance > KALMAN_ERROR_THRESHOLD) {
    ak->q = ak->q_base * 2.0;  // İşlem gürültüsü artır (daha az güven)
    ak->r = ak->r_base * 2.0;  // Ölçüm gürültüsü artır (daha az güven)
  } else {
    ak->q = ak->q_base;
    ak->r = ak->r_base;
  }
  
  return ak->x;
}
// =====================================================
// FİLTRELİ OKUMA (Dinamik Kalman ile)
// =====================================================
float filteredADSVoltage() {
  // Ortama göre parametreleri seç
  float q_base = (currentEnv == ENV_LAB) ? KALMAN_Q_LAB : KALMAN_Q_FIELD;
  float r_base = (currentEnv == ENV_LAB) ? KALMAN_R_LAB : KALMAN_R_FIELD;
  
  AdaptiveKalman ak = initAdaptiveKalman(q_base, r_base);
  float value = 0;
  
  for (int i = 0; i < ADC_SAMPLES; i++) {
    int16_t raw = ads1115.readADC_SingleEnded(0);
    float voltage = ads1115.computeVolts(raw);
    value = adaptiveKalmanFilter(&ak, voltage);
    delay(ADC_DELAY);
  }
  return value;
}

float filteredCurrent() {
  // Ortama göre parametreleri seç
  float q_base = (currentEnv == ENV_LAB) ? KALMAN_Q_LAB : KALMAN_Q_FIELD;
  float r_base = (currentEnv == ENV_LAB) ? KALMAN_R_LAB : KALMAN_R_FIELD;
  
  AdaptiveKalman ak = initAdaptiveKalman(q_base, r_base);
  float value = 0;
  
  for (int i = 0; i < ADC_SAMPLES; i++) {
    float current = ina219.getCurrent_mA() / 1000.0;
    value = adaptiveKalmanFilter(&ak, current);
    delay(ADC_DELAY);
  }
  return value;
}
// =====================================================
// BATARYA
// =====================================================
float batteryVoltage() {
  int raw = analogRead(BATTERY_PIN);
  return
    ((float)raw / 4095.0) *
    3.3 *
    11.0;
}
// =====================================================
// AKIM AÇ / KAPA
// =====================================================
void inj(bool state) {
  digitalWrite(
    CURRENT_CTRL,
    state ? HIGH : LOW
  );
  delay(20);
}
// =====================================================
// PROB TEMAS DİRENCİ ÖLÇÜMÜ (ÇOKLU FREKANS)
// =====================================================
// Üç farklı darbe süresiyle (100ms, 500ms, 1500ms) test yapıp
// ortalamasını alır. Bu şekilde frekans bağımlılığı azalır ve
// daha güvenilir bir temas direnci değeri elde edilir.
float checkContactResistance() {
  float rValues[CONTACT_TEST_DURATIONS];
  
  for (int idx = 0; idx < CONTACT_TEST_DURATIONS; idx++) {
    int duration = contactTestDurations[idx];
    
    inj(true);
    delay(duration);
    
    // Stabilizasyon: darbe sonrası geçici etkileri atlamak için bekle
    delay(CONTACT_STABILIZE_DELAY);
    
    float v = filteredADSVoltage() - cal.zero_offset_voltage;
    float i = filteredCurrent() - cal.zero_offset_current;
    inj(false);
    
    // Direnç hesapla
    if (fabs(i) > 0.000001) {
      rValues[idx] = fabs(v) / fabs(i);
    } else {
      rValues[idx] = 999999.0;  // Açık devre
    }
    
    // Elektrotlar arasında sakinleşme süresi
    delay(200);
  }
  
  // Üç ölçümün ortalamasını döndür
  float avgR = (rValues[0] + rValues[1] + rValues[2]) / 3.0;
  
  // Debug: Serial'de tüm değerleri yaz
  Serial.printf(
    "Temas R: %.0f (100ms), %.0f (500ms), %.0f (1500ms) -> Ort: %.0f Ohm\n",
    rValues[0], rValues[1], rValues[2], avgR
  );
  
  return avgR;
}
// =====================================================
// ARRAY ÖLÇÜMÜ
// =====================================================
SingleMeas measureSingle() {
  SingleMeas r;
  r.reverse = false;
  inj(true);
  delay(TON_TIME);
  r.V1 =
    filteredADSVoltage()
    - cal.zero_offset_voltage;
  r.I =
    filteredCurrent()
    - cal.zero_offset_current;
  inj(false);
  delay(100);
  r.V2 =
    filteredADSVoltage()
    - cal.zero_offset_voltage;
  if (fabs(r.V1) > 0.000001) {
    r.IP =
      fabs(r.V2 / r.V1) *
      100.0;
  } else {
    r.IP = 0;
  }
  r.rho = 0;
  return r;
}
// =====================================================
// ORTALAMA
// =====================================================
float average(
  float* data,
  int count
) {
  if (count <= 0)
    return 0;
  float sum = 0;
  for (int i = 0; i < count; i++)
    sum += data[i];
  return sum / count;
}
// =====================================================
// STANDART SAPMA
// =====================================================
float standardDeviation(
  float* data,
  int count,
  float mean
) {
  if (count <= 1)
    return 0;
  float sum = 0;
  for (int i = 0; i < count; i++) {
    float d =
      data[i] - mean;
    sum += d * d;
  }
  return sqrt(
    sum / (count - 1)
  );
}
// =====================================================
// WENNER ÖLÇÜMÜ
// =====================================================
WennerResult measureWenner(
  float spacing
) {
  WennerResult result;
  result.spacing = spacing;
  result.rho = 0;
  result.ip = 0;
  result.rhoStd = 0;
  result.ipStd = 0;
  result.anomaly = 0;
  result.valid = false;
  result.contactR = 0;
  float rhoValues[
    MEASUREMENTS_PER_POINT
  ];
  float ipValues[
    MEASUREMENTS_PER_POINT
  ];
  float K =
    wennerK(spacing);
  for (
    int n = 0;
    n < MEASUREMENTS_PER_POINT;
    n++
  ) {
    SingleMeas m =
      measureSingle();
    if (
      fabs(m.I) >
      0.000001
    ) {
      m.rho =
        K *
        fabs(m.V1) /
        fabs(m.I);
    } else {
      m.rho = 0;
    }
    rhoValues[n] = m.rho;
    ipValues[n] = m.IP;
    delay(REST_TIME);
  }
  result.rho =
    average(
      rhoValues,
      MEASUREMENTS_PER_POINT
    );
  result.ip =
    average(
      ipValues,
      MEASUREMENTS_PER_POINT
    );
  result.rhoStd =
    standardDeviation(
      rhoValues,
      MEASUREMENTS_PER_POINT,
      result.rho
    );
  result.ipStd =
    standardDeviation(
      ipValues,
      MEASUREMENTS_PER_POINT,
      result.ip
    );
  // --------------------------------
  // GEÇERLİLİK
  // --------------------------------
  bool rhoOK =
    result.rho > 0 &&
    result.rhoStd <
      result.rho * 0.10;
  bool ipOK =
    fabs(result.ip) > 0.0001 &&
    result.ipStd <
      fabs(result.ip) * 0.10;
  result.valid = rhoOK && ipOK;
  return result;
}
// =====================================================
// ANOMALİ HESABI
// =====================================================
float calculateAnomaly(
  float rho,
  float ip
) {
  if (rho <= 0)
    return 0;
  float rhoScore;
  if (rho < 1)
    rhoScore = 1.0;
  else if (rho < 5)
    rhoScore = 0.9;
  else if (rho < 10)
    rhoScore = 0.8;
  else if (rho < 50)
    rhoScore = 0.6;
  else if (rho < 100)
    rhoScore = 0.4;
  else if (rho < 500)
    rhoScore = 0.2;
  else
    rhoScore = 0.05;
  float ipScore;
  if (ip > 20)
    ipScore = 1.0;
  else if (ip > 15)
    ipScore = 0.9;
  else if (ip > 10)
    ipScore = 0.75;
  else if (ip > 5)
    ipScore = 0.55;
  else if (ip > 2)
    ipScore = 0.30;
  else
    ipScore = 0.05;
  return
    (
      rhoScore * 0.50 +
      ipScore  * 0.50
    ) * 100.0;
}
// =====================================================
// DERİNLİK TAHMİNİ
// =====================================================
float estimatedDepth(
  float spacing
) {
  return spacing * 0.5;
}
// =====================================================
// YÖN SKORU
// =====================================================
float directionScore(
  int direction
) {
  float total = 0;
  int count = 0;
  for (
    int l = 0;
    l < WENNER_LEVELS;
    l++
  ) {
    if (
      results[direction][l].valid
    ) {
      total +=
        results[direction][l].anomaly;
      count++;
    }
  }
  if (count == 0)
    return 0;
  return total / count;
}
// =====================================================
// EN GÜÇLÜ YÖN
// =====================================================
int strongestDirection() {
  int best = DIR_CENTER;
  float bestScore = -1;
  for (
    int d = 0;
    d < DIRECTION_COUNT;
    d++
  ) {
    float score =
      directionScore(d);
    if (score > bestScore) {
      bestScore = score;
      best = d;
    }
  }
  return best;
}
// =====================================================
// EN GÜÇLÜ SEVİYE
// =====================================================
int strongestLevel(
  int direction
) {
  int best = -1;
  float score = -1;
  for (
    int l = 0;
    l < WENNER_LEVELS;
    l++
  ) {
    if (
      results[direction][l].valid &&
      results[direction][l].anomaly >
      score
    ) {
      score =
        results[direction][l].anomaly;
      best = l;
    }
  }
  if (best == -1)
    best = 0;
  return best;
}
// =====================================================
// MESAFE / DERİNLİK TAHMİNİ
// =====================================================
void calculateTarget(
  int direction,
  float &distance,
  float &depth,
  float &confidence
) {
  int level =
    strongestLevel(direction);
  WennerResult &r =
    results[direction][level];
  distance =
    r.spacing;
  depth =
    estimatedDepth(r.spacing);
  confidence =
    r.anomaly;
  if (confidence > 100)
    confidence = 100;
}
// =====================================================
// ORTAM MODU EKRANI
// =====================================================
void showEnvironmentMenu() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println(
    "ORTAM MODU SECIMI"
  );
  display.println();
  display.println(
    "START: LAB (stabil)"
  );
  display.println();
  display.println(
    "STOP: SAHA (gurultulu)"
  );
  display.display();
}

void selectEnvironment() {
  showEnvironmentMenu();
  
  while (true) {
    if (
      digitalRead(BUTTON_START)
      == LOW
    ) {
      delay(50);
      if (
        digitalRead(BUTTON_START)
        == LOW
      ) {
        while (
          digitalRead(BUTTON_START)
          == LOW
        ) {
          delay(10);
        }
        currentEnv = ENV_LAB;
        break;
      }
    }
    if (
      digitalRead(BUTTON_STOP)
      == LOW
    ) {
      delay(50);
      if (
        digitalRead(BUTTON_STOP)
        == LOW
      ) {
        while (
          digitalRead(BUTTON_STOP)
          == LOW
        ) {
          delay(10);
        }
        currentEnv = ENV_FIELD;
        break;
      }
    }
    delay(10);
  }
  
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println(
    "ORTAM SECILDI:"
  );
  display.println();
  display.printf(
    "%s\n",
    (currentEnv == ENV_LAB) ? "LAB" : "SAHA"
  );
  display.display();
  
  Serial.printf(
    "Ortam Modu: %s\n",
    (currentEnv == ENV_LAB) ? "LAB" : "SAHA"
  );
  
  delay(1500);
  showMain();
}
// =====================================================
// OLED ANA EKRAN
// =====================================================
void showMain() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println(
    " MADEN ARAMA"
  );
  display.printf(
    "%s\n",
    VERSION
  );
  display.printf(
    "Pil: %.2fV\n",
    batteryVoltage()
  );
  display.printf(
    "Ortam: %s\n",
    (currentEnv == ENV_LAB) ? "LAB" : "SAHA"
  );
  display.printf(
    "Wenner: %.0f/%.0f/%.0fm\n",
    wennerSpacing[0],
    wennerSpacing[1],
    wennerSpacing[2]
  );
  display.println(
    "START: TARAMA"
  );
  display.println(
    "CAL: PROB SIFIR"
  );
  display.display();
}
// =====================================================
// START BEKLE
// =====================================================
bool waitForStart();
// =====================================================
// PROB SIFIR KALİBRASYONU
// =====================================================
void zeroCalibration() {
  calibrating = true;
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println(
    " PROB SIFIR"
  );
  display.println();
  display.println(
    "Problari birbirinden"
  );
  display.println(
    "izole edin."
  );
  display.println();
  display.println(
    "START = BASLAT"
  );
  display.println(
    "STOP  = IPTAL"
  );
  display.display();
  if (!waitForStart()) {
    calibrating = false;
    showMain();
    return;
  }
  inj(false);
  delay(500);
  float v =
    filteredADSVoltage();
  float i =
    filteredCurrent();
  cal.zero_offset_voltage = v;
  cal.zero_offset_current = i;
  cal.calibrated = true;
  cal.last_cal = millis();
  EEPROM.put(
    CALIBRATION_ADDR,
    cal
  );
  EEPROM.commit();
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println(
    " SIFIR KAL. TAMAM"
  );
  display.printf(
    "V0: %.4fV\n",
    v
  );
  display.printf(
    "I0: %.4fA\n",
    i
  );
  display.display();
  digitalWrite(
    LED_CAL,
    HIGH
  );
  delay(2000);
  digitalWrite(
    LED_CAL,
    LOW
  );
  calibrating = false;
  showMain();
}
// =====================================================
// YÖN EKRANI
// =====================================================
void showDirection(
  int direction
) {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println(
    "=== YON ==="
  );
  display.println();
  display.printf(
    "%s\n",
    directionNames[direction]
  );
  display.println();
  display.println(
    "START = BASLA"
  );
  display.println(
    "STOP  = IPTAL"
  );
  display.display();
}
// =====================================================
// PROB DIRENCI UYARI EKRANI
// =====================================================
void showContactWarning(
  float r
) {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println(
    "PROB DIRENCI"
  );
  display.printf(
    "%.0f Ohm\n",
    r
  );
  if (r >= CONTACT_POOR_OHM) {
    display.println(
      "COK YUKSEK!"
    );
    display.println(
      "Veri guvenilmez"
    );
    display.println(
      "olabilir."
    );
  } else {
    display.println(
      "YUKSEK"
    );
    display.println(
      "Zemini nemlendirin"
    );
  }
  display.println(
    "Devam ediliyor..."
  );
  display.display();
}
// =====================================================
// WENNER SONUCU OLED
// =====================================================
void showWennerResult(
  int direction,
  int level,
  WennerResult r
) {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.printf(
    "%s\n",
    directionNames[direction]
  );
  display.printf(
    "WENNER %.0fm\n",
    r.spacing
  );
  display.printf(
    "rho: %.2f\n",
    r.rho
  );
  display.printf(
    "IP: %.2f%%\n",
    r.ip
  );
  display.printf(
    "Anom: %.0f%%\n",
    r.anomaly
  );
  display.printf(
    "Derinlik: ~%.1fm\n",
    estimatedDepth(
      r.spacing
    )
  );
  display.display();
}
// =====================================================
// SONUÇ EKRANI
// =====================================================
void showFinalResult() {
  int bestDirection =
    strongestDirection();
  int bestLevel =
    strongestLevel(
      bestDirection
    );
  float distance;
  float depth;
  float confidence;
  calculateTarget(
    bestDirection,
    distance,
    depth,
    confidence
  );
  if (
    !results[bestDirection][bestLevel].valid
  ) {
    Serial.println(
      "UYARI: secilen sonuc "
      "gecersiz olarak "
      "isaretlenmisti - guven "
      "duzeyi dusuk olabilir."
    );
  }
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println(
    "=== HEDEF ==="
  );
  display.printf(
    "Yon: %s\n",
    directionNames[
      bestDirection
    ]
  );
  display.printf(
    "Mesafe: ~%.1fm\n",
    distance
  );
  display.printf(
    "Derinlik: ~%.1fm\n",
    depth
  );
  display.printf(
    "Guven: %.0f%%\n",
    confidence
  );
  display.printf(
    "Seviye: %.0fm\n",
    wennerSpacing[
      bestLevel
    ]
  );
  display.println();
  display.println(
    "STOP = ANA MENU"
  );
  display.display();
  Serial.println();
  Serial.println(
    "================================"
  );
  Serial.println(
    "        HEDEF ANALIZI"
  );
  Serial.println(
    "================================"
  );
  Serial.printf(
    "YON       : %s\n",
    directionNames[
      bestDirection
    ]
  );
  Serial.printf(
    "MESAFE    : ~%.1f m\n",
    distance
  );
  Serial.printf(
    "DERINLIK  : ~%.1f m\n",
    depth
  );
  Serial.printf(
    "SEVIYE    : %.1f m\n",
    wennerSpacing[
      bestLevel
    ]
  );
  Serial.printf(
    "GUVEN     : %.1f %%\n",
    confidence
  );
  Serial.println(
    "================================"
  );
}
// =====================================================
// TÜM SONUÇLARI SERIAL
// =====================================================
void printAllResults() {
  Serial.println();
  Serial.println(
    "========================================"
  );
  Serial.println(
    "       WENNER COK SEVIYELI TARAMA"
  );
  Serial.printf(
    "       ORTAM: %s\n",
    (currentEnv == ENV_LAB) ? "LAB" : "SAHA"
  );
  Serial.println(
    "========================================"
  );
  for (
    int d = 0;
    d < DIRECTION_COUNT;
    d++
  ) {
    Serial.println();
    Serial.printf(
      "--- %s ---\n",
      directionNames[d]
    );
    for (
      int l = 0;
      l < WENNER_LEVELS;
      l++
    ) {
      WennerResult &r =
        results[d][l];
      Serial.printf(
        "a=%.1fm | rho=%.2f | IP=%.2f%% | "
        "Anom=%.1f%% | Derinlik~%.1fm | "
        "Gecerli=%s\n",
        r.spacing,
        r.rho,
        r.ip,
        r.anomaly,
        estimatedDepth(
          r.spacing
        ),
        r.valid ? "EVET" : "HAYIR"
      );
    }
    Serial.printf(
      "YON SKORU = %.1f%%\n",
      directionScore(d)
    );
  }
  Serial.println(
    "========================================"
  );
}
// =====================================================
// SD KAYIT (HATA KONTROLLÜ)
// =====================================================
void saveToSD() {
  if (!sdOK) {
    Serial.println("HATA: SD kart baslatilamadi!");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("SD HATA!");
    display.println("Kart baslatilamadi");
    display.display();
    delay(2000);
    return;
  }
  
  File f = SD.open("/wenner.csv", FILE_APPEND);
  if (!f) {
    Serial.println("HATA: /wenner.csv acilamadi!");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("SD HATA!");
    display.println("/wenner.csv");
    display.println("acilamadi!");
    display.display();
    digitalWrite(LED_ERROR, HIGH);
    delay(2000);
    digitalWrite(LED_ERROR, LOW);
    return;
  }
  
  // CSV başlığı yoksa ekle
  if (f.size() == 0) {
    int header_result = f.println("Tarih,Saat,Yon,Wenner,Rho,IP,RhoStd,IPStd,"
                                   "Anomali,Derinlik,ProbDirenci,Gecerli,Ortam");
    if (header_result == 0) {
      Serial.println("HATA: CSV baslik yazilemiyor!");
      f.close();
      digitalWrite(LED_ERROR, HIGH);
      delay(2000);
      digitalWrite(LED_ERROR, LOW);
      return;
    }
  }
  
  // Veri yaz
  int lines_written = 0;
  DateTime now = rtc.now();
  const char* envStr = (currentEnv == ENV_LAB) ? "LAB" : "SAHA";
  
  for (int d = 0; d < DIRECTION_COUNT; d++) {
    for (int l = 0; l < WENNER_LEVELS; l++) {
      WennerResult &r = results[d][l];
      int bytes_written = f.printf(
        "%04d-%02d-%02d,%02d:%02d:%02d,%s,%.1f,%.3f,%.3f,"
        "%.3f,%.3f,%.2f,%.2f,%.0f,%s,%s\n",
        now.year(), now.month(), now.day(),
        now.hour(), now.minute(), now.second(),
        directionNames[d], r.spacing, r.rho, r.ip,
        r.rhoStd, r.ipStd, r.anomaly,
        estimatedDepth(r.spacing), r.contactR,
        r.valid ? "EVET" : "HAYIR",
        envStr
      );
      
      if (bytes_written > 0) {
        lines_written++;
      } else {
        Serial.printf("HATA: Satir %d yazilemiyor!\n", lines_written + 1);
      }
    }
  }
  
  if (!f.close()) {
    Serial.println("HATA: Dosya kapatma basarısız!");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("SD HATA!");
    display.println("Dosya kapatma");
    display.println("basarısız!");
    display.display();
    digitalWrite(LED_ERROR, HIGH);
    delay(2000);
    digitalWrite(LED_ERROR, LOW);
  } else {
    Serial.printf("OK: SD'ye %d satir yazildi\n", lines_written);
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("SD BASARILI!");
    display.printf("%d satir\n", lines_written);
    display.println("kaydedildi");
    display.display();
    digitalWrite(LED_SD, HIGH);
    delay(1000);
    digitalWrite(LED_SD, LOW);
  }
}
// =====================================================
// TARAMA
// =====================================================
void startMeasurement() {
  if (measuring ||
      calibrating)
    return;
  if (
    batteryVoltage() <
    LOW_BATTERY_WARNING
  ) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println(
      " PIL DUSUK!"
    );
    display.printf(
      "%.2fV",
      batteryVoltage()
    );
    display.display();
    delay(2000);
    showMain();
    return;
  }
  if (!cal.calibrated) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println(
      "ONCE SIFIR"
    );
    display.println(
      "KALIBRASYON"
    );
    display.println();
    display.println(
      "CAL -> BASLAT"
    );
    display.display();
    delay(2000);
    return;
  }
  measuring = true;
  digitalWrite(
    LED_MEASURE,
    HIGH
  );
  for (
    int d = 0;
    d < DIRECTION_COUNT;
    d++
  ) {
    if (
      digitalRead(BUTTON_STOP)
      == LOW
    ) {
      stopMeasurement();
      return;
    }
    showDirection(d);
    if (
      !waitForStart()
    ) {
      stopMeasurement();
      return;
    }
    // -----------------------------
    // YÖN TEKRAR DONGUSU
    // -----------------------------
    int attempt = 0;
    bool directionOK = false;
    while (
      attempt <= MAX_DIRECTION_RETRY &&
      !directionOK
    ) {
      if (attempt > 0) {
        if (
          digitalRead(BUTTON_STOP)
          == LOW
        ) {
          stopMeasurement();
          return;
        }
        display.clearDisplay();
        display.setCursor(0, 0);
        display.println(
          "YON TEKRAR"
        );
        display.printf(
          "%s\n",
          directionNames[d]
        );
        display.printf(
          "Deneme %d/%d\n",
          attempt + 1,
          MAX_DIRECTION_RETRY + 1
        );
        display.println(
          "Tum seviyeler"
        );
        display.println(
          "gecersizdi."
        );
        display.display();
        delay(2000);
      }
      for (
        int level = 0;
        level < WENNER_LEVELS;
        level++
      ) {
        if (
          digitalRead(BUTTON_STOP)
          == LOW
        ) {
          stopMeasurement();
          return;
        }
        float spacing =
          wennerSpacing[level];
        // --------------------------------
        // PROB TEMAS KONTROLU (ÇOKLU FREKANS)
        // --------------------------------
        float contactR =
          checkContactResistance();
        if (contactR > CONTACT_WARN_OHM) {
          showContactWarning(
            contactR
          );
          delay(2500);
        }
        display.clearDisplay();
        display.setCursor(0, 0);
        display.println(
          "WENNER OLCUM"
        );
        display.printf(
          "Yon: %s\n",
          directionNames[d]
        );
        display.printf(
          "Seviye: %d/%d\n",
          level + 1,
          WENNER_LEVELS
        );
        display.printf(
          "Mesafe: %.0fm\n",
          spacing
        );
        display.println();
        display.println(
          "Olculuyor..."
        );
        display.display();
        WennerResult r =
          measureWenner(
            spacing
          );
        r.anomaly =
          calculateAnomaly(
            r.rho,
            r.ip
          );
        r.contactR = contactR;
        results[d][level] = r;
        showWennerResult(
          d,
          level,
          r
        );
        Serial.printf(
          "%s | %.1fm | "
          "rho=%.3f | IP=%.3f | "
          "Anomali=%.1f%% | "
          "ProbR=%.0f Ohm | "
          "Gecerli=%s\n",
          directionNames[d],
          spacing,
          r.rho,
          r.ip,
          r.anomaly,
          r.contactR,
          r.valid ? "EVET" : "HAYIR"
        );
        delay(1500);
      }
      // --------------------------------
      // BU YONDE EN AZ BIR SEVIYE GECERLI MI?
      // --------------------------------
      directionOK = false;
      for (
        int level = 0;
        level < WENNER_LEVELS;
        level++
      ) {
        if (results[d][level].valid) {
          directionOK = true;
          break;
        }
      }
      attempt++;
    }
    if (!directionOK) {
      Serial.printf(
        "UYARI: %s yonunde "
        "%d denemeden sonra "
        "hicbir seviye gecerli "
        "olmadi.\n",
        directionNames[d],
        MAX_DIRECTION_RETRY + 1
      );
      display.clearDisplay();
      display.setCursor(0, 0);
      display.println(
        "UYARI:"
      );
      display.printf(
        "%s yonu\n",
        directionNames[d]
      );
      display.println(
        "guvenilir veri"
      );
      display.println(
        "alinamadi."
      );
      display.display();
      delay(2000);
    }
  }
  printAllResults();
  saveToSD();
  showFinalResult();
  measuring = false;
  digitalWrite(
    LED_MEASURE,
    LOW
  );
  while (
    digitalRead(BUTTON_STOP)
    != LOW
  ) {
    delay(10);
  }
  delay(50);
  while (
    digitalRead(BUTTON_STOP)
    == LOW
  ) {
    delay(10);
  }
  showMain();
}
// =====================================================
// STOP
// =====================================================
void stopMeasurement() {
  inj(false);
  measuring = false;
  digitalWrite(
    LED_MEASURE,
    LOW
  );
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println(
    "OLCUM IPTAL"
  );
  display.display();
  delay(1000);
  showMain();
}
// =====================================================
// START BEKLE
// =====================================================
bool waitForStart() {
  while (true) {
    if (
      digitalRead(
        BUTTON_START
      ) == LOW
    ) {
      delay(50);
      if (
        digitalRead(
          BUTTON_START
        ) == LOW
      ) {
        while (
          digitalRead(
            BUTTON_START
          ) == LOW
        ) {
          delay(10);
        }
        return true;
      }
    }
    if (
      digitalRead(
        BUTTON_STOP
      ) == LOW
    ) {
      delay(50);
      if (
        digitalRead(
          BUTTON_STOP
        ) == LOW
      ) {
        while (
          digitalRead(
            BUTTON_STOP
          ) == LOW
        ) {
          delay(10);
        }
        return false;
      }
    }
    delay(10);
  }
}
// =====================================================
// EEPROM YÜKLE
// =====================================================
void loadCalibration() {
  EEPROM.get(
    CALIBRATION_ADDR,
    cal
  );
  if (
    isnan(
      cal.zero_offset_voltage
    ) ||
    isnan(
      cal.zero_offset_current
    )
  ) {
    cal.zero_offset_voltage = 0;
    cal.zero_offset_current = 0;
    cal.calibrated = false;
    cal.last_cal = 0;
  }
}
// =====================================================
// SETUP
// =====================================================
void setup() {
  Serial.begin(115200);
  pinMode(
    CURRENT_CTRL,
    OUTPUT
  );
  digitalWrite(
    CURRENT_CTRL,
    LOW
  );
  pinMode(
    BUTTON_START,
    INPUT_PULLUP
  );
  pinMode(
    BUTTON_STOP,
    INPUT_PULLUP
  );
  pinMode(
    BUTTON_CAL,
    INPUT_PULLUP
  );
  pinMode(
    LED_MEASURE,
    OUTPUT
  );
  pinMode(
    LED_ERROR,
    OUTPUT
  );
  pinMode(
    LED_FAULT,
    OUTPUT
  );
  pinMode(
    LED_SD,
    OUTPUT
  );
  pinMode(
    LED_CAL,
    OUTPUT
  );
  analogReadResolution(12);
  Wire.begin(
    I2C_SDA,
    I2C_SCL
  );
  if (
    display.begin(
      SSD1306_SWITCHCAPVCC,
      OLED_ADDR
    )
  ) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(
      SSD1306_WHITE
    );
    display.setCursor(0, 0);
    display.println(
      "MADEN ARAMA"
    );
    display.println(
      VERSION
    );
    display.display();
  } else {
    Serial.println(
      "OLED HATA!"
    );
  }
  if (
    !ads1115.begin()
  ) {
    Serial.println(
      "ADS1115 HATA!"
    );
    digitalWrite(
      LED_ERROR,
      HIGH
    );
  }
  if (
    !ina219.begin()
  ) {
    Serial.println(
      "INA219 HATA!"
    );
    digitalWrite(
      LED_ERROR,
      HIGH
    );
  }
  if (
    !rtc.begin()
  ) {
    Serial.println(
      "DS3231 HATA!"
    );
  } else {
    if (
      rtc.lostPower()
    ) {
      rtc.adjust(
        DateTime(
          F(__DATE__),
          F(__TIME__)
        )
      );
    }
  }
  EEPROM.begin(
    EEPROM_SIZE
  );
  loadCalibration();
  SPI.begin(
    SD_SCK,
    SD_MISO,
    SD_MOSI,
    SD_CS
  );
  if (
    SD.begin(SD_CS)
  ) {
    sdOK = true;
    digitalWrite(
      LED_SD,
      HIGH
    );
    Serial.println(
      "SD OK"
    );
  } else {
    sdOK = false;
    Serial.println(
      "SD HATA!"
    );
  }
  Serial.println();
  Serial.println(
    "======================================"
  );
  Serial.println(
    " MADEN ARAMA SISTEMI"
  );
  Serial.println(
    VERSION
  );
  Serial.println(
    "======================================"
  );
  Serial.println(
    "ARRAY : WENNER"
  );
  Serial.println(
    "NOKTA : 5"
  );
  Serial.println(
    "SEVIYE: 5m / 10m / 20m (sabit)"
  );
  Serial.println(
    "TEMAS KONTROLU: ÇOKLU FREKANS (100/500/1500ms)"
  );
  Serial.println(
    "KALMAN FILTRE: DINAMIK UYARLANABILIR"
  );
  Serial.printf(
    "PROB DIRENC KONTROLU: VAR (uyari>%.0fOhm)\n",
    CONTACT_WARN_OHM
  );
  Serial.println(
    "======================================"
  );
  
  // Ortam modu seçimi
  selectEnvironment();
}
// =====================================================
// LOOP
// =====================================================
void loop() {
  if (
    digitalRead(BUTTON_START)
    == LOW
    &&
    !measuring
    &&
    !calibrating
  ) {
    delay(50);
    if (
      digitalRead(BUTTON_START)
      == LOW
    ) {
      startMeasurement();
    }
  }
  if (
    digitalRead(BUTTON_STOP)
    == LOW
    &&
    measuring
  ) {
    delay(50);
    if (
      digitalRead(BUTTON_STOP)
      == LOW
    ) {
      stopMeasurement();
    }
  }
  if (
    digitalRead(BUTTON_CAL)
    == LOW
    &&
    !measuring
    &&
    !calibrating
  ) {
    delay(50);
    if (
      digitalRead(BUTTON_CAL)
      == LOW
    ) {
      while (
        digitalRead(BUTTON_CAL)
        == LOW
      ) {
        delay(10);
      }
      zeroCalibration();
    }
  }
  delay(10);
}
