#include <Wire.h>
#include <VL53L0X.h>

// =========================
// ПИНЫ ESP32
// =========================
#define SDA_PIN 21
#define SCL_PIN 22

#define XSHUT_LEFT  19
#define XSHUT_RIGHT 18

// =========================
// АДРЕСА ДАТЧИКОВ
// =========================
#define VL53_ADDR_LEFT  0x30   // левому задаём новый адрес
#define VL53_ADDR_RIGHT 0x29   // правый остаётся стандартным

// =========================
// ОБЪЕКТЫ ДАТЧИКОВ
// =========================
VL53L0X sensorLeft;
VL53L0X sensorRight;

// =========================
// ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ
// =========================
uint16_t distLeft = 0;
uint16_t distRight = 0;

bool vl53LeftOk = false;
bool vl53RightOk = false;
bool vl53NewData = false;

// =========================
// НАСТРОЙКИ ОПРОСА
// =========================
unsigned long lastVL53Update = 0;
const unsigned long VL53_INTERVAL_MS = 100;

// =========================
// ИНИЦИАЛИЗАЦИЯ ДАТЧИКОВ
// =========================
void initVL53()
{
  pinMode(XSHUT_LEFT, OUTPUT);
  pinMode(XSHUT_RIGHT, OUTPUT);

  // Сначала выключаем оба датчика
  digitalWrite(XSHUT_LEFT, LOW);
  digitalWrite(XSHUT_RIGHT, LOW);
  delay(50);

  Wire.begin(SDA_PIN, SCL_PIN);
  delay(10);

  // -------------------------
  // ЛЕВЫЙ ДАТЧИК
  // -------------------------
  digitalWrite(XSHUT_LEFT, HIGH);
  delay(50);

  sensorLeft.setTimeout(100);

  if (sensorLeft.init())
  {
    sensorLeft.setAddress(VL53_ADDR_LEFT);
    sensorLeft.setMeasurementTimingBudget(30000);
    sensorLeft.startContinuous();
    vl53LeftOk = true;
    Serial.println("VL53 LEFT init OK, addr = 0x30");
  }
  else
  {
    vl53LeftOk = false;
    Serial.println("VL53 LEFT init FAIL");
  }

  // -------------------------
  // ПРАВЫЙ ДАТЧИК
  // -------------------------
  digitalWrite(XSHUT_RIGHT, HIGH);
  delay(50);

  sensorRight.setTimeout(100);

  if (sensorRight.init())
  {
    // Правый остаётся на стандартном адресе 0x29
    sensorRight.setMeasurementTimingBudget(30000);
    sensorRight.startContinuous();
    vl53RightOk = true;
    Serial.println("VL53 RIGHT init OK, addr = 0x29");
  }
  else
  {
    vl53RightOk = false;
    Serial.println("VL53 RIGHT init FAIL");
  }
}

// =========================
// ФОНОВОЕ ОБНОВЛЕНИЕ ДАТЧИКОВ
// Вызывать часто из loop()
// =========================
void updateVL53()
{
  unsigned long now = millis();

  if (now - lastVL53Update < VL53_INTERVAL_MS)
    return;

  lastVL53Update = now;
  vl53NewData = false;

  if (vl53LeftOk)
  {
    uint16_t value = sensorLeft.readRangeContinuousMillimeters();

    if (!sensorLeft.timeoutOccurred())
    {
      distLeft = value;
    }
  }

  if (vl53RightOk)
  {
    uint16_t value = sensorRight.readRangeContinuousMillimeters();

    if (!sensorRight.timeoutOccurred())
    {
      distRight = value;
    }
  }

  vl53NewData = true;
}

// =========================
// SETUP
// =========================
void setup()
{
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("Start VL53L0X x2");

  initVL53();
}

// =========================
// LOOP
// =========================
void loop()
{
  updateVL53();

  // Пример использования:
  // печатаем только когда пришли новые данные
  if (vl53NewData)
  {
    vl53NewData = false;

    Serial.print("Left: ");
    if (vl53LeftOk) Serial.print(distLeft);
    else Serial.print("ERR");

    Serial.print(" mm | Right: ");
    if (vl53RightOk) Serial.print(distRight);
    else Serial.print("ERR");

    Serial.println(" mm");
  }

  // Здесь может быть любой остальной код робота
}