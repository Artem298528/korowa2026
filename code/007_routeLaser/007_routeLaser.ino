/*
  ============================================================
  РОБОТ: 2 ШАГОВЫХ ДВИГАТЕЛЯ + 2 VL53L0X + 3 СЕРВЫ + 2 КНОПКИ
  ============================================================

  ПОДКЛЮЧЕНИЕ:

  1. Шаговые двигатели

  Мотор 1:
    STEP -> GPIO 25
    DIR  -> GPIO 33
    EN   -> GPIO 14

  Мотор 2:
    STEP -> GPIO 27
    DIR  -> GPIO 26
    EN   -> GPIO 32

  ВАЖНО:
  В этой версии моторы НЕ включаются при загрузке ESP32.
  Они включаются только тогда, когда маршрут реально дошёл до шага MOVE_STEPPERS.

  ------------------------------------------------------------

  2. Дальномеры VL53L0X

  SDA -> GPIO 21
  SCL -> GPIO 22

  XSHUT левого датчика  -> GPIO 19
  XSHUT правого датчика -> GPIO 18

  Левый датчик получает адрес 0x30.
  Правый остаётся на стандартном адресе 0x29.

  ------------------------------------------------------------

  3. Сервы

  Серва 0 -> GPIO 12
  Серва 1 -> GPIO 16
  Серва 2 -> GPIO 17

  ВАЖНО:
  GPIO 12 — загрузочный пин ESP32.
  Если ESP32 плохо стартует с подключённой сервой, лучше перенести эту серву.

  Сервы желательно питать от отдельного 5V источника.
  GND серв, ESP32 и драйверов моторов должен быть общим.

  ------------------------------------------------------------

  4. Кнопки

  STARTER:
    GPIO 23 -> кнопка -> GND

  SIDE:
    GPIO 13 -> кнопка -> GND

  Используется INPUT_PULLUP:
    отпущена -> HIGH
    нажата   -> LOW

  ------------------------------------------------------------

  ЛОГИКА РАБОТЫ:

  1. ESP32 включается.
  2. Моторы НЕ включаются.
  3. Инициализируются датчики и выбирается маршрут кнопкой SIDE.
  4. Робот ждёт кнопку STARTER.
  5. После старта начинается отсчёт 98 секунд.
  6. На первом шаге MOVE_STEPPERS включаются драйверы моторов.
  7. Если дальномер видит препятствие ближе 200 мм — робот ждёт.
  8. В некоторых шагах защиту дальномерами можно отключить.
  9. После завершения маршрута или после 98 секунд:
     - шаговые двигатели отключаются;
     - колёса больше не едут;
     - две сервы начинают плавно двигаться в цикле.
*/

#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <GyverStepper.h>
#include <ESP32Servo.h>

// ======================================================
// ПИНЫ КНОПОК
// ======================================================

const int STARTER_PIN = 23;
const int SIDE_PIN    = 13;

// ======================================================
// ПИНЫ VL53L0X
// ======================================================

#define SDA_PIN 21
#define SCL_PIN 22

#define XSHUT_LEFT  19
#define XSHUT_RIGHT 18

#define VL53_ADDR_LEFT  0x30
#define VL53_ADDR_RIGHT 0x29

// ======================================================
// ШАГОВЫЕ ДВИГАТЕЛИ
// ======================================================

GStepper<STEPPER2WIRE> stepper1(800, 25, 33, 14);
GStepper<STEPPER2WIRE> stepper2(800, 27, 26, 32);

bool steppersEnabled = false;

// ======================================================
// СЕРВЫ
// ======================================================

#define SERVO_UNUSED -1

const int SERVO_COUNT = 3;

const int SERVO_PINS[SERVO_COUNT] = {
  12,   // servo 0
  16,   // servo 1
  17    // servo 2
};

Servo servos[SERVO_COUNT];

const int SERVO_MIN_US = 500;
const int SERVO_MAX_US = 2500;

// ======================================================
// ОСНОВНЫЕ НАСТРОЙКИ
// ======================================================

const unsigned long MAX_RUN_TIME_MS = 93000;
const unsigned long VL53_INTERVAL_MS = 100;

const uint16_t OBSTACLE_LIMIT_MM = 200;

// ======================================================
// VL53L0X
// ======================================================

VL53L0X sensorLeft;
VL53L0X sensorRight;

uint16_t distLeft = 0;
uint16_t distRight = 0;

bool vl53LeftOk = false;
bool vl53RightOk = false;
bool vl53NewData = false;

unsigned long lastVL53Update = 0;

// ======================================================
// ТИПЫ ШАГОВ МАРШРУТА
// ======================================================

enum StepType {
  MOVE_STEPPERS,
  SERVO_MOVE,
  SERVO_MOVE_2,
  PAUSE_STEP,
  END_ROUTE
};

// ======================================================
// СТРУКТУРА ШАГА МАРШРУТА
// ======================================================

struct RouteStep {
  StepType type;

  long motor1Steps;
  long motor2Steps;

  int servoIndex1;
  int servoAngle1;

  int servoIndex2;
  int servoAngle2;

  unsigned long timeMs;

  bool useDistanceSafety;
};

/*
  Формат:

  {ТИП, motor1, motor2, servo1, angle1, servo2, angle2, timeMs, safety}

  Движение:
  {MOVE_STEPPERS, 1000, 1000, -1, 0, -1, 0, 0, true}

  Одна серва:
  {SERVO_MOVE, 0, 0, 0, 90, -1, 0, 700, false}

  Две сервы одновременно:
  {SERVO_MOVE_2, 0, 0, 0, 45, 1, 135, 700, false}

  Пауза:
  {PAUSE_STEP, 0, 0, -1, 0, -1, 0, 1000, false}

  Конец:
  {END_ROUTE, 0, 0, -1, 0, -1, 0, 0, false}

  safety:
    true  — дальномеры контролируют препятствия;
    false — защита отключена для этого шага.
*/

// ======================================================
// МАРШРУТ 1
// ======================================================

RouteStep route1[] = {
  {SERVO_MOVE_2, 0, 0, 1, 0, 2, 180, 800, false},
  {MOVE_STEPPERS, 1000, 1000, -1, 0, -1, 0, 0, true},

  {PAUSE_STEP, 0, 0, -1, 0, -1, 0, 1000, false},

  // Две сервы одновременно
  {SERVO_MOVE_2, 0, 0, 0, 40, 1, 140, 800, false},

  // Подъезд к предмету — защита отключена
  {MOVE_STEPPERS, 500, 500, -1, 0, -1, 0, 0, false},

  {SERVO_MOVE_2, 0, 0, 0, 100, 1, 80, 800, false},

  {MOVE_STEPPERS, -500, -500, -1, 0, -1, 0, 0, true},

  {MOVE_STEPPERS, -800, 800, -1, 0, -1, 0, 0, true},

  {END_ROUTE, 0, 0, -1, 0, -1, 0, 0, false}
};

// ======================================================
// МАРШРУТ 2
// ======================================================

RouteStep route2[] = {
  {MOVE_STEPPERS, 1500, 1500, -1, 0, -1, 0, 0, true},

  {SERVO_MOVE, 0, 0, 2, 45, -1, 0, 700, false},

  {SERVO_MOVE_2, 0, 0, 0, 20, 1, 160, 900, false},

  {PAUSE_STEP, 0, 0, -1, 0, -1, 0, 1000, false},

  // Участок без контроля дальномерами
  {MOVE_STEPPERS, 700, 700, -1, 0, -1, 0, 0, false},

  {MOVE_STEPPERS, 800, -800, -1, 0, -1, 0, 0, true},

  {END_ROUTE, 0, 0, -1, 0, -1, 0, 0, false}
};

// ======================================================
// ПЕРЕМЕННЫЕ МАРШРУТА
// ======================================================

RouteStep* currentRoute = nullptr;

int currentStepIndex = 0;

bool stepStarted = false;
bool motorsWereMoving = false;

bool routeFinished = false;
bool timeoutFinished = false;

unsigned long stepStartTime = 0;
unsigned long robotStartTime = 0;

// ======================================================
// ИНИЦИАЛИЗАЦИЯ ДАЛЬНОМЕРОВ
// ======================================================

void initVL53() {
  pinMode(XSHUT_LEFT, OUTPUT);
  pinMode(XSHUT_RIGHT, OUTPUT);

  digitalWrite(XSHUT_LEFT, LOW);
  digitalWrite(XSHUT_RIGHT, LOW);
  delay(50);

  Wire.begin(SDA_PIN, SCL_PIN);
  delay(10);

  digitalWrite(XSHUT_LEFT, HIGH);
  delay(50);

  sensorLeft.setTimeout(100);

  if (sensorLeft.init()) {
    sensorLeft.setAddress(VL53_ADDR_LEFT);
    sensorLeft.setMeasurementTimingBudget(30000);
    sensorLeft.startContinuous();

    vl53LeftOk = true;
    Serial.println("VL53 LEFT init OK, addr = 0x30");
  } else {
    vl53LeftOk = false;
    Serial.println("VL53 LEFT init FAIL");
  }

  digitalWrite(XSHUT_RIGHT, HIGH);
  delay(50);

  sensorRight.setTimeout(100);

  if (sensorRight.init()) {
    sensorRight.setMeasurementTimingBudget(30000);
    sensorRight.startContinuous();

    vl53RightOk = true;
    Serial.println("VL53 RIGHT init OK, addr = 0x29");
  } else {
    vl53RightOk = false;
    Serial.println("VL53 RIGHT init FAIL");
  }
}

// ======================================================
// ОБНОВЛЕНИЕ ДАЛЬНОМЕРОВ
// ======================================================

void updateVL53() {
  unsigned long now = millis();

  if (now - lastVL53Update < VL53_INTERVAL_MS) {
    return;
  }

  lastVL53Update = now;
  vl53NewData = false;

  if (vl53LeftOk) {
    uint16_t value = sensorLeft.readRangeContinuousMillimeters();

    if (!sensorLeft.timeoutOccurred()) {
      distLeft = value;
    }
  }

  if (vl53RightOk) {
    uint16_t value = sensorRight.readRangeContinuousMillimeters();

    if (!sensorRight.timeoutOccurred()) {
      distRight = value;
    }
  }

  vl53NewData = true;
}

// ======================================================
// ПРОВЕРКА ПРЕПЯТСТВИЯ
// ======================================================

bool isObstacleDetected() {
  bool leftBlocked = false;
  bool rightBlocked = false;

  if (vl53LeftOk && distLeft > 0 && distLeft < OBSTACLE_LIMIT_MM) {
    leftBlocked = true;
  }

  if (vl53RightOk && distRight > 0 && distRight < OBSTACLE_LIMIT_MM) {
    rightBlocked = true;
  }

  return leftBlocked || rightBlocked;
}

// ======================================================
// ПЕЧАТЬ ДАЛЬНОМЕРОВ
// ======================================================

void printDistances() {
  Serial.print("VL53 LEFT: ");

  if (vl53LeftOk) Serial.print(distLeft);
  else Serial.print("ERR");

  Serial.print(" mm | RIGHT: ");

  if (vl53RightOk) Serial.print(distRight);
  else Serial.print("ERR");

  Serial.println(" mm");
}

// ======================================================
// НАСТРОЙКА ШАГОВЫХ ДВИГАТЕЛЕЙ
// ======================================================

void setupSteppers() {
  stepper1.setRunMode(FOLLOW_POS);
  stepper2.setRunMode(FOLLOW_POS);

  stepper1.setMaxSpeed(500);
  stepper1.setAcceleration(500);

  stepper2.setMaxSpeed(500);
  stepper2.setAcceleration(500);

  stepper1.autoPower(true);
  stepper2.autoPower(true);

  stepper1.reverse(false);
  stepper2.reverse(true);

  // ВАЖНО:
  // Здесь НЕ вызываем enable().
  // Моторы не включаются при загрузке ESP32.

  stepper1.disable();
  stepper2.disable();

  steppersEnabled = false;

  Serial.println("STEPPERS CONFIGURED, BUT DISABLED");
}

// ======================================================
// ВКЛЮЧИТЬ ШАГОВЫЕ ТОЛЬКО ПРИ ДВИЖЕНИИ
// ======================================================

void enableSteppersIfNeeded() {
  if (!steppersEnabled) {
    stepper1.enable();
    stepper2.enable();

    steppersEnabled = true;

    Serial.println("STEPPERS ENABLED");
  }
}

// ======================================================
// ОТКЛЮЧИТЬ ШАГОВЫЕ
// ======================================================

void disableSteppers() {
  stepper1.disable();
  stepper2.disable();

  steppersEnabled = false;

  Serial.println("STEPPERS DISABLED");
}

// ======================================================
// ПРОВЕРКА СЕРВЫ
// ======================================================

bool isServoAvailable(int index) {
  if (index < 0) return false;
  if (index >= SERVO_COUNT) return false;
  if (SERVO_PINS[index] == SERVO_UNUSED) return false;

  return true;
}

// ======================================================
// ПОВЕРНУТЬ ОДНУ СЕРВУ
// ======================================================

void attachAndWriteServo(int index, int angle) {
  if (!isServoAvailable(index)) {
    Serial.print("SERVO ");
    Serial.print(index);
    Serial.println(" is unused. Skip.");
    return;
  }

  angle = constrain(angle, 0, 180);

  servos[index].attach(SERVO_PINS[index], SERVO_MIN_US, SERVO_MAX_US);
  servos[index].write(angle);

  Serial.print("SERVO ");
  Serial.print(index);
  Serial.print(" GPIO ");
  Serial.print(SERVO_PINS[index]);
  Serial.print(" -> ");
  Serial.println(angle);
}

// ======================================================
// ПОВЕРНУТЬ ДВЕ СЕРВЫ ОДНОВРЕМЕННО
// ======================================================

void attachAndWriteTwoServos(int index1, int angle1, int index2, int angle2) {
  attachAndWriteServo(index1, angle1);
  attachAndWriteServo(index2, angle2);
}

// ======================================================
// ОТКЛЮЧИТЬ ВСЕ СЕРВЫ
// ======================================================

void detachAllServos() {
  for (int i = 0; i < SERVO_COUNT; i++) {
    if (isServoAvailable(i)) {
      servos[i].detach();
    }
  }

  Serial.println("ALL SERVOS DETACHED");
}

// ======================================================
// ФИНАЛЬНОЕ ДВИЖЕНИЕ ДВУХ СЕРВ
// ======================================================

void finalServoLoop() {
  const int finalServoA = 0;
  const int finalServoB = 1;

  if (!isServoAvailable(finalServoA) || !isServoAvailable(finalServoB)) {
    delay(1000);
    return;
  }

  servos[finalServoA].attach(SERVO_PINS[finalServoA], SERVO_MIN_US, SERVO_MAX_US);
  servos[finalServoB].attach(SERVO_PINS[finalServoB], SERVO_MIN_US, SERVO_MAX_US);

  for (int angle = 0; angle <= 180; angle++) {
    servos[finalServoA].write(angle);
    servos[finalServoB].write(180 - angle);
    delay(15);
  }

  for (int angle = 180; angle >= 0; angle--) {
    servos[finalServoA].write(angle);
    servos[finalServoB].write(180 - angle);
    delay(15);
  }
}

// ======================================================
// ЗАПУСК ШАГА МАРШРУТА
// ======================================================

void startRouteStep(RouteStep step) {
  stepStartTime = millis();

  if (step.type == MOVE_STEPPERS) {
    motorsWereMoving = false;

    Serial.print("MOVE: ");
    Serial.print(step.motor1Steps);
    Serial.print(" / ");
    Serial.println(step.motor2Steps);

    // ВАЖНО:
    // Шаговые включаются только здесь,
    // то есть только когда реально начинается движение.
    enableSteppersIfNeeded();

    stepper1.setTarget(step.motor1Steps, RELATIVE);
    stepper2.setTarget(step.motor2Steps, RELATIVE);
  }

  else if (step.type == SERVO_MOVE) {
    Serial.println("SERVO_MOVE");

    attachAndWriteServo(step.servoIndex1, step.servoAngle1);
  }

  else if (step.type == SERVO_MOVE_2) {
    Serial.println("SERVO_MOVE_2");

    attachAndWriteTwoServos(
      step.servoIndex1,
      step.servoAngle1,
      step.servoIndex2,
      step.servoAngle2
    );
  }

  else if (step.type == PAUSE_STEP) {
    Serial.print("PAUSE: ");
    Serial.print(step.timeMs);
    Serial.println(" ms");
  }

  stepStarted = true;
}

// ======================================================
// ПРОВЕРКА ЗАВЕРШЕНИЯ ШАГА
// ======================================================

bool isRouteStepFinished(RouteStep step, byte stepper1Move, byte stepper2Move) {
  if (step.type == MOVE_STEPPERS) {
    if (stepper1Move != 0 || stepper2Move != 0) {
      motorsWereMoving = true;
    }

    return motorsWereMoving && stepper1Move == 0 && stepper2Move == 0;
  }

  if (step.type == SERVO_MOVE || step.type == SERVO_MOVE_2) {
    if (millis() - stepStartTime >= step.timeMs) {
      detachAllServos();
      return true;
    }

    return false;
  }

  if (step.type == PAUSE_STEP) {
    return millis() - stepStartTime >= step.timeMs;
  }

  return false;
}

// ======================================================
// SETUP
// ======================================================

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println("========================================");
  Serial.println("ROBOT PROGRAM START");
  Serial.println("STEPPERS ENABLE ONLY ON MOVE STEP");
  Serial.println("========================================");

  pinMode(STARTER_PIN, INPUT_PULLUP);
  pinMode(SIDE_PIN, INPUT_PULLUP);

  setupSteppers();
  initVL53();

  Serial.println();
  Serial.println("Servo configuration:");

  for (int i = 0; i < SERVO_COUNT; i++) {
    Serial.print("Servo ");
    Serial.print(i);
    Serial.print(": ");

    if (SERVO_PINS[i] == SERVO_UNUSED) {
      Serial.println("UNUSED");
    } else {
      Serial.print("GPIO ");
      Serial.println(SERVO_PINS[i]);
    }
  }

  Serial.println();

  if (digitalRead(SIDE_PIN) == LOW) {
    currentRoute = route2;
    Serial.println("Selected route: 2");
  } else {
    currentRoute = route1;
    Serial.println("Selected route: 1");
  }

  Serial.println("Waiting STARTER press...");

  while (digitalRead(STARTER_PIN) == HIGH) {
    updateVL53();

    if (vl53NewData) {
      vl53NewData = false;
      printDistances();
    }

    delay(20);
  }

  delay(100);

  robotStartTime = millis();

  Serial.println("START!");
}

// ======================================================
// LOOP
// ======================================================

void loop() {
  updateVL53();

  if (vl53NewData) {
    vl53NewData = false;
    printDistances();
  }

  // ------------------------------------------------------
  // Контроль 98 секунд
  // ------------------------------------------------------

  if (!routeFinished && !timeoutFinished) {
    if (millis() - robotStartTime >= MAX_RUN_TIME_MS) {
      timeoutFinished = true;
      routeFinished = true;

      disableSteppers();
      detachAllServos();

      Serial.println("TIMEOUT 98 SEC. ROUTE STOPPED.");
    }
  }

  // ------------------------------------------------------
  // После финиша работают только сервы
  // ------------------------------------------------------

  if (routeFinished) {
    finalServoLoop();
    return;
  }

  RouteStep currentStep = currentRoute[currentStepIndex];

  // ------------------------------------------------------
  // Конец маршрута
  // ------------------------------------------------------

  if (currentStep.type == END_ROUTE) {
    routeFinished = true;

    disableSteppers();
    detachAllServos();

    Serial.println("ROUTE FINISHED. FINAL SERVO LOOP STARTED.");
    return;
  }

  // ------------------------------------------------------
  // Проверка препятствия
  // ------------------------------------------------------

  bool blocked = currentStep.useDistanceSafety && isObstacleDetected();

  static bool wasBlocked = false;

  if (blocked) {
    if (!wasBlocked) {
      Serial.println("OBSTACLE DETECTED. WAITING...");
      printDistances();
      wasBlocked = true;
    }

    // tick() не вызываем — моторы стоят на паузе.
    delay(5);
    return;
  }

  if (wasBlocked) {
    Serial.println("OBSTACLE CLEAR. CONTINUE.");
    printDistances();
    wasBlocked = false;
  }

  // ------------------------------------------------------
  // Обслуживание шаговых
  // ------------------------------------------------------

  byte stepper1Move = stepper1.tick();
  byte stepper2Move = stepper2.tick();

  // ------------------------------------------------------
  // Запуск нового шага
  // ------------------------------------------------------

  if (!stepStarted) {
    startRouteStep(currentStep);
  }

  // ------------------------------------------------------
  // Проверка завершения шага
  // ------------------------------------------------------

  if (isRouteStepFinished(currentStep, stepper1Move, stepper2Move)) {
    Serial.print("Step finished: ");
    Serial.println(currentStepIndex);

    currentStepIndex++;
    stepStarted = false;
    motorsWereMoving = false;

    delay(50);
  }
}