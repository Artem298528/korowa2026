#include <Arduino.h>
#include <GyverStepper.h>
#include <ESP32Servo.h>

// ===================== ПИНЫ =====================

const int ROUTE_PIN = 26;
const int START_PIN = 27;

// До 6 серв
#define SERVO_UNUSED -1

const int SERVO_PINS[6] = {
  33,            // servo 0 подключена
  25,            // servo 1 подключена
  SERVO_UNUSED,            // servo 2 подключена
  SERVO_UNUSED,  // servo 3 не используется
  SERVO_UNUSED,  // servo 4 не используется
  SERVO_UNUSED   // servo 5 не используется
};

const int SERVO_COUNT = 6;

// Шаговые двигатели
GStepper<STEPPER2WIRE> stepper1(800, 19, 18, 17);
GStepper<STEPPER2WIRE> stepper2(800, 23, 22, 21);

// Сервы
Servo servos[SERVO_COUNT];

// ===================== ТИПЫ ШАГОВ =====================

enum StepType {
  MOVE_STEPPERS,
  SERVO_MOVE,
  PAUSE_STEP,
  END_ROUTE
};

struct RouteStep {
  StepType type;
  long motor1Steps;
  long motor2Steps;
  int servoIndex;
  int servoAngle;
  unsigned long timeMs;
};

// ===================== МАРШРУТЫ =====================

// Формат:
// {ТИП, motor1, motor2, servoIndex, servoAngle, timeMs}

// servoIndex:
// 0..5 — номер сервы
// -1   — серва не используется

RouteStep route1[] = {
  {MOVE_STEPPERS, 1000, 1000, -1,  0, 0},
  {PAUSE_STEP,    0,    0,    -1,  0, 1000},

  {SERVO_MOVE,    0,    0,     0,  0, 500},   // серва 1 на 0°
  {PAUSE_STEP,    0,    0,    -1,  0, 2000},
  {SERVO_MOVE,    0,    0,     0, 35, 500},   // серва 1 на 35°

  {MOVE_STEPPERS, -800, 800,  -1,  0, 0},
  {END_ROUTE,     0,    0,    -1,  0, 0}
};

RouteStep route2[] = {
  {MOVE_STEPPERS, 1500, 1500, -1,  0, 0},

  {SERVO_MOVE,    0,    0,     1, 15, 500},   // серва 2 на 15°
  {PAUSE_STEP,    0,    0,    -1,  0, 2000},
  {SERVO_MOVE,    0,    0,     1,  0, 500},   // серва 2 на 0°

  // Пример шага с незадействованной сервой:
  // Серва 4 в массиве SERVO_PINS = SERVO_UNUSED,
  // поэтому команда будет пропущена безопасно.
  {SERVO_MOVE,    0,    0,     4, 90, 500},

  {PAUSE_STEP,    0,    0,    -1,  0, 2000},
  {MOVE_STEPPERS, 800, -800,  -1,  0, 0},
  {END_ROUTE,     0,    0,    -1,  0, 0}
};

// ===================== ПЕРЕМЕННЫЕ =====================

RouteStep* currentRoute;

int currentStepIndex = 0;

bool stepStarted = false;
bool motorsWereMoving = false;

unsigned long stepStartTime = 0;

int activeServoIndex = -1;

// ===================== НАСТРОЙКА МОТОРОВ =====================

void setupSteppers() {
  stepper1.setRunMode(FOLLOW_POS);
  stepper2.setRunMode(FOLLOW_POS);

  stepper1.setMaxSpeed(200);
  stepper1.setAcceleration(300);

  stepper2.setMaxSpeed(200);
  stepper2.setAcceleration(300);

  stepper1.autoPower(true);
  stepper2.autoPower(true);

  stepper1.enable();
  stepper2.enable();
}

// ===================== СЕРВЫ =====================

bool isServoAvailable(int index) {
  if (index < 0 || index >= SERVO_COUNT) return false;
  if (SERVO_PINS[index] == SERVO_UNUSED) return false;
  return true;
}

void attachAndWriteServo(int index, int angle) {
  if (!isServoAvailable(index)) {
    Serial.print("SERVO ");
    Serial.print(index);
    Serial.println(" is unused. Skip.");
    return;
  }

  angle = constrain(angle, 0, 180);

  servos[index].attach(SERVO_PINS[index], 500, 2500);
  servos[index].write(angle);

  activeServoIndex = index;

  Serial.print("SERVO ");
  Serial.print(index);
  Serial.print(" on pin ");
  Serial.print(SERVO_PINS[index]);
  Serial.print(" -> ");
  Serial.println(angle);
}

void detachActiveServo() {
  if (isServoAvailable(activeServoIndex)) {
    servos[activeServoIndex].detach();

    Serial.print("SERVO ");
    Serial.print(activeServoIndex);
    Serial.println(" DETACH");
  }

  activeServoIndex = -1;
}

// ===================== ЗАПУСК ШАГА =====================

void startRouteStep(RouteStep step) {
  stepStartTime = millis();

  if (step.type == MOVE_STEPPERS) {
    motorsWereMoving = false;

    Serial.print("MOVE: ");
    Serial.print(step.motor1Steps);
    Serial.print(" / ");
    Serial.println(step.motor2Steps);

    stepper1.setTarget(step.motor1Steps, RELATIVE);
    stepper2.setTarget(step.motor2Steps, RELATIVE);
  }

  else if (step.type == SERVO_MOVE) {
    attachAndWriteServo(step.servoIndex, step.servoAngle);
  }

  else if (step.type == PAUSE_STEP) {
    Serial.print("PAUSE: ");
    Serial.print(step.timeMs);
    Serial.println(" ms");
  }

  stepStarted = true;
}

// ===================== ПРОВЕРКА ЗАВЕРШЕНИЯ ШАГА =====================

bool isRouteStepFinished(RouteStep step, byte stepper1Move, byte stepper2Move) {
  if (step.type == MOVE_STEPPERS) {
    if (stepper1Move != 0 || stepper2Move != 0) {
      motorsWereMoving = true;
    }

    return motorsWereMoving && stepper1Move == 0 && stepper2Move == 0;
  }

  if (step.type == SERVO_MOVE) {
    if (millis() - stepStartTime >= step.timeMs) {
      detachActiveServo();
      return true;
    }

    return false;
  }

  if (step.type == PAUSE_STEP) {
    return millis() - stepStartTime >= step.timeMs;
  }

  return false;
}

// ===================== SETUP =====================

void setup() {
  Serial.begin(115200);

  pinMode(ROUTE_PIN, INPUT_PULLUP);
  pinMode(START_PIN, INPUT_PULLUP);

  setupSteppers();

  Serial.println("Robot route program");

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

  if (digitalRead(ROUTE_PIN) == LOW) {
    currentRoute = route2;
    Serial.println("Selected route: 2");
  } else {
    currentRoute = route1;
    Serial.println("Selected route: 1");
  }

  Serial.println("Waiting START_PIN press...");

  while (digitalRead(START_PIN) == HIGH) {
    stepper1.tick();
    stepper2.tick();
    delay(10);
  }

  Serial.println("START!");
}

// ===================== LOOP =====================

void loop() {
  byte stepper1Move = stepper1.tick();
  byte stepper2Move = stepper2.tick();

  RouteStep currentStep = currentRoute[currentStepIndex];

  if (currentStep.type == END_ROUTE) {
    Serial.println("ROUTE FINISHED");

    while (true) {
      stepper1.tick();
      stepper2.tick();
      delay(10);
    }
  }

  if (!stepStarted) {
    startRouteStep(currentStep);
  }

  if (isRouteStepFinished(currentStep, stepper1Move, stepper2Move)) {
    Serial.print("Step finished: ");
    Serial.println(currentStepIndex);

    currentStepIndex++;
    stepStarted = false;
    motorsWereMoving = false;

    delay(50);
  }
}