
#include <GyverStepper.h>
GStepper<STEPPER2WIRE> stepper1(800, 25, 33, 14);
GStepper<STEPPER2WIRE> stepper2(800, 27, 26, 32);
//GStepper<STEPPER2WIRE> stepper(steps, step, dir, en); 

void setup() {
  Serial.begin(115200);


  // режим следования к целевй позиции
  stepper1.setRunMode(FOLLOW_POS);
  stepper2.setRunMode(FOLLOW_POS);


  stepper1.setMaxSpeed(500);
  stepper1.setAcceleration(500);

  stepper2.setMaxSpeed(500);
  stepper2.setAcceleration(500);

  // отключать мотор при достижении цели
  stepper1.autoPower(true);
  stepper2.autoPower(true);

  stepper1.reverse(false);
  stepper2.reverse(true);

  // включить мотор (если указан пин en)
  stepper1.enable();
  stepper2.enable();
  stepper1.setTarget(500,RELATIVE);
  stepper2.setTarget(500,RELATIVE);
}

void loop() {

stepper1.tick();
stepper2.tick();
}
