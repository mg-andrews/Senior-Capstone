#include "MotorSetup.h"
#include "pindef.h"
#include <TMCStepper.h>
#include <FastAccelStepper.h>

  static const int microstep = 32;
  static const float full_steps = 200.0;
  static const float steps_per_rev = microstep * full_steps;
  static const int deadband = 2; // In steps

  #define R_SENSE 0.11f
  static TMC2209Stepper driver(&Serial1, R_SENSE, 0b00);

  static FastAccelStepperEngine engine = FastAccelStepperEngine();
  static FastAccelStepper *stepper = NULL;

void setup_motor() {

  // Motor driver setup
  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  digitalWrite(EN_PIN, HIGH); // Disable driver for config
  digitalWrite(DIR_PIN, HIGH); // LOW: CCW

  Serial1.begin(115200, SERIAL_8N1, UART_RX, UART_TX);
  delay(100);

  driver.begin();
  driver.toff(4);
  driver.blank_time(24);
  driver.internal_Rsense(false);
  driver.I_scale_analog(false);
  driver.mstep_reg_select(true);
  driver.microsteps(microstep);

  driver.en_spreadCycle(false);   // stable torque
  driver.TPWMTHRS(250);            // disable StealthChop
  driver.semin(0);               // disable CoolStep
  driver.rms_current(150);       // Max 0.15A RMS
  driver.hold_multiplier(0.2);   // Reduce idle heat & noise

  driver.pdn_disable(true);

  // Motor engine setup
  engine.init();
  stepper = engine.stepperConnectToPin(STEP_PIN);
  stepper->setDirectionPin(DIR_PIN);
  stepper->setEnablePin(EN_PIN);
  stepper->setAutoEnable(false);  // True sets the enable pin and turns the TMC2209 on/off atomatically

  /*
  stepper->setSpeedInHz(set_vel);
  stepper->setAcceleration(set_accel);
  */

  digitalWrite(EN_PIN, LOW); // Enable driver

  // Check UART works
  Serial.print("TMC version: 0x");
  Serial.println(driver.version(), HEX);
  Serial.print("Current readback: ");
  Serial.println(driver.rms_current());

  stepper->setSpeedInHz(2000);
  stepper->setAcceleration(5000);
}

void motor_move_to(int position) {
  if (stepper) stepper->moveTo(position);
}

int motor_get_position() {
  if (stepper) return stepper->getCurrentPosition();
  return 0;
}

float motor_steps_per_rev() {
  return steps_per_rev;
}

int motor_deadband() {
  return deadband;
}