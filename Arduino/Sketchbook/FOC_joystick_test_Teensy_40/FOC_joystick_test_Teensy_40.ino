
#define JOYSTICK_ACTIVATE_PIN 32
#define JOYSTICK_PRESSED_PIN 33
#define JOYSTICK_X_PIN A2
#define JOYSTICK_Y_PIN A9

#define JOYSTICK_TRIM_X 0
#define JOYSTICK_TRIM_Y -14

#define DEADZONE_JS 0.0

const int batteryVoltageInPin = A10;  // Analog input pin that the battery 1/3 divider is attached to. "800" = 3.90V per cell (11.72V).

// desired speed can be set by joystick:
// comes in the range -100...100 - it has a meaning of "percent of max speed":
double joystickSpeedR = 0.0;
double joystickSpeedL = 0.0;

double pwm_R, pwm_L;    // pwm -255..255 accumulated here and ultimately sent to motor (H-Bridge or servos) pins (will be constrained by set_motor())


void setup() {
  // put your setup code here, to run once:

}

void loop() {

  // Test and adjust joystick:
  int xx = analogRead(JOYSTICK_X_PIN) + JOYSTICK_TRIM_X;
  int yy = analogRead(JOYSTICK_Y_PIN) + JOYSTICK_TRIM_Y;
  double jx = joystickX();
  double jy = joystickY();
  Serial.print("X: ");
  Serial.print(xx);
  Serial.print(" / ");
  Serial.print(jx);
  Serial.print("\t\tY: ");
  Serial.print(yy);
  Serial.print(" / ");
  Serial.print(jy);
  Serial.print("\t\tactive: ");
  Serial.print(isJoystickActive());
  Serial.print("  pressed: ");
  Serial.print(isJoystickPressed());

  computeJoystickSpeeds();

  // Test servos:
  pwm_R = map(joystickSpeedR, -100, 100, -255, 255);
  pwm_L = map(joystickSpeedL, -100, 100, -255, 255);

  Serial.print("\t\tPWM: R: ");
  Serial.print(pwm_R);
  Serial.print("\tL: ");
  Serial.print(pwm_L);

  //constrainPwm();

  //set_motors();

  long voltage_mv = analogRead(batteryVoltageInPin); // "800" here relates to battery voltage 11.72V = 3.90V per cell
  voltage_mv = voltage_mv * 16204l / 1000l; // millivolts, returns "12000" for 12.0V

  Serial.print("\t\tmV: ");
  Serial.println(voltage_mv);

  delay(100);
  return;

}
