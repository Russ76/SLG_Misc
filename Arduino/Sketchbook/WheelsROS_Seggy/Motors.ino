//
// see https://github.com/slgrobotics/robots_bringup/tree/main/Docs/miniPRO
//
// Seggy ("Smart Table" robot is based on Segway Ninebot miniPRO
//

#include <SimpleFOC.h>

#define POWER_SUPPLY_VOLTAGE 12
#define DRIVER_VOLTAGE_LIMIT 6
#define MOTOR_VOLTAGE_ALIGN 2
#define POLE_PAIRS 15

const float voltage_limit = 6.0;  // safe startup limit
const float current_limit = 3.0;  // A, conservative

//target velocity and voltage limit variables:
float target_velocity_L = 0;
float target_velocity_R = 0;
float volt_limit = 1; // [V] miniPRO wheels are very low resistance. 0.5A at this setting

BLDCMotor motorL = BLDCMotor(POLE_PAIRS);
BLDCMotor motorR = BLDCMotor(POLE_PAIRS);

//BLDCDriver6PWM driver(A_H, B_H, C_H, A_L, B_L, C_L);
// Note: For 6PWM action GP 26...31 are available, if not used for LEDs

BLDCDriver3PWM driverL = BLDCDriver3PWM(5, 6, 7, 8);    // Left wheel
BLDCDriver3PWM driverR = BLDCDriver3PWM(9, 10, 11, 13); // Right wheel. 13 is also BUILTIN_LED

//HallSensor sensor(HALL1, HALL2, HALL3, POLE_PAIRS);
HallSensor sensorL = HallSensor(2, 3, 4, POLE_PAIRS);    // Left wheel
HallSensor sensorR = HallSensor(12, 14, 15, POLE_PAIRS); // Right wheel

// Note: A2 (16) and A9 (23) are used by joystick, as are A32 (Active) and A33 (Pressed). A10 (24) used by battery monitor.

// InlineCurrentSensor - https://docs.simplefoc.com/inline_current_sense
//  - shunt_resistor  - shunt resistor value (0.005 Ohm)
//  - gain  - current-sense op-amp gain (24 in this case)
//  Option: if measuring 2 out of 3 currents, put the flag _NC to the phase you are not using.
InlineCurrentSense current_senseL = InlineCurrentSense(0.005, 24, A6, A7, A8); // Left wheel
InlineCurrentSense current_senseR = InlineCurrentSense(0.005, 24, A3, A4, A5); // Right wheel

// -------- PID tuning values from last message --------
void setupPID() {
  // current q/d
  motorL.PID_current_q.P = 1.00;
  motorL.PID_current_q.I = 0.03;
  motorL.PID_current_q.D = 0.00;

  motorL.PID_current_d.P = 1.00;
  motorL.PID_current_d.I = 0.03;
  motorL.PID_current_d.D = 0.00;

  // velocity
  motorL.PID_velocity.P = 0.08;   // 0.60;
  motorL.PID_velocity.I = 0.75;   // 0.015;
  motorL.PID_velocity.D = 0.004;  //0.0002;

  // position (optional)
  // motorL.P_angle.P = 18.0;
  // motorL.P_angle.I = 0.0;
  // motorL.P_angle.D = 0.0;

  // current q/d
  motorR.PID_current_q.P = 1.00;
  motorR.PID_current_q.I = 0.03;
  motorR.PID_current_q.D = 0.00;

  motorR.PID_current_d.P = 1.00;
  motorR.PID_current_d.I = 0.03;
  motorR.PID_current_d.D = 0.00;

  // velocity
  motorR.PID_velocity.P = 0.08;   // 0.60;
  motorR.PID_velocity.I = 0.75;   // 0.015;
  motorR.PID_velocity.D = 0.004;  //0.0002;

  // position (optional)
  // motorR.P_angle.P = 18.0;
  // motorR.P_angle.I = 0.0;
  // motorR.P_angle.D = 0.0;
}

bool MotorsInit()
{
  pwm_R = 0;
  pwm_L = 0;

  // enable more verbose output for debugging
  // comment out if not needed. See https://docs.simplefoc.com/debugging
  SimpleFOCDebug::enable(&Serial);

  // driver config - power supply voltage [V]
  driverL.voltage_power_supply = driverR.voltage_power_supply = POWER_SUPPLY_VOLTAGE;

  // limit the maximum dc voltage the driver can set
  // as a protection measure for the low-resistance motors
  // this value is fixed on startup
  driverL.voltage_limit = driverR.voltage_limit = DRIVER_VOLTAGE_LIMIT;
  if(!driverL.init() || !driverR.init()){
    Serial.println("Error: Drivers init failed!");
    return false;
  }

  // link motors and drivers:
  motorL.linkDriver(&driverL);
  motorR.linkDriver(&driverR);

  // During the sensor align procedure, SimpleFOC moves the wheels and measures
  //     the direction of the sensor and the zero electrical angle offset.
  // set aligning voltage [Volts]:
  motorL.voltage_sensor_align = motorR.voltage_sensor_align = MOTOR_VOLTAGE_ALIGN;

  // Link sensors to motors:
  motorL.linkSensor(&sensorL);
  motorR.linkSensor(&sensorR);

  // limiting motor movements
  // limit the voltage to be set to the motor
  // start very low for high resistance motors
  // current = voltage / resistance, so try to be well under 1Amp
  motorL.voltage_limit = volt_limit;
  motorR.voltage_limit = volt_limit;
 
  // open loop control config
  motorL.controller = MotionControlType::velocity_openloop;
  motorR.controller = MotionControlType::velocity_openloop;
  //motorL.controller = MotionControlType::velocity;
  //motorR.controller = MotionControlType::velocity;

  setupPID();

  // init motor hardware
  if(!motorL.init() || !motorR.init()){
    Serial.println("Error: Motors init failed!");
    return false;
  }

  if (!motorL.initFOC() || !motorR.initFOC()) {
    Serial.println("Error: FOC init failed!");
    return false;
  }

  set_motors();

  return true;
}

void constrainPwm()
{
  // Maximum / Minimum Limitations:
  pwm_R = constrain(pwm_R, -255, 255);
  pwm_L = constrain(pwm_L, -255, 255);
}

// ******************************************************************************************
//
//   Set motor power for both motors. Positive is forward.
//
// ******************************************************************************************
void set_motors()
{
  constrainPwm();

  target_velocity_L = map(pwm_L, -255, 255, -3.0, 3.0);
  target_velocity_R = map(pwm_R, -255, 255, -3.0, 3.0);
}

void motorLoop()
{
  // Execute FOC algorithm
  motorL.loopFOC();
  motorR.loopFOC();

  // open loop velocity movement
  // using motor.voltage_limit and motor.velocity_limit
  // to turn the motor "backwards", just set a negative target_velocity
  motorL.move(-target_velocity_L);
  motorR.move(target_velocity_R);
}
