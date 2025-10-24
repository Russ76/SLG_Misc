/**

   Velocity motion control example
   Steps:
   1) Configure the motor and sensor
   2) Run the code
   3) Set the target velocity (in radians per second) from serial terminal



   NOTE :
   > Specifically for Arduino UNO example code for running velocity motion control using a hall sensor
   > Since Arduino UNO doesn't have enough interrupt pins we have to use software interrupt library PciManager.

   > If running this code with Nucleo or Bluepill or any other board which has more than 2 interrupt pins
   > you can supply doC directly to the sensor.enableInterrupts(doA,doB,doC) and avoid using PciManger

*/
#include <SimpleFOC.h>

// miniPRO wheels are very low resistance, "1" is fine:
#define POWER_SUPPLY_VOLTAGE 12
//#define DRIVER_VOLTAGE_LIMIT 6
#define MOTOR_VOLTAGE_LIMIT 8
#define MOTOR_VOLTAGE_ALIGN 2
#define POLE_PAIRS 15

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(POLE_PAIRS);
BLDCDriver3PWM driver = BLDCDriver3PWM(5, 6, 7, 8);  // Left wheel
//BLDCDriver3PWM driver = BLDCDriver3PWM(9, 10, 11, 13); // Right wheel. 13 is also BUILTIN_LED

// hall sensor instance
//HallSensor sensor(HALL1, HALL2, HALL3, POLE_PAIRS);
HallSensor sensor = HallSensor(2, 3, 4, POLE_PAIRS);  // Left wheel
//HallSensor sensor = HallSensor(12, 14, 15, POLE_PAIRS); // Right wheel

// MOT: Align sensor.
// MOT: sensor_direction==CCW
// MOT: PP check: OK!
// MOT: Zero elec. angle: 5.24
// MOT: No current sense.

bool state_A = false;
bool state_B = false;
//bool state_C = false;
bool prev_state_A = false;
bool prev_state_B = false;

volatile long long Ldistance = 0;

/*
My version of the ISR:
void leftEncoder() {
  state_A = digitalRead(sensor.pinA);
  state_B = digitalRead(sensor.pinB);
  //state_C = digitalRead(sensor.pinC);
  if (state_A != prev_state_A || state_B != prev_state_B) {
    // this is one tick - A or B changed. We have 60 ticks per rotation for miniPRO wheels (15 pole pairs)
    bool dir = state_A && !prev_state_A && state_B
           || !state_A && prev_state_A && !state_B
           || !state_B && prev_state_B && state_A
           || state_B && !prev_state_B && !state_A;

  //Serial.println(dir);

    if (dir) {
      Ldistance--;
    } else {
      Ldistance++;  // wheel moves forward, positive increase
    }
    prev_state_A = state_A;
    prev_state_B = state_B;
  }
}
*/

// ChatGPT 5 version of the ISR:
// - Executes in constant time (no conditional branching),
// - Is extremely fast (a single table lookup)
void leftEncoder() {
  uint8_t A = digitalRead(sensor.pinA);
  uint8_t B = digitalRead(sensor.pinB);

  uint8_t curr = (A << 1) | B;
  uint8_t prev = (prev_state_A << 1) | prev_state_B;

  // Lookup table for direction based on previous and current 2-bit states
  static const int8_t dir_table[4][4] = {
    { 0,  1, -1,  0},
    {-1,  0,  0,  1},
    { 1,  0,  0, -1},
    { 0, -1,  1,  0}
  };

  int8_t dir = dir_table[prev][curr];
  if (dir) Ldistance += dir;

  prev_state_A = A;
  prev_state_B = B;
} 

// Interrupt routine intialization
// channel A and B callbacks
void doA() {
  //Serial.println("doA");
  //uint8_t hall_state = sensor.readHallState(); // bitmask 0-2 bits; not in this library version (2.3) yet
  leftEncoder();
  sensor.handleA();
}
void doB() {
  //Serial.println("doB");
  leftEncoder();
  sensor.handleB();
}
void doC() {
  //Serial.println("doC");
  leftEncoder();
  sensor.handleC();
}

// velocity set point variable
float target_velocity = 0;
// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) {
  command.scalar(&target_velocity, cmd);
}

bool init_ok = false;

ulong last_print = 0;

void setup() {

  // use monitoring with serial
  Serial.begin(115200);
  // enable more verbose output for debugging
  // comment out if not needed. See https://docs.simplefoc.com/debugging
  SimpleFOCDebug::enable(&Serial);

  // initialize HALL sensor hardware
  sensor.init();  // void sub

  sensor.enableInterrupts(doA, doB, doC);

  // link the motor to the sensor
  motor.linkSensor(&sensor);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = POWER_SUPPLY_VOLTAGE;
  //driver.voltage_limit = DRIVER_VOLTAGE_LIMIT;
  driver.pwm_frequency = 20000;
  // Force center-aligned PWM mode to avoid warning at startup (library version 2.4.x+):
  //driverL.center_aligned = driverR.center_aligned = true;

  if (!driver.init()) {
    SIMPLEFOC_DEBUG("Error: Driver init failed!");
    return;
  }

  // link the motor and the driver
  motor.linkDriver(&driver);

  // During the sensor align procedure, SimpleFOC moves the wheels and measures
  //     the direction of the sensor and the zero electrical angle offset.
  // set aligning voltage [Volts]:
  motor.voltage_sensor_align = MOTOR_VOLTAGE_ALIGN;
  // index search velocity [rad/s]
  //motor.velocity_index_search = 3;

  // set motion control loop to be used
  motor.controller = MotionControlType::velocity;

  // contoller configuration
  // default parameters in defaults.h

  // velocity PID controller parameters
  motor.PID_velocity.P = 0.4f;
  motor.PID_velocity.I = 2.0f;
  motor.PID_velocity.D = 0.0f;

  // limiting motor movements
  // Note: enables basic anti-windup mechanism if you limit the voltage or current.
  // limit the voltage to be set to the motor. Start very low for high resistance motors.
  // current = voltage / resistance, so try to be well under 1Amp
  motor.voltage_limit = MOTOR_VOLTAGE_LIMIT;
  motor.LPF_velocity = 0.2;  // a low-pass filter to smooth out noisy velocity measurements, derived from position sensor.
  motor.PID_velocity.output_ramp = 300.0;

  // To skip alignment, supply known parameters:
  motor.zero_electric_angle = 5.24;         // rad
  motor.sensor_direction = Direction::CCW;  // CW or CCW

  // comment out if not needed
  //motor.useMonitoring(Serial);

  SIMPLEFOC_DEBUG("IP: initializing motor and FOC...");

  // initialize motor
  if (!motor.init()) {
    SIMPLEFOC_DEBUG("Error: Motor init failed!");
    return;
  }

  // If not skipped, will align sensor and start FOC
  if (!motor.initFOC()) {
    SIMPLEFOC_DEBUG("Error: FOC init failed!");
    return;
  }

  // add target velocity command T
  command.add('T', doTarget, "target velocity");

  Serial.println(F("OK: Motor ready."));
  Serial.println(F("Set the target velocity using serial terminal:"));
  _delay(1000);

  init_ok = true;
}

long long LdistancePrev = 0;
long long pLdistancePrev = 0;

double speedMeasured_L = 0;

long distL;

void speed_calculate() {
  distL = Ldistance - LdistancePrev;

  // note: this is the place to scale -100..100% speed to 255 pwm:
  speedMeasured_L = (double)distL;  //map(distL, -80, 80, -100, 100);

  // we don't need to keep track of total distances per wheel, only increments - for speed loop.
  // beware - if EncodersReset() is not called often enough, L/Rdistance will
  // overflow at 32K and cause violent jerking of the wheels!
  //EncodersReset();

  LdistancePrev = Ldistance;
}

bool pprev_state_A = false;
bool pprev_state_B = false;

void loop() {

  if (!init_ok) {
    _delay(1000);
    return;
  }

/*
  if (state_A != pprev_state_A || state_B != pprev_state_B) {
    Serial.print("AB: ");
    Serial.print(state_A);
    Serial.print("\t");
    Serial.println(state_B);
    pprev_state_A = state_A;
    pprev_state_B = state_B;
  }
*/

  if (Ldistance != pLdistancePrev) {
    Serial.print("dist: ");
    Serial.println(Ldistance);
    pLdistancePrev = Ldistance;
  }

  ulong now = millis();
  if (now - last_print > 1000) {
    float angle = sensor.getAngle();
    float velocity = sensor.getVelocity();
    Serial.print("Ang: ");
    Serial.print(angle);
    Serial.print("  Vel: ");
    Serial.println(velocity);

    speed_calculate();
    Serial.print("Ldistance: ");
    Serial.print(Ldistance);
    Serial.print("\tMeasured speed %: ");
    Serial.println(speedMeasured_L);

    last_print = now;
  }

  // main FOC algorithm function
  // the faster you run this function the better
  // Arduino UNO loop  ~1kHz
  // Bluepill loop ~10kHz
  motor.loopFOC();

  // Motion control function
  // velocity, position or voltage (defined in motor.controller)
  // this function can be run at much lower frequency than loopFOC() function
  // You can also use motor.move() and set the motor.target in the code
  //motor.move(target_velocity);

  // function intended to be used with serial plotter to monitor motor variables
  // significantly slowing the execution down!!!!
  // motor.monitor();

  // user communication
  command.run();
}
