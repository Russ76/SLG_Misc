//#define TRACE
//#define USE_EMA
//#define USE_PIDS
//#define HAS_ENCODERS
//#define HAS_LEDS

// See https://github.com/slgrobotics/robots_bringup

#ifdef USE_PIDS
#include <PID_v1.h>
#endif // USE_PIDS

//
// This code is an adaptation of Dragger and Plucky robots code, which runs on Mega 2560
// The "Comm" part complies with articubot_one and can be used with ROS2 interchangibly.
//
// As of 2025-10-20 this code is deployed on Seggy robot and works with its joystick and ROS2 Jazzy
//
// See https://github.com/slgrobotics/robots_bringup/tree/main/Docs
//     https://github.com/slgrobotics/Misc/blob/master/Arduino/Sketchbook/DraggerROS
//     https://github.com/slgrobotics/Misc/blob/master/Arduino/Sketchbook/PluckyWheelsROS
//
// Sergei Grichine - slg@quakemap.com
//
// Note: using Arduino IDE 2.3.6 + Teensy plugins + Teensy 4.0 on Ubunti 24.04
//

#define JOYSTICK_ACTIVATE_PIN 32
#define JOYSTICK_PRESSED_PIN 33
#define JOYSTICK_X_PIN A2
#define JOYSTICK_Y_PIN A9

#define JOYSTICK_TRIM_X 12
#define JOYSTICK_TRIM_Y -2

#define DEADZONE_JS 0.0

const int batteryVoltageInPin = A10;  // Analog input pin that the battery 1/3 divider is attached to. 4.7K / 1.2K does the job for 15V max.

const int mainLedPin = LED_BUILTIN;

#ifdef HAS_LEDS
// additional diagnostic LEDs:
const int redLedPin = 28;
const int greenLedPin = 29;
const int whiteLedPin = 30;
const int blueLedPin = 31;
#endif // HAS_LEDS

const int mydt = 5;           // 5 milliseconds make for 200 Hz operating cycle
const int controlLoopFactor = 20; // factor of 20 make for 100 ms Control (PID) cycle

//-------------------------------------- Variable definitions --------------------------------------------- //

#ifdef HAS_ENCODERS
// 64-bit integers to avoid overflow
volatile long long Ldistance, Rdistance;   // encoders - distance traveled, in ticks
long long LdistancePrev = 0;   // last encoders values - distance traveled, in ticks
long long RdistancePrev = 0;

double speedMeasured_R = 0;  // percent of max speed for this drive configuration.
double speedMeasured_L = 0;

long distR; // ticks per 100ms cycle, as measured
long distL;
#endif // HAS_ENCODERS

double pwm_R, pwm_L;    // pwm -255..255 accumulated here and ultimately sent to motor (H-Bridge or servos) pins (will be constrained by set_motor())
#ifdef USE_PIDS
double dpwm_R, dpwm_L;  // correction output, calculated by PID, constrained -250..250 normally, will be added to the above
#endif // USE_PIDS

int angle_steer{0};
int angle_throttle{0};

// desired speed can be set by joystick:
// comes in the range -100...100 - it has a meaning of "percent of max speed":
double joystickSpeedR = 0.0;
double joystickSpeedL = 0.0;

// desired speed is set by Comm or Joystick.
// comes in the range -100...100 - it has a meaning of "percent of max possible speed"
double desiredSpeedR = 0.0;
double desiredSpeedL = 0.0;

// PID Setpoints (desired speed after ema, -100...100 ):
double setpointSpeedR = 0.0;
double setpointSpeedL = 0.0;

const int RightMotorChannel = 0;  // index to ema*[] arrays
const int LeftMotorChannel = 1;

// EMA period to smooth wheels movement. 100 is smooth but fast, 300 is slow.
const int EmaPeriod = 20;

// Robot physical parameters:
double wheelBaseMeters = 0.600;
double wheelRadiusMeters = 0.192;
double encoderTicksPerRevolution = 2506; // one wheel rotation

#ifdef USE_PIDS
// higher Ki causes residual rotation, higher Kd - jerking movement
PID myPID_R(&speedMeasured_R, &dpwm_R, &setpointSpeedR, 1.0, 0, 0.05, DIRECT);    // in, out, setpoint, double Kp, Ki, Kd, DIRECT or REVERSE
PID myPID_L(&speedMeasured_L, &dpwm_L, &setpointSpeedL, 1.0, 0, 0.05, DIRECT);
#endif // USE_PIDS

bool testDir = false;

/* Stop the robot if it hasn't received a movement command
  in this number of milliseconds */
unsigned long AUTO_STOP_INTERVAL = 2000;

// milliseconds from last events:
unsigned long lastMotorCommandMs = 0;
unsigned long lastCommMs = 0;
unsigned long lastSonarMs = 0;

//-------------------------------------- End of variable definitions -----------------------------------------//

bool init_ok = false;

unsigned long timer = 0;     // general purpose timer, microseconds
unsigned long timer_old;

unsigned int loopCnt = 0;
unsigned int lastLoopCnt = 0;

void setup()
{
  InitSerial(); // ArticuBots uplink, uses Serial/USB.

  InitLeds();

#ifdef USE_PIDS
  int PID_SAMPLE_TIME = mydt * controlLoopFactor;  // milliseconds.

  // turn the PID on and set its parameters:
  myPID_R.SetOutputLimits(-250.0, 250.0);  // match to maximum PID outputs in both directions. PID output will be added to PWM on each cycle.
  myPID_R.SetSampleTime(PID_SAMPLE_TIME);  // milliseconds. Regardless of how frequently Compute() is called, the PID algorithm will be evaluated at a regular interval (no more often than this).
  myPID_R.SetMode(AUTOMATIC);              // AUTOMATIC means the calculations take place, while MANUAL just turns off the PID and lets the man drive

  myPID_L.SetOutputLimits(-250.0, 250.0);
  myPID_L.SetSampleTime(PID_SAMPLE_TIME);
  myPID_L.SetMode(AUTOMATIC);
#endif // USE_PIDS

  // ======================== init motors and encoders: ===================================

  init_ok = MotorsInit();

#ifdef HAS_ENCODERS
  EncodersInit();    // Initialize the encoders - attach interrupts
#endif // HAS_ENCODERS

  setEmaPeriod(RightMotorChannel, EmaPeriod);
  setEmaPeriod(LeftMotorChannel, EmaPeriod);

  timer = micros();
  delay(20);
  loopCnt = 0;
}

// =========================== main loop timing ============================================
unsigned long STD_LOOP_TIME  = mydt * 1000;  // Fixed time loop of 5 milliseconds
unsigned long lastLoopUsefulTime = STD_LOOP_TIME;  // see Kalman dt

#ifdef TRACE
const int printLoopFactor = 200;
int printLoopCnt = 0;
#endif

// =========================================================================================

void loop() //Main Loop
{
  if (!init_ok) {
    // Error: blinking LED_BUILTIN very slowly
    blinkLED(10, 2000);
    return;
  }

  // Test and adjust joystick: https://github.com/slgrobotics/Misc/tree/master/Arduino/Sketchbook/FOC_joystick_test_Teensy_40

  // Wait here, if not time yet - we use a time fixed loop
  if (lastLoopUsefulTime < STD_LOOP_TIME)
  {
    while ((micros() - timer) < STD_LOOP_TIME)
    {
      motorLoop(); // run as often as possible
      readCommCommand();  // reads desiredSpeed
    }
  }

  loopCnt++;
  timer_old = timer;
  timer = micros();

  boolean isControlLoop = loopCnt % controlLoopFactor == 0;

#ifdef TRACE
  boolean isPrintLoop = printLoopCnt++ >= printLoopFactor;

  if (isPrintLoop)
  {
    printLoopCnt = 0;
    /*
      Serial.print("Dist/cycle: ");
      Serial.print(distR);
      Serial.print("   Desired: ");
      Serial.print(desiredSpeedR);
      Serial.print("   Measured: ");
      Serial.println(speedMeasured_R);
    */
    printAll();    // takes 34ms and completely stops the 5ms cycle
  }
#endif

  if (millis() - lastCommMs > AUTO_STOP_INTERVAL || millis() - lastMotorCommandMs > AUTO_STOP_INTERVAL)
  {
    // failsafe - Comm signal or motor command is not detected for more than a second
    desiredSpeedR = desiredSpeedL = 0.0;
  }

  // test - controller should hold this speed:
  //desiredSpeedR = 20;
  //desiredSpeedL = 20;

  if (isJoystickActive())
  {
    // ignore Comm values and override by joystick on A0 and A1:

    computeJoystickSpeeds();

    desiredSpeedL = joystickSpeedL;
    desiredSpeedR = joystickSpeedR;

    //  myPID_L.SetMode(MANUAL);  // Disables PID, input goes straight to PWM
    //  myPID_R.SetMode(MANUAL);
    //} else {
    //  myPID_L.SetMode(AUTOMATIC);
    //  myPID_R.SetMode(AUTOMATIC);
  }

#ifdef USE_EMA
  // smooth movement by using ema: take desiredSpeed and produce setpointSpeed
  ema(RightMotorChannel);
  ema(LeftMotorChannel);
#else // USE_EMA
  setpointSpeedR = desiredSpeedR; // no ema
  setpointSpeedL = desiredSpeedL;
#endif // USE_EMA

#ifdef USE_PIDS
  // compute control inputs - increments to current PWM
  myPID_R.Compute();
  myPID_L.Compute();
#endif // USE_PIDS

  if (isControlLoop) // we do speed PID calculation on a slower scale, about 10Hz
  {
#ifdef HAS_ENCODERS
    // based on PWM increments, calculate speed:
    speed_calculate();
#endif // HAS_ENCODERS

#ifdef USE_PIDS
    // Calculate the pwm, given the desired speed (pick up values computed by PIDs):
    pwm_calculate();
#else // USE_PIDS
    pwm_R = constrain(map(setpointSpeedR, -100, 100, -255, 255), -255.0, 255.0);
    pwm_L = constrain(map(setpointSpeedL, -100, 100, -255, 255), -255.0, 255.0);
#endif // USE_PIDS

    // if(abs(pwm_R) > 250 || abs(desiredSpeedR - speedMeasured_R) > 3) {
    //   Serial.print(speedMeasured_R);
    //   Serial.print("  ");
    //   Serial.println(pwm_R);
    // }

    // test: At both motors set to +80 expect Ldistance and Rdistance to increase
    //pwm_R = 80.0;
    //pwm_L = 80.0;
    //if(!isJoystickActive()) {
    //  pwm_L = 255.0;  // measure full speed distR and distL. See SpeedControl tab.
    //  pwm_R = 255.0;
    //}

    set_motors();

#ifdef HAS_LEDS
    digitalWrite(redLedPin, millis() - lastCommMs > 1000 ? HIGH : LOW);
    digitalWrite(whiteLedPin, millis() - lastMotorCommandMs > 1000 ? HIGH : LOW);
    //digitalWrite(blueLedPin, millis() - lastSonarMs > 1000 ? HIGH : LOW);

    digitalWrite(greenLedPin, !digitalRead(greenLedPin)); // blinking at 10 Hz
#endif // HAS_LEDS

    // Normal cycle blinking: 5 Hz
    digitalWrite(mainLedPin,!digitalRead(mainLedPin));
  }

  // we are done in about 3.4ms

  // mark how much time we spent:
  lastLoopUsefulTime = micros() - timer;

}
