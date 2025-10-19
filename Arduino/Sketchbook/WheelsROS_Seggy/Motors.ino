//
// see https://github.com/slgrobotics/robots_bringup/tree/main/Docs/miniPRO
//
// Seggy ("Smart Table" robot is based on Segway Ninebot miniPRO
//

void MotorsInit()
{
  pwm_R = 0;
  pwm_L = 0;

  set_motors();
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

}
