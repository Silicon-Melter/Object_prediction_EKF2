/**
 * Core/Src/pid.c
 *
 * Ported from https://github.com/simplefoc/Arduino-FOC/tree/master
 *
 * @author Nonpawit Ekburanawat
 */

#include "pid.h"

#include "foc_utils.h"

float pid_calculate(struct pid_s *pid, const float err, const float dt)
{
  const float proportional = pid->p * err;

  float integral = pid->int_old + pid->i * dt * 0.5f * (err - pid->err_old);

  integral = _constrain(integral, -pid->limit, pid->limit);

  const float derivative = pid->d * (err - pid->err_old) / dt;

  float output = proportional + integral + derivative;
  output       = _constrain(output, -pid->limit, pid->limit);

  if (pid->ramp > 0)
  {
    const float output_rate = (output - pid->out_old) / dt;
    if (output_rate > pid->ramp)
    {
      output = pid->out_old + pid->ramp * dt;
    }
    else if (output_rate < -pid->ramp)
    {
      output = pid->out_old - pid->ramp * dt;
    }
  }

  pid->int_old = integral;
  pid->out_old = output;
  pid->err_old = err;
  return output;
}
