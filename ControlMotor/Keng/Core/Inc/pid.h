/**
 * Core/Inc/pid.h
 *
 * Ported from https://github.com/simplefoc/Arduino-FOC/tree/master
 *
 * @author Nonpawit Ekburanawat
 */

#ifndef PID_H
#define PID_H

#include "main.h"

struct pid_s
{
  float         p;
  float         i;
  float         d;
  float         ramp;
  float         limit;
  float         err_old;
  float         out_old;
  float         int_old;
};

float pid_calculate(struct pid_s *pid, float err, float dt);

#endif /* PID_H */
