/**
 * Core/Src/lpf.c
 *
 * Ported from https://github.com/simplefoc/Arduino-FOC/tree/master
 *
 * @author Nonpawit Ekburanawat
 */

#include "lpf.h"

float lpf_calc(struct lpf_s *lpf, const float dat, const float dt)
{
  const float alpha = lpf->tf / (lpf->tf + dt);
  const float y     = alpha * lpf->prev_x + (1.0f - alpha) * dat;
  lpf->prev_x       = y;
  return y;
}
