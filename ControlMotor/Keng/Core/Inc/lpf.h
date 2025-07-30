/**
 * Core/Inc/lpf.h
 *
 * Ported from https://github.com/simplefoc/Arduino-FOC/tree/master
 *
 * @author Nonpawit Ekburanawat
 */

#ifndef LPF_H
#define LPF_H

struct lpf_s
{
  float tf;
  float prev_x;
};

float lpf_calc(struct lpf_s *lpf, float dat, float dt);

#endif /* LPF_H */
