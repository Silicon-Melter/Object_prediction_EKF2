/**
 * Core/Src/foc_utils.c
 *
 * Ported from https://github.com/simplefoc/Arduino-FOC/tree/master
 *
 * @author Nonpawit Ekburanawat
 */

#include <stdint.h>
#include <math.h>

#include "foc_utils.h"

/* Public Functions *********************************************************/

float _normalize_angle(const float angle)
{
  const float a = fmod(angle, _2PI);

  return a >= 0 ? a : a + _2PI;
}

void _sincos(const float a, float *s, float *c)
{
  *s = _sin(a);
  *c = _cos(a);
}

float _sin(float a)
{
  static uint16_t sine_array[65] = {
    0,
    804,
    1608,
    2411,
    3212,
    4011,
    4808,
    5602,
    6393,
    7180,
    7962,
    8740,
    9512,
    10279,
    11039,
    11793,
    12540,
    13279,
    14010,
    14733,
    15447,
    16151,
    16846,
    17531,
    18205,
    18868,
    19520,
    20160,
    20788,
    21403,
    22006,
    22595,
    23170,
    23732,
    24279,
    24812,
    25330,
    25833,
    26320,
    26791,
    27246,
    27684,
    28106,
    28511,
    28899,
    29269,
    29622,
    29957,
    30274,
    30572,
    30853,
    31114,
    31357,
    31581,
    31786,
    31972,
    32138,
    32286,
    32413,
    32522,
    32610,
    32679,
    32729,
    32758,
    32768
  };
  int32_t      t1, t2;
  unsigned int i    = (unsigned int)(a * (64 * 4 * 256.0f / _2PI));
  const int    frac = i & 0xff;
  i                 = i >> 8 & 0xff;
  if (i < 64)
  {
    t1 = (int32_t)sine_array[i];
    t2 = (int32_t)sine_array[i + 1];
  }
  else if (i < 128)
  {
    t1 = (int32_t)sine_array[128 - i];
    t2 = (int32_t)sine_array[127 - i];
  }
  else if (i < 192)
  {
    t1 = -(int32_t)sine_array[-128 + i];
    t2 = -(int32_t)sine_array[-127 + i];
  }
  else
  {
    t1 = -(int32_t)sine_array[256 - i];
    t2 = -(int32_t)sine_array[255 - i];
  }

  return 1.0f / 32768.0f * (t1 + ((t2 - t1) * frac >> 8));
}

float _cos(float a)
{
  float a_sin = a + _PI_2;
  a_sin       = a_sin > _2PI ? a_sin - _2PI : a_sin;
  return _sin(a_sin);
}

float _electricalAngle(float shaft_angle, int pole_pairs)
{
  return shaft_angle * pole_pairs;
}
