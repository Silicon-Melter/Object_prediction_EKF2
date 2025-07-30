/**
 * Core/Src/bldc_driver.c
 *
 * Ported from https://github.com/simplefoc/Arduino-FOC/tree/master
 *
 * @author Nonpawit Ekburanawat
 */

#include <math.h>

#include "bldc_motor.h"
#include "bldc_driver.h"
#include "foc_utils.h"
#include "foc_motor.h"
#include "foc_driver.h"
#include "as5048a.h"

/* Public Variables *********************************************************/

float motor_voltage_limit = 0.0f;

enum foc_mod_type_e foc_mod = SINE_PWM;

float target;

float u_alpha;
float u_beta;

int8_t enabled;

int8_t modulation_centered = 1;

float shaft_velocity_sp;

struct DQVoltage_s voltage;

uint8_t             sensor;
enum ss_direction_e ss_dir;

bool pp_check_result = false;

float zero_electric_angle = 0;

float current_sp;

/* Closed loop. */

struct pid_s bldc_pid;
struct lpf_s bldc_lpf;
float        low_pass_tf;
float        voltage_sensor_align = 0.0f;
float        velocity_index_search;

/* Private Variables *********************************************************/

/* Motor controller. */

enum bldc_ctrl_type_e motor_controller;

float ua, ub, uc;

int pole_pairs;

/* Private Function Declarations ********************************************/

float vel_open_loop(float target_vel, float dt);
void  set_phrase_voltage(float uq, float ud, float andel_el);
int   align_sensor(void);

/* Private Functions ********************************************************/

float vel_open_loop(const float target_vel, float dt)
{
  if (dt <= 0 || dt > 0.5f)
  {
    dt = 1e-3f;
  }

  shaft_angle = _normalize_angle(shaft_angle + target_vel * dt);

  //  shaft_velocity = target_vel;

  const float uq = motor_voltage_limit;

  set_phrase_voltage(uq, 0, _electricalAngle(shaft_angle, pole_pairs));

  return uq;
}

void set_phrase_voltage(const float uq, const float ud, const float andel_el)
{
  float center;
  //  int   sector;
  float _ca, _sa;

  switch (foc_mod)
  {
    case SINE_PWM:
    case SPACE_VEC_PWM:
      _sincos(andel_el, &_sa, &_ca);

      u_alpha = _ca * ud - _sa * uq;
      u_beta  = _sa * ud + _ca * uq;

      ua = u_alpha;
      ub = -0.5f * u_alpha + _SQRT3_2 * u_beta;
      uc = -0.5f * u_alpha - _SQRT3_2 * u_beta;

      center = driver_voltage_limit / 2;

    // TODO: Add Space Vector PWM if clause

      if (!modulation_centered)
      {
        const float umin = min(ua, min(ub, uc));

        ua -= umin;
        ub -= umin;
        uc -= umin;
      }
      else
      {
        ua += center;
        ub += center;
        uc += center;
      }
      break;

    default:
      break;
  }

  set_pwm(ua, ub, uc);
}

int align_sensor(void)
{
  ss_dir                    = UNKNOWN;
  const float voltage_align = voltage_sensor_align;

  /* Find natural direction. */

  for (int i = 0; i <= 500; i++)
  {
    const float angle = _3PI_2 + _2PI * (float)i / 500.0f;
    set_phrase_voltage(voltage_align, 0, angle);
    HAL_Delay(2);
  }

  const float mid_angle = get_angle();

  for (int i = 500; i >= 0; i--)
  {
    const float angle = _3PI_2 + _2PI * (float)i / 500.0f;
    set_phrase_voltage(voltage_align, 0, angle);
    HAL_Delay(2);
  }

  const float end_angle = get_angle();

  HAL_Delay(200);

  const float moved = fabsf(mid_angle - end_angle);
  if (moved < MIN_ANGLE_DETECT_MOVEMENT)
  {
    return 0;
  }

  if (mid_angle < end_angle)
  {
    ss_dir = CCW;
  }
  else
  {
    ss_dir = CW;
  }

  pp_check_result = !(fabsf(moved * (float)pole_pairs - _2PI) > 0.5f);
  if (pp_check_result == false)
  {
    return 0;
  }

  /* Set zero electric angle. */

  set_phrase_voltage(voltage_align, 0, _3PI_2);
  HAL_Delay(700);

  zero_electric_angle = get_angle();

  HAL_Delay(20);

  set_phrase_voltage(0, 0, 0);
  HAL_Delay(200);
  return 1;
}

/* Public Functions *********************************************************/

/**
 * @brief Initialize BLDC motor parameters.
 * @param pp        BLDC motor pole pairs.
 * @param mot_v_lim BLDC motor voltage limit.
 * @return 1 if success.
 */
uint8_t bldc_open_loop_init(const uint8_t pp,
                            const float   mot_v_lim)
{
  motor_voltage_limit = mot_v_lim > driver_voltage_limit
                          ? driver_voltage_limit
                          : mot_v_lim;

  motor_controller = VELOCITY_OPENLOOP;
  pole_pairs       = pp;

  voltage.q = 0;
  voltage.d = 0;

  bldc_enable();

  sensor = 0;

  HAL_Delay(500);

  return 1;
}

uint8_t bldc_vel_ctrl_init(const uint8_t      pp,
                           const float        v_ss_align,
                           const float        vel_idx_s,
                           const struct lpf_s lpf,
                           const struct pid_s pid,
                           const float        mot_v_lim)
{
  motor_voltage_limit = mot_v_lim > driver_voltage_limit
                          ? driver_voltage_limit
                          : mot_v_lim;
  voltage_sensor_align = v_ss_align > motor_voltage_limit
                           ? motor_voltage_limit
                           : v_ss_align;
  velocity_index_search = vel_idx_s;

  bldc_pid       = pid;
  bldc_lpf       = lpf;
  bldc_pid.limit = motor_voltage_limit;

  motor_controller = VELOCITY;
  pole_pairs       = pp;

  sensor = 1;

  voltage.q = 0;
  voltage.d = 0;

  bldc_enable();

  HAL_Delay(500);

  return 1;
}

int foc_init(void)
{
  int rslt = 0;

  if (sensor)
  {
    rslt = align_sensor();
  }

  return rslt;
}

void bldc_enable(void)
{
  driver_enable();

  enabled = 1;
}

void bldc_disable(void)
{
  driver_disable();

  enabled = 0;
}

void bldc_move(const float target, const float dt)
{
  shaft_velocity = lpf_calc(&bldc_lpf, get_vel_foc(dt), dt);

  if (!enabled)
  {
    return;
  }

  switch (motor_controller)
  {
    case VELOCITY:
      shaft_velocity_sp = target;
      current_sp = pid_calculate(&bldc_pid,
                                 shaft_velocity_sp - shaft_velocity,
                                 dt);
      voltage.q = current_sp;
      voltage.d = 0;
      break;

    case VELOCITY_OPENLOOP:
      shaft_velocity_sp = target;
      voltage.q = vel_open_loop(shaft_velocity_sp, dt);
      voltage.d = 0;
      break;

    default:
      break;
  }
}

void loop_foc(void)
{
  if (motor_controller == VELOCITY_OPENLOOP ||
    motor_controller == ANGLE_OPENLOOP)
  {
    return;
  }

  if (!enabled)
  {
    return;
  }

  electrical_angle = (float)(ss_dir * pole_pairs) *
    _normalize_angle(get_angle());

  set_phrase_voltage(voltage.q, voltage.d, electrical_angle);
}
