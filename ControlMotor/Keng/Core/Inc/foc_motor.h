/**
 * Core/Inc/foc_motor.h
 *
 * Ported from https://github.com/simplefoc/Arduino-FOC/tree/master
 *
 * @author Nonpawit Ekburanawat
 */

#ifndef FOC_MOTOR_H
#define FOC_MOTOR_H

#include <stdbool.h>

#include "foc_utils.h"

enum foc_mod_type_e
{
  SINE_PWM = 0x00,
  SPACE_VEC_PWM,
  TRAPEZOID_120,
  TRAPEZOID_150,
};

enum ss_direction_e
{
  CW      = 1,
  CCW     = -1,
  UNKNOWN = 0
};

extern enum foc_mod_type_e foc_mod;

extern float u_alpha;

extern float u_beta;

extern int8_t modulation_centered;

/* Current target velocity. */

extern float shaft_velocity_sp;

/* Current target angle. */

extern float shaft_angle_sp;

/* Current motor angle. */

float shaft_angle;

/* Current electrical angle. */

float electrical_angle;

/* Current motor velocity. */

float shaft_velocity;

extern struct DQVoltage_s voltage;

/* Motor closed loop control PID. */

extern struct pid_s bldc_pid;

/* Motor closed loop low pass filter parameter. */

extern float low_pass_tf;

/* Motor configuration parameters. */

extern float voltage_sensor_align;
extern float velocity_index_search;

/* Variable to store whether sensor is used. */

extern uint8_t sensor;

extern enum ss_direction_e ss_dir;

extern bool pp_check_result;

extern float zero_electric_angle;

extern float current_sp;

extern struct lpf_s bldc_lpf;

#endif /* FOC_MOTOR_H */
