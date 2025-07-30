/**
 * Core/Inc/foc_driver.h
 *
 * Ported from https://github.com/simplefoc/Arduino-FOC/tree/master
 *
 * @author Nonpawit Ekburanawat
 */

#ifndef FOC_DRIVER_H
#define FOC_DRIVER_H

/* Public Variables *********************************************************/

/* PWM frequency. */

extern long pwm_frequency;

/* Power supply voltage. */

extern float voltage_power_supply;

/* Limiting voltage set to the motor */

extern float driver_voltage_limit;
extern float motor_voltage_limit;

/* Current target value. */

extern float target;

/* Motor enable status. */

extern int8_t enabled;

enum bldc_ctrl_type_e
{
  TORQUE = 0x00,
  VELOCITY,
  ANGLE,
  VELOCITY_OPENLOOP,
  ANGLE_OPENLOOP
};

#endif /* FOC_DRIVER_H */
