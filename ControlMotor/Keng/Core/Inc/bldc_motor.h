/**
 * Core/Inc/bldc_driver.h
 *
 * Ported from https://github.com/simplefoc/Arduino-FOC/tree/master
 *
 * @author Nonpawit Ekburanawat
 */

#ifndef BLDC_MOTOR_H
#define BLDC_MOTOR_H

#include "main.h"
#include "pid.h"
#include "lpf.h"

uint8_t bldc_open_loop_init(uint8_t pp,
                            float   mot_v_lim);
uint8_t bldc_vel_ctrl_init(uint8_t      pp,
                           float        v_ss_align,
                           float        vel_idx_s,
                           struct lpf_s lpf,
                           struct pid_s pid,
                           float        mot_v_lim);
void bldc_enable(void);
void bldc_disable(void);
void bldc_move(float target, float dt);
void loop_foc(void);

#endif /* BLDC_MOTOR_H */
