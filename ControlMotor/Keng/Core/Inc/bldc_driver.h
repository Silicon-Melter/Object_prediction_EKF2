/**
 * Core/Inc/bldc_driver.h
 *
 * Ported from https://github.com/simplefoc/Arduino-FOC/tree/master
 *
 * @author Nonpawit Ekburanawat
 */

#ifndef BLDC_DRIVER_H
#define BLDC_DRIVER_H

#include "main.h"

/* Public Function Declarations *********************************************/

void set_pwm(float ua, float ub, float uc);
int  driver_init(const TIM_HandleTypeDef *_htim1,
                uint32_t                  ph_a_ch,
                const TIM_HandleTypeDef * _htim2,
                uint32_t                  ph_b_ch,
                const TIM_HandleTypeDef * _htim3,
                uint32_t                  ph_c_ch,
                GPIO_TypeDef *            en_port,
                uint16_t                  en_pin,
                float                     v_sup,
                float                     drv_v_lim);
int  foc_init(void);
void driver_enable(void);
void driver_disable(void);

#endif /* BLDC_DRIVER_H */
