/**
 * Core/Src/bldc_driver.h
 *
 * Ported from https://github.com/simplefoc/Arduino-FOC/tree/master
 *
 * @author Nonpawit Ekburanawat
 */

/* Includes *****************************************************************/

#include "bldc_driver.h"
#include "foc_utils.h"

/* Public Variables *********************************************************/

float voltage_power_supply = 0.0f;
float driver_voltage_limit = 0.0f;

/* Private Variables ********************************************************/

float dc_a = 0.0f;
float dc_b = 0.0f;
float dc_c = 0.0f;

/* PWM timer handler. */

TIM_HandleTypeDef foc_htim1;
TIM_HandleTypeDef foc_htim2;
TIM_HandleTypeDef foc_htim3;

/* PWM phrases timer channels. */

uint32_t ph_a_tim;
uint32_t ph_b_tim;
uint32_t ph_c_tim;

/* Enable pin. */

GPIO_TypeDef *foc_en_port;
uint16_t      foc_en_pin;

/* Public Functions *********************************************************/

void set_pwm(float ua, float ub, float uc)
{
  /* Limit the voltage in the driver. */

  ua = _constrain(ua, 0.0f, driver_voltage_limit);
  ub = _constrain(ub, 0.0f, driver_voltage_limit);
  uc = _constrain(uc, 0.0f, driver_voltage_limit);

  /* Calculate duty cycle. */

  dc_a = _constrain(ua / voltage_power_supply, 0.0f, 1.0f);
  dc_b = _constrain(ub / voltage_power_supply, 0.0f, 1.0f);
  dc_c = _constrain(uc / voltage_power_supply, 0.0f, 1.0f);

  /* PWM_A */

  uint32_t pwm_range = foc_htim1.Instance->ARR;

  __HAL_TIM_SET_COMPARE(&foc_htim1, ph_a_tim, dc_a * pwm_range);

  /* PWM_B */

  pwm_range = foc_htim2.Instance->ARR;

  __HAL_TIM_SET_COMPARE(&foc_htim2, ph_b_tim, dc_b * pwm_range);

  /* PWM_C */

  pwm_range = foc_htim3.Instance->ARR;

  __HAL_TIM_SET_COMPARE(&foc_htim3, ph_c_tim, dc_c * pwm_range);
}

int driver_init(const TIM_HandleTypeDef *_htim1,
                const uint32_t           ph_a_ch,
                const TIM_HandleTypeDef *_htim2,
                const uint32_t           ph_b_ch,
                const TIM_HandleTypeDef *_htim3,
                const uint32_t           ph_c_ch,
                GPIO_TypeDef *           en_port,
                const uint16_t           en_pin,
                const float              v_sup,
                const float              drv_v_lim)
{
  foc_htim1            = *_htim1;
  foc_htim2            = *_htim2;
  foc_htim3            = *_htim3;
  voltage_power_supply = v_sup;
  driver_voltage_limit = drv_v_lim > v_sup ? v_sup : drv_v_lim;

  /* Store configured ports. */

  ph_a_tim = ph_a_ch;
  ph_b_tim = ph_b_ch;
  ph_c_tim = ph_c_ch;

  foc_en_port = en_port;
  foc_en_pin  = en_pin;

  HAL_TIM_PWM_Start(&foc_htim1, ph_a_tim);
  HAL_TIM_PWM_Start(&foc_htim2, ph_b_tim);
  HAL_TIM_PWM_Start(&foc_htim3, ph_c_tim);

  return 1;
}

void driver_enable(void)
{
  HAL_GPIO_WritePin(foc_en_port, foc_en_pin, GPIO_PIN_SET);

  set_pwm(0, 0, 0);
}

void driver_disable(void)
{
  set_pwm(0, 0, 0);

  HAL_GPIO_WritePin(foc_en_port, foc_en_pin, GPIO_PIN_RESET);
}
