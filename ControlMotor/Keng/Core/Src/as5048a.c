/**
 * Core/Src/as5048a.c
 *
 * Ported from https://github.com/raimapo/Magnetic_encoder_AS5048A/tree/master
 *
 * @author Nonpawit Ekburanawat
 */

#include "as5048a.h"
#include "foc_utils.h"
#include "main.h"

/* Private Variables ********************************************************/

const uint16_t AS5048A_CLEAR_ERROR_FLAG           = 0x0001;
const uint16_t AS5048A_PROGRAMMING_CONTROL        = 0x0003;
const uint16_t AS5048A_OTP_REGISTER_ZERO_POS_HIGH = 0x0016;
const uint16_t AS5048A_OTP_REGISTER_ZERO_POS_LOW  = 0x0017;
const uint16_t AS5048A_DIAG_AGC                   = 0x3FFD;
const uint16_t AS5048A_MAGNITUDE                  = 0x3FFE;
const uint16_t AS5048A_ANGLE                      = 0x3FFF;

GPIO_TypeDef *cs_port;
uint16_t      cs_pin;

SPI_HandleTypeDef ss_hspi;

uint8_t  err_flg = 0;
uint16_t pos;

float zero_angle = 0;

float vel = 0;

/* Private Macros ***********************************************************/

#define EN_SPI HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
#define DIS_SPI HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);

/* Private Function Declarations ********************************************/

uint8_t  spi_calc_event_parity(uint16_t value);
uint16_t read(uint16_t reg_addr);
uint16_t write(uint16_t reg_addr, uint16_t data);

/* Private Functions ********************************************************/

uint8_t spi_calc_event_parity(uint16_t value)
{
  uint8_t cnt = 0;

  for (uint8_t i = 0; i < 16; i++)
  {
    if (value & 0x1)
    {
      cnt++;
    }
    value >>= 1;
  }

  return cnt & 0x1;
}

uint16_t read(const uint16_t reg_addr)
{
  uint8_t data[2];

  uint16_t command = 0b0100000000000000;
  command          = command | reg_addr;

  command |= (uint16_t)spi_calc_event_parity(command) << 15;

  data[1] = command & 0xFF;
  data[0] = command >> 8 & 0xFF;

  EN_SPI;
  HAL_SPI_Transmit(&ss_hspi, (uint8_t *)&data, 2, 0xFFFF);
  DIS_SPI;

  EN_SPI;
  HAL_SPI_Receive(&ss_hspi, (uint8_t *)&data, 2, 0xFFFF);
  DIS_SPI;

  if (data[1] & 0x40)
  {
    err_flg = 1;
  }
  else
  {
    err_flg = 0;
  }

  return ((data[1] & 0xFF) << 8 | data[0] & 0xFF) & ~0xC000;
}

uint16_t write(const uint16_t reg_addr, const uint16_t data)
{
  uint8_t dat[2];

  uint16_t command = 0b0000000000000000;
  command |= reg_addr;

  command |= (uint16_t)spi_calc_event_parity(command) << 15;

  dat[1] = command & 0xFF;
  dat[0] = command >> 8 & 0xFF;

  EN_SPI;
  HAL_SPI_Transmit(&ss_hspi, (uint8_t *)&dat, 2, 0xFFFF);
  DIS_SPI;

  dat[1] = command & 0xFF;
  dat[0] = command >> 8 & 0xFF;

  EN_SPI;
  HAL_SPI_Transmit(&ss_hspi, (uint8_t *)&dat, 2, 0xFFFF);
  DIS_SPI;

  dat[1] = 0x00;
  dat[0] = 0x00;

  EN_SPI;
  HAL_SPI_Transmit(&ss_hspi, (uint8_t *)&dat, 2, 0xFFFF);
  HAL_SPI_Receive(&ss_hspi, (uint8_t *)&dat, 2, 0xFFFF);
  DIS_SPI;

  return ((dat[1] & 0xFF) << 8 | dat[0] & 0xFF) & ~0xC000;
}

/* Public Functions *********************************************************/

void as5048a_init(const SPI_HandleTypeDef *hspi,
                  GPIO_TypeDef *           spi_cs_port,
                  const uint16_t           spi_cs_pin)
{
  ss_hspi = *hspi;
  cs_port = spi_cs_port;
  cs_pin  = spi_cs_pin;
}

float get_angle(void)
{
  const float current_angle = (float)read(AS5048A_ANGLE) * ((float)360 /
    16383);

  return current_angle * _PI / 180.0f;
}

float get_vel_foc(const float dt)
{
  const float angle = get_angle();

  float diff = angle - zero_angle;

  if (diff > _PI)
  {
    diff -= _PI;
  }
  else if (diff < -_PI)
  {
    diff += _PI;
  }

  vel = diff / dt;

  zero_angle = angle;

  return vel;
}

float get_vel(void)
{
  return vel;
}
