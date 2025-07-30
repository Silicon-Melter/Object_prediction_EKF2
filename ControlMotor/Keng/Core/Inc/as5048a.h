/**
 * Core/Inc/as5048a.h
 *
 * @author Nonpawit Ekburanawat
 */

#ifndef AS5048A_H
#define AS5048A_H

#include "main.h"

void as5048a_init(const SPI_HandleTypeDef *hspi,
                  GPIO_TypeDef *           spi_cs_port,
                  uint16_t                 spi_cs_pin);

float get_angle(void);
float get_vel_foc(float dt);
float get_vel(void);

#endif /* AS5048A_H */
