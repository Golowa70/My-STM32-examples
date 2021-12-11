#ifndef _W25QXXCONFIG_H
#define _W25QXXCONFIG_H

#include "main.h"

#define _W25QXX_SPI                   hspi1
extern SPI_HandleTypeDef _W25QXX_SPI;

#define _W25QXX_CS_GPIO               W25Q_CS_GPIO_Port
#define _W25QXX_CS_PIN                W25Q_CS_Pin
#define _W25QXX_USE_FREERTOS          0
#define _W25QXX_DEBUG                 0

#endif
