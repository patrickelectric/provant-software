/**
  ******************************************************************************
  * @file    modules/io/c_io_sonar.h
  * @author  Patrick Jose Pereira
  * @version V1.0.0
  * @date    11-fevereiro-2014
  * @brief   Implementação da leitura do sonar XL-MaxSonar-EZ MB1200.
  *****************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef C_IO_SONAR_H
#define C_IO_SONAR_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_conf.h"

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
void c_io_sonar_init();
int  c_io_sonar_read();

#ifdef __cplusplus
}
#endif

#endif //C_IO_BLCTRL20_H
