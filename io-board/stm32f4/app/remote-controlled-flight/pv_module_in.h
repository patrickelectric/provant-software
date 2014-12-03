/**
  ******************************************************************************
  * @file    modules/io/pv_module_io.h
  * @author  Martin Vincent Bloedorn
  * @version V1.0.0
  * @date    02-Dezember-2013
  * @brief   Implementação do módulo de gerenciamento de sensores.
  *
  * TODO
  *
  *****************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef PV_MODULE_IN_H
#define PV_MODULE_IN_H

#ifdef __cplusplus
 extern "C" {
#endif

 /** \brief Define que modulos serao utilizados.
    *
    * Se ENABLE_* 1 então tal modulo será utilizado
    *
    */
#define ENABLE_IMU		1
#define ENABLE_SERVO	0
#define ENABLE_ESC		0
#define ENABLE_SONAR	0
#define ENALBE_DEBUG	1

/* Includes ------------------------------------------------------------------*/

/* FreeRTOS kernel includes */
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

/* proVANT includes */
#include "c_common_gpio.h"
#include "c_common_i2c.h"
#include "c_common_uart.h"
#include "c_common_utils.h"

#include "c_io_blctrl.h"
#include "c_io_rx24f.h"
#include "c_io_imu.h"
#include "c_io_sonar.h"
#include "c_rc_receiver.h"

#include "pv_typedefs.h"
#include "c_datapr_MultWii.h"
#include "c_datapr_MahonyAHRS.h"

/* Exported types ------------------------------------------------------------*/
struct pv_interface_in
{
  xQueueHandle oInputData;  
} pv_interface_in;
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
void module_in_init();
void module_in_run();

#ifdef __cplusplus
}
#endif

#endif //PV_MODULE_IO_H
