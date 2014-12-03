/**
  ******************************************************************************
  * @file    modules/rc/pv_module_rc.h
  * @author  Martin Vincent Bloedorn
  * @version V1.0.0
  * @date    30-November-2013
  * @brief   Definição do módulo de controle + output.
  *****************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef PV_MODULE_CO_H
#define PV_MODULE_CO_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* FreeRTOS kernel includes */
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

/* proVANT includes */
#include "c_common_gpio.h"
#include "c_common_i2c.h"
#include "c_rc_control.h"
#include "c_io_blctrl.h"
#include "c_io_rx24f.h"

#include "pv_typedefs.h"

/* Exported types ------------------------------------------------------------*/
struct pv_interface_co 
{
  xQueueHandle iInputData;  
  xQueueHandle oControlOutputData;  
} pv_interface_co;
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
void module_co_init();
void module_co_run();

#ifdef __cplusplus
}
#endif

#endif //PV_MODULE_CO_H
