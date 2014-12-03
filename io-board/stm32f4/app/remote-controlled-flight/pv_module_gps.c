/**
  ******************************************************************************
  * @file    app/remote-controlled-flight/pv_module_gps.c
  * @author  Patrick Jose Pereira
  * @version V1.0.0
  * @date    27-August-2014
  * @brief   Implementação do módulo de leitura de dados do GPS.
  ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "pv_module_gps.h"

/** @addtogroup ProVANT_app
  * @{
  */

/** @addtogroup app_gps
  * \brief Módulo responsavel por tratar os dados do GPS.
  *
  * Definição do módulo de tratamento dos dados de GPS.
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define MODULE_PERIOD	   100//ms

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
portTickType lastWakeTime;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions definitions --------------------------------------------*/

/** \brief Inicializacao do módulo de GPS.
  *
  * Instancia as Queues de comunicação inter-thread.
  * @param  None
  * @retval None
  */
void module_gps_init() 
{
}

/** \brief Função principal do módulo de GPS.
  * @param  None
  * @retval None
  *
  */
void module_gps_run()
{
  while(1)
  {
    lastWakeTime = xTaskGetTickCount();
    vTaskDelayUntil( &lastWakeTime, (MODULE_PERIOD / portTICK_RATE_MS));
  }
}
/* IRQ handlers ------------------------------------------------------------- */

/**
  * @}
  */

/**
  * @}
  */