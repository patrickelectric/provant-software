/**
  ******************************************************************************
  * @file    app/remote-controlled-flight/pv_module_sm.c
  * @author  Patrick Jose Pereira
  * @version V1.0.0
  * @date    27-August-2014
  * @brief   Implementação do módulo da maquinas de estados do VANT.
  ******************************************************************************/

  /* Includes ------------------------------------------------------------------*/
#include "pv_module_sm.h"

/** @addtogroup ProVANT_app
  * @{
  */

/** @addtogroup app_sm
  * \brief Módulo responsavel pela maquina de estados do VANT.
  *
  * Definição do módulo da maquina de estados.
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define MODULE_PERIOD	   20//ms

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
portTickType lastWakeTime;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions definitions --------------------------------------------*/

/** \brief Inicializacao do módulo de sm.
  *
  * Instancia as Queues de comunicação inter-thread.
  * @param  None
  * @retval None
  */
void module_sm_init() 
{
}

/** \brief Função principal do módulo da sm.
  * @param  None
  * @retval None
  *
  */
void module_sm_run()
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
