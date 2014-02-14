/**
  ******************************************************************************
  * @file    modules/rc/pv_module_rc.c
  * @author  Martin Vincent Bloedorn
  * @version V1.0.0
  * @date    30-November-2013
  * @brief   ...
  ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "pv_module_rc.h"

/** @addtogroup ProVANT_Modules
  * @{
  */

/** @addtogroup Module_RC
  * \brief Módulo com as principais funcionalidades para operação em modo rádio controlado.
  *
  * Definição do módulo de controle e comunicação via rádio manual.
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define MODULE_PERIOD	   10//ms

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
pv_msg_io_actuation actuation;
pv_type_receiverChannels receiver;
portTickType lastWakeTime;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions definitions --------------------------------------------*/

/** \brief Inicializacao do módulo de RC.
  *
  * Instancia as Queues de comunicação inter-thread, inicializa a pinagem necessária para
  * o controle remoto e aloca o que for necessário para as equações de controle.
  * @param  None
  * @retval None
  */
void module_rc_init() {
	c_rc_receiver_init();

	pv_interface_rc.iAttitude = xQueueCreate(1, sizeof(pv_msg_datapr_attitude));

	actuation.servoTorqueControlEnable = 0;
	actuation.escLeftSpeed  = 0.0;
	actuation.escRightSpeed = 0.0;
	actuation.servoLeft	    = 0.0;
	actuation.servoRight    = 0.0;
}

/** \brief Função principal do módulo de RC.
  * @param  None
  * @retval None
  *
  * Interpreta o recebimento de PPM, calcula sinais de controle e os envia
  * via interface.
  */
void module_rc_run() {
	while(1) {
		lastWakeTime = xTaskGetTickCount();
		/*
		//get radio-control channel values
		for(int i=0; i<8; i++)
			receiver.channel[i] = c_rc_receiver_getChannel(i);

		actuation.escLeftSpeed  = 20.0;
		actuation.escRightSpeed = 20.0;

		//send actuation values
		if(pv_interface_rc.oActuation != 0)
			xQueueOverwrite(pv_interface_rc.oActuation, &actuation);
		*/
        vTaskDelayUntil( &lastWakeTime, MODULE_PERIOD / portTICK_RATE_MS);
	}
}
/* IRQ handlers ------------------------------------------------------------- */

/**
  * @}
  */

/**
  * @}
  */
