/**
  ******************************************************************************
  * @file    app/remote-controlled-flight/pv_module_do.c
  * @author  Patrick Jose Pereira
  * @version V1.0.0
  * @date    27-August-2014
  * @brief   Implementação do módulo de transmissao de dados para fora do ARM.
  ******************************************************************************/

      /* Includes ------------------------------------------------------------------*/
#include "pv_module_do.h"

/** @addtogroup ProVANT_app
  * @{
  */

/** @addtogroup app_do
  * \brief Módulo responsavel por transmitir dados.
  *
  * Definição do módulo de transmissão de dados.
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define MODULE_PERIOD	     100//ms
#define USART_BAUDRATE     460800
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
portTickType lastWakeTime;
unsigned int heartBeat=0;
pv_msg_input iInputData;
pv_msg_controlOutput iControlOutputData; 
//GPIOPin debugPin;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions definitions --------------------------------------------*/

/** \brief Inicializacao do módulo de data out.
  *
  * Instancia as Queues de comunicação inter-thread.
  * @param  None
  * @retval None
  */
void module_do_init() 
{
  /* Inicia a usart2 */
	c_common_usart2_init(USART_BAUDRATE);

  /* Reserva o local de memoria compartilhado */
	pv_interface_do.iInputData          = xQueueCreate(1, sizeof(pv_msg_input));
  pv_interface_do.iControlOutputData  = xQueueCreate(1, sizeof(pv_msg_controlOutput));

  /* Pin for debug */
  //debugPin = c_common_gpio_init(GPIOE, GPIO_Pin_13, GPIO_Mode_OUT);
}

/** \brief Função principal do módulo de data out.
  * @param  None
  * @retval None
  *
  */
void module_do_run()
{
	while(1)
	{
		lastWakeTime = xTaskGetTickCount();
		heartBeat++;

    /* toggle pin for debug */
    //c_common_gpio_toggle(debugPin);

		xQueueReceive(pv_interface_do.iInputData, &iInputData, 0);
    xQueueReceive(pv_interface_do.iControlOutputData, &iControlOutputData, 0);

		arm_scale_f32(iInputData.imuOutput.accRaw,RAD_TO_DEG,iInputData.imuOutput.accRaw,3);
		arm_scale_f32(iInputData.imuOutput.gyrRaw,RAD_TO_DEG,iInputData.imuOutput.gyrRaw,3);
    int channel[]={iInputData.receiverOutput.joystick[0],iInputData.receiverOutput.joystick[1],iInputData.receiverOutput.joystick[2],iInputData.receiverOutput.joystick[3],iInputData.receiverOutput.aButton,iInputData.receiverOutput.bButton,iInputData.receiverOutput.vrPot};

		c_common_datapr_multwii_raw_imu(iInputData.imuOutput.accRaw,iInputData.imuOutput.gyrRaw,iInputData.imuOutput.magRaw);
    c_common_datapr_multwii_attitude(iInputData.attitude.roll*RAD_TO_DEG,iInputData.attitude.pitch*RAD_TO_DEG,iInputData.attitude.yaw*RAD_TO_DEG);
		c_common_datapr_multwii2_rcNormalize(channel);
    c_common_datapr_multwii_altitude(iInputData.sonarOutput.altitude,0);
    c_common_datapr_multwii_debug(iInputData.heartBeat,iControlOutputData.heartBeat,1,heartBeat);
    c_common_datapr_multwii_sendstack(USART2);
  
    c_common_datapr_multwii2_sendControldatain(iControlOutputData.vantBehavior.rpy, iControlOutputData.vantBehavior.drpy, iControlOutputData.vantBehavior.xyz, iControlOutputData.vantBehavior.dxyz);
    c_common_datapr_multwii2_sendControldataout(iControlOutputData.actuation.servoPosition, iControlOutputData.actuation.escNewtons, iControlOutputData.actuation.escRpm);
    c_common_datapr_multwii_sendstack(USART2);

    /* toggle pin for debug */
    //c_common_gpio_toggle(debugPin);

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