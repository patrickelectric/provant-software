/**
  ******************************************************************************
  * @file    modules/io/pv_module_io.c
  * @author  Martin Vincent Bloedorn
  * @version V1.0.0
  * @date    02-Dezember-2013
  * @brief   Implementação do módulo de gerenciamento de sensores.
  ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "pv_module_in.h"

/** @addtogroup ProVANT_app
  * @{
  */

/** @addtogroup app_in
  * \brief Componentes para o sensoriamento do VANT.
  *
  * Reunião de todos os componentes relacionados às operações de input do VANT.
  * Leituras de todos os sensores. O processamento destes
  * dados brutos é feito neste módulo.
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define MODULE_PERIOD	   10//ms

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
portTickType lastWakeTime;
pv_msg_input oInputData;
//GPIOPin debugPin;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions definitions --------------------------------------------*/

/** \brief Inicializacao componentes de IO.
  *
  * Incializa o hardware para comunicar com os sensores. Rotinas de teste
  * ainda precisam ser executadas.
  * @param  None
  * @retval None
  */
void module_in_init() 
{
	/* Inicialização da imu */
	c_common_i2c_init(I2C1); 
	c_io_imu_init(I2C1); 

  /* Inicializador do sonar */
  c_io_sonar_init();

  /* Inicializador do receiver */
	c_rc_receiver_init();

  /* Pin for debug */
  //debugPin = c_common_gpio_init(GPIOE, GPIO_Pin_13, GPIO_Mode_OUT);

  /* Resevar o espaco para a variavel compartilhada */
	pv_interface_in.oInputData  = xQueueCreate(1, sizeof(pv_msg_input));
}

/** \brief Função principal do módulo de IO.
  * @param  None
  * @retval None
  *
  * Loop que amostra sensores como necessário.
  *
  */
void module_in_run() 
{
  unsigned int heartBeat=0;
  float rpy[3]={};

	while(1)
	{
    oInputData.heartBeat=heartBeat+=1;

    /* toggle pin for debug */
    //c_common_gpio_toggle(debugPin);

    /* Leitura do numero de ciclos atuais */
		lastWakeTime = xTaskGetTickCount();

    /* Pega e trata os valores da imu */
		c_io_imu_getRaw(oInputData.imuOutput.accRaw, oInputData.imuOutput.gyrRaw, oInputData.imuOutput.magRaw);
    oInputData.imuOutput.sampleTime =xTaskGetTickCount() -lastWakeTime;
    c_io_imu_ComplimentaryRPY(rpy,oInputData.imuOutput.accRaw,oInputData.imuOutput.gyrRaw,oInputData.imuOutput.magRaw);
    oInputData.attitude.roll  =rpy[0];
    oInputData.attitude.pitch =rpy[1];
    oInputData.attitude.yaw   =rpy[2];

    /* Realiza a laitura dos canais do radio-controle */
		oInputData.receiverOutput.joystick[0]=c_rc_receiver_getChannel(C_RC_CHANNEL_THROTTLE);
		oInputData.receiverOutput.joystick[1]=c_rc_receiver_getChannel(C_RC_CHANNEL_PITCH);
		oInputData.receiverOutput.joystick[2]=c_rc_receiver_getChannel(C_RC_CHANNEL_ROLL);
		oInputData.receiverOutput.joystick[3]=c_rc_receiver_getChannel(C_RC_CHANNEL_YAW);
		oInputData.receiverOutput.vrPot		   =c_rc_receiver_getChannel(C_RC_CHANNEL_A);
		oInputData.receiverOutput.aButton	   =c_rc_receiver_getChannel(C_RC_CHANNEL_VR);
    oInputData.receiverOutput.sampleTime =xTaskGetTickCount();

    /* Executra a leitura do sonar */
		oInputData.sonarOutput.altitude      =c_io_sonar_read();
    oInputData.sonarOutput.sampleTime    =xTaskGetTickCount() - lastWakeTime;

    oInputData.cicleTime                 =xTaskGetTickCount() - lastWakeTime;

    /* toggle pin for debug */
    //c_common_gpio_toggle(debugPin);

    /* Realiza o trabalho de mutex */
		if(pv_interface_in.oInputData != 0)
      xQueueOverwrite(pv_interface_in.oInputData, &oInputData);

    /* A thread dorme ate o tempo final ser atingido */
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



