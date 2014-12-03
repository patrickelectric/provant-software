/**
  ******************************************************************************
  * @file    modules/common/pv_typedefs.h
  * @author  Martin Vincent Bloedorn
  * @version V1.0.0
  * @date    10-February-2014
  * @brief   Definições de tipos e estruturas de mensagens para o projeto.
  *
  *****************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef PV_TYPEDEFS_H
#define PV_TYPEDEFS_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_conf.h"

#ifdef __cplusplus
 extern "C" {
#endif

 /** @addtogroup Common_Components
   * @{
   */

 /** @addtogroup PV_Typedefs
   *
   * Definições de tipagem e estruras de mensagem para o projeto.
   * @{
   */

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/** \brief Um par de floats. */
typedef struct
{
	float accRaw[3];
  float gyrRaw[3];
  float magRaw[3];
  float temp;
  unsigned int  sampleTime;
} pv_type_imuOutput;

/** \brief Informações dos dados do controle remoto.*/
typedef struct 
{
  int  joystick[4];
  unsigned int vrPot;
  bool aButton;
  bool bButton;
  unsigned int  sampleTime;
} pv_type_receiverOutput;

/** \brief Informações do sonar*/
typedef struct 
{
  float altitude;
  unsigned int  sampleTime;
} pv_type_sonarOutput;

/** \brief Tipo do ESC com informações.*/
typedef struct 
{
	char  ID;
	float angularSpeed;
	float current;
	float voltage;
  float rpm;
  unsigned int  sampleTime;
} pv_type_escOutput;

/** \brief Dados do servo*/
typedef struct 
{
  char  ID;
  float angularSpeed;
  float angle;
  float torque;
  float rpm;
  unsigned int  sampleTime;
} pv_type_servoOutput;

/** \brief Estrutura para orientação do VANT.*/
typedef struct
{
  float roll, pitch, yaw;
  float dotRoll, dotPitch, dotYaw;
} pv_type_datapr_attitude;

/** \brief Estrutura para dados de atuação.*/
typedef struct
{
  float servoTorque[2];
  float servoPosition[2];
  float escNewtons[2];
  float escRpm[2];
} pv_type_actuation;

/** \brief Estrutura para dados de comportamento.*/
typedef struct
{
  float rpy[3];
  float drpy[3];
  float xyz[3];
  float dxyz[3];
} pv_type_vantBehavior;

/** \brief Estrutura para dados de comportamento.*/
typedef struct
{
  int timeStamp;
  bool validity;
  float lat;
  bool latDirection;
  float lon;
  bool lonDirection;
  float speedOverGround;
  float trueCourse;
  int dateStamp;
  float variation;
  bool variationDirection;
  char ModeIndicator;
  unsigned int sampleTime;
} pv_type_gpsOutput;

/* Exported messages ---------------------------------------------------------*/

/** \brief Estrutura de mensagens para os dois RX24f e ESCs.
 *
 * Pacote prevê passagem de referências de velocidade para os ESCs em <b>rad/s</b>,
 * referências agulares em <b>rad</b> e torque em <b>N.m</b>. Para o servo, o módulo de
 * IO automaticamente alternará entre referências de ângulo para torque se a flag
 *  \b servoTorqueControlEnable estiver setada.
 */

/** \brief Estrutura de mensagem de saida da estrutura thread input.*/
typedef struct
{
  pv_type_imuOutput       imuOutput;
  pv_type_receiverOutput  receiverOutput;
  pv_type_sonarOutput     sonarOutput;
  pv_type_escOutput       escOutput;
  pv_type_servoOutput     servoOutput;
  pv_type_datapr_attitude attitude;
  unsigned int cicleTime;
  unsigned int heartBeat;
} pv_msg_input;

/** \brief Estrutura de mensagem de saida da estrutura thread de controle.*/
typedef struct
{
  pv_type_actuation    actuation;
  pv_type_vantBehavior vantBehavior;
  unsigned int cicleTime;
  unsigned int heartBeat;
} pv_msg_controlOutput;

/** \brief Estrutura de mensagem de saida da estrutura thread de gps.*/
typedef struct
{
  pv_type_gpsOutput gpsOutput;
  unsigned int cicleTime;
  unsigned int heartBeat;
} pv_msg_gps;

/** \brief Estrutura de mensagem de saida da estrutura thread de state machine.*/
typedef struct
{
  unsigned int cicleTime;
  unsigned int heartBeat;
} pv_msg_sm;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
/* Header-defined wrapper functions ----------------------------------------- */

 /**
   * @}
   */
 /**
   * @}
   */

#ifdef __cplusplus
}
#endif

#endif //PV_TYPEDEFS_H

