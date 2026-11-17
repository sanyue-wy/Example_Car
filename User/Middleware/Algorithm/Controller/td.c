/**
 * @file    td.c
 * @brief   跟踪微分器(实现层)
 * @version 1.0
 * @date    2026-09-25
 */

/* Includes ------------------------------------------------------------------*/
#include "td.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Exported functions ---------------------------------------------------------*/

/**
  * @brief  跟踪微分器初始化
  */
void TD_Init(TD_t *td, float r, float control_period)
{
  if (td == NULL)
  {
    return;
  }

  *td = (TD_t){0};

  td->R = (r > 0.0f) ? r : 1.0f;
  td->ControlPeriod = (control_period > 0.0f) ? control_period : 0.001f;
}

/**
  * @brief  复位跟踪微分器运行态
  */
void TD_Clear(TD_t *td)
{
  if (td == NULL)
  {
    return;
  }
  td->Input   = 0.0f;
  td->X       = 0.0f;
  td->DX      = 0.0f;
  td->DDX     = 0.0f;
}
