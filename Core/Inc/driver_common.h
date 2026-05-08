/**
  ******************************************************************************
  * @file    driver_common.h
  * @brief   驱动层公用头文件：统一包含 HAL 基础类型，并提供可选别名/宏，供各外设驱动共享。
  * @note    各模块 *.c / *.h 在需要 HAL_StatusTypeDef 或驱动层约定类型时，优先包含本文件，
  *          避免在多个头文件中重复直连 HAL 细节。
  ******************************************************************************
  */

#ifndef DRIVER_COMMON_H
#define DRIVER_COMMON_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"

/**
  * @brief 驱动层统一返回类型（与 HAL_StatusTypeDef 等价，便于日后替换或映射）
  */
typedef HAL_StatusTypeDef Drv_StatusTypeDef;

/** @name 驱动层常用结果别名（与 HAL 枚举值一致） */
#define DRV_OK       HAL_OK
#define DRV_ERROR    HAL_ERROR
#define DRV_BUSY     HAL_BUSY
#define DRV_TIMEOUT  HAL_TIMEOUT

#ifdef __cplusplus
}
#endif

#endif /* DRIVER_COMMON_H */
