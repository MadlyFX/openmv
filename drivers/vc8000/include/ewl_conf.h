/**
  ******************************************************************************
  * @file    ewl_conf.h
  * @brief   Encoder Wrapper Layer configuration for OpenMV.
  ******************************************************************************
  */
#ifndef EWL_CONF_H
#define EWL_CONF_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define EWL_USE_MALLOC_MM       0
#define EWL_USE_FREERTOS_MM     1
#define EWL_USE_THREADX_MM      2
#define EWL_USE_STM32MPM_MM     3
#define EWL_USER_MM             4

#define EWL_ALLOC_API           EWL_USER_MM

#define EWL_USE_POLLING_SYNC    0
#define EWL_USE_FREERTOS_SYNC   1
#define EWL_USE_THREADX_SYNC    2
#define EWL_USER_SYNC           4

#define EWL_SYNC_API            EWL_USE_POLLING_SYNC

#define ALIGNMENT_INCR          8UL
#define MEM_CHUNKS              32

#ifdef USE_FULL_ASSERT
#ifndef assert_param
#define assert_param(expr) ((expr) ? (void) 0U : assert_failed((uint8_t *) __FILE__, __LINE__))
void assert_failed(uint8_t *file, uint32_t line);
#endif
#else
#ifndef assert_param
#define assert_param(expr) ((void) 0U)
#endif
#endif

#ifdef __cplusplus
}
#endif

#endif /* EWL_CONF_H */
