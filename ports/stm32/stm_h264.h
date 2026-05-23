/*
 * SPDX-License-Identifier: MIT
 *
 * Copyright (C) 2026 OpenMV, LLC.
 *
 * STM32N6 H.264 encoder wrapper.
 */
#ifndef __STM_H264_H__
#define __STM_H264_H__

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "imlib.h"

typedef enum {
    STM_H264_OK = 0,
    STM_H264_ERROR = -1,
    STM_H264_UNSUPPORTED = -2,
    STM_H264_OVERFLOW = -3,
    STM_H264_MEMORY = -4,
    STM_H264_INVALID_ARGUMENT = -5,
    STM_H264_NULL_ARGUMENT = -6,
    STM_H264_EWL = -7,
    STM_H264_INVALID_STATUS = -8,
    STM_H264_HW_BUS = -9,
    STM_H264_HW_DATA = -10,
    STM_H264_HW_TIMEOUT = -11,
    STM_H264_HW_RESERVED = -12,
    STM_H264_SYSTEM = -13,
    STM_H264_INSTANCE = -14,
    STM_H264_HRD = -15,
    STM_H264_HW_RESET = -16,
    STM_H264_FUSE = -17,
    STM_H264_BUSY = -18,
    STM_H264_RATE_UNSUPPORTED = -19,
} stm_h264_error_t;

typedef struct {
    int width;
    int height;
    int fps;
    int bitrate;
    int gop;
    int qp;
    pixformat_t pixformat;
} stm_h264_config_t;

typedef struct {
    const void *inst;
    uint8_t *input_scratch;
    size_t input_scratch_size;
    uint32_t frames;
    uint32_t gop;
    bool use_input_scratch;
    bool started;
    bool hw_initialized;
} stm_h264_t;

int stm_h264_init(stm_h264_t *ctx, const stm_h264_config_t *config);
int stm_h264_encode(stm_h264_t *ctx, image_t *src,
                    uint8_t *out, size_t out_size,
                    size_t *out_len, bool force_idr);
int stm_h264_end(stm_h264_t *ctx, uint8_t *out, size_t out_size, size_t *out_len);
void stm_h264_deinit(stm_h264_t *ctx);
const char *stm_h264_strerror(int error);

#endif /* __STM_H264_H__ */
