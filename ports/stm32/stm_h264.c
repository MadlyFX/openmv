/*
 * SPDX-License-Identifier: MIT
 *
 * Copyright (C) 2026 OpenMV, LLC.
 *
 * STM32N6 H.264 encoder wrapper.
 */
#include "stm_h264.h"

#if defined(OMV_VC8000_ENABLE) && (OMV_VC8000_ENABLE == 1)

#include <string.h>
#undef MIN
#undef MAX
#include "h264encapi.h"
#include "ewl.h"
#include "omv_common.h"
#include "stm32n6xx_hal.h"
#include "stm32n6xx_ll_venc.h"
#include "umalloc.h"

#define STM_H264_MIN_BITRATE        (10000)
#define STM_H264_MAX_BITRATE        (60000000)
#define STM_H264_DEFAULT_QP         (25)
#define STM_H264_MIN_QP             (10)
#define STM_H264_MIN_USER_QP        (0)
#define STM_H264_MAX_QP             (51)
#define STM_H264_RGB888_BYTES_PER_PIXEL (4)
#define STM_H264_MAX_MB_PER_SECOND  (8160U * 15U)
#define STM_H264_MB_SIZE            (16U)

static bool stm_h264_active = false;

static void stm_h264_cache_clean(const void *addr, size_t size) {
    #if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1)
    if (addr && size) {
        uintptr_t start = OMV_ALIGN_DOWN((uintptr_t) addr, OMV_CACHE_LINE_SIZE);
        uintptr_t end = OMV_ALIGN_TO((uintptr_t) addr + size, OMV_CACHE_LINE_SIZE);
        SCB_CleanDCache_by_Addr((uint32_t *) start, end - start);
    }
    #else
    (void) addr;
    (void) size;
    #endif
}

static void stm_h264_cache_invalidate(const void *addr, size_t size) {
    #if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1)
    if (addr && size) {
        uintptr_t start = OMV_ALIGN_DOWN((uintptr_t) addr, OMV_CACHE_LINE_SIZE);
        uintptr_t end = OMV_ALIGN_TO((uintptr_t) addr + size, OMV_CACHE_LINE_SIZE);
        SCB_InvalidateDCache_by_Addr((uint32_t *) start, end - start);
    }
    #else
    (void) addr;
    (void) size;
    #endif
}

static void stm_h264_cache_clean_invalidate(const void *addr, size_t size) {
    #if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1)
    if (addr && size) {
        uintptr_t start = OMV_ALIGN_DOWN((uintptr_t) addr, OMV_CACHE_LINE_SIZE);
        uintptr_t end = OMV_ALIGN_TO((uintptr_t) addr + size, OMV_CACHE_LINE_SIZE);
        SCB_CleanInvalidateDCache_by_Addr((uint32_t *) start, end - start);
    }
    #else
    (void) addr;
    (void) size;
    #endif
}

static int stm_h264_ret_to_error(H264EncRet ret) {
    if ((ret == H264ENC_OK) || (ret == H264ENC_FRAME_READY)) {
        return STM_H264_OK;
    }

    switch (ret) {
        case H264ENC_ERROR:
            return STM_H264_ERROR;
        case H264ENC_NULL_ARGUMENT:
            return STM_H264_NULL_ARGUMENT;
        case H264ENC_INVALID_ARGUMENT:
            return STM_H264_INVALID_ARGUMENT;
        case H264ENC_MEMORY_ERROR:
        case H264ENC_EWL_MEMORY_ERROR:
            return STM_H264_MEMORY;
        case H264ENC_EWL_ERROR:
            return STM_H264_EWL;
        case H264ENC_INVALID_STATUS:
            return STM_H264_INVALID_STATUS;
        case H264ENC_OUTPUT_BUFFER_OVERFLOW:
            return STM_H264_OVERFLOW;
        case H264ENC_HW_BUS_ERROR:
            return STM_H264_HW_BUS;
        case H264ENC_HW_DATA_ERROR:
            return STM_H264_HW_DATA;
        case H264ENC_HW_TIMEOUT:
            return STM_H264_HW_TIMEOUT;
        case H264ENC_HW_RESERVED:
            return STM_H264_HW_RESERVED;
        case H264ENC_SYSTEM_ERROR:
            return STM_H264_SYSTEM;
        case H264ENC_INSTANCE_ERROR:
            return STM_H264_INSTANCE;
        case H264ENC_HRD_ERROR:
            return STM_H264_HRD;
        case H264ENC_HW_RESET:
            return STM_H264_HW_RESET;
        case H264ENC_FUSE_ERROR:
            return STM_H264_FUSE;
        default:
            return STM_H264_ERROR;
    }
}

static int stm_h264_picture_type(pixformat_t pixformat, H264EncPictureType *type) {
    switch (pixformat) {
        case PIXFORMAT_GRAYSCALE:
        case PIXFORMAT_RGB565:
        case PIXFORMAT_YUV_ANY:
            // ST's N6 H.264 reference path uses RGB888 input; expand captures into that mode.
            *type = H264ENC_RGB888;
            return STM_H264_OK;
        default:
            return STM_H264_UNSUPPORTED;
    }
}

static uint32_t stm_h264_macroblocks(int width, int height) {
    uint32_t mb_width = ((uint32_t) width + STM_H264_MB_SIZE - 1U) / STM_H264_MB_SIZE;
    uint32_t mb_height = ((uint32_t) height + STM_H264_MB_SIZE - 1U) / STM_H264_MB_SIZE;
    return mb_width * mb_height;
}

static int stm_h264_append_padding(uint8_t *out, size_t out_size, size_t *out_len) {
    uintptr_t out_addr = (uintptr_t) out;
    int pad_size = 8 - (out_addr % 8);
    int pad_len = 0;

    *out_len = 0;

    if ((out_addr % 8) == 0) {
        return STM_H264_OK;
    }

    if (pad_size < 6) {
        pad_size += 8;
    }

    if ((size_t) pad_size > out_size) {
        return STM_H264_OVERFLOW;
    }

    out[pad_len++] = 0x00;
    out[pad_len++] = 0x00;
    out[pad_len++] = 0x00;
    out[pad_len++] = 0x01;
    out[pad_len++] = 0x2c;
    pad_size -= 5;

    while (pad_size--) {
        out[pad_len++] = 0xff;
    }

    *out_len = pad_len;
    return STM_H264_OK;
}

static int stm_h264_stream_start(stm_h264_t *ctx, uint8_t *out, size_t out_size, size_t *out_len) {
    H264EncIn enc_in = { 0 };
    H264EncOut enc_out = { 0 };
    H264EncRet ret;
    size_t pad_len = 0;

    enc_in.pOutBuf = (u32 *) out;
    enc_in.busOutBuf = (ptr_t) out;
    enc_in.outBufSize = out_size;

    stm_h264_cache_clean_invalidate(out, out_size);
    ret = H264EncStrmStart(ctx->inst, &enc_in, &enc_out);
    if (ret != H264ENC_OK) {
        return stm_h264_ret_to_error(ret);
    }

    int error = stm_h264_append_padding(out + enc_out.streamSize,
                                        out_size - enc_out.streamSize,
                                        &pad_len);
    if (error != STM_H264_OK) {
        return error;
    }

    *out_len = enc_out.streamSize + pad_len;
    ctx->started = true;
    return STM_H264_OK;
}

static int stm_h264_alloc_input_scratch(stm_h264_t *ctx, const stm_h264_config_t *config) {
    ctx->input_scratch_size =
        OMV_ALIGN_TO((size_t) config->width * (size_t) config->height * STM_H264_RGB888_BYTES_PER_PIXEL,
                     OMV_CACHE_LINE_SIZE);
    ctx->input_scratch = uma_malign(ctx->input_scratch_size, OMV_CACHE_LINE_SIZE,
                                    UMA_PERSIST | UMA_CACHE | UMA_MAYBE);
    if (!ctx->input_scratch) {
        ctx->input_scratch_size = 0;
        return STM_H264_MEMORY;
    }

    ctx->use_input_scratch = true;
    return STM_H264_OK;
}

static int stm_h264_pack_luma_rgb888(const image_t *src, uint8_t *dst) {
    uint32_t *rgb = (uint32_t *) dst;
    size_t pixels = (size_t) src->w * (size_t) src->h;

    switch (src->pixfmt) {
        case PIXFORMAT_GRAYSCALE: {
            const uint8_t *gray = src->data;
            for (size_t i = 0; i < pixels; i++) {
                *rgb++ = COLOR_Y_TO_RGB888(gray[i]);
            }
            return STM_H264_OK;
        }
        case PIXFORMAT_RGB565: {
            const uint16_t *rgb565 = (const uint16_t *) src->data;
            for (size_t i = 0; i < pixels; i++) {
                uint16_t pixel = rgb565[i];
                uint32_t y = COLOR_RGB565_TO_Y(pixel);
                *rgb++ = COLOR_Y_TO_RGB888(y);
            }
            return STM_H264_OK;
        }
        case PIXFORMAT_YUV_ANY: {
            const uint16_t *yuv = (const uint16_t *) src->data;
            for (size_t i = 0; i < pixels; i++) {
                *rgb++ = COLOR_Y_TO_RGB888(yuv[i] & 0xff);
            }
            return STM_H264_OK;
        }
        default:
            return STM_H264_UNSUPPORTED;
    }
}

static void stm_h264_setup_rate_control(H264EncRateCtrl *rate, int bitrate, int gop, int qp) {
    rate->pictureSkip = 0;
    rate->hrd = 0;
    rate->bitPerSecond = bitrate;
    rate->gopLen = gop;
    rate->intraQpDelta = 0;
    rate->fixedIntraQp = 0;
    rate->mbQpAdjustment = 0;
    rate->mbQpAutoBoost = 0;

    if (qp >= STM_H264_MIN_USER_QP) {
        rate->pictureRc = 0;
        rate->mbRc = 0;
        rate->qpHdr = qp;
        rate->qpMin = qp;
        rate->qpMax = qp;
    } else {
        rate->pictureRc = 1;
        rate->mbRc = 1;
        rate->qpHdr = STM_H264_DEFAULT_QP;
        rate->qpMin = STM_H264_MIN_QP;
        rate->qpMax = STM_H264_MAX_QP;
    }
}

int stm_h264_init(stm_h264_t *ctx, const stm_h264_config_t *config) {
    H264EncPictureType input_type;
    H264EncConfig enc_config;
    H264EncPreProcessingCfg preproc;
    H264EncCodingCtrl coding;
    H264EncRateCtrl rate;
    H264EncRet ret;

    if (!ctx || !config) {
        return STM_H264_INVALID_ARGUMENT;
    }

    if (stm_h264_active) {
        return STM_H264_BUSY;
    }

    if ((config->width <= 0) || (config->height <= 0) ||
        (config->fps <= 0) || (config->fps > 300) ||
        (config->qp < -1) || (config->qp > STM_H264_MAX_QP) ||
        (config->bitrate < STM_H264_MIN_BITRATE) ||
        (config->bitrate > STM_H264_MAX_BITRATE) ||
        (config->width % 4) || (config->height % 2)) {
        return STM_H264_INVALID_ARGUMENT;
    }

    int error = stm_h264_picture_type(config->pixformat, &input_type);
    if (error != STM_H264_OK) {
        return error;
    }

    if (((uint64_t) stm_h264_macroblocks(config->width, config->height) *
         (uint64_t) config->fps) > STM_H264_MAX_MB_PER_SECOND) {
        return STM_H264_RATE_UNSUPPORTED;
    }

    memset(ctx, 0, sizeof(*ctx));
    ctx->gop = (config->gop > 0) ? config->gop : config->fps;
    ctx->gop = OMV_MIN(OMV_MAX(ctx->gop, 1), 300);

    error = stm_h264_alloc_input_scratch(ctx, config);
    if (error != STM_H264_OK) {
        return error;
    }

    HAL_SYSCFG_EnableVENCRAMReserved();
    LL_VENC_Init();
    ctx->hw_initialized = true;
    stm_h264_active = true;

    memset(&enc_config, 0, sizeof(enc_config));
    enc_config.streamType = H264ENC_BYTE_STREAM;
    enc_config.viewMode = H264ENC_BASE_VIEW_SINGLE_BUFFER;
    enc_config.level = H264ENC_LEVEL_5_1;
    enc_config.width = config->width;
    enc_config.height = config->height;
    enc_config.frameRateNum = config->fps;
    enc_config.frameRateDenom = 1;
    enc_config.refFrameAmount = 1;

    ret = H264EncInit(&enc_config, &ctx->inst);
    if (ret != H264ENC_OK) {
        stm_h264_deinit(ctx);
        return stm_h264_ret_to_error(ret);
    }

    ret = H264EncGetPreProcessing(ctx->inst, &preproc);
    if (ret != H264ENC_OK) {
        stm_h264_deinit(ctx);
        return stm_h264_ret_to_error(ret);
    }

    preproc.origWidth = config->width;
    preproc.origHeight = config->height;
    preproc.xOffset = 0;
    preproc.yOffset = 0;
    preproc.inputType = input_type;
    preproc.rotation = H264ENC_ROTATE_0;
    preproc.videoStabilization = 0;
    preproc.colorConversion.type = H264ENC_RGBTOYUV_BT601;
    preproc.scaledOutput = 0;
    preproc.interlacedFrame = 0;

    ret = H264EncSetPreProcessing(ctx->inst, &preproc);
    if (ret != H264ENC_OK) {
        stm_h264_deinit(ctx);
        return stm_h264_ret_to_error(ret);
    }

    ret = H264EncGetCodingCtrl(ctx->inst, &coding);
    if (ret != H264ENC_OK) {
        stm_h264_deinit(ctx);
        return stm_h264_ret_to_error(ret);
    }

    coding.idrHeader = 1;

    ret = H264EncSetCodingCtrl(ctx->inst, &coding);
    if (ret != H264ENC_OK) {
        stm_h264_deinit(ctx);
        return stm_h264_ret_to_error(ret);
    }

    ret = H264EncGetRateCtrl(ctx->inst, &rate);
    if (ret != H264ENC_OK) {
        stm_h264_deinit(ctx);
        return stm_h264_ret_to_error(ret);
    }

    stm_h264_setup_rate_control(&rate, config->bitrate, ctx->gop, config->qp);
    ret = H264EncSetRateCtrl(ctx->inst, &rate);
    if (ret != H264ENC_OK) {
        stm_h264_deinit(ctx);
        return stm_h264_ret_to_error(ret);
    }

    return STM_H264_OK;
}

int stm_h264_encode(stm_h264_t *ctx, image_t *src,
                    uint8_t *out, size_t out_size,
                    size_t *out_len, bool force_idr) {
    H264EncIn enc_in = { 0 };
    H264EncOut enc_out = { 0 };
    size_t header_len = 0;

    if (!ctx || !ctx->inst || !src || !out || !out_len || !src->data) {
        return STM_H264_INVALID_ARGUMENT;
    }

    *out_len = 0;

    if (!ctx->started) {
        int error = stm_h264_stream_start(ctx, out, out_size, &header_len);
        if (error != STM_H264_OK) {
            return error;
        }
    }

    uint8_t *frame_out = out + header_len;
    size_t frame_out_size = out_size - header_len;
    size_t src_size = image_size(src);
    uint8_t *input = src->data;

    stm_h264_cache_invalidate(src->data, src_size);
    if (ctx->use_input_scratch) {
        if (!ctx->input_scratch) {
            return STM_H264_INVALID_ARGUMENT;
        }

        int error = stm_h264_pack_luma_rgb888(src, ctx->input_scratch);
        if (error != STM_H264_OK) {
            return error;
        }

        stm_h264_cache_clean(ctx->input_scratch, ctx->input_scratch_size);
        input = ctx->input_scratch;
    }

    enc_in.busLuma = (ptr_t) input;
    enc_in.busChromaU = 0;
    enc_in.busChromaV = 0;
    enc_in.pOutBuf = (u32 *) frame_out;
    enc_in.busOutBuf = (ptr_t) frame_out;
    enc_in.outBufSize = frame_out_size;
    enc_in.codingType = ((ctx->frames % ctx->gop) == 0 || force_idr) ?
                        H264ENC_INTRA_FRAME : H264ENC_PREDICTED_FRAME;
    enc_in.timeIncrement = 1;
    enc_in.ipf = H264ENC_REFERENCE_AND_REFRESH;
    enc_in.ltrf = H264ENC_NO_REFERENCE_NO_REFRESH;
    enc_in.lineBufWrCnt = 0;
    enc_in.sendAUD = 0;

    stm_h264_cache_clean_invalidate(frame_out, frame_out_size);

    H264EncRet ret = H264EncStrmEncode(ctx->inst, &enc_in, &enc_out, NULL, NULL, NULL);
    if (ret != H264ENC_FRAME_READY) {
        return stm_h264_ret_to_error(ret);
    }

    stm_h264_cache_invalidate(frame_out, enc_out.streamSize);
    ctx->frames++;
    *out_len = header_len + enc_out.streamSize;
    return STM_H264_OK;
}

int stm_h264_end(stm_h264_t *ctx, uint8_t *out, size_t out_size, size_t *out_len) {
    H264EncIn enc_in = { 0 };
    H264EncOut enc_out = { 0 };

    if (!ctx || !ctx->inst || !out || !out_len) {
        return STM_H264_INVALID_ARGUMENT;
    }

    *out_len = 0;

    if (!ctx->started) {
        return STM_H264_OK;
    }

    enc_in.pOutBuf = (u32 *) out;
    enc_in.busOutBuf = (ptr_t) out;
    enc_in.outBufSize = out_size;

    stm_h264_cache_clean_invalidate(out, out_size);
    H264EncRet ret = H264EncStrmEnd(ctx->inst, &enc_in, &enc_out);
    if (ret != H264ENC_OK) {
        return stm_h264_ret_to_error(ret);
    }

    *out_len = enc_out.streamSize;
    return STM_H264_OK;
}

void stm_h264_deinit(stm_h264_t *ctx) {
    if (!ctx) {
        return;
    }

    if (ctx->inst) {
        H264EncRelease(ctx->inst);
        ctx->inst = NULL;
    }

    if (ctx->input_scratch) {
        uma_free(ctx->input_scratch);
        ctx->input_scratch = NULL;
        ctx->input_scratch_size = 0;
        ctx->use_input_scratch = false;
    }

    if (ctx->hw_initialized) {
        LL_VENC_DeInit();
        HAL_SYSCFG_DisableVENCRAMReserved();
        ctx->hw_initialized = false;
    }

    ctx->started = false;
    stm_h264_active = false;
}

const char *stm_h264_strerror(int error) {
    switch (error) {
        case STM_H264_OK:
            return "no error";
        case STM_H264_ERROR:
            return "H.264 encoder returned a generic error; check VENC hardware ID/support";
        case STM_H264_UNSUPPORTED:
            return "unsupported H.264 input pixel format";
        case STM_H264_OVERFLOW:
            return "H.264 output buffer overflow";
        case STM_H264_MEMORY:
            return "H.264 encoder memory allocation failed";
        case STM_H264_INVALID_ARGUMENT:
            return "invalid H.264 encoder argument";
        case STM_H264_NULL_ARGUMENT:
            return "internal H.264 encoder null argument";
        case STM_H264_EWL:
            return "H.264 encoder wrapper layer error";
        case STM_H264_INVALID_STATUS:
            return "H.264 encoder was called in an invalid state";
        case STM_H264_HW_BUS:
            return "H.264 hardware bus access error";
        case STM_H264_HW_DATA:
            return "H.264 hardware data error";
        case STM_H264_HW_TIMEOUT:
            return "H.264 hardware timeout";
        case STM_H264_HW_RESERVED:
            return "H.264 hardware is busy";
        case STM_H264_SYSTEM:
            return "H.264 hardware system error";
        case STM_H264_INSTANCE:
            return "invalid H.264 encoder instance";
        case STM_H264_HRD:
            return "H.264 HRD rate-control error";
        case STM_H264_HW_RESET:
            return "H.264 hardware reset during encode";
        case STM_H264_FUSE:
            return "H.264 hardware rejected the selected encoding mode or input format; lower resolution/fps";
        case STM_H264_BUSY:
            return "H.264 encoder is already active";
        case STM_H264_RATE_UNSUPPORTED:
            return "requested H.264 resolution/fps exceeds STM32N6 VENC limit (about 1080p15 or 720p30)";
        default:
            return "H.264 encoder error";
    }
}

#else

int stm_h264_init(stm_h264_t *ctx, const stm_h264_config_t *config) {
    (void) ctx;
    (void) config;
    return STM_H264_UNSUPPORTED;
}

int stm_h264_encode(stm_h264_t *ctx, image_t *src,
                    uint8_t *out, size_t out_size,
                    size_t *out_len, bool force_idr) {
    (void) ctx;
    (void) src;
    (void) out;
    (void) out_size;
    (void) force_idr;

    if (out_len) {
        *out_len = 0;
    }

    return STM_H264_UNSUPPORTED;
}

int stm_h264_end(stm_h264_t *ctx, uint8_t *out, size_t out_size, size_t *out_len) {
    (void) ctx;
    (void) out;
    (void) out_size;

    if (out_len) {
        *out_len = 0;
    }

    return STM_H264_UNSUPPORTED;
}

void stm_h264_deinit(stm_h264_t *ctx) {
    (void) ctx;
}

const char *stm_h264_strerror(int error) {
    (void) error;
    return "H.264 encoder is not enabled";
}

#endif
