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
#define STM_H264_MAX_QP             (51)

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

    if (ret == H264ENC_OUTPUT_BUFFER_OVERFLOW) {
        return STM_H264_OVERFLOW;
    }

    if ((ret == H264ENC_MEMORY_ERROR) || (ret == H264ENC_EWL_MEMORY_ERROR)) {
        return STM_H264_MEMORY;
    }

    if (ret == H264ENC_INVALID_ARGUMENT) {
        return STM_H264_INVALID_ARGUMENT;
    }

    return STM_H264_ERROR;
}

static int stm_h264_picture_type(pixformat_t pixformat, H264EncPictureType *type) {
    switch (pixformat) {
        case PIXFORMAT_RGB565:
            *type = H264ENC_RGB565;
            return STM_H264_OK;
        case PIXFORMAT_YUV422:
            *type = H264ENC_YUV422_INTERLEAVED_YUYV;
            return STM_H264_OK;
        default:
            return STM_H264_UNSUPPORTED;
    }
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

    stm_h264_cache_invalidate(out, enc_out.streamSize);

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

static void stm_h264_setup_rate_control(H264EncRateCtrl *rate, int bitrate, int gop) {
    rate->pictureRc = 1;
    rate->mbRc = 1;
    rate->pictureSkip = 0;
    rate->hrd = 0;
    rate->qpHdr = STM_H264_DEFAULT_QP;
    rate->qpMin = STM_H264_MIN_QP;
    rate->qpMax = STM_H264_MAX_QP;
    rate->bitPerSecond = bitrate;
    rate->gopLen = gop;
    rate->intraQpDelta = 0;
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
        return STM_H264_ERROR;
    }

    if ((config->width <= 0) || (config->height <= 0) ||
        (config->fps <= 0) || (config->fps > 300) ||
        (config->bitrate < STM_H264_MIN_BITRATE) ||
        (config->bitrate > STM_H264_MAX_BITRATE) ||
        (config->width % 4) || (config->height % 2)) {
        return STM_H264_INVALID_ARGUMENT;
    }

    int error = stm_h264_picture_type(config->pixformat, &input_type);
    if (error != STM_H264_OK) {
        return error;
    }

    memset(ctx, 0, sizeof(*ctx));
    ctx->gop = (config->gop > 0) ? config->gop : config->fps;
    ctx->gop = OMV_MIN(OMV_MAX(ctx->gop, 1), 300);

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
    coding.seiMessages = 0;
    coding.videoFullRange = 1;
    coding.enableCabac = 0;
    coding.transform8x8Mode = 0;
    coding.quarterPixelMv = 0;

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

    stm_h264_setup_rate_control(&rate, config->bitrate, ctx->gop);
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

    enc_in.busLuma = (ptr_t) src->data;
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

    stm_h264_cache_clean(src->data, src_size);
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

    stm_h264_cache_invalidate(out, enc_out.streamSize);
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
        case STM_H264_UNSUPPORTED:
            return "unsupported H.264 input pixel format";
        case STM_H264_OVERFLOW:
            return "H.264 output buffer overflow";
        case STM_H264_MEMORY:
            return "H.264 encoder memory allocation failed";
        case STM_H264_INVALID_ARGUMENT:
            return "invalid H.264 encoder argument";
        default:
            return "H.264 encoder error";
    }
}

void *EWLmalloc(u32 n) {
    return uma_malloc(n, UMA_PERSIST | UMA_MAYBE);
}

void EWLfree(void *p) {
    uma_free(p);
}

void *EWLcalloc(u32 n, u32 s) {
    size_t size = (size_t) n * (size_t) s;
    return uma_calloc(size, UMA_PERSIST | UMA_MAYBE);
}

i32 EWLMallocLinear(const void *instance, u32 size, EWLLinearMem_t *info) {
    (void) instance;

    if (!info || !size) {
        return EWL_ERROR;
    }

    size_t aligned_size = OMV_ALIGN_TO(size, OMV_CACHE_LINE_SIZE);
    void *ptr = uma_malign(aligned_size, OMV_CACHE_LINE_SIZE, UMA_PERSIST | UMA_CACHE | UMA_MAYBE);
    if (!ptr) {
        return EWL_ERROR;
    }

    memset(ptr, 0, aligned_size);
    stm_h264_cache_clean(ptr, aligned_size);

    info->size = aligned_size;
    info->virtualAddress = (u32 *) ptr;
    info->busAddress = (ptr_t) ptr;
    return EWL_OK;
}

void EWLFreeLinear(const void *instance, EWLLinearMem_t *info) {
    (void) instance;

    if (info && info->virtualAddress) {
        uma_free(info->virtualAddress);
        info->virtualAddress = NULL;
        info->busAddress = 0;
        info->size = 0;
    }
}

void EWLPoolChoiceCb(uint8_t **pool_ptr, size_t *size) {
    if (pool_ptr) {
        *pool_ptr = NULL;
    }

    if (size) {
        *size = 0;
    }
}

void EWLPoolReleaseCb(uint8_t **pool_ptr) {
    if (pool_ptr) {
        *pool_ptr = NULL;
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
