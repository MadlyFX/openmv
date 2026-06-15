/*
 * SPDX-License-Identifier: MIT
 *
 * Copyright (C) 2026 OpenMV, LLC.
 *
 * Native video recorder module.
 */
#include "imlib_config.h"

#if MICROPY_PY_VIDEO && defined(IMLIB_ENABLE_IMAGE_FILE_IO)

#include <string.h>
#include "py/obj.h"
#include "py/nlr.h"
#include "py/mphal.h"
#include "py/runtime.h"
#include "extmod/vfs.h"
#include "file_utils.h"
#include "framebuffer.h"
#include "omv_common.h"
#include "omv_csi.h"
#include "py_csi.h"
#include "stm_h264.h"
#include "umalloc.h"

#define PY_VIDEO_CODEC_H264             (1)
#define PY_VIDEO_CODEC_MJPEG            (2)
#define PY_VIDEO_CODEC_RAW              (3)
#define PY_VIDEO_DEFAULT_FPS            (30)
#define PY_VIDEO_DEFAULT_BITRATE        (50000000)
#define PY_VIDEO_DEFAULT_QUALITY        (90)
#define PY_VIDEO_DEFAULT_OUT_BUFFER     (2 * 1024 * 1024)
#define PY_VIDEO_DEFAULT_WRITE_BUFFER   (256 * 1024)
#define PY_VIDEO_MIN_WRITE_BUFFER       (64 * 1024)

static const mp_obj_type_t py_video_recorder_type;

typedef struct py_video_recorder {
    mp_obj_base_t base;
    omv_csi_t *csi;
    file_t fp;
    stm_h264_t h264;
    uint8_t *out_buf;
    uint8_t *write_buf;
    size_t out_size;
    size_t write_size;
    size_t write_pos;
    uint32_t mjpeg_bytes;
    uint32_t us_old;
    uint32_t us_intervals;
    uint32_t elapsed_ms;
    uint64_t us_total;
    uint32_t frames;
    uint32_t dropped;
    uint64_t bytes;
    int codec;
    int fps;
    int bitrate;
    int qp;
    int quality;
    int width;
    int height;
    bool closed;
    bool recording;
    bool stream_was_enabled;
    bool mjpeg_opened;
} py_video_recorder_t;

static void py_video_raise_csi_error(int error) {
    mp_raise_msg(&mp_type_RuntimeError, (mp_rom_error_text_t) omv_csi_strerror(error));
}

static void py_video_raise_h264_error(const char *operation, int error) {
    mp_raise_msg_varg(&mp_type_RuntimeError,
                      MP_ERROR_TEXT("H.264 %s failed: %s"),
                      operation, stm_h264_strerror(error));
}

static void py_video_raise_mjpeg_error(const char *operation, int error) {
    const char *message = "MJPEG encoder error";
    switch (error) {
        case -1:
            message = "captured frame size does not match recorder size";
            break;
        case -2:
            message = "JPEG output buffer overflow; lower quality or increase buffer_size";
            break;
        case -3:
            message = "MJPEG AVI exceeded 4 GiB limit";
            break;
        default:
            break;
    }

    mp_raise_msg_varg(&mp_type_RuntimeError,
                      MP_ERROR_TEXT("MJPEG %s failed: %s"),
                      operation, message);
}

static void py_video_raise_raw_error(const char *operation, int error) {
    const char *message = "RAW recorder error";
    switch (error) {
        case -1:
            message = "captured frame size does not match recorder size";
            break;
        case -2:
            message = "RAW requires grayscale or Bayer capture";
            break;
        default:
            break;
    }

    mp_raise_msg_varg(&mp_type_RuntimeError,
                      MP_ERROR_TEXT("RAW %s failed: %s"),
                      operation, message);
}

static void py_video_validate_output_path(const char *path) {
    const char *path_out = path;
    if (mp_vfs_lookup_path(path, &path_out) == MP_VFS_NONE) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("video output path is not mounted; check /sdcard or cwd"));
    }
}

static void py_video_cache_invalidate(const void *addr, size_t size) {
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

static void py_video_free_buffers(py_video_recorder_t *self) {
    if (self->out_buf) {
        uma_free(self->out_buf);
        self->out_buf = NULL;
    }

    if (self->write_buf) {
        uma_free(self->write_buf);
        self->write_buf = NULL;
    }
}

static void py_video_writer_flush(py_video_recorder_t *self) {
    if (self->write_pos) {
        file_write(&self->fp, self->write_buf, self->write_pos);
        self->write_pos = 0;
    }
}

static void py_video_writer_write(py_video_recorder_t *self, const uint8_t *data, size_t size) {
    if (size >= self->write_size) {
        py_video_writer_flush(self);
        file_write(&self->fp, data, size);
        return;
    }

    while (size) {
        size_t space = self->write_size - self->write_pos;
        size_t chunk = OMV_MIN(space, size);

        memcpy(self->write_buf + self->write_pos, data, chunk);
        self->write_pos += chunk;
        data += chunk;
        size -= chunk;

        if (self->write_pos == self->write_size) {
            py_video_writer_flush(self);
        }
    }
}

static void py_video_writer_write_u32(py_video_recorder_t *self, uint32_t value) {
    uint8_t data[4] = {
        (uint8_t) value,
        (uint8_t) (value >> 8),
        (uint8_t) (value >> 16),
        (uint8_t) (value >> 24),
    };
    py_video_writer_write(self, data, sizeof(data));
}

static bool py_video_mjpeg_pixformat_supported(pixformat_t pixformat) {
    switch (pixformat) {
        case PIXFORMAT_BINARY:
        case PIXFORMAT_GRAYSCALE:
        case PIXFORMAT_RGB565:
        case PIXFORMAT_BAYER_ANY:
        case PIXFORMAT_YUV_ANY:
        case PIXFORMAT_JPEG:
            return true;
        default:
            return false;
    }
}

static bool py_video_raw_pixformat_supported(pixformat_t pixformat) {
    switch (pixformat) {
        case PIXFORMAT_GRAYSCALE:
        case PIXFORMAT_BAYER_ANY:
            return true;
        default:
            return false;
    }
}

static int py_video_mjpeg_encode(py_video_recorder_t *self, image_t *image) {
    static const uint8_t pad[3] = { 0 };

    if ((image->w != self->width) || (image->h != self->height)) {
        return -1;
    }

    image_t jpeg;

    py_video_cache_invalidate(image->data, image_size(image));
    bool passthrough = image->pixfmt == PIXFORMAT_JPEG;
    if (passthrough) {
        jpeg = *image;
    } else {
        jpeg = (image_t) {
            .w = image->w,
            .h = image->h,
            .pixfmt = PIXFORMAT_JPEG,
            .size = self->out_size,
            .data = self->out_buf,
        };

        if (jpeg_compress(image, &jpeg, self->quality, false, JPEG_SUBSAMPLING_444)) {
            return -2;
        }
    }

    uint32_t jpeg_size = jpeg.size;
    uint32_t size_padded = OMV_ALIGN_TO(jpeg_size, 4);
    if ((size_padded < jpeg_size) || (!passthrough && (size_padded > self->out_size))) {
        return -2;
    }

    if ((((uint32_t) -1) - self->mjpeg_bytes) < size_padded) {
        return -3;
    }

    py_video_writer_write(self, (const uint8_t *) "00dc", 4);
    py_video_writer_write_u32(self, size_padded);
    py_video_writer_write(self, jpeg.data, jpeg_size);
    if (size_padded > jpeg_size) {
        py_video_writer_write(self, pad, size_padded - jpeg_size);
    }

    self->mjpeg_bytes += size_padded;
    self->bytes += 8 + size_padded;

    uint32_t ticks = mp_hal_ticks_us();
    if (self->us_old) {
        self->us_total += ticks - self->us_old;
        self->us_intervals++;
    }
    self->us_old = ticks;

    return 0;
}

static int py_video_raw_write(py_video_recorder_t *self, image_t *image) {
    if ((image->w != self->width) || (image->h != self->height)) {
        return -1;
    }

    if (!py_video_raw_pixformat_supported(image->pixfmt)) {
        return -2;
    }

    size_t size = image_size(image);
    py_video_cache_invalidate(image->data, size);
    py_video_writer_write(self, image->data, size);
    self->bytes += size;
    return 0;
}

static int py_video_recorder_start(py_video_recorder_t *self) {
    if (self->recording) {
        return 0;
    }

    framebuffer_t *stream_fb = framebuffer_get(FB_STREAM_ID);
    self->stream_was_enabled = stream_fb->enabled;
    framebuffer_set_enabled(stream_fb, false);

    int error = omv_csi_recorder_start(self->csi, true);
    if (error < 0) {
        framebuffer_set_enabled(stream_fb, self->stream_was_enabled);
        return error;
    }

    self->us_old = 0;
    self->recording = true;
    return 0;
}

static void py_video_recorder_stop_internal(py_video_recorder_t *self) {
    if (!self->recording) {
        return;
    }

    self->dropped = self->csi->recorder_dropped_frames;
    omv_csi_recorder_stop(self->csi);

    framebuffer_t *stream_fb = framebuffer_get(FB_STREAM_ID);
    framebuffer_set_enabled(stream_fb, self->stream_was_enabled);
    self->recording = false;
}

static mp_obj_t py_video_recorder_status_obj(py_video_recorder_t *self) {
    mp_obj_t dict = mp_obj_new_dict(0);
    mp_obj_dict_store(dict, MP_OBJ_NEW_QSTR(MP_QSTR_frames), mp_obj_new_int_from_uint(self->frames));
    mp_obj_dict_store(dict, MP_OBJ_NEW_QSTR(MP_QSTR_dropped), mp_obj_new_int_from_uint(self->dropped));
    mp_obj_dict_store(dict, MP_OBJ_NEW_QSTR(MP_QSTR_bytes), mp_obj_new_int_from_ull(self->bytes));
    mp_obj_dict_store(dict, MP_OBJ_NEW_QSTR(MP_QSTR_codec), mp_obj_new_int(self->codec));
    mp_obj_dict_store(dict, MP_OBJ_NEW_QSTR(MP_QSTR_width), mp_obj_new_int(self->width));
    mp_obj_dict_store(dict, MP_OBJ_NEW_QSTR(MP_QSTR_height), mp_obj_new_int(self->height));
    mp_obj_dict_store(dict, MP_OBJ_NEW_QSTR(MP_QSTR_fps), mp_obj_new_int(self->fps));
    mp_obj_dict_store(dict, MP_OBJ_NEW_QSTR(MP_QSTR_actual_fps),
                      mp_obj_new_float(self->elapsed_ms ? ((self->frames * 1000.0f) / self->elapsed_ms) : 0));
    mp_obj_dict_store(dict, MP_OBJ_NEW_QSTR(MP_QSTR_bitrate), mp_obj_new_int(self->bitrate));
    mp_obj_dict_store(dict, MP_OBJ_NEW_QSTR(MP_QSTR_qp), mp_obj_new_int(self->qp));
    mp_obj_dict_store(dict, MP_OBJ_NEW_QSTR(MP_QSTR_quality), mp_obj_new_int(self->quality));
    mp_obj_dict_store(dict, MP_OBJ_NEW_QSTR(MP_QSTR_closed), mp_obj_new_bool(self->closed));
    mp_obj_dict_store(dict, MP_OBJ_NEW_QSTR(MP_QSTR_recording), mp_obj_new_bool(self->recording));
    return dict;
}

static void py_video_recorder_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind) {
    py_video_recorder_t *self = MP_OBJ_TO_PTR(self_in);
    (void) kind;
    mp_printf(print, "{\"closed\":%s, \"recording\":%s, \"frames\":%u, \"dropped\":%u, \"bytes\":%llu}",
              self->closed ? "true" : "false",
              self->recording ? "true" : "false",
              (unsigned) self->frames,
              (unsigned) self->dropped,
              (unsigned long long) self->bytes);
}

static mp_obj_t py_video_recorder_record(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_seconds, ARG_frames };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_seconds, MP_ARG_INT | MP_ARG_KW_ONLY, {.u_int = -1 } },
        { MP_QSTR_frames,  MP_ARG_INT | MP_ARG_KW_ONLY, {.u_int = -1 } },
    };

    py_video_recorder_t *self = MP_OBJ_TO_PTR(pos_args[0]);
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    if (self->closed) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("video recorder is closed"));
    }

    if ((args[ARG_seconds].u_int < 0) && (args[ARG_frames].u_int < 0)) {
        mp_raise_msg(&mp_type_ValueError, MP_ERROR_TEXT("seconds or frames must be set"));
    }

    if (args[ARG_frames].u_int == 0 || args[ARG_seconds].u_int == 0) {
        return py_video_recorder_status_obj(self);
    }

    int error = py_video_recorder_start(self);
    if (error < 0) {
        py_video_raise_csi_error(error);
    }

    nlr_buf_t nlr;
    if (nlr_push(&nlr) == 0) {
        uint32_t start_ms = mp_hal_ticks_ms();
        uint32_t start_frames = self->frames;
        uint32_t duration_ms = (args[ARG_seconds].u_int > 0) ?
                               ((uint32_t) args[ARG_seconds].u_int * 1000) : 0;

        while (true) {
            uint32_t recorded = self->frames - start_frames;
            if ((args[ARG_frames].u_int >= 0) && (recorded >= (uint32_t) args[ARG_frames].u_int)) {
                break;
            }

            if ((args[ARG_seconds].u_int > 0) && check_timeout_ms(start_ms, duration_ms)) {
                break;
            }

            image_t image;
            error = omv_csi_recorder_acquire(self->csi, &image, 0);
            if (error < 0) {
                py_video_raise_csi_error(error);
            }

            size_t out_len = 0;
            nlr_buf_t encode_nlr;
            if (nlr_push(&encode_nlr) == 0) {
                if (self->codec == PY_VIDEO_CODEC_H264) {
                    error = stm_h264_encode(&self->h264, &image,
                                            self->out_buf, self->out_size,
                                            &out_len, false);
                } else if (self->codec == PY_VIDEO_CODEC_MJPEG) {
                    error = py_video_mjpeg_encode(self, &image);
                } else {
                    error = py_video_raw_write(self, &image);
                }
                nlr_pop();
            } else {
                omv_csi_recorder_release(self->csi);
                nlr_jump(encode_nlr.ret_val);
            }
            omv_csi_recorder_release(self->csi);

            if (error < 0) {
                if (self->codec == PY_VIDEO_CODEC_H264) {
                    py_video_raise_h264_error("encode", error);
                } else if (self->codec == PY_VIDEO_CODEC_MJPEG) {
                    py_video_raise_mjpeg_error("encode", error);
                } else {
                    py_video_raise_raw_error("write", error);
                }
            }

            if (self->codec == PY_VIDEO_CODEC_H264) {
                py_video_writer_write(self, self->out_buf, out_len);
                self->bytes += out_len;
            }

            self->frames++;
            self->dropped = self->csi->recorder_dropped_frames;

            mp_event_handle_nowait();
        }

        py_video_recorder_stop_internal(self);
        self->elapsed_ms += mp_hal_ticks_ms() - start_ms;
        py_video_writer_flush(self);
        nlr_pop();
    } else {
        py_video_recorder_stop_internal(self);
        nlr_jump(nlr.ret_val);
    }

    return py_video_recorder_status_obj(self);
}
static MP_DEFINE_CONST_FUN_OBJ_KW(py_video_recorder_record_obj, 1, py_video_recorder_record);

static mp_obj_t py_video_recorder_stop(mp_obj_t self_in) {
    py_video_recorder_t *self = MP_OBJ_TO_PTR(self_in);
    py_video_recorder_stop_internal(self);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(py_video_recorder_stop_obj, py_video_recorder_stop);

static mp_obj_t py_video_recorder_status(mp_obj_t self_in) {
    py_video_recorder_t *self = MP_OBJ_TO_PTR(self_in);
    return py_video_recorder_status_obj(self);
}
static MP_DEFINE_CONST_FUN_OBJ_1(py_video_recorder_status_obj_fun, py_video_recorder_status);

static mp_obj_t py_video_recorder_close(mp_obj_t self_in) {
    py_video_recorder_t *self = MP_OBJ_TO_PTR(self_in);

    if (!self->closed) {
        py_video_recorder_stop_internal(self);

        if (self->codec == PY_VIDEO_CODEC_H264) {
            size_t out_len = 0;
            int error = stm_h264_end(&self->h264, self->out_buf, self->out_size, &out_len);
            if (error < 0) {
                py_video_raise_h264_error("finalize", error);
            }

            if (out_len) {
                py_video_writer_write(self, self->out_buf, out_len);
                self->bytes += out_len;
            }
        }

        py_video_writer_flush(self);
        if (self->codec == PY_VIDEO_CODEC_MJPEG && self->mjpeg_opened) {
            uint32_t us_avg = self->us_intervals ? (self->us_total / self->us_intervals) :
                              ((self->fps > 0) ? (1000000 / self->fps) : 0);
            mjpeg_sync(&self->fp, self->frames, self->mjpeg_bytes, us_avg);
        }

        file_sync(&self->fp);
        file_close(&self->fp);
        if (self->codec == PY_VIDEO_CODEC_H264) {
            stm_h264_deinit(&self->h264);
        }
        py_video_free_buffers(self);
        self->closed = true;
    }

    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(py_video_recorder_close_obj, py_video_recorder_close);

static mp_obj_t py_video_recorder_make_new(const mp_obj_type_t *type,
                                           size_t n_args, size_t n_kw,
                                           const mp_obj_t *all_args) {
    enum {
        ARG_cam,
        ARG_path,
        ARG_codec,
        ARG_fps,
        ARG_bitrate,
        ARG_qp,
        ARG_quality,
        ARG_gop,
        ARG_buffer_size,
        ARG_write_buffer,
    };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_cam,          MP_ARG_OBJ | MP_ARG_REQUIRED, {.u_rom_obj = MP_ROM_NONE} },
        { MP_QSTR_path,         MP_ARG_OBJ | MP_ARG_REQUIRED, {.u_rom_obj = MP_ROM_NONE} },
        { MP_QSTR_codec,        MP_ARG_INT | MP_ARG_KW_ONLY,  {.u_int = PY_VIDEO_CODEC_H264} },
        { MP_QSTR_fps,          MP_ARG_INT | MP_ARG_KW_ONLY,  {.u_int = PY_VIDEO_DEFAULT_FPS} },
        { MP_QSTR_bitrate,      MP_ARG_INT | MP_ARG_KW_ONLY,  {.u_int = PY_VIDEO_DEFAULT_BITRATE} },
        { MP_QSTR_qp,           MP_ARG_INT | MP_ARG_KW_ONLY,  {.u_int = -1} },
        { MP_QSTR_quality,      MP_ARG_INT | MP_ARG_KW_ONLY,  {.u_int = PY_VIDEO_DEFAULT_QUALITY} },
        { MP_QSTR_gop,          MP_ARG_INT | MP_ARG_KW_ONLY,  {.u_int = -1} },
        { MP_QSTR_buffer_size,  MP_ARG_INT | MP_ARG_KW_ONLY,  {.u_int = PY_VIDEO_DEFAULT_OUT_BUFFER} },
        { MP_QSTR_write_buffer, MP_ARG_INT | MP_ARG_KW_ONLY,  {.u_int = PY_VIDEO_DEFAULT_WRITE_BUFFER} },
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all_kw_array(n_args, n_kw, all_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    if (!mp_obj_is_type(args[ARG_cam].u_obj, &py_csi_type)) {
        mp_raise_msg(&mp_type_TypeError, MP_ERROR_TEXT("expected csi.CSI object"));
    }

    if ((args[ARG_codec].u_int != PY_VIDEO_CODEC_H264) &&
        (args[ARG_codec].u_int != PY_VIDEO_CODEC_MJPEG) &&
        (args[ARG_codec].u_int != PY_VIDEO_CODEC_RAW)) {
        mp_raise_msg(&mp_type_ValueError, MP_ERROR_TEXT("unsupported video codec"));
    }

    if (args[ARG_buffer_size].u_int <= 0 ||
        args[ARG_write_buffer].u_int < PY_VIDEO_MIN_WRITE_BUFFER) {
        mp_raise_msg(&mp_type_ValueError, MP_ERROR_TEXT("invalid video buffer size"));
    }

    if ((args[ARG_quality].u_int < 0) || (args[ARG_quality].u_int > 100)) {
        mp_raise_msg(&mp_type_ValueError, MP_ERROR_TEXT("quality ranges between 0 and 100"));
    }

    py_csi_obj_t *cam = MP_OBJ_TO_PTR(args[ARG_cam].u_obj);
    const char *path = mp_obj_str_get_str(args[ARG_path].u_obj);
    py_video_validate_output_path(path);

    omv_csi_t *csi = cam->csi;

    if (!csi || !csi->fb) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("CSI is not initialized"));
    }

    py_video_recorder_t *self = mp_obj_malloc_with_finaliser(py_video_recorder_t, type);
    memset(self, 0, sizeof(*self));
    self->base.type = type;
    self->csi = csi;
    self->closed = true;
    self->codec = args[ARG_codec].u_int;
    self->fps = args[ARG_fps].u_int;
    self->bitrate = args[ARG_bitrate].u_int;
    self->qp = args[ARG_qp].u_int;
    self->quality = args[ARG_quality].u_int;
    self->out_size = args[ARG_buffer_size].u_int;
    self->write_size = args[ARG_write_buffer].u_int;

    self->out_buf = uma_malign(self->out_size, OMV_CACHE_LINE_SIZE, UMA_PERSIST | UMA_CACHE | UMA_MAYBE);
    self->write_buf = uma_malign(self->write_size, OMV_CACHE_LINE_SIZE, UMA_PERSIST | UMA_CACHE | UMA_MAYBE);
    if (!self->out_buf || !self->write_buf) {
        py_video_free_buffers(self);
        mp_raise_msg(&mp_type_MemoryError, MP_ERROR_TEXT("failed to allocate video buffers"));
    }

    nlr_buf_t nlr;
    if (nlr_push(&nlr) == 0) {
        file_open(&self->fp, path, FA_WRITE | FA_CREATE_ALWAYS);
        nlr_pop();
    } else {
        py_video_free_buffers(self);
        nlr_jump(nlr.ret_val);
    }

    self->closed = false;

    self->width = csi->fb->u ? csi->fb->u : csi->fb->w;
    self->height = csi->fb->v ? csi->fb->v : csi->fb->h;

    if (self->codec == PY_VIDEO_CODEC_H264) {
        stm_h264_config_t config = {
            .width = self->width,
            .height = self->height,
            .fps = self->fps,
            .bitrate = self->bitrate,
            .gop = args[ARG_gop].u_int,
            .qp = self->qp,
            .pixformat = csi->pixformat,
        };

        int error = stm_h264_init(&self->h264, &config);
        if (error < 0) {
            file_close(&self->fp);
            self->closed = true;
            py_video_free_buffers(self);
            py_video_raise_h264_error("init", error);
        }
    } else if (self->codec == PY_VIDEO_CODEC_MJPEG) {
        if (!py_video_mjpeg_pixformat_supported(csi->pixformat)) {
            file_close(&self->fp);
            self->closed = true;
            py_video_free_buffers(self);
            mp_raise_msg(&mp_type_RuntimeError,
                         MP_ERROR_TEXT("MJPEG requires grayscale, RGB565, Bayer, or YUV capture"));
        }

        mjpeg_open(&self->fp, self->width, self->height);
        self->mjpeg_opened = true;
        self->bytes = file_tell(&self->fp);
    } else {
        if (!py_video_raw_pixformat_supported(csi->pixformat)) {
            file_close(&self->fp);
            self->closed = true;
            py_video_free_buffers(self);
            mp_raise_msg(&mp_type_RuntimeError,
                         MP_ERROR_TEXT("RAW requires grayscale or Bayer capture"));
        }
    }

    return MP_OBJ_FROM_PTR(self);
}

static const mp_rom_map_elem_t py_video_recorder_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),  MP_ROM_QSTR(MP_QSTR_Recorder) },
    { MP_ROM_QSTR(MP_QSTR___del__),   MP_ROM_PTR(&py_video_recorder_close_obj) },
    { MP_ROM_QSTR(MP_QSTR_record),    MP_ROM_PTR(&py_video_recorder_record_obj) },
    { MP_ROM_QSTR(MP_QSTR_stop),      MP_ROM_PTR(&py_video_recorder_stop_obj) },
    { MP_ROM_QSTR(MP_QSTR_status),    MP_ROM_PTR(&py_video_recorder_status_obj_fun) },
    { MP_ROM_QSTR(MP_QSTR_close),     MP_ROM_PTR(&py_video_recorder_close_obj) },
};
static MP_DEFINE_CONST_DICT(py_video_recorder_locals_dict, py_video_recorder_locals_dict_table);

static MP_DEFINE_CONST_OBJ_TYPE(
    py_video_recorder_type,
    MP_QSTR_Recorder,
    MP_TYPE_FLAG_NONE,
    make_new, py_video_recorder_make_new,
    print, py_video_recorder_print,
    locals_dict, &py_video_recorder_locals_dict
    );

static const mp_rom_map_elem_t py_video_globals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_video) },
    { MP_ROM_QSTR(MP_QSTR_Recorder), MP_ROM_PTR(&py_video_recorder_type) },
    { MP_ROM_QSTR(MP_QSTR_H264),     MP_ROM_INT(PY_VIDEO_CODEC_H264) },
    { MP_ROM_QSTR(MP_QSTR_MJPEG),    MP_ROM_INT(PY_VIDEO_CODEC_MJPEG) },
    { MP_ROM_QSTR(MP_QSTR_RAW),      MP_ROM_INT(PY_VIDEO_CODEC_RAW) },
};
static MP_DEFINE_CONST_DICT(py_video_globals_dict, py_video_globals_dict_table);

const mp_obj_module_t video_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_t) &py_video_globals_dict,
};

MP_REGISTER_MODULE(MP_QSTR_video, video_module);
#endif // MICROPY_PY_VIDEO && IMLIB_ENABLE_IMAGE_FILE_IO
