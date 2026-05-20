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
#include "file_utils.h"
#include "framebuffer.h"
#include "omv_common.h"
#include "omv_csi.h"
#include "py_csi.h"
#include "stm_h264.h"
#include "umalloc.h"

#define PY_VIDEO_CODEC_H264             (1)
#define PY_VIDEO_DEFAULT_FPS            (120)
#define PY_VIDEO_DEFAULT_BITRATE        (50000000)
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
    uint32_t frames;
    uint32_t dropped;
    uint64_t bytes;
    int fps;
    int bitrate;
    bool closed;
    bool recording;
    bool stream_was_enabled;
} py_video_recorder_t;

static void py_video_raise_csi_error(int error) {
    mp_raise_msg(&mp_type_RuntimeError, (mp_rom_error_text_t) omv_csi_strerror(error));
}

static void py_video_raise_h264_error(int error) {
    mp_raise_msg(&mp_type_RuntimeError, (mp_rom_error_text_t) stm_h264_strerror(error));
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
    mp_obj_dict_store(dict, MP_OBJ_NEW_QSTR(MP_QSTR_fps), mp_obj_new_int(self->fps));
    mp_obj_dict_store(dict, MP_OBJ_NEW_QSTR(MP_QSTR_bitrate), mp_obj_new_int(self->bitrate));
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
            error = stm_h264_encode(&self->h264, &image,
                                    self->out_buf, self->out_size,
                                    &out_len, false);
            omv_csi_recorder_release(self->csi);

            if (error < 0) {
                py_video_raise_h264_error(error);
            }

            py_video_writer_write(self, self->out_buf, out_len);
            self->frames++;
            self->bytes += out_len;
            self->dropped = self->csi->recorder_dropped_frames;

            mp_event_handle_nowait();
        }

        py_video_recorder_stop_internal(self);
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

        size_t out_len = 0;
        int error = stm_h264_end(&self->h264, self->out_buf, self->out_size, &out_len);
        if (error < 0) {
            py_video_raise_h264_error(error);
        }

        if (out_len) {
            py_video_writer_write(self, self->out_buf, out_len);
            self->bytes += out_len;
        }

        py_video_writer_flush(self);
        file_sync(&self->fp);
        file_close(&self->fp);
        stm_h264_deinit(&self->h264);
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
        { MP_QSTR_gop,          MP_ARG_INT | MP_ARG_KW_ONLY,  {.u_int = -1} },
        { MP_QSTR_buffer_size,  MP_ARG_INT | MP_ARG_KW_ONLY,  {.u_int = PY_VIDEO_DEFAULT_OUT_BUFFER} },
        { MP_QSTR_write_buffer, MP_ARG_INT | MP_ARG_KW_ONLY,  {.u_int = PY_VIDEO_DEFAULT_WRITE_BUFFER} },
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all_kw_array(n_args, n_kw, all_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    if (!mp_obj_is_type(args[ARG_cam].u_obj, &py_csi_type)) {
        mp_raise_msg(&mp_type_TypeError, MP_ERROR_TEXT("expected csi.CSI object"));
    }

    if (args[ARG_codec].u_int != PY_VIDEO_CODEC_H264) {
        mp_raise_msg(&mp_type_ValueError, MP_ERROR_TEXT("only H264 codec is supported"));
    }

    if (args[ARG_buffer_size].u_int <= 0 ||
        args[ARG_write_buffer].u_int < PY_VIDEO_MIN_WRITE_BUFFER) {
        mp_raise_msg(&mp_type_ValueError, MP_ERROR_TEXT("invalid video buffer size"));
    }

    py_csi_obj_t *cam = MP_OBJ_TO_PTR(args[ARG_cam].u_obj);
    const char *path = mp_obj_str_get_str(args[ARG_path].u_obj);
    omv_csi_t *csi = cam->csi;

    if (!csi || !csi->fb) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("CSI is not initialized"));
    }

    py_video_recorder_t *self = mp_obj_malloc_with_finaliser(py_video_recorder_t, type);
    memset(self, 0, sizeof(*self));
    self->base.type = type;
    self->csi = csi;
    self->closed = true;
    self->fps = args[ARG_fps].u_int;
    self->bitrate = args[ARG_bitrate].u_int;
    self->out_size = args[ARG_buffer_size].u_int;
    self->write_size = args[ARG_write_buffer].u_int;

    self->out_buf = uma_malign(self->out_size, OMV_CACHE_LINE_SIZE, UMA_PERSIST | UMA_CACHE | UMA_MAYBE);
    self->write_buf = uma_malign(self->write_size, OMV_CACHE_LINE_SIZE, UMA_PERSIST | UMA_CACHE | UMA_MAYBE);
    if (!self->out_buf || !self->write_buf) {
        py_video_free_buffers(self);
        mp_raise_msg(&mp_type_MemoryError, MP_ERROR_TEXT("failed to allocate video buffers"));
    }

    int width = csi->fb->u ? csi->fb->u : csi->fb->w;
    int height = csi->fb->v ? csi->fb->v : csi->fb->h;

    stm_h264_config_t config = {
        .width = width,
        .height = height,
        .fps = self->fps,
        .bitrate = self->bitrate,
        .gop = args[ARG_gop].u_int,
        .pixformat = csi->pixformat,
    };

    int error = stm_h264_init(&self->h264, &config);
    if (error < 0) {
        py_video_free_buffers(self);
        py_video_raise_h264_error(error);
    }

    nlr_buf_t nlr;
    if (nlr_push(&nlr) == 0) {
        file_open(&self->fp, path, FA_WRITE | FA_CREATE_ALWAYS);
        nlr_pop();
    } else {
        stm_h264_deinit(&self->h264);
        py_video_free_buffers(self);
        nlr_jump(nlr.ret_val);
    }

    self->closed = false;

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
};
static MP_DEFINE_CONST_DICT(py_video_globals_dict, py_video_globals_dict_table);

const mp_obj_module_t video_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_t) &py_video_globals_dict,
};

MP_REGISTER_MODULE(MP_QSTR_video, video_module);
#endif // MICROPY_PY_VIDEO && IMLIB_ENABLE_IMAGE_FILE_IO
