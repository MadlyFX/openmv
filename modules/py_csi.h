/*
 * Copyright (C) 2026 OpenMV, LLC.
 *
 * This file is part of the OpenMV project.
 *
 * SPDX-License-Identifier: MIT
 */
#ifndef __PY_CSI_H__
#define __PY_CSI_H__

#include "py/obj.h"
#include "omv_csi.h"

typedef struct _py_csi_obj_t {
    mp_obj_base_t base;
    omv_csi_t *csi;
    void *raw;
    mp_obj_t vsync_cb;
    mp_obj_t frame_cb;
} py_csi_obj_t;

extern const mp_obj_type_t py_csi_type;

#endif /* __PY_CSI_H__ */
