/*
 * SPDX-License-Identifier: MIT
 *
 * Copyright (C) 2013-2024 OpenMV, LLC.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * IMU Python module.
 */
#include "board_config.h"

#if MICROPY_PY_IMU
#include "py/obj.h"
#include "py/mphal.h"
#include "py/runtime.h"

#include "py_helper.h"
#include "py_imu.h"
#include "omv_gpio.h"
#include "omv_spi.h"
#include "omv_i2c.h"

#if defined(OMV_IMU_CHIP_LSM6DS3)
#include "lsm6ds3tr_c_reg.h"

typedef union {
    int16_t i16bit[3];
    uint8_t u8bit[6];
} axis3bit16_t;

typedef union {
    int16_t i16bit;
    uint8_t u8bit[2];
} axis1bit16_t;


#define LSM_FUNC(f)                lsm6ds3tr_c_##f
#define LSM_CONST(c)               LSM6DS3TR_C_##c

#define lsm_from_fs2_to_mg         lsm6ds3tr_c_from_fs2g_to_mg
#define lsm_from_fs4_to_mg         lsm6ds3tr_c_from_fs4g_to_mg
#define lsm_from_fs8_to_mg         lsm6ds3tr_c_from_fs8g_to_mg
#define lsm_from_fs16_to_mg        lsm6ds3tr_c_from_fs16g_to_mg
#define lsm_from_fs125_to_mdps     lsm6ds3tr_c_from_fs125dps_to_mdps
#define lsm_from_fs250_to_mdps     lsm6ds3tr_c_from_fs250dps_to_mdps
#define lsm_from_fs500_to_mdps     lsm6ds3tr_c_from_fs500dps_to_mdps
#define lsm_from_fs1000_to_mdps    lsm6ds3tr_c_from_fs1000dps_to_mdps
#define lsm_from_fs2000_to_mdps    lsm6ds3tr_c_from_fs2000dps_to_mdps
#define lsm_from_lsb_to_celsius    lsm6ds3tr_c_from_lsb_to_celsius

#elif defined(OMV_IMU_CHIP_LSM6DSM)
#include "lsm6dsm_reg.h"

typedef union {
    int16_t i16bit[3];
    int16_t u8bit[3];
} axis3bit16_t;

typedef union {
    int16_t i16bit;
    int16_t u8bit[1];
} axis1bit16_t;

#define LSM_FUNC(f)                lsm6dsm_##f
#define LSM_CONST(c)               LSM6DSM_##c

#define lsm_from_fs2_to_mg         lsm6dsm_from_fs2g_to_mg
#define lsm_from_fs4_to_mg         lsm6dsm_from_fs4g_to_mg
#define lsm_from_fs8_to_mg         lsm6dsm_from_fs8g_to_mg
#define lsm_from_fs16_to_mg        lsm6dsm_from_fs16g_to_mg
#define lsm_from_fs125_to_mdps     lsm6dsm_from_fs125dps_to_mdps
#define lsm_from_fs250_to_mdps     lsm6dsm_from_fs250dps_to_mdps
#define lsm_from_fs500_to_mdps     lsm6dsm_from_fs500dps_to_mdps
#define lsm_from_fs1000_to_mdps    lsm6dsm_from_fs1000dps_to_mdps
#define lsm_from_fs2000_to_mdps    lsm6dsm_from_fs2000dps_to_mdps
#define lsm_from_lsb_to_celsius    lsm6dsm_from_lsb_to_celsius

#elif defined(OMV_IMU_CHIP_LSM6DSOX)
#include "lsm6dsox_reg.h"

typedef union {
    int16_t i16bit[3];
    int16_t u8bit[3];
} axis3bit16_t;

typedef union {
    int16_t i16bit;
    int16_t u8bit[1];
} axis1bit16_t;

#define LSM_FUNC(f)                lsm6dsox_##f
#define LSM_CONST(c)               LSM6DSOX_##c

#define lsm_from_fs2_to_mg         lsm6dsox_from_fs2_to_mg
#define lsm_from_fs4_to_mg         lsm6dsox_from_fs4_to_mg
#define lsm_from_fs8_to_mg         lsm6dsox_from_fs8_to_mg
#define lsm_from_fs16_to_mg        lsm6dsox_from_fs16_to_mg
#define lsm_from_fs125_to_mdps     lsm6dsox_from_fs125_to_mdps
#define lsm_from_fs250_to_mdps     lsm6dsox_from_fs250_to_mdps
#define lsm_from_fs500_to_mdps     lsm6dsox_from_fs500_to_mdps
#define lsm_from_fs1000_to_mdps    lsm6dsox_from_fs1000_to_mdps
#define lsm_from_fs2000_to_mdps    lsm6dsox_from_fs2000_to_mdps
#define lsm_from_lsb_to_celsius    lsm6dsox_from_lsb_to_celsius
#else
#error "imu chip variant is not defined."
#endif  // IMU chip

static bool imu_initialized = false;

#if defined(OMV_IMU_SPI_ID)

#if !defined(IMU_SPI_BUS_TIMEOUT)
#define IMU_SPI_BUS_TIMEOUT    (5000)
#endif

static omv_spi_t imu_bus;

static void platform_init(void *imu_bus) {
    omv_spi_config_t spi_config;
    omv_spi_default_config(&spi_config, OMV_IMU_SPI_ID);

    spi_config.baudrate = OMV_IMU_SPI_BAUDRATE;
    spi_config.clk_pol = OMV_SPI_CPOL_HIGH;
    spi_config.clk_pha = OMV_SPI_CPHA_2EDGE;
    spi_config.nss_enable = false; // Soft NSS

    omv_spi_init(imu_bus, &spi_config);
}

static void platform_deinit(void *imu_bus) {
    omv_spi_deinit(imu_bus);
    imu_initialized = false;
}

static int32_t platform_write(void *imu_bus, uint8_t Reg, const uint8_t *Bufp, uint16_t len) {
    omv_spi_t *spi_bus = imu_bus;

    omv_spi_transfer_t spi_xfer = {
        .timeout = IMU_SPI_BUS_TIMEOUT,
        .flags = OMV_SPI_XFER_BLOCKING,
        .callback = NULL,
        .userdata = NULL,
    };

    omv_gpio_write(spi_bus->cs, 0);
    spi_xfer.size = 1;
    spi_xfer.txbuf = &Reg;
    spi_xfer.rxbuf = NULL;
    omv_spi_transfer_start(spi_bus, &spi_xfer);

    spi_xfer.size = len;
    spi_xfer.txbuf = (uint8_t *) Bufp;
    spi_xfer.rxbuf = NULL;
    omv_spi_transfer_start(spi_bus, &spi_xfer);

    omv_gpio_write(spi_bus->cs, 1);
    return 0;
}

static int32_t platform_read(void *imu_bus, uint8_t Reg, uint8_t *Bufp, uint16_t len) {
    omv_spi_t *spi_bus = imu_bus;

    Reg |= 0x80;
    omv_spi_transfer_t spi_xfer = {
        .timeout = IMU_SPI_BUS_TIMEOUT,
        .flags = OMV_SPI_XFER_BLOCKING,
        .callback = NULL,
        .userdata = NULL,
    };

    omv_gpio_write(spi_bus->cs, 0);
    spi_xfer.size = 1;
    spi_xfer.txbuf = &Reg;
    spi_xfer.rxbuf = NULL;
    omv_spi_transfer_start(spi_bus, &spi_xfer);

    spi_xfer.size = len;
    spi_xfer.txbuf = NULL;
    spi_xfer.rxbuf = Bufp;
    omv_spi_transfer_start(spi_bus, &spi_xfer);

    omv_gpio_write(spi_bus->cs, 1);
    return 0;
}
#elif defined(OMV_IMU_I2C_ID)
static omv_i2c_t imu_bus = {};

static void platform_init(void *imu_bus) {
    omv_i2c_init(imu_bus, OMV_IMU_I2C_ID, OMV_IMU_I2C_SPEED);
}

static void platform_deinit(void *imu_bus) {
    omv_i2c_deinit(imu_bus);
    imu_initialized = false;
}

static int32_t platform_write(void *imu_bus, uint8_t reg, const uint8_t *bufp, uint16_t len) {
    omv_i2c_t *i2c_bus = imu_bus;

    if (omv_i2c_write(i2c_bus, LSM6DSM_I2C_ADD_L, (uint8_t *) &reg, 1, OMV_I2C_XFER_SUSPEND) != 0) {
        return -1;
    }
    if (omv_i2c_write(i2c_bus, LSM6DSM_I2C_ADD_L, (uint8_t *) bufp, len, OMV_I2C_XFER_NO_FLAGS) != 0) {
        return -1;
    }
    return 0;
}

static int32_t platform_read(void *imu_bus, uint8_t reg, uint8_t *bufp, uint16_t len) {
    omv_i2c_t *i2c_bus = imu_bus;

    if (omv_i2c_write(i2c_bus, LSM6DSM_I2C_ADD_L, (uint8_t *) &reg, 1, OMV_I2C_XFER_NO_STOP) != 0) {
        return -1;
    }
    if (omv_i2c_read(i2c_bus, LSM6DSM_I2C_ADD_L, bufp, len, OMV_I2C_XFER_NO_FLAGS) != 0) {
        return -1;
    }
    return 0;
}
#else
#error "imu bus is not defined."
#endif

static stmdev_ctx_t dev_ctx = {
    .handle = &imu_bus,
    .read_reg = platform_read,
    .write_reg = platform_write,
};

// Defaults match py_imu_init() below: 52 Hz ODR, 8g accel, 2000 dps gyro.
// Enum codes are identical across LSM6DS3/DSM/DSOX, so a uint8_t is portable.
static uint8_t xl_odr_code = 3;  // LSM_XL_ODR_52Hz
static uint8_t gy_odr_code = 3;  // LSM_GY_ODR_52Hz
static uint8_t xl_fs_code  = 3;  // LSM_8g
static uint8_t gy_fs_code  = 6;  // LSM_2000dps

static float xl_lsb_to_mg(int16_t lsb) {
    switch (xl_fs_code) {
        case LSM_CONST(2g):  return lsm_from_fs2_to_mg(lsb);
        case LSM_CONST(4g):  return lsm_from_fs4_to_mg(lsb);
        case LSM_CONST(16g): return lsm_from_fs16_to_mg(lsb);
        default:             return lsm_from_fs8_to_mg(lsb);
    }
}

static float gy_lsb_to_mdps(int16_t lsb) {
    switch (gy_fs_code) {
        case LSM_CONST(125dps):  return lsm_from_fs125_to_mdps(lsb);
        case LSM_CONST(250dps):  return lsm_from_fs250_to_mdps(lsb);
        case LSM_CONST(500dps):  return lsm_from_fs500_to_mdps(lsb);
        case LSM_CONST(1000dps): return lsm_from_fs1000_to_mdps(lsb);
        default:                 return lsm_from_fs2000_to_mdps(lsb);
    }
}

// ODR enum codes (0..10) are common across LSM6DS3/DSM/DSOX. Code 11 (1.6 Hz
// low-power, accel only) is intentionally excluded so accel and gyro can share
// the same setting.
static const struct {
    uint8_t code;
    float hz;
} odr_table[] = {
    { 0,    0.0f }, { 1,   12.5f }, { 2,   26.0f }, { 3,   52.0f },
    { 4,  104.0f }, { 5,  208.0f }, { 6,  416.0f }, { 7,  833.0f },
    { 8, 1660.0f }, { 9, 3330.0f }, { 10, 6660.0f },
};

static uint8_t hz_to_odr_code(float hz) {
    if (hz <= 0.0f) {
        return 0;
    }
    // Pick the lowest table entry whose rate is >= the request; if none, max out.
    for (size_t i = 1; i < MP_ARRAY_SIZE(odr_table); i++) {
        if (hz <= odr_table[i].hz) {
            return odr_table[i].code;
        }
    }
    return odr_table[MP_ARRAY_SIZE(odr_table) - 1].code;
}

static float odr_code_to_hz(uint8_t code) {
    for (size_t i = 0; i < MP_ARRAY_SIZE(odr_table); i++) {
        if (odr_table[i].code == code) {
            return odr_table[i].hz;
        }
    }
    return 0.0f;
}

static void error_on_not_ready() {
    if (!imu_initialized) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("IMU Not Ready!"));
    }
}

static mp_obj_t py_imu_tuple(float x, float y, float z) {
    return mp_obj_new_tuple(3, (mp_obj_t [3]) {mp_obj_new_float(x),
                                               mp_obj_new_float(y),
                                               mp_obj_new_float(z)});
}

// For when the camera board is lying on a table face up.

// X points to the right of the camera
// Y points down below the camera
// Z points in the reverse direction of the camera

// Thus (https://www.nxp.com/docs/en/application-note/AN3461.pdf):
//
// Roll = atan2(Y, Z)
// Pitch = atan2(-X, sqrt(Y^2, + Z^2)) -> assume Y=0 -> atan2(-X, Z)

// For when the camera board is standing right-side up.

// X points to the right of the camera (still X)
// Y points down below the camera (now Z)
// Z points in the reverse direction of the camera (now -Y)

// So:
//
// Roll = atan2(-X, sqrt(Z^2, + Y^2)) -> assume Z=0 -> atan2(-X, Y)
// Pitch = atan2(Z, -Y)

#if (OMV_IMU_X_Y_ROTATION_DEGREES != 0) &&   \
    (OMV_IMU_X_Y_ROTATION_DEGREES != 90) &&  \
    (OMV_IMU_X_Y_ROTATION_DEGREES != 180) && \
    (OMV_IMU_X_Y_ROTATION_DEGREES != 270)
#error "OMV_IMU_X_Y_ROTATION_DEGREES must be 0, 90, 180, or 270!"
#endif

#if (OMV_IMU_MOUNTING_Z_DIRECTION != -1) && \
    (OMV_IMU_MOUNTING_Z_DIRECTION != 1)
#error "OMV_IMU_MOUNTING_Z_DIRECTION must be -1 or 1!"
#endif

static float py_imu_get_roll() {
    axis3bit16_t data_raw_acceleration = {};
    LSM_FUNC(acceleration_raw_get) (&dev_ctx, data_raw_acceleration.u8bit);
    #if OMV_IMU_X_Y_ROTATION_DEGREES == 0
    float xr = xl_lsb_to_mg(data_raw_acceleration.i16bit[0]); // x
    float yr = xl_lsb_to_mg(data_raw_acceleration.i16bit[1]); // y
    #elif OMV_IMU_X_Y_ROTATION_DEGREES == 90
    float xr = -xl_lsb_to_mg(data_raw_acceleration.i16bit[1]); // y
    float yr = xl_lsb_to_mg(data_raw_acceleration.i16bit[0]); // x
    #elif OMV_IMU_X_Y_ROTATION_DEGREES == 180
    float xr = -xl_lsb_to_mg(data_raw_acceleration.i16bit[0]); // x
    float yr = -xl_lsb_to_mg(data_raw_acceleration.i16bit[1]); // y
    #elif OMV_IMU_X_Y_ROTATION_DEGREES == 270
    float xr = xl_lsb_to_mg(data_raw_acceleration.i16bit[1]); // y
    float yr = -xl_lsb_to_mg(data_raw_acceleration.i16bit[0]); // x
    #endif
    #if OMV_IMU_MOUNTING_Z_DIRECTION == 1 // default is -1 (IMU pointing reverse of camera)
    xr = -xr;
    yr = -yr;
    #endif
    return fmodf((IM_RAD2DEG(fast_atan2f(-xr, yr)) + 180), 360); // rotate 180
}

static float py_imu_get_pitch() {
    axis3bit16_t data_raw_acceleration = {};
    LSM_FUNC(acceleration_raw_get) (&dev_ctx, data_raw_acceleration.u8bit);
    #if OMV_IMU_X_Y_ROTATION_DEGREES == 0
    float yr = xl_lsb_to_mg(data_raw_acceleration.i16bit[1]); // y
    float zr = xl_lsb_to_mg(data_raw_acceleration.i16bit[2]); // z
    #elif OMV_IMU_X_Y_ROTATION_DEGREES == 90
    float yr = xl_lsb_to_mg(data_raw_acceleration.i16bit[0]); // x
    float zr = xl_lsb_to_mg(data_raw_acceleration.i16bit[2]); // z
    #elif OMV_IMU_X_Y_ROTATION_DEGREES == 180
    float yr = -xl_lsb_to_mg(data_raw_acceleration.i16bit[1]); // y
    float zr = xl_lsb_to_mg(data_raw_acceleration.i16bit[2]); // z
    #elif OMV_IMU_X_Y_ROTATION_DEGREES == 270
    float yr = -xl_lsb_to_mg(data_raw_acceleration.i16bit[0]); // x
    float zr = xl_lsb_to_mg(data_raw_acceleration.i16bit[2]); // z
    #endif
    #if OMV_IMU_MOUNTING_Z_DIRECTION == 1 // default is -1 (IMU pointing reverse of camera)
    yr = -yr;
    zr = -zr;
    #endif
    return IM_RAD2DEG(fast_atan2f(zr, -yr));
}
void py_imu_init();

static mp_obj_t py_imu_acceleration_mg() {
    error_on_not_ready();

    axis3bit16_t data_raw_acceleration = {};
    LSM_FUNC(acceleration_raw_get) (&dev_ctx, data_raw_acceleration.u8bit);
    return py_imu_tuple(xl_lsb_to_mg(data_raw_acceleration.i16bit[0]),
                        xl_lsb_to_mg(data_raw_acceleration.i16bit[1]),
                        xl_lsb_to_mg(data_raw_acceleration.i16bit[2]));
}
static MP_DEFINE_CONST_FUN_OBJ_0(py_imu_acceleration_mg_obj, py_imu_acceleration_mg);

static mp_obj_t py_imu_angular_rate_mdps() {
    error_on_not_ready();

    axis3bit16_t data_raw_angular_rate = {};
    LSM_FUNC(angular_rate_raw_get) (&dev_ctx, data_raw_angular_rate.u8bit);
    return py_imu_tuple(gy_lsb_to_mdps(data_raw_angular_rate.i16bit[0]),
                        gy_lsb_to_mdps(data_raw_angular_rate.i16bit[1]),
                        gy_lsb_to_mdps(data_raw_angular_rate.i16bit[2]));
}
static MP_DEFINE_CONST_FUN_OBJ_0(py_imu_angular_rate_mdps_obj, py_imu_angular_rate_mdps);

static mp_obj_t py_imu_temperature_c() {
    error_on_not_ready();

    axis1bit16_t data_raw_temperature = {};
    LSM_FUNC(temperature_raw_get) (&dev_ctx, data_raw_temperature.u8bit);
    return mp_obj_new_float(LSM_FUNC(from_lsb_to_celsius) (data_raw_temperature.i16bit));
}
static MP_DEFINE_CONST_FUN_OBJ_0(py_imu_temperature_c_obj, py_imu_temperature_c);

static mp_obj_t py_imu_roll() {
    error_on_not_ready();

    return mp_obj_new_float(py_imu_get_roll());
}
static MP_DEFINE_CONST_FUN_OBJ_0(py_imu_roll_obj, py_imu_roll);

static mp_obj_t py_imu_pitch() {
    error_on_not_ready();

    return mp_obj_new_float(py_imu_get_pitch());
}
static MP_DEFINE_CONST_FUN_OBJ_0(py_imu_pitch_obj, py_imu_pitch);

static mp_obj_t py_imu_sleep(mp_obj_t enable) {
    error_on_not_ready();

    bool en = mp_obj_get_int(enable);
    LSM_FUNC(xl_data_rate_set) (&dev_ctx, en ? LSM_CONST(XL_ODR_OFF) : xl_odr_code);
    LSM_FUNC(gy_data_rate_set) (&dev_ctx, en ? LSM_CONST(GY_ODR_OFF) : gy_odr_code);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(py_imu_sleep_obj, py_imu_sleep);

// imu.data_rate([hz]) — get/set the accelerometer + gyroscope output data
// rate. Pass 0 to power both axes down. The chip only supports a fixed set of
// ODRs; the request is snapped up to the next supported rate (returned).
static mp_obj_t py_imu_data_rate(size_t n_args, const mp_obj_t *args) {
    error_on_not_ready();

    if (n_args > 0) {
        uint8_t code = hz_to_odr_code(mp_obj_get_float(args[0]));
        if (LSM_FUNC(xl_data_rate_set) (&dev_ctx, code) ||
            LSM_FUNC(gy_data_rate_set) (&dev_ctx, code)) {
            mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("IMU data rate set failed"));
        }
        xl_odr_code = code;
        gy_odr_code = code;
    }
    return mp_obj_new_float(odr_code_to_hz(xl_odr_code));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(py_imu_data_rate_obj, 0, 1, py_imu_data_rate);

// imu.accel_full_scale([g]) — get/set the accelerometer full scale.
// Valid values: 2, 4, 8, 16. Returns the active full scale.
static mp_obj_t py_imu_accel_full_scale(size_t n_args, const mp_obj_t *args) {
    error_on_not_ready();

    if (n_args > 0) {
        int g = mp_obj_get_int(args[0]);
        uint8_t code;
        switch (g) {
            case 2:  code = LSM_CONST(2g);  break;
            case 4:  code = LSM_CONST(4g);  break;
            case 8:  code = LSM_CONST(8g);  break;
            case 16: code = LSM_CONST(16g); break;
            default:
                mp_raise_ValueError(MP_ERROR_TEXT("accel full scale must be 2, 4, 8, or 16"));
        }
        if (LSM_FUNC(xl_full_scale_set) (&dev_ctx, code)) {
            mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("IMU accel scale set failed"));
        }
        xl_fs_code = code;
    }
    switch (xl_fs_code) {
        case LSM_CONST(2g):  return mp_obj_new_int(2);
        case LSM_CONST(4g):  return mp_obj_new_int(4);
        case LSM_CONST(16g): return mp_obj_new_int(16);
        default:             return mp_obj_new_int(8);
    }
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(py_imu_accel_full_scale_obj, 0, 1, py_imu_accel_full_scale);

// imu.gyro_full_scale([dps]) — get/set the gyroscope full scale.
// Valid values: 125, 250, 500, 1000, 2000. Returns the active full scale.
static mp_obj_t py_imu_gyro_full_scale(size_t n_args, const mp_obj_t *args) {
    error_on_not_ready();

    if (n_args > 0) {
        int dps = mp_obj_get_int(args[0]);
        uint8_t code;
        switch (dps) {
            case 125:  code = LSM_CONST(125dps);  break;
            case 250:  code = LSM_CONST(250dps);  break;
            case 500:  code = LSM_CONST(500dps);  break;
            case 1000: code = LSM_CONST(1000dps); break;
            case 2000: code = LSM_CONST(2000dps); break;
            default:
                mp_raise_ValueError(MP_ERROR_TEXT("gyro full scale must be 125, 250, 500, 1000, or 2000"));
        }
        if (LSM_FUNC(gy_full_scale_set) (&dev_ctx, code)) {
            mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("IMU gyro scale set failed"));
        }
        gy_fs_code = code;
    }
    switch (gy_fs_code) {
        case LSM_CONST(125dps):  return mp_obj_new_int(125);
        case LSM_CONST(250dps):  return mp_obj_new_int(250);
        case LSM_CONST(500dps):  return mp_obj_new_int(500);
        case LSM_CONST(1000dps): return mp_obj_new_int(1000);
        default:                 return mp_obj_new_int(2000);
    }
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(py_imu_gyro_full_scale_obj, 0, 1, py_imu_gyro_full_scale);

static mp_obj_t py_imu_write_reg(mp_obj_t addr, mp_obj_t val) {
    error_on_not_ready();

    uint8_t v = mp_obj_get_int(val);
    LSM_FUNC(write_reg) (&dev_ctx, mp_obj_get_int(addr), &v, sizeof(v));
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(py_imu_write_reg_obj, py_imu_write_reg);

static mp_obj_t py_imu_read_reg(size_t n_args, const mp_obj_t *args) {
    error_on_not_ready();

    // Burst read into a caller-provided bytearray when buf is given.
    if (n_args > 1 && args[1] != mp_const_none) {
        mp_buffer_info_t bufinfo;
        mp_get_buffer_raise(args[1], &bufinfo, MP_BUFFER_WRITE);
        if (bufinfo.len > 0) {
            LSM_FUNC(read_reg) (&dev_ctx, mp_obj_get_int(args[0]), bufinfo.buf, bufinfo.len);
        }
        return mp_const_none;
    }

    uint8_t v;
    LSM_FUNC(read_reg) (&dev_ctx, mp_obj_get_int(args[0]), &v, sizeof(v));
    return mp_obj_new_int(v);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(py_imu_read_reg_obj, 1, 2, py_imu_read_reg);

static const mp_rom_map_elem_t globals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),            MP_OBJ_NEW_QSTR(MP_QSTR_imu) },
    { MP_ROM_QSTR(MP_QSTR_acceleration_mg),     MP_ROM_PTR(&py_imu_acceleration_mg_obj) },
    { MP_ROM_QSTR(MP_QSTR_angular_rate_mdps),   MP_ROM_PTR(&py_imu_angular_rate_mdps_obj) },
    { MP_ROM_QSTR(MP_QSTR_temperature_c),       MP_ROM_PTR(&py_imu_temperature_c_obj) },
    { MP_ROM_QSTR(MP_QSTR_roll),                MP_ROM_PTR(&py_imu_roll_obj) },
    { MP_ROM_QSTR(MP_QSTR_pitch),               MP_ROM_PTR(&py_imu_pitch_obj) },
    { MP_ROM_QSTR(MP_QSTR_sleep),               MP_ROM_PTR(&py_imu_sleep_obj) },
    { MP_ROM_QSTR(MP_QSTR_data_rate),           MP_ROM_PTR(&py_imu_data_rate_obj) },
    { MP_ROM_QSTR(MP_QSTR_accel_full_scale),    MP_ROM_PTR(&py_imu_accel_full_scale_obj) },
    { MP_ROM_QSTR(MP_QSTR_gyro_full_scale),     MP_ROM_PTR(&py_imu_gyro_full_scale_obj) },
    { MP_ROM_QSTR(MP_QSTR___write_reg),         MP_ROM_PTR(&py_imu_write_reg_obj) },
    { MP_ROM_QSTR(MP_QSTR___read_reg),          MP_ROM_PTR(&py_imu_read_reg_obj) },
};

static MP_DEFINE_CONST_DICT(globals_dict, globals_dict_table);

const mp_obj_module_t imu_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_t) &globals_dict
};

void py_imu_init() {
    uint8_t rst = 1;
    uint8_t whoamI = 0;

    platform_init(&imu_bus);

    // Try to read device id...
    for (int i = 0; (i < 10) && (whoamI != LSM_CONST(ID)); i++) {
        LSM_FUNC(device_id_get) (&dev_ctx, &whoamI);
        mp_event_wait_ms(1);
    }

    if (whoamI != LSM_CONST(ID)) {
        platform_deinit(&imu_bus);
        return;
    }

    LSM_FUNC(reset_set) (&dev_ctx, PROPERTY_ENABLE);

    for (int i = 0; (i < 10000) && rst; i++) {
        LSM_FUNC(reset_get) (&dev_ctx, &rst);
    }

    if (rst) {
        platform_deinit(&imu_bus);
        return;
    }

    LSM_FUNC(block_data_update_set) (&dev_ctx, PROPERTY_ENABLE);

    xl_odr_code = LSM_CONST(XL_ODR_52Hz);
    gy_odr_code = LSM_CONST(GY_ODR_52Hz);
    xl_fs_code  = LSM_CONST(8g);
    gy_fs_code  = LSM_CONST(2000dps);

    LSM_FUNC(xl_data_rate_set) (&dev_ctx, xl_odr_code);
    LSM_FUNC(gy_data_rate_set) (&dev_ctx, gy_odr_code);

    LSM_FUNC(xl_full_scale_set) (&dev_ctx, xl_fs_code);
    LSM_FUNC(gy_full_scale_set) (&dev_ctx, gy_fs_code);

    imu_initialized = true;
}

float py_imu_roll_rotation() {
    if (imu_initialized) {
        return py_imu_get_roll();
    }
    return 0.0f;
}

float py_imu_pitch_rotation() {
    if (imu_initialized) {
        return py_imu_get_pitch();
    }
    return 0.0f;
}

MP_REGISTER_MODULE(MP_QSTR_imu, imu_module);
#endif // MICROPY_PY_IMU
