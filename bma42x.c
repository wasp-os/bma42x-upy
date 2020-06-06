/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 Daniel Thompson
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
 */

#include <stdio.h>
#include <string.h>

#include "py/obj.h"
#include "py/objmodule.h"
#include "py/runtime.h"
#include "py/builtin.h"
#include "py/mphal.h"
#include "extmod/machine_i2c.h"
#include "bma421.h"

STATIC const mp_obj_type_t bma42x_BMA42X_type;

// this is the actual C-structure for our new object
typedef struct _bma42x_BMA42X_obj_t {
    mp_obj_base_t base;

    mp_obj_base_t *i2c_obj;
    uint16_t i2c_addr;
    bool debug;

    struct bma4_dev dev;
} bma42x_BMA42X_obj_t;

STATIC void check_result(int res)
{
    switch (res) {
    case BMA4_OK:
	return;
    case BMA4_E_NULL_PTR:
	mp_raise_ValueError("null pointer");
	// noreturn
    case BMA4_E_OUT_OF_RANGE:
	mp_raise_ValueError("out of range");
	// noreturn
    case BMA4_E_INVALID_SENSOR:
	mp_raise_ValueError("invalid sensor");
	// noreturn
    case BMA4_E_CONFIG_STREAM_ERROR:
	mp_raise_ValueError("config stream error");
	// noreturn
    case BMA4_E_SELF_TEST_FAIL:
	mp_raise_ValueError("self test fail");
	// noreturn
    case BMA4_E_COM_FAIL:
	mp_raise_ValueError("comms failure");
	// noreturn
    default:
	mp_raise_ValueError("unknown error");
    }
}

STATIC int i2c_transfer(mp_obj_base_t *i2c_obj, uint16_t addr, size_t n,
		        mp_machine_i2c_buf_t *bufs, unsigned int flags)
{
    mp_machine_i2c_p_t *i2c_p = (mp_machine_i2c_p_t*)i2c_obj->type->protocol;

    int mp_res = i2c_p->transfer(i2c_obj, addr, n, bufs, flags);

    return mp_res < 0 ? BMA4_E_COM_FAIL : BMA4_OK;
}

STATIC void hexdump(const uint8_t *buf, unsigned int len)
{
    for (unsigned int i=0; i<len; i++)
	printf("%02x", buf[i]);
}

STATIC int8_t i2c_reg_write(uint8_t reg_addr, const uint8_t *reg_data,
		              uint32_t length, void *intf_ptr)
{
    bma42x_BMA42X_obj_t *self = intf_ptr;
    mp_machine_i2c_buf_t bufs[2] = {
        { .buf = &reg_addr, .len = 1 },
        { .buf = (uint8_t *) reg_data, .len = length },
    };

    if (self->debug) {
	printf("BMA42x write: %u %02x [", self->i2c_addr, reg_addr);
	hexdump(reg_data, length);
	printf("]\n");
    }
    return i2c_transfer(self->i2c_obj, self->i2c_addr,
		        2, bufs, MP_MACHINE_I2C_FLAG_STOP);
}

STATIC int8_t i2c_reg_read(uint8_t reg_addr, uint8_t *reg_data,
		           uint32_t length, void *intf_ptr)
{
    bma42x_BMA42X_obj_t *self = intf_ptr;
    mp_machine_i2c_buf_t buf;
    int res;

    if (self->debug)
	printf("BMA42x read: %u %02x [", self->i2c_addr, reg_addr);

    /* send register address */
    buf.buf = &reg_addr;
    buf.len = 1;
    res = i2c_transfer(self->i2c_obj, self->i2c_addr, 1, &buf, 0);
    if (0 != res)
	return res;

    /* STOP */
    buf.buf = NULL;
    buf.len = 0;
    res = i2c_transfer(self->i2c_obj, self->i2c_addr,
		       1, &buf, MP_MACHINE_I2C_FLAG_STOP);
    if (0 != res)
	return res;

    /* read the data */
    buf.buf = reg_data;
    buf.len = length;
    res = i2c_transfer(self->i2c_obj, self->i2c_addr, 1, &buf,
	               MP_MACHINE_I2C_FLAG_READ | MP_MACHINE_I2C_FLAG_STOP);
    if (self->debug) {
	if (0 == res)
	    hexdump(reg_data, length);
	printf("]\n");
    }

    return res;
}

STATIC void delay_us(uint32_t period, void *intf_ptr)
{
    mp_hal_delay_us(period);
}

#define define_bma42x_BMA42X_get(arg_t, prefix, getter) \
STATIC mp_obj_t bma42x_BMA42X_##getter(mp_obj_t self_in) \
{ \
    bma42x_BMA42X_obj_t *self = MP_OBJ_TO_PTR(self_in); \
    arg_t arg; \
    check_result(prefix##_##getter(&arg, &self->dev)); \
    return MP_OBJ_NEW_SMALL_INT(arg); \
} \
STATIC MP_DEFINE_CONST_FUN_OBJ_1(bma42x_BMA42X_##getter##_obj, \
	                         bma42x_BMA42X_##getter)

#define define_bma42x_BMA42X_set(arg_t, prefix, setter) \
STATIC mp_obj_t bma42x_BMA42X_##setter(mp_obj_t self_in, mp_obj_t arg_in) \
{ \
    bma42x_BMA42X_obj_t *self = MP_OBJ_TO_PTR(self_in); \
    arg_t arg = mp_obj_get_int(arg_in); \
    check_result(prefix##_##setter(arg, &self->dev)); \
    return mp_const_none; \
} \
STATIC MP_DEFINE_CONST_FUN_OBJ_2(bma42x_BMA42X_##setter##_obj, \
	                         bma42x_BMA42X_##setter)

mp_obj_t bma42x_BMA42X_make_new(const mp_obj_type_t *type, size_t n_args,
                                size_t n_kw, const mp_obj_t *all_args )
{
    enum { ARG_i2c, };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_i2c, MP_ARG_OBJ | MP_ARG_REQUIRED },
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all_kw_array(n_args, n_kw, all_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    // create new object
    bma42x_BMA42X_obj_t *self = m_new_obj(bma42x_BMA42X_obj_t);
    self->base.type = &bma42x_BMA42X_type;

    // set parameters
    mp_obj_base_t *i2c_obj = (mp_obj_base_t*)MP_OBJ_TO_PTR(args[ARG_i2c].u_obj);
    self->i2c_obj = i2c_obj;
    self->i2c_addr = BMA4_I2C_ADDR_PRIMARY;
    self->debug = false;

    memset(&self->dev, 0, sizeof(struct bma4_dev));
    self->dev.intf = BMA4_I2C_INTF;
    self->dev.intf_ptr = self;
    self->dev.bus_read = i2c_reg_read;
    self->dev.bus_write = i2c_reg_write;
    self->dev.delay_us = delay_us;
    self->dev.read_write_len = 8;

    return MP_OBJ_FROM_PTR(self);
}

STATIC void bma42x_BMA42X_print(const mp_print_t *print, mp_obj_t self_in,
	                        mp_print_kind_t kind )
{
    (void)kind;
    bma42x_BMA42X_obj_t *self = MP_OBJ_TO_PTR(self_in);
    mp_printf(print, "<BMA42X i2c=%p>", self->i2c_obj);
}

STATIC mp_obj_t bma42x_BMA42X_debug(mp_obj_t self_in, mp_obj_t enable_in)
{
    bma42x_BMA42X_obj_t *self = MP_OBJ_TO_PTR(self_in);
    mp_int_t enable = mp_obj_get_int(enable_in);

    self->debug = enable;

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(bma42x_BMA42X_debug_obj,
	                         bma42x_BMA42X_debug);

STATIC mp_obj_t bma42x_BMA42X_feature_enable(
		mp_obj_t self_in, mp_obj_t feature_in, mp_obj_t enable_in)
{
    bma42x_BMA42X_obj_t *self = MP_OBJ_TO_PTR(self_in);
    uint8_t feature = mp_obj_get_int(feature_in);
    bool enable = mp_obj_get_int(enable_in);

    check_result(bma421_feature_enable(feature, enable, &self->dev));

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(bma42x_BMA42X_feature_enable_obj,
	                         bma42x_BMA42X_feature_enable);

define_bma42x_BMA42X_get(uint8_t, bma4, get_offset_comp);

STATIC mp_obj_t bma42x_BMA42X_get_reg(mp_obj_t self_in, mp_obj_t reg_in)
{
    bma42x_BMA42X_obj_t *self = MP_OBJ_TO_PTR(self_in);
    uint8_t reg = mp_obj_get_int(reg_in);
    uint8_t val;

    check_result(bma4_read_regs(reg, &val, 1u, &self->dev));

    return MP_OBJ_NEW_SMALL_INT(val);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(bma42x_BMA42X_get_reg_obj,
	                         bma42x_BMA42X_get_reg);

STATIC mp_obj_t bma42x_BMA42X_get_temperature(mp_obj_t self_in, mp_obj_t temp_unit_in)
{
    bma42x_BMA42X_obj_t *self = MP_OBJ_TO_PTR(self_in);
    uint8_t temp_unit = mp_obj_get_int(temp_unit_in);
    int32_t temp;

    check_result(bma4_get_temperature(&temp, temp_unit, &self->dev));

    return MP_OBJ_NEW_SMALL_INT(temp);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(bma42x_BMA42X_get_temperature_obj,
	                         bma42x_BMA42X_get_temperature);


STATIC mp_obj_t bma42x_BMA42X_init(mp_obj_t self_in)
{
    bma42x_BMA42X_obj_t *self = MP_OBJ_TO_PTR(self_in);

    check_result(bma421_init(&self->dev));

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(bma42x_BMA42X_init_obj, bma42x_BMA42X_init);

STATIC mp_obj_t bma42x_BMA42X_map_interrupt(size_t n_args, const mp_obj_t *args)
{
    bma42x_BMA42X_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    uint8_t int_line = mp_obj_get_int(args[1]);
    uint16_t int_map = mp_obj_get_int(args[2]);
    bool enable = mp_obj_get_int(args[3]);

    check_result(bma421_map_interrupt(int_line, int_map, enable, &self->dev));

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR(bma42x_BMA42X_map_interrupt_obj, 4,
	                           bma42x_BMA42X_map_interrupt);

STATIC mp_obj_t bma42x_BMA42X_read_accel_xyz(mp_obj_t self_in)
{
    bma42x_BMA42X_obj_t *self = MP_OBJ_TO_PTR(self_in);
    struct bma4_accel data;

    check_result(bma4_read_accel_xyz(&data, &self->dev));

    mp_obj_t tuple[3] = {
	MP_OBJ_NEW_SMALL_INT(data.x),
	MP_OBJ_NEW_SMALL_INT(data.y),
	MP_OBJ_NEW_SMALL_INT(data.z),
    };
    return mp_obj_new_tuple(3, tuple);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(bma42x_BMA42X_read_accel_xyz_obj,
				 bma42x_BMA42X_read_accel_xyz);

define_bma42x_BMA42X_get(uint16_t, bma421, read_int_status);

STATIC mp_obj_t bma42x_BMA42X_set_accel_config(size_t n_args, const mp_obj_t *args,
	                                       mp_map_t *kw_args)
{
    bma42x_BMA42X_obj_t *self = MP_OBJ_TO_PTR(args[0]);

    enum { ARG_odr, ARG_range, ARG_bandwidth, ARG_perf_mode, };
    static const mp_arg_t allowed_args[] = {
	{ MP_QSTR_odr,       MP_ARG_INT, {.u_int = BMA4_OUTPUT_DATA_RATE_100HZ } },
	{ MP_QSTR_range,     MP_ARG_INT, {.u_int = BMA4_ACCEL_RANGE_2G } },
	{ MP_QSTR_bandwidth, MP_ARG_INT, {.u_int = BMA4_ACCEL_NORMAL_AVG4 } },
	{ MP_QSTR_perf_mode, MP_ARG_INT, {.u_int = BMA4_CIC_AVG_MODE } },
    };
    mp_arg_val_t vals[ARG_perf_mode+1];
    mp_arg_parse_all(n_args-1, args+1, kw_args, ARG_perf_mode+1, allowed_args, vals);

    struct bma4_accel_config conf = {
        .odr = vals[ARG_odr].u_int,
        .range = vals[ARG_range].u_int,
        .bandwidth = vals[ARG_bandwidth].u_int,
        .perf_mode = vals[ARG_perf_mode].u_int,
    };
    check_result(bma4_set_accel_config(&conf, &self->dev));

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(bma42x_BMA42X_set_accel_config_obj, 1,
	                          bma42x_BMA42X_set_accel_config);

STATIC bma42x_BMA42X_obj_t *parse_args_any_no_mot(
		size_t n_args, const mp_obj_t *args, mp_map_t *kw_args,
		struct bma421_any_no_mot_config *config)
{
    bma42x_BMA42X_obj_t *self = MP_OBJ_TO_PTR(args[0]);

    static const mp_arg_t allowed_args[] = {
	{ MP_QSTR_duration,  MP_ARG_INT, {.u_int = 5 } },
	{ MP_QSTR_threshold, MP_ARG_INT, {.u_int = 0xaa } },
	{ MP_QSTR_axes_en,   MP_ARG_INT, {.u_int = BMA421_EN_ALL_AXIS } },
    };
    mp_arg_val_t vals[3];
    mp_arg_parse_all(n_args-1, args+1, kw_args, 3, allowed_args, vals);

    // order must match allowed_args
    config->duration = vals[0].u_int;
    config->threshold = vals[1].u_int;
    config->axes_en = vals[2].u_int;

    return self;
}

STATIC mp_obj_t bma42x_BMA42X_set_any_mot_config(size_t n_args, const mp_obj_t *args,
	                                         mp_map_t *kw_args)
{
    struct bma421_any_no_mot_config conf;
    bma42x_BMA42X_obj_t *self = parse_args_any_no_mot(n_args, args, kw_args, &conf);

    check_result(bma421_set_any_mot_config(&conf, &self->dev));

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(bma42x_BMA42X_set_any_mot_config_obj, 1,
	                          bma42x_BMA42X_set_any_mot_config);

STATIC mp_obj_t bma42x_BMA42X_set_no_mot_config(size_t n_args, const mp_obj_t *args,
	                                         mp_map_t *kw_args)
{
    struct bma421_any_no_mot_config conf;
    bma42x_BMA42X_obj_t *self = parse_args_any_no_mot(n_args, args, kw_args, &conf);

    check_result(bma421_set_no_mot_config(&conf, &self->dev));

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(bma42x_BMA42X_set_no_mot_config_obj, 1,
	                          bma42x_BMA42X_set_no_mot_config);

define_bma42x_BMA42X_set(bool, bma4, set_accel_enable);
define_bma42x_BMA42X_set(uint8_t, bma4, set_command_register);
define_bma42x_BMA42X_set(uint8_t, bma4, set_offset_comp);

STATIC mp_obj_t bma42x_BMA42X_set_reg(mp_obj_t self_in, mp_obj_t reg_in,
				      mp_obj_t val_in)
{
    bma42x_BMA42X_obj_t *self = MP_OBJ_TO_PTR(self_in);
    uint8_t reg = mp_obj_get_int(reg_in);
    uint8_t val = mp_obj_get_int(val_in);

    check_result(bma4_write_regs(reg, &val, 1u, &self->dev));

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(bma42x_BMA42X_set_reg_obj,
	                         bma42x_BMA42X_set_reg);

define_bma42x_BMA42X_set(bool, bma421, step_detector_enable);
define_bma42x_BMA42X_get(uint32_t, bma421, step_counter_output);
define_bma42x_BMA42X_set(uint8_t, bma421, step_counter_set_watermark);

STATIC mp_obj_t bma42x_BMA42X_write_config_file(mp_obj_t self_in)
{
    bma42x_BMA42X_obj_t *self = MP_OBJ_TO_PTR(self_in);

    check_result(bma421_write_config_file(&self->dev));

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(bma42x_BMA42X_write_config_file_obj,
	                         bma42x_BMA42X_write_config_file);

#define BMA4_EXPORT_OBJ(x) \
    { MP_ROM_QSTR(MP_QSTR_##x), MP_ROM_PTR(&bma42x_BMA42X_##x##_obj) }

STATIC const mp_rom_map_elem_t bma42x_BMA42X_locals_dict_table[] = {
    BMA4_EXPORT_OBJ(debug),
    BMA4_EXPORT_OBJ(feature_enable),
    BMA4_EXPORT_OBJ(get_offset_comp),
    BMA4_EXPORT_OBJ(get_reg),
    BMA4_EXPORT_OBJ(get_temperature),
    BMA4_EXPORT_OBJ(init),
    BMA4_EXPORT_OBJ(map_interrupt),
    BMA4_EXPORT_OBJ(read_accel_xyz),
    BMA4_EXPORT_OBJ(read_int_status),
    BMA4_EXPORT_OBJ(set_accel_config),
    BMA4_EXPORT_OBJ(set_accel_enable),
    BMA4_EXPORT_OBJ(set_any_mot_config),
    BMA4_EXPORT_OBJ(set_no_mot_config),
    BMA4_EXPORT_OBJ(set_command_register),
    BMA4_EXPORT_OBJ(set_offset_comp),
    BMA4_EXPORT_OBJ(set_reg),
    BMA4_EXPORT_OBJ(step_detector_enable),
    BMA4_EXPORT_OBJ(step_counter_output),
    BMA4_EXPORT_OBJ(step_counter_set_watermark),
    BMA4_EXPORT_OBJ(write_config_file),
};
STATIC MP_DEFINE_CONST_DICT(bma42x_BMA42X_locals_dict, bma42x_BMA42X_locals_dict_table);

STATIC const mp_obj_type_t bma42x_BMA42X_type = {
    { &mp_type_type },
    .name = MP_QSTR_BMA42X,
    .print = bma42x_BMA42X_print,
    .make_new = bma42x_BMA42X_make_new,
    .locals_dict = (mp_obj_dict_t*)&bma42x_BMA42X_locals_dict,
};

#define BMA4_EXPORT_CONST(x) \
    { MP_ROM_QSTR(MP_QSTR_##x), MP_ROM_INT(BMA4_##x) }
#define BMA421_EXPORT_CONST(x) \
    { MP_ROM_QSTR(MP_QSTR_##x), MP_ROM_INT(BMA421_##x) }

STATIC const mp_map_elem_t bma42x_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_OBJ_NEW_QSTR(MP_QSTR_bma42x) },
    { MP_ROM_QSTR(MP_QSTR_BMA42X), (mp_obj_t)&bma42x_BMA42X_type },

    // Registers
    BMA4_EXPORT_CONST(ACCEL_CONFIG_ADDR),
    BMA4_EXPORT_CONST(POWER_CONF_ADDR),
    BMA4_EXPORT_CONST(POWER_CTRL_ADDR),
    BMA4_EXPORT_CONST(NV_CONFIG_ADDR),

    // Conf values
    BMA4_EXPORT_CONST(ACCEL_RANGE_2G),
    BMA4_EXPORT_CONST(ACCEL_RANGE_4G),
    BMA4_EXPORT_CONST(ACCEL_RANGE_8G),
    BMA4_EXPORT_CONST(ACCEL_RANGE_16G),
    BMA4_EXPORT_CONST(ACCEL_NORMAL_AVG4),
    BMA4_EXPORT_CONST(OUTPUT_DATA_RATE_50HZ),
    BMA4_EXPORT_CONST(OUTPUT_DATA_RATE_100HZ),
    BMA4_EXPORT_CONST(CIC_AVG_MODE),
    BMA4_EXPORT_CONST(CONTINUOUS_MODE),

    // Temp unit
    BMA4_EXPORT_CONST(SCALE_TEMP),
    BMA4_EXPORT_CONST(DEG),
    BMA4_EXPORT_CONST(FAHREN),
    BMA4_EXPORT_CONST(KELVIN),

    BMA421_EXPORT_CONST(STEP_CNTR),
    BMA421_EXPORT_CONST(STEP_ACT),
    BMA421_EXPORT_CONST(WRIST_WEAR),
    BMA421_EXPORT_CONST(SINGLE_TAP),
    BMA421_EXPORT_CONST(DOUBLE_TAP),

    BMA4_EXPORT_CONST(INTR1_MAP),
    BMA4_EXPORT_CONST(INTR2_MAP),

    BMA421_EXPORT_CONST(STEP_CNTR_INT),
    BMA421_EXPORT_CONST(ANY_MOT_INT),
    BMA421_EXPORT_CONST(NO_MOT_INT),

    BMA421_EXPORT_CONST(DIS_ALL_AXIS),
    BMA421_EXPORT_CONST(X_AXIS_EN),
    BMA421_EXPORT_CONST(Y_AXIS_EN),
    BMA421_EXPORT_CONST(Z_AXIS_EN),
    BMA421_EXPORT_CONST(EN_ALL_AXIS),
};

STATIC MP_DEFINE_CONST_DICT (mp_module_bma42x_globals, bma42x_module_globals_table );

const mp_obj_module_t mp_module_bma42x = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&mp_module_bma42x_globals,
};

MP_REGISTER_MODULE(MP_QSTR_bma42x, mp_module_bma42x, MODULE_BMA42X_ENABLED);
