/**
 * The MIT License (MIT)
 *
 * Copyright (c) 2018-2019 Erik Moqvist
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * This file was generated by cantools version 37.1.0 Sat Aug 13 19:55:59 2022.
 */

#include <string.h>

#include "megasquirt_simplified_dash_broadcast.h"

static inline uint8_t pack_left_shift_u8(
    uint8_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint8_t)((uint8_t)(value << shift) & mask);
}

static inline uint8_t pack_left_shift_u16(
    uint16_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint8_t)((uint8_t)(value << shift) & mask);
}

static inline uint8_t pack_right_shift_u16(
    uint16_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint8_t)((uint8_t)(value >> shift) & mask);
}

static inline uint16_t unpack_left_shift_u16(
    uint8_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint16_t)((uint16_t)(value & mask) << shift);
}

static inline uint8_t unpack_right_shift_u8(
    uint8_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint8_t)((uint8_t)(value & mask) >> shift);
}

static inline uint16_t unpack_right_shift_u16(
    uint8_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint16_t)((uint16_t)(value & mask) >> shift);
}

int megasquirt_simplified_dash_broadcast_megasquirt_dash4_pack(
    uint8_t *dst_p,
    const struct megasquirt_simplified_dash_broadcast_megasquirt_dash4_t *src_p,
    size_t size)
{
    uint16_t launch_timing;
    uint16_t tc_retard;

    if (size < 8u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 8);

    dst_p[0] |= pack_right_shift_u16(src_p->vss1, 8u, 0xffu);
    dst_p[1] |= pack_left_shift_u16(src_p->vss1, 0u, 0xffu);
    tc_retard = (uint16_t)src_p->tc_retard;
    dst_p[2] |= pack_right_shift_u16(tc_retard, 8u, 0xffu);
    dst_p[3] |= pack_left_shift_u16(tc_retard, 0u, 0xffu);
    launch_timing = (uint16_t)src_p->launch_timing;
    dst_p[4] |= pack_right_shift_u16(launch_timing, 8u, 0xffu);
    dst_p[5] |= pack_left_shift_u16(launch_timing, 0u, 0xffu);

    return (8);
}

int megasquirt_simplified_dash_broadcast_megasquirt_dash4_unpack(
    struct megasquirt_simplified_dash_broadcast_megasquirt_dash4_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    uint16_t launch_timing;
    uint16_t tc_retard;

    if (size < 8u) {
        return (-EINVAL);
    }

    dst_p->vss1 = unpack_left_shift_u16(src_p[0], 8u, 0xffu);
    dst_p->vss1 |= unpack_right_shift_u16(src_p[1], 0u, 0xffu);
    tc_retard = unpack_left_shift_u16(src_p[2], 8u, 0xffu);
    tc_retard |= unpack_right_shift_u16(src_p[3], 0u, 0xffu);
    dst_p->tc_retard = (int16_t)tc_retard;
    launch_timing = unpack_left_shift_u16(src_p[4], 8u, 0xffu);
    launch_timing |= unpack_right_shift_u16(src_p[5], 0u, 0xffu);
    dst_p->launch_timing = (int16_t)launch_timing;

    return (0);
}

uint16_t megasquirt_simplified_dash_broadcast_megasquirt_dash4_vss1_encode(float value)
{
    return (uint16_t)(value / 0.1f);
}

float megasquirt_simplified_dash_broadcast_megasquirt_dash4_vss1_decode(uint16_t value)
{
    return ((float)value * 0.1f);
}

bool megasquirt_simplified_dash_broadcast_megasquirt_dash4_vss1_is_in_range(uint16_t value)
{
    (void)value;

    return (true);
}

int16_t megasquirt_simplified_dash_broadcast_megasquirt_dash4_tc_retard_encode(float value)
{
    return (int16_t)(value / 0.1f);
}

float megasquirt_simplified_dash_broadcast_megasquirt_dash4_tc_retard_decode(int16_t value)
{
    return ((float)value * 0.1f);
}

bool megasquirt_simplified_dash_broadcast_megasquirt_dash4_tc_retard_is_in_range(int16_t value)
{
    (void)value;

    return (true);
}

int16_t megasquirt_simplified_dash_broadcast_megasquirt_dash4_launch_timing_encode(float value)
{
    return (int16_t)(value / 0.1f);
}

float megasquirt_simplified_dash_broadcast_megasquirt_dash4_launch_timing_decode(int16_t value)
{
    return ((float)value * 0.1f);
}

bool megasquirt_simplified_dash_broadcast_megasquirt_dash4_launch_timing_is_in_range(int16_t value)
{
    (void)value;

    return (true);
}

int megasquirt_simplified_dash_broadcast_megasquirt_dash3_pack(
    uint8_t *dst_p,
    const struct megasquirt_simplified_dash_broadcast_megasquirt_dash3_t *src_p,
    size_t size)
{
    uint16_t batt;
    uint16_t sensors1;
    uint16_t sensors2;

    if (size < 8u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 8);

    batt = (uint16_t)src_p->batt;
    dst_p[0] |= pack_right_shift_u16(batt, 8u, 0xffu);
    dst_p[1] |= pack_left_shift_u16(batt, 0u, 0xffu);
    sensors1 = (uint16_t)src_p->sensors1;
    dst_p[2] |= pack_right_shift_u16(sensors1, 8u, 0xffu);
    dst_p[3] |= pack_left_shift_u16(sensors1, 0u, 0xffu);
    sensors2 = (uint16_t)src_p->sensors2;
    dst_p[4] |= pack_right_shift_u16(sensors2, 8u, 0xffu);
    dst_p[5] |= pack_left_shift_u16(sensors2, 0u, 0xffu);
    dst_p[6] |= pack_left_shift_u8(src_p->knk_rtd, 0u, 0xffu);

    return (8);
}

int megasquirt_simplified_dash_broadcast_megasquirt_dash3_unpack(
    struct megasquirt_simplified_dash_broadcast_megasquirt_dash3_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    uint16_t batt;
    uint16_t sensors1;
    uint16_t sensors2;

    if (size < 8u) {
        return (-EINVAL);
    }

    batt = unpack_left_shift_u16(src_p[0], 8u, 0xffu);
    batt |= unpack_right_shift_u16(src_p[1], 0u, 0xffu);
    dst_p->batt = (int16_t)batt;
    sensors1 = unpack_left_shift_u16(src_p[2], 8u, 0xffu);
    sensors1 |= unpack_right_shift_u16(src_p[3], 0u, 0xffu);
    dst_p->sensors1 = (int16_t)sensors1;
    sensors2 = unpack_left_shift_u16(src_p[4], 8u, 0xffu);
    sensors2 |= unpack_right_shift_u16(src_p[5], 0u, 0xffu);
    dst_p->sensors2 = (int16_t)sensors2;
    dst_p->knk_rtd = unpack_right_shift_u8(src_p[6], 0u, 0xffu);

    return (0);
}

int16_t megasquirt_simplified_dash_broadcast_megasquirt_dash3_batt_encode(float value)
{
    return (int16_t)(value / 0.1f);
}

float megasquirt_simplified_dash_broadcast_megasquirt_dash3_batt_decode(int16_t value)
{
    return ((float)value * 0.1f);
}

bool megasquirt_simplified_dash_broadcast_megasquirt_dash3_batt_is_in_range(int16_t value)
{
    (void)value;

    return (true);
}

int16_t megasquirt_simplified_dash_broadcast_megasquirt_dash3_sensors1_encode(float value)
{
    return (int16_t)(value / 0.01f);
}

float megasquirt_simplified_dash_broadcast_megasquirt_dash3_sensors1_decode(int16_t value)
{
    return ((float)value * 0.01f);
}

bool megasquirt_simplified_dash_broadcast_megasquirt_dash3_sensors1_is_in_range(int16_t value)
{
    (void)value;

    return (true);
}

int16_t megasquirt_simplified_dash_broadcast_megasquirt_dash3_sensors2_encode(float value)
{
    return (int16_t)(value / 0.01f);
}

float megasquirt_simplified_dash_broadcast_megasquirt_dash3_sensors2_decode(int16_t value)
{
    return ((float)value * 0.01f);
}

bool megasquirt_simplified_dash_broadcast_megasquirt_dash3_sensors2_is_in_range(int16_t value)
{
    (void)value;

    return (true);
}

uint8_t megasquirt_simplified_dash_broadcast_megasquirt_dash3_knk_rtd_encode(float value)
{
    return (uint8_t)(value / 0.1f);
}

float megasquirt_simplified_dash_broadcast_megasquirt_dash3_knk_rtd_decode(uint8_t value)
{
    return ((float)value * 0.1f);
}

bool megasquirt_simplified_dash_broadcast_megasquirt_dash3_knk_rtd_is_in_range(uint8_t value)
{
    (void)value;

    return (true);
}

int megasquirt_simplified_dash_broadcast_megasquirt_dash2_pack(
    uint8_t *dst_p,
    const struct megasquirt_simplified_dash_broadcast_megasquirt_dash2_t *src_p,
    size_t size)
{
    uint16_t egocor1;
    uint16_t egt1;
    uint16_t pwseq1;

    if (size < 8u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 8);

    dst_p[0] |= pack_left_shift_u8(src_p->afrtgt1, 0u, 0xffu);
    dst_p[1] |= pack_left_shift_u8(src_p->afr1, 0u, 0xffu);
    egocor1 = (uint16_t)src_p->egocor1;
    dst_p[2] |= pack_right_shift_u16(egocor1, 8u, 0xffu);
    dst_p[3] |= pack_left_shift_u16(egocor1, 0u, 0xffu);
    egt1 = (uint16_t)src_p->egt1;
    dst_p[4] |= pack_right_shift_u16(egt1, 8u, 0xffu);
    dst_p[5] |= pack_left_shift_u16(egt1, 0u, 0xffu);
    pwseq1 = (uint16_t)src_p->pwseq1;
    dst_p[6] |= pack_right_shift_u16(pwseq1, 8u, 0xffu);
    dst_p[7] |= pack_left_shift_u16(pwseq1, 0u, 0xffu);

    return (8);
}

int megasquirt_simplified_dash_broadcast_megasquirt_dash2_unpack(
    struct megasquirt_simplified_dash_broadcast_megasquirt_dash2_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    uint16_t egocor1;
    uint16_t egt1;
    uint16_t pwseq1;

    if (size < 8u) {
        return (-EINVAL);
    }

    dst_p->afrtgt1 = unpack_right_shift_u8(src_p[0], 0u, 0xffu);
    dst_p->afr1 = unpack_right_shift_u8(src_p[1], 0u, 0xffu);
    egocor1 = unpack_left_shift_u16(src_p[2], 8u, 0xffu);
    egocor1 |= unpack_right_shift_u16(src_p[3], 0u, 0xffu);
    dst_p->egocor1 = (int16_t)egocor1;
    egt1 = unpack_left_shift_u16(src_p[4], 8u, 0xffu);
    egt1 |= unpack_right_shift_u16(src_p[5], 0u, 0xffu);
    dst_p->egt1 = (int16_t)egt1;
    pwseq1 = unpack_left_shift_u16(src_p[6], 8u, 0xffu);
    pwseq1 |= unpack_right_shift_u16(src_p[7], 0u, 0xffu);
    dst_p->pwseq1 = (int16_t)pwseq1;

    return (0);
}

uint8_t megasquirt_simplified_dash_broadcast_megasquirt_dash2_afrtgt1_encode(float value)
{
    return (uint8_t)(value / 0.1f);
}

float megasquirt_simplified_dash_broadcast_megasquirt_dash2_afrtgt1_decode(uint8_t value)
{
    return ((float)value * 0.1f);
}

bool megasquirt_simplified_dash_broadcast_megasquirt_dash2_afrtgt1_is_in_range(uint8_t value)
{
    (void)value;

    return (true);
}

uint8_t megasquirt_simplified_dash_broadcast_megasquirt_dash2_afr1_encode(float value)
{
    return (uint8_t)(value / 0.1f);
}

float megasquirt_simplified_dash_broadcast_megasquirt_dash2_afr1_decode(uint8_t value)
{
    return ((float)value * 0.1f);
}

bool megasquirt_simplified_dash_broadcast_megasquirt_dash2_afr1_is_in_range(uint8_t value)
{
    (void)value;

    return (true);
}

int16_t megasquirt_simplified_dash_broadcast_megasquirt_dash2_egocor1_encode(float value)
{
    return (int16_t)(value / 0.1f);
}

float megasquirt_simplified_dash_broadcast_megasquirt_dash2_egocor1_decode(int16_t value)
{
    return ((float)value * 0.1f);
}

bool megasquirt_simplified_dash_broadcast_megasquirt_dash2_egocor1_is_in_range(int16_t value)
{
    (void)value;

    return (true);
}

int16_t megasquirt_simplified_dash_broadcast_megasquirt_dash2_egt1_encode(float value)
{
    return (int16_t)(value / 0.1f);
}

float megasquirt_simplified_dash_broadcast_megasquirt_dash2_egt1_decode(int16_t value)
{
    return ((float)value * 0.1f);
}

bool megasquirt_simplified_dash_broadcast_megasquirt_dash2_egt1_is_in_range(int16_t value)
{
    (void)value;

    return (true);
}

int16_t megasquirt_simplified_dash_broadcast_megasquirt_dash2_pwseq1_encode(float value)
{
    return (int16_t)(value / 0.001f);
}

float megasquirt_simplified_dash_broadcast_megasquirt_dash2_pwseq1_decode(int16_t value)
{
    return ((float)value * 0.001f);
}

bool megasquirt_simplified_dash_broadcast_megasquirt_dash2_pwseq1_is_in_range(int16_t value)
{
    (void)value;

    return (true);
}

int megasquirt_simplified_dash_broadcast_megasquirt_dash1_pack(
    uint8_t *dst_p,
    const struct megasquirt_simplified_dash_broadcast_megasquirt_dash1_t *src_p,
    size_t size)
{
    uint16_t adv_deg;
    uint16_t mat;

    if (size < 8u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 8);

    dst_p[0] |= pack_right_shift_u16(src_p->pw1, 8u, 0xffu);
    dst_p[1] |= pack_left_shift_u16(src_p->pw1, 0u, 0xffu);
    dst_p[2] |= pack_right_shift_u16(src_p->pw2, 8u, 0xffu);
    dst_p[3] |= pack_left_shift_u16(src_p->pw2, 0u, 0xffu);
    mat = (uint16_t)src_p->mat;
    dst_p[4] |= pack_right_shift_u16(mat, 8u, 0xffu);
    dst_p[5] |= pack_left_shift_u16(mat, 0u, 0xffu);
    adv_deg = (uint16_t)src_p->adv_deg;
    dst_p[6] |= pack_right_shift_u16(adv_deg, 8u, 0xffu);
    dst_p[7] |= pack_left_shift_u16(adv_deg, 0u, 0xffu);

    return (8);
}

int megasquirt_simplified_dash_broadcast_megasquirt_dash1_unpack(
    struct megasquirt_simplified_dash_broadcast_megasquirt_dash1_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    uint16_t adv_deg;
    uint16_t mat;

    if (size < 8u) {
        return (-EINVAL);
    }

    dst_p->pw1 = unpack_left_shift_u16(src_p[0], 8u, 0xffu);
    dst_p->pw1 |= unpack_right_shift_u16(src_p[1], 0u, 0xffu);
    dst_p->pw2 = unpack_left_shift_u16(src_p[2], 8u, 0xffu);
    dst_p->pw2 |= unpack_right_shift_u16(src_p[3], 0u, 0xffu);
    mat = unpack_left_shift_u16(src_p[4], 8u, 0xffu);
    mat |= unpack_right_shift_u16(src_p[5], 0u, 0xffu);
    dst_p->mat = (int16_t)mat;
    adv_deg = unpack_left_shift_u16(src_p[6], 8u, 0xffu);
    adv_deg |= unpack_right_shift_u16(src_p[7], 0u, 0xffu);
    dst_p->adv_deg = (int16_t)adv_deg;

    return (0);
}

uint16_t megasquirt_simplified_dash_broadcast_megasquirt_dash1_pw1_encode(float value)
{
    return (uint16_t)(value / 0.001f);
}

float megasquirt_simplified_dash_broadcast_megasquirt_dash1_pw1_decode(uint16_t value)
{
    return ((float)value * 0.001f);
}

bool megasquirt_simplified_dash_broadcast_megasquirt_dash1_pw1_is_in_range(uint16_t value)
{
    (void)value;

    return (true);
}

uint16_t megasquirt_simplified_dash_broadcast_megasquirt_dash1_pw2_encode(float value)
{
    return (uint16_t)(value / 0.001f);
}

float megasquirt_simplified_dash_broadcast_megasquirt_dash1_pw2_decode(uint16_t value)
{
    return ((float)value * 0.001f);
}

bool megasquirt_simplified_dash_broadcast_megasquirt_dash1_pw2_is_in_range(uint16_t value)
{
    (void)value;

    return (true);
}

int16_t megasquirt_simplified_dash_broadcast_megasquirt_dash1_mat_encode(float value)
{
    return (int16_t)(value / 0.1f);
}

float megasquirt_simplified_dash_broadcast_megasquirt_dash1_mat_decode(int16_t value)
{
    return ((float)value * 0.1f);
}

bool megasquirt_simplified_dash_broadcast_megasquirt_dash1_mat_is_in_range(int16_t value)
{
    (void)value;

    return (true);
}

int16_t megasquirt_simplified_dash_broadcast_megasquirt_dash1_adv_deg_encode(float value)
{
    return (int16_t)(value / 0.1f);
}

float megasquirt_simplified_dash_broadcast_megasquirt_dash1_adv_deg_decode(int16_t value)
{
    return ((float)value * 0.1f);
}

bool megasquirt_simplified_dash_broadcast_megasquirt_dash1_adv_deg_is_in_range(int16_t value)
{
    (void)value;

    return (true);
}

int megasquirt_simplified_dash_broadcast_megasquirt_dash0_pack(
    uint8_t *dst_p,
    const struct megasquirt_simplified_dash_broadcast_megasquirt_dash0_t *src_p,
    size_t size)
{
    uint16_t clt;
    uint16_t map;
    uint16_t tps;

    if (size < 8u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 8);

    map = (uint16_t)src_p->map;
    dst_p[0] |= pack_right_shift_u16(map, 8u, 0xffu);
    dst_p[1] |= pack_left_shift_u16(map, 0u, 0xffu);
    dst_p[2] |= pack_right_shift_u16(src_p->rpm, 8u, 0xffu);
    dst_p[3] |= pack_left_shift_u16(src_p->rpm, 0u, 0xffu);
    clt = (uint16_t)src_p->clt;
    dst_p[4] |= pack_right_shift_u16(clt, 8u, 0xffu);
    dst_p[5] |= pack_left_shift_u16(clt, 0u, 0xffu);
    tps = (uint16_t)src_p->tps;
    dst_p[6] |= pack_right_shift_u16(tps, 8u, 0xffu);
    dst_p[7] |= pack_left_shift_u16(tps, 0u, 0xffu);

    return (8);
}

int megasquirt_simplified_dash_broadcast_megasquirt_dash0_unpack(
    struct megasquirt_simplified_dash_broadcast_megasquirt_dash0_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    uint16_t clt;
    uint16_t map;
    uint16_t tps;

    if (size < 8u) {
        return (-EINVAL);
    }

    map = unpack_left_shift_u16(src_p[0], 8u, 0xffu);
    map |= unpack_right_shift_u16(src_p[1], 0u, 0xffu);
    dst_p->map = (int16_t)map;
    dst_p->rpm = unpack_left_shift_u16(src_p[2], 8u, 0xffu);
    dst_p->rpm |= unpack_right_shift_u16(src_p[3], 0u, 0xffu);
    clt = unpack_left_shift_u16(src_p[4], 8u, 0xffu);
    clt |= unpack_right_shift_u16(src_p[5], 0u, 0xffu);
    dst_p->clt = (int16_t)clt;
    tps = unpack_left_shift_u16(src_p[6], 8u, 0xffu);
    tps |= unpack_right_shift_u16(src_p[7], 0u, 0xffu);
    dst_p->tps = (int16_t)tps;

    return (0);
}

int16_t megasquirt_simplified_dash_broadcast_megasquirt_dash0_map_encode(float value)
{
    return (int16_t)(value / 0.1f);
}

float megasquirt_simplified_dash_broadcast_megasquirt_dash0_map_decode(int16_t value)
{
    return ((float)value * 0.1f);
}

bool megasquirt_simplified_dash_broadcast_megasquirt_dash0_map_is_in_range(int16_t value)
{
    (void)value;

    return (true);
}

uint16_t megasquirt_simplified_dash_broadcast_megasquirt_dash0_rpm_encode(float value)
{
    return (uint16_t)(value);
}

float megasquirt_simplified_dash_broadcast_megasquirt_dash0_rpm_decode(uint16_t value)
{
    return ((float)value);
}

bool megasquirt_simplified_dash_broadcast_megasquirt_dash0_rpm_is_in_range(uint16_t value)
{
    (void)value;

    return (true);
}

int16_t megasquirt_simplified_dash_broadcast_megasquirt_dash0_clt_encode(float value)
{
    return (int16_t)(value / 0.1f);
}

float megasquirt_simplified_dash_broadcast_megasquirt_dash0_clt_decode(int16_t value)
{
    return ((float)value * 0.1f);
}

bool megasquirt_simplified_dash_broadcast_megasquirt_dash0_clt_is_in_range(int16_t value)
{
    (void)value;

    return (true);
}

int16_t megasquirt_simplified_dash_broadcast_megasquirt_dash0_tps_encode(float value)
{
    return (int16_t)(value / 0.1f);
}

float megasquirt_simplified_dash_broadcast_megasquirt_dash0_tps_decode(int16_t value)
{
    return ((float)value * 0.1f);
}

bool megasquirt_simplified_dash_broadcast_megasquirt_dash0_tps_is_in_range(int16_t value)
{
    (void)value;

    return (true);
}
