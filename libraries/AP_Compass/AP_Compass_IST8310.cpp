/*
 * Copyright (C) 2016  Emlid Ltd. All rights reserved.
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Driver by Georgii Staroselskii, Sep 2016
 */
#include "AP_Compass_IST8310.h"

#if AP_COMPASS_IST8310_ENABLED

#include <stdio.h>
#include <utility>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Math/AP_Math.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <GCS_MAVLink/GCS.h>

#define WAI_REG 0x0
#define DEVICE_ID 0x10
#define DEVICE_ID_IST8308 0x08

#define OUTPUT_X_L_REG 0x3
#define OUTPUT_X_L_REG_IST8308 0x2

#define CNTL1_REG 0xA
#define CNTL1_VAL_SINGLE_MEASUREMENT_MODE 0x1

#define CNTL2_REG 0xB
#define CNTL2_VAL_SRST 1

#define AVGCNTL_REG 0x41
#define AVGCNTL_VAL_XZ_16 (4)
#define AVGCNTL_VAL_Y_16 (4 << 3)

#define PDCNTL_REG 0x42
#define PDCNTL_VAL_PULSE_DURATION_NORMAL 0xC0

#define SAMPLING_PERIOD_USEC (10 * AP_USEC_PER_MSEC)
#define IST8310_RESOLUTION 0.3

static const int16_t IST8310_MAX_VAL_XY = (1600 / IST8310_RESOLUTION) + 1;
static const int16_t IST8310_MIN_VAL_XY = -IST8310_MAX_VAL_XY;
static const int16_t IST8310_MAX_VAL_Z  = (2500 / IST8310_RESOLUTION) + 1;
static const int16_t IST8310_MIN_VAL_Z  = -IST8310_MAX_VAL_Z;

static uint8_t output_data_reg = OUTPUT_X_L_REG;

extern const AP_HAL::HAL &hal;

AP_Compass_Backend *AP_Compass_IST8310::probe(AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                              bool force_external,
                                              enum Rotation rotation)
{
    if (!dev) {
        return nullptr;
    }

    // --- Fast One-Time I2C Bus Scan on Bus 0 ---
    static bool bus_scanned = false;
    if (!bus_scanned && hal.i2c_mgr) {
        bus_scanned = true;
        uint8_t found_count = 0;
        
        for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
            auto test_dev = hal.i2c_mgr->get_device(dev->bus_num(), addr);
            if (!test_dev) {
                continue;
            }

            test_dev->get_semaphore()->take_blocking();
            test_dev->set_retries(1);

            uint8_t reg0 = 0;
            if (test_dev->read_registers(0x00, &reg0, 1)) {
                GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "I2C%u: ACK at 0x%02X (Reg0=0x%02X)",
                              dev->bus_num() + 1, addr, reg0);
                found_count++;
            }
            test_dev->get_semaphore()->give();
        }

        if (found_count == 0) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "I2C%u: 0 devices responded across 0x08-0x77",
                          dev->bus_num() + 1);
        }
    }

    AP_Compass_IST8310 *sensor = NEW_NOTHROW AP_Compass_IST8310(std::move(dev), force_external, rotation);
    if (!sensor || !sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

AP_Compass_IST8310::AP_Compass_IST8310(AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                       bool force_external,
                                       enum Rotation rotation)
    : _dev(std::move(dev))
    , _rotation(rotation)
    , _force_external(force_external)
{
}

bool AP_Compass_IST8310::init()
{
    uint8_t reset_count = 0;

    _dev->get_semaphore()->take_blocking();
    _dev->set_retries(10);

    // Issue soft reset
    _dev->write_register(CNTL2_REG, CNTL2_VAL_SRST);
    hal.scheduler->delay(20);

    uint8_t whoami = 0;
    if (!_dev->read_registers(WAI_REG, &whoami, 1)) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "IST8310: No ACK at 0x%02X on bus %u",
                      _dev->get_bus_address(), _dev->bus_num() + 1);
        _dev->get_semaphore()->give();
        return false;
    }

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "IST8310: Addr 0x%02X replied WHOAMI=0x%02X",
                  _dev->get_bus_address(), whoami);

    if (whoami != DEVICE_ID && whoami != DEVICE_ID_IST8308) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "IST8310: Rejected unknown WHOAMI 0x%02X", whoami);
        _dev->get_semaphore()->give();
        return false;
    }

    // Configure register offsets based on silicon variant
    if (whoami == DEVICE_ID_IST8308) {
        output_data_reg = OUTPUT_X_L_REG_IST8308;
    } else {
        output_data_reg = OUTPUT_X_L_REG;
    }

    for (; reset_count < 5; reset_count++) {
        if (!_dev->write_register(CNTL2_REG, CNTL2_VAL_SRST)) {
            hal.scheduler->delay(10);
            continue;
        }

        hal.scheduler->delay(10);

        uint8_t cntl2 = 0xFF;
        if (_dev->read_registers(CNTL2_REG, &cntl2, 1) && (cntl2 & 0x01) == 0) {
            break;
        }
    }

    // Configure filter and pulse duration registers
    _dev->write_register(AVGCNTL_REG, AVGCNTL_VAL_Y_16 | AVGCNTL_VAL_XZ_16);
    _dev->write_register(PDCNTL_REG, PDCNTL_VAL_PULSE_DURATION_NORMAL);

    _dev->set_retries(3);
    start_conversion();
    _dev->get_semaphore()->give();

    _dev->set_device_type(DEVTYPE_IST8310);
    if (!register_compass(_dev->get_bus_id())) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "IST8310: Failed to register compass backend");
        return false;
    }

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "IST8310: Registered successfully on Bus %u (Addr 0x%02X)",
                  _dev->bus_num() + 1, _dev->get_bus_address());

    set_rotation(_rotation);

    if (_force_external) {
        set_external(true);
    }
    
    _periodic_handle = _dev->register_periodic_callback(SAMPLING_PERIOD_USEC,
        FUNCTOR_BIND_MEMBER(&AP_Compass_IST8310::timer, void));

    return true;
}

void AP_Compass_IST8310::start_conversion()
{
    if (!_dev->write_register(CNTL1_REG, CNTL1_VAL_SINGLE_MEASUREMENT_MODE)) {
        _ignore_next_sample = true;
    }
}

void AP_Compass_IST8310::timer()
{
    if (_ignore_next_sample) {
        _ignore_next_sample = false;
        start_conversion();
        return;
    }

    struct PACKED {
        le16_t rx;
        le16_t ry;
        le16_t rz;
    } buffer;

    bool ret = _dev->read_registers(output_data_reg, (uint8_t *) &buffer, sizeof(buffer));
    if (!ret) {
        return;
    }

    start_conversion();
    _dev->adjust_periodic_callback(_periodic_handle, SAMPLING_PERIOD_USEC);

    auto x = static_cast<int16_t>(le16toh(buffer.rx));
    auto y = static_cast<int16_t>(le16toh(buffer.ry));
    auto z = static_cast<int16_t>(le16toh(buffer.rz));

    if (x > IST8310_MAX_VAL_XY || x < IST8310_MIN_VAL_XY ||
        y > IST8310_MAX_VAL_XY || y < IST8310_MIN_VAL_XY ||
        z > IST8310_MAX_VAL_Z  || z < IST8310_MIN_VAL_Z) {
        return;
    }

    z = -z;
    Vector3f field = Vector3f{x * 3.0f, y * 3.0f, z * 3.0f};
    accumulate_sample(field);
}

#endif  // AP_COMPASS_IST8310_ENABLED