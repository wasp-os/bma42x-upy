#
# Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
# Copyright (c) 2020 Daniel Thompson.
#
# SPDX-License-Identifier: BSD-3-Clause
#

"""example to showcase reading accelerometer data

This example is a translation of the C example of the same name:
../BMA423-Sensor-API/examples/generic/accelerometer.c .
"""

import bma42x
import time

GRAVITY_EARTH = 9.80665

# Holds the total number of accel x, y and z axes sample counts to be printed
ACCEL_SAMPLE_COUNT = 100

def accelerometer(i2c):
    bma = bma42x.BMA42X(i2c)
    accel_conf = {}

    bma.init()

    # There is no hardware reset capability so issue a software reset
    # instead.
    bma.set_command_register(0xb6)
    time.sleep(0.20)

    # Upload the configuration file to enable the features of the sensor.
    bma.write_config_file()

    # Enable the accelerometer
    bma.set_accel_enable(True)

    # Accelerometer Configuration Setting
    # Output data Rate
    accel_conf['odr'] = bma42x.OUTPUT_DATA_RATE_100HZ

    # Gravity range of the sensor (+/- 2G, 4G, 8G, 16G)
    accel_conf['range'] = bma42x.ACCEL_RANGE_2G

    # Bandwidth configure number of sensor samples required to average
    # if value = 2, then 4 samples are averaged
    # averaged samples = 2^(val(accel bandwidth))
    # Note1 : More info refer datasheets
    # Note2 : A higher number of averaged samples will result in a lower noise
    # level of the signal, but since the performance power mode phase is
    # increased, the power consumption will also rise.
    accel_conf['bandwidth'] = bma42x.ACCEL_NORMAL_AVG4

    # Enable the filter performance mode where averaging of samples
    # will be done based on above set bandwidth and ODR.
    # There are two modes
    #  0 -> Averaging samples (Default)
    #  1 -> No averaging
    # For more info on No Averaging mode refer datasheets.
    accel_conf['perf_mode'] = bma42x.CIC_AVG_MODE

    # Set the accel configurations
    bma.set_accel_config(**accel_conf)

    print("Ax[m/s2], Ay[m/s2], Az[m/s2]")

    for i in range(ACCEL_SAMPLE_COUNT):
        (x, y, z) = bma.read_accel_xyz()

        # Converting lsb to meters per seconds square for 12 bit accelerometer
        # at 2G range
        x = lsb_to_ms2(x, 2, 12)
        y = lsb_to_ms2(y, 2, 12)
        z = lsb_to_ms2(z, 2, 12)

        # Print the data in m/s2/
        print("{:.2f}, {:.2f}, {:.2f}".format(x, y, z))

        # Pause for 10ms, 100Hz output data rate
        time.sleep(0.01)

def lsb_to_ms2(val, g_range, bit_width):
    """Converts raw sensor values(LSB) to meters per seconds square.

    :param val: Raw sensor value
    :param g_range: Accel Range selected (2G, 4G, 8G, 16G).
    :param bit_width: Resolution of the sensor.
    :return: Accel values in meters per second square.
    """
    half_scale = (1 << bit_width) / 2

    return GRAVITY_EARTH * val * g_range / half_scale;
