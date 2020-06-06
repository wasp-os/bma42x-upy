#
# Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
# Copyright (c) 2020 Daniel Thompson.
#
# SPDX-License-Identifier: BSD-3-Clause
#

"""Example shows basic application of reading sensor temperature.

This example is a translation of the C example of the same name:
../BMA423-Sensor-API/examples/generic/temperature/temperature.c .
"""

import bma42x
import time

def temperature(i2c):
    bma = bma42x.BMA42X(i2c)
    accel_conf = {}

    # Sensor initialization
    bma.init()

    # There is no hardware reset capability so issue a software reset
    # instead.
    bma.set_command_register(0xb6)
    time.sleep(0.20)

    # Upload the configuration file to enable the features of the sensor.
    bma.write_config_file()

    # Get temperature in degree C
    get_temp_C = bma.get_temperature(bma42x.DEG)

    # Get temperature in degree F
    get_temp_F = bma.get_temperature(bma42x.FAHREN)

    # Get temperature in degree K
    get_temp_K = bma.get_temperature(bma42x.KELVIN)

    # Scale the output to get the actual temperature
    actual_temp = get_temp_C / bma42x.SCALE_TEMP
    print("Actual temperature in degree celsius is {:.2f} degrees C".format(actual_temp))
    actual_temp = get_temp_F / bma42x.SCALE_TEMP
    print("Actual temperature in degree fahranheit is {:.2f} degrees F".format(actual_temp))
    actual_temp = get_temp_K / bma42x.SCALE_TEMP
    print("Actual temperature in degree kelvin is {:.2f} degrees K".format(actual_temp))
