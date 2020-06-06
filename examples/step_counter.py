#
# Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
# Copyright (c) 2020 Daniel Thompson.
#
# SPDX-License-Identifier: BSD-3-Clause
#

"""Step counter

Example shows basic setup application of step counter feature.

This example is a translation of the C example of the same name:
../BMA423-Sensor-API/examples/generic/step_counter.c .
"""

import bma42x
import time

def step_counter(i2c):
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

    # Enable step counter
    bma.feature_enable(bma42x.STEP_CNTR, True)

    # Map the interrupt pin with that of step counter interrupts
    # Interrupt will  be generated when step activity is generated.
    bma.map_interrupt(bma42x.INTR1_MAP, bma42x.STEP_CNTR_INT, True)

    # Set water-mark level 1 to get interrupt after 20 steps.
    # Range of step counter interrupt is 0 to 20460(resolution of 20 steps).
    bma.step_counter_set_watermark(1)

    print("Move/perform the walk/step action with the sensor")
    while True:
        # Read the interrupt register to check whether step counter interrupt is
        # received
        int_status = bma.read_int_status()

        # Check if step counter interrupt is triggered
        if int_status & bma42x.STEP_CNTR_INT:
            print("Step counter interrupt received")

            # On interrupt, Get step counter output
            step_out = bma.step_counter_output()
            break

    print("The step counter output is {}".format(step_out))
