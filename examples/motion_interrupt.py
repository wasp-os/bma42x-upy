#
# Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause
#

"""Motion interrupt

Example shows basic application of motion interrupt where no motion or/and any
motion are detected.

This example is a translation of the C example of the same name:
../BMA423-Sensor-API/examples/generic/motion_interrupt.py .
"""

import bma42x
import time

def motion_interrupt(i2c):
    bma = bma42x.BMA42X(i2c)
    accel_conf = {}
    any_no_mot = {}
    iteration = 20

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

    # Select the axis for which any/no motion interrupt should be generated
    any_no_mot['axes_en'] = bma42x.EN_ALL_AXIS;

    # Set the slope threshold:
    # Interrupt will be generated if the slope of all the axis exceeds the
    # threshold (1 bit = 0.48mG)
    any_no_mot['threshold'] = 10;

    # Set the duration for any/no motion interrupt:
    # Duration defines the number of consecutive data points for which
    # threshold condition must be true(1 bit = 20ms)
    any_no_mot['duration'] = 4;

    # Set the threshold, duration and axis enable configuration
    bma.set_any_mot_config(**any_no_mot);
    bma.set_no_mot_config(**any_no_mot);

    # Map the interrupt pin with that of any-motion and no-motion interrupts.
    # Interrupt will be generated when any or no-motion is recognized.
    bma.map_interrupt(bma42x.INTR1_MAP,
                      bma42x.ANY_MOT_INT | bma42x.NO_MOT_INT, True)

    print("Shake the board for any-motion interrupt whereas do not shake the "
          "board for no-motion interrupt")

    while True:
        # Read the interrupt register to check whether any-motion or no-motion
        # interrupt is received
        int_status = bma.read_int_status()

        # Check if any-motion interrupt is triggered
        if int_status & bma42x.ANY_MOT_INT:
            print("Any-Motion interrupt received");
            iteration -= 1

        # Check if no-motion interrupt is triggered
        if int_status & bma42x.NO_MOT_INT:
            print("No-Motion interrupt received");
            iteration -= 1

        # Break out of the loop when iteration has reached zero
        if iteration == 0:
            print("Iterations are done. Exiting !");
            break;
