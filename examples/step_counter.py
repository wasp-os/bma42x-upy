#
# Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
# Copyright (c) 2020 Daniel Thompson.
#
# SPDX-License-Identifier: BSD-3-Clause
#

"""Step counter

Example shows basic setup application of step counter feature.

This example is a translation of the equivalent code in C
(although the loop at the end is slightly different):
../BMA423-Sensor-API/examples/generic/step_counter.c .
"""

import bma42x
import time
import watch

bma = bma42x.BMA42X(watch.i2c)
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

print("Move/perform the walk/step action with the sensor")
while True:
    # Get step counter output
    step_out = bma.step_counter_output()
    print("The step counter output is {}\r".format(step_out), end='')

    time.sleep(2)
