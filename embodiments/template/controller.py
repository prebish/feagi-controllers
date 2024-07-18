#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Copyright 2016-2022 The FEAGI Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================
"""

import threading
from time import sleep
from feagi_connector import sensors
from feagi_connector import actuators
from feagi_connector import retina as retina
from feagi_connector import pns_gateway as pns
from feagi_connector.version import __version__
from feagi_connector import feagi_interface as feagi


def action(obtained_data, capabilities):
    if "servo" in capabilities:
        if 'servo' in obtained_data:
            recieve_servo_data = actuators.get_servo_data(obtained_data, True)
        if 'servo_position' in obtained_data:
            for data_point in obtained_data:
                recieve_servo_position_data = actuators.get_position_data(obtained_data['servo_position'][data_point], capabilities, device_id)
                # Do a command to move
    if "motor" in capabilities:
        recieve_motor_data = actuators.get_motor_data(obtained_data,
                                                      capabilities['motor']['power_amount'],
                                                      capabilities['motor']['count'],
                                                      rolling_window)
    # Halted here as it needs to be improved
    recieve_gpio_data = actuators.get_gpio_data(obtained_data)
    check_input_request = actuators.check_convert_gpio_to_input(obtained_data)

if __name__ == "__main__":
    # Generate runtime dictionary
    runtime_data = {"vision": [], "stimulation_period": None, "feagi_state": None, "feagi_network": None}

    config = feagi.build_up_from_configuration()
    feagi_settings = config['feagi_settings'].copy()
    agent_settings = config['agent_settings'].copy()
    default_capabilities = config['default_capabilities'].copy()
    message_to_feagi = config['message_to_feagi'].copy()
    capabilities = config['capabilities'].copy()

    # # # FEAGI registration # # # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    # - - - - - - - - - - - - - - - - - - #
    feagi_settings, runtime_data, api_address, feagi_ipu_channel, feagi_opu_channel = \
        feagi.connect_to_feagi(feagi_settings, runtime_data, agent_settings, capabilities,
                               __version__)
    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    default_capabilities = {}  # It will be generated in process_visual_stimuli. See the
    default_capabilities = pns.create_runtime_default_list(default_capabilities, capabilities)
    if "camera" in capabilities:
        threading.Thread(target=retina.vision_progress, args=(default_capabilities, feagi_settings, camera_data['vision'],), daemon=True).start()
    else:
        # No vision included so using no_vision_progress background
        pns.check_genome_status_no_vision(message_from_feagi)
        feagi_settings['feagi_burst_speed'] = pns.check_refresh_rate(message_from_feagi, feagi_settings['feagi_burst_speed'])

    while True:
        message_from_feagi = pns.message_from_feagi
        if message_from_feagi:
            obtained_signals = pns.obtain_opu_data(message_from_feagi)
            action(obtained_signals, capabilities)
        pns.signals_to_feagi(message_to_feagi, feagi_ipu_channel, agent_settings, feagi_settings)
        message_to_feagi.clear()
        sleep(feagi_settings['feagi_burst_speed'])
