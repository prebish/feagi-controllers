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
from datetime import datetime
from feagi_connector import router
from feagi_connector import testing_mode
from feagi_connector import retina as retina
from feagi_connector import pns_gateway as pns
from feagi_connector.version import __version__
from feagi_connector import trainer as feagi_trainer
from feagi_connector import feagi_interface as feagi

if __name__ == "__main__":
    # Generate runtime dictionary
    runtime_data = {"vision": {}, "current_burst_id": None, "stimulation_period": None,
                    "feagi_state": None,
                    "feagi_network": None}

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
    msg_counter = runtime_data["feagi_state"]['burst_counter']
    if not pns.full_list_dimension:
        pns.full_list_dimension = pns.fetch_full_dimensions()
    rgb = dict()
    rgb['camera'] = dict()
    previous_frame_data = {}
    start_timer = 0
    raw_frame = []
    continue_loop = True
    total = 0
    success = 0
    success_rate = 0
    # overwrite manual
    camera_data = dict()
    camera_data['vision'] = dict()
    temporary_previous = dict()
    default_capabilities = {}  # It will be generated in process_visual_stimuli. See the
    default_capabilities = pns.create_runtime_default_list(default_capabilities, capabilities)
    threading.Thread(target=retina.vision_progress,
                     args=(default_capabilities, feagi_opu_channel, api_address, feagi_settings,
                           camera_data['vision'],), daemon=True).start()
    while continue_loop:
        image_obj = feagi_trainer.scan_the_folder(capabilities['image_reader']['path'])
        for image in image_obj:
            raw_frame = image[0]
            camera_data['vision'] = raw_frame
            name_id = image[1]
            message_to_feagi = feagi_trainer.id_training_with_image(message_to_feagi, name_id)
            # Post image into vision
            if start_timer == 0:
                start_timer = datetime.now()
            while capabilities['image_reader']['pause'] >= int((datetime.now() - start_timer).total_seconds()):
                size_list = pns.resize_list
                temporary_previous, rgb, default_capabilities = \
                    retina.process_visual_stimuli(
                        raw_frame,
                        default_capabilities,
                        previous_frame_data,
                        rgb, capabilities, False)
                if 'camera' in rgb:
                    if rgb['camera'] == {}:
                        break
                    else:
                        message_to_feagi = pns.generate_feagi_data(rgb, msg_counter, datetime.now(),
                                                                   message_to_feagi)
                # Testing mode section
                if capabilities['image_reader']['test_mode']:
                    success_rate, success, total = testing_mode.mode_testing(name_id,
                                                                             pns.message_from_feagi,
                                                                             total, success,
                                                                             success_rate)
                else:
                    success_rate, success, total = 0, 0, 0
                pns.signals_to_feagi(message_to_feagi, feagi_ipu_channel, agent_settings, feagi_settings)
                sleep(feagi_settings['burst_duration'])
            previous_frame_data = temporary_previous.copy()
            start_timer = 0
            message_to_feagi.clear()
        sleep(feagi_settings['burst_duration'])
        continue_loop = capabilities['image_reader']['loop']
