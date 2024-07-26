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
from datetime import datetime
from utils.process_image import process_image
# import shutil
import threading
from time import sleep
import webbrowser
from feagi_connector import testing_mode
from feagi_connector import retina
from feagi_connector import pns_gateway as pns
from feagi_connector.version import __version__
from feagi_connector import trainer as feagi_trainer
from feagi_connector import feagi_interface as feagi

# Open browser window to display images
def open_browser():
    webbrowser.open('http://localhost:5000')

threading.Timer(1.0, open_browser).start()

# Latest location data for browser image display
latest_image_info = {"image": None, "location_data": None}

# This block of code will execute if this script is run as the main module
if __name__ == "__main__":
    # Initialize a runtime dictionary to store various runtime data
    runtime_data = {"vision": {}, "current_burst_id": None, "stimulation_period": None,
                    "feagi_state": None,
                    "feagi_network": None}
    
    # Load configurations and settings for FEAGI, agents, capabilities, and messages
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
    # Initialize a message counter from the FEAGI state
    msg_counter = runtime_data["feagi_state"]['burst_counter']

    # Fetch the full dimensions if not already set
    if not pns.full_list_dimension:
        pns.full_list_dimension = pns.fetch_full_dimensions()

    # Initialize variables for vision processing
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
    camera_data['vision'] = []
    temporary_previous = dict()

    # Create runtime default capabilities list
    default_capabilities = {}  # It will be generated in process_visual_stimuli. See the
    default_capabilities = pns.create_runtime_default_list(default_capabilities, capabilities)
    default_capabilities = retina.convert_new_json_to_old_json(default_capabilities)
    threading.Thread(target=retina.vision_progress, args=(default_capabilities, feagi_settings, camera_data['vision'],), daemon=True).start()
    # Main loop for processing images
    default_capabilities['hardcode_for_kat'] = []
    while continue_loop:
        # Grabs all images in this directory
        image_obj = feagi_trainer.scan_the_folder(capabilities['input']['image_reader']['0']['image_path'])
        for image in image_obj:
            latest_image_info = {"image": None, "location_data": None}
            raw_frame = image[0]
            camera_data['vision'] = raw_frame
            name_id = image[1]
            message_to_feagi = feagi_trainer.id_training_with_image(message_to_feagi, name_id)
            if start_timer == 0:
                start_timer = datetime.now()
            while capabilities['input']['image_reader']['0']['image_display_duration'] >= int((datetime.now() - start_timer).total_seconds()):
                size_list = pns.resize_list
                temporary_previous, rgb, default_capabilities, modified_data = \
                    retina.process_visual_stimuli_trainer(
                        raw_frame,
                        default_capabilities,
                        previous_frame_data,
                        rgb, capabilities, False) # processes visual data into FEAGI-comprehensible form

                # Process the image to show FEAGI selection boxes on image in browser
                # if len(default_capabilities['hardcode_for_kat']) > 0:
                #     process_image(default_capabilities['hardcode_for_kat'])

                # If camera data is available, generate data for FEAGI
                if 'camera' in rgb: # This is the data wrapped for feagi data to read
                    if rgb['camera'] == {}:
                        break
                    else:
                        message_to_feagi = pns.generate_feagi_data(rgb, message_to_feagi)

                message_from_feagi = pns.message_from_feagi # Needs to re-structure this code to be
                # more consistent
                # if (message_from_feagi):
                #     print('message', message_from_feagi)
                    # get id recognition message

                # location section
                location_data = pns.recognize_location_data(message_from_feagi)
                if location_data:
                    # print("location: ", location_data)
                    latest_image_info["location_data"] = location_data
                    latest_image_info["image"] = default_capabilities['hardcode_for_kat']
                process_image(modified_data['00_C'], latest_image_info['image'], latest_image_info['location_data'])

                # Testing mode section
                if capabilities['input']['image_reader']['0']['test_mode']:
                    success_rate, success, total = testing_mode.mode_testing(name_id,
                                                                             message_from_feagi,
                                                                             total, success,
                                                                             success_rate)
                else:
                    success_rate, success, total = 0, 0, 0
                # Send signals to FEAGI
                pns.signals_to_feagi(message_to_feagi, feagi_ipu_channel, agent_settings, feagi_settings)
                # Sleep for the burst duration specified in the settings
                sleep(feagi_settings['burst_duration'])
            sleep(capabilities['input']['image_reader']['0']['image_gap_duration'])
            previous_frame_data = temporary_previous.copy()
            start_timer = 0
            message_to_feagi.clear()
        # Sleep for the burst duration before the next iteration
        sleep(feagi_settings['burst_duration'])
        continue_loop = capabilities['input']['image_reader']['0']['loop']
