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
import cv2 # OpenCV
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

    # Customize and show actual image
    def process_image(raw_frame):
        # Define the coordinates for the box
        top_left = (160, 60)
        bottom_right = (400, 240)
        
        # Define the border thickness
        border_thickness = 3

        # Draw the outer black rectangle (border)
        cv2.rectangle(raw_frame, 
                    (top_left[0] - border_thickness, top_left[1] - border_thickness),
                    (bottom_right[0] + border_thickness, bottom_right[1] + border_thickness),
                    (0, 0, 0), border_thickness)

        # Draw the inner green rectangle
        cv2.rectangle(raw_frame, top_left, bottom_right, (0, 255, 0), 2)

        # Add text to the image in the top left of the box
        text = "FEAGI Perception: dog"
        text_position = (top_left[0], top_left[1] + 5)  # Adjusted to fit background

        # Font and text size
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.6
        font_thickness = 2

        # Calculate the width and height of the text box
        (text_width, text_height), baseline = cv2.getTextSize(text, font, font_scale, font_thickness)
        text_height += baseline

        # Create a rectangle filled with black color for the text background
        background_top_left = (text_position[0], text_position[1] - text_height)
        background_bottom_right = (text_position[0] + text_width, text_position[1] + baseline)
        cv2.rectangle(raw_frame, background_top_left, background_bottom_right, (0, 0, 0), cv2.FILLED)

        # Put the text over the black rectangle
        cv2.putText(raw_frame, text, text_position, font, font_scale, (0, 255, 0), font_thickness)

        # Display the image in a window with this string title
        cv2.imshow('Image Shown to FEAGI', raw_frame)
        cv2.waitKey(0)  # Wait for a key press to close the window
        cv2.destroyAllWindows()  # Close all OpenCV windows

    # Start a thread to process vision progress
    threading.Thread(target=retina.vision_progress, args=(default_capabilities, feagi_settings, camera_data['vision'],), daemon=True).start()
    # Main loop for processing images
    while continue_loop:
        # Scan the folder for images
        image_obj = feagi_trainer.scan_the_folder(capabilities['image_reader']['path'])
        for image in image_obj:
            raw_frame = image[0]
            camera_data['vision'] = raw_frame
            name_id = image[1]
            message_to_feagi = feagi_trainer.id_training_with_image(message_to_feagi, name_id)
            if start_timer == 0:
                start_timer = datetime.now()

            # Process images until the specified pause duration
            while capabilities['image_reader']['pause'] >= int((datetime.now() - start_timer).total_seconds()):
                size_list = pns.resize_list
                temporary_previous, rgb, default_capabilities = \
                    retina.process_visual_stimuli(
                        raw_frame,
                        default_capabilities,
                        previous_frame_data,
                        rgb, capabilities, False)
                
                # Process the image
                process_image(raw_frame)

                # If camera data is available, generate data for FEAGI
                if 'camera' in rgb:
                    if rgb['camera'] == {}:
                        break
                    else:
                        message_to_feagi = pns.generate_feagi_data(rgb, message_to_feagi)

                # If in test mode, perform mode testing
                if capabilities['image_reader']['test_mode']:
                    success_rate, success, total = testing_mode.mode_testing(name_id,
                                                                             pns.message_from_feagi,
                                                                             total, success,
                                                                             success_rate)
                else:
                    success_rate, success, total = 0, 0, 0
                # Send signals to FEAGI
                pns.signals_to_feagi(message_to_feagi, feagi_ipu_channel, agent_settings, feagi_settings)
                # Sleep for the burst duration specified in the settings
                sleep(feagi_settings['burst_duration'])

            # Update the previous frame data and reset the timer
            previous_frame_data = temporary_previous.copy()
            start_timer = 0
            message_to_feagi.clear()
        # Sleep for the burst duration before the next iteration
        sleep(feagi_settings['burst_duration'])
        # Continue the loop based on the loop setting in capabilities
        continue_loop = capabilities['image_reader']['loop']
