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
import flask_server
from time import sleep
from process_image import *
from datetime import datetime
from feagi_connector import retina
from feagi_connector import testing_mode
from feagi_connector import pns_gateway as pns
import dynamic_image_coordinates as img_coords
from feagi_connector.version import __version__
from feagi_connector import feagi_interface as feagi
from feagi_connector import trainer as feagi_trainer


def run_app():
    config = feagi.build_up_from_configuration()
    image_reader_config = config["capabilities"]["input"]["image_reader"]["0"]
    flask_server.apply_config_settings(image_reader_config)
    flask_server.start_app()


# Needs to add configuration to toggle this. It should be default to false.
app_thread = threading.Thread(target=run_app)
app_thread.start()

# This block of code will execute if this script is run as the main module
if __name__ == "__main__":
    # Initialize a runtime dictionary to store various runtime data
    runtime_data = {
        "vision": {},
        "current_burst_id": None,
        "stimulation_period": None,
        "feagi_state": None,
        "feagi_network": None,
    }

    # Load configurations and settings for FEAGI, agents, capabilities, and messages
    config = feagi.build_up_from_configuration()
    feagi_settings = config["feagi_settings"].copy()
    agent_settings = config["agent_settings"].copy()
    default_capabilities = config["default_capabilities"].copy()
    message_to_feagi = config["message_to_feagi"].copy()
    capabilities = config["capabilities"].copy()

    # # # FEAGI registration # # # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    # - - - - - - - - - - - - - - - - - - #
    feagi_settings, runtime_data, api_address, feagi_ipu_channel, feagi_opu_channel = (
        feagi.connect_to_feagi(
            feagi_settings, runtime_data, agent_settings, capabilities, __version__
        )
    )
    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    # Initialize a message counter from the FEAGI state
    msg_counter = runtime_data["feagi_state"]["burst_counter"]

    # Fetch the full dimensions if not already set
    if not pns.full_list_dimension:
        pns.full_list_dimension = pns.fetch_full_dimensions()

    # Initialize variables for vision processing
    rgb = dict()
    rgb["camera"] = dict()
    previous_frame_data = {}
    start_timer = 0
    raw_frame = []
    continue_loop = True
    total = 0
    success = 0
    success_rate = 0
    # overwrite manual
    flag = True
    camera_data = dict()
    camera_data["vision"] = []
    temporary_previous = dict()

    # Create runtime default capabilities list
    default_capabilities = {}  # It will be generated in process_visual_stimuli. See the
    default_capabilities = pns.create_runtime_default_list(
        default_capabilities, capabilities
    )
    default_capabilities = retina.convert_new_json_to_old_json(default_capabilities)
    threading.Thread(
        target=retina.vision_progress,
        args=(
            default_capabilities,
            feagi_settings,
            camera_data["vision"],
        ),
        daemon=True,
    ).start()

    # Main loop for processing images
    # new_cam = cv2.VideoCapture(2) # webcam
    while continue_loop:
        # Grabs all images in this directory
        image_obj = feagi_trainer.scan_the_folder(
            capabilities["input"]["image_reader"]["0"]["image_path"]
        )
        latest_image_id = None
        # Iterate through images
        for image in image_obj:
            raw_frame = image[0]
            flask_server.latest_raw_image = raw_frame
            flask_server.latest_static.raw_image_dimensions = (
                f"{raw_frame.shape[1]} x {raw_frame.shape[0]}"
            )
            # check, raw_frame = new_cam.read() # webcam
            camera_data["vision"] = raw_frame
            name_id = image[1]
            # name_id = "0-0-0" # webcam
            # Update image ID for Flask server to display
            image_id = key = next(iter(name_id))
            # image_id = "0-0-0" # webcam
            flask_server.latest_static = img_coords.update_image_ids(
                new_image_id=image_id,
                new_feagi_image_id=None,
                static=flask_server.latest_static,
            )
            # Carry on with the image processing
            message_to_feagi = feagi_trainer.id_training_with_image(
                message_to_feagi, name_id
            )
            if start_timer == 0.0:
                start_timer = datetime.now()
            while (
                float(
                    capabilities["input"]["image_reader"]["0"]["image_display_duration"]
                )
                >= (datetime.now() - start_timer).total_seconds()
            ):
                size_list = pns.resize_list
                message_from_feagi = pns.message_from_feagi
                temporary_previous, rgb, default_capabilities, modified_data = (
                    retina.process_visual_stimuli_trainer(
                        raw_frame,
                        default_capabilities,
                        previous_frame_data,
                        rgb,
                        capabilities,
                        False,
                    )
                )  # processes visual data into FEAGI-comprehensible form

                # When FEAGI sends a recognition ID, update it for Flask server to display
                if "opu_data" in message_from_feagi:
                    recognition_id = pns.detect_ID_data(message_from_feagi)
                    if recognition_id:
                        feagi_image_id = key = next(
                            iter(recognition_id)
                        )  # example recognition_id: {'0-5-0': 100}
                        flask_server.latest_static = img_coords.update_image_ids(
                            new_image_id=None,
                            new_feagi_image_id=feagi_image_id,
                            static=flask_server.latest_static,
                        )

                # Process current image sent to FEAGI with bounding box
                location_data = pns.recognize_location_data(message_from_feagi)
                if pns.full_list_dimension:
                    size_of_cortical = pns.full_list_dimension["o__loc"][
                        "cortical_dimensions"
                    ]  # Get cortical dimensions from FEAGI db
                if previous_frame_data:
                    # Add image's dimensions to HTML display data
                    flask_server.latest_static.image_dimensions = f"{modified_data['00_C'].shape[1]} x {modified_data['00_C'].shape[0]}"
                    new_image_id = getattr(flask_server.latest_static, "image_id", "")
                    feagi_image_id = getattr(
                        flask_server.latest_static, "feagi_image_id", ""
                    )
                    if location_data:
                        if "00_C" in modified_data:
                            flask_server.latest_image = process_image(
                                modified_data["00_C"], location_data, size_of_cortical
                            )
                    elif latest_image_id != new_image_id:
                        latest_image_id = new_image_id
                        if "00_C" in modified_data:
                            flask_server.latest_image = process_image(
                                modified_data["00_C"]
                            )
                # If camera data is available, generate data for FEAGI
                if "camera" in rgb:  # This is the data wrapped for feagi data to read
                    if rgb["camera"] == {}:
                        break
                    else:
                        message_to_feagi = pns.generate_feagi_data(
                            rgb, message_to_feagi
                        )
                message_from_feagi = (
                    pns.message_from_feagi
                )  # Needs to re-structure this code to be
                # more consistent

                # location section
                location_data = pns.recognize_location_data(message_from_feagi)
                if capabilities["input"]["image_reader"]["0"]["test_mode"]:
                    success_rate, success, total = testing_mode.mode_testing(
                        name_id, message_from_feagi, total, success, success_rate
                    )
                else:
                    success_rate, success, total = 0, 0, 0
                # Send signals to FEAGI
                pns.signals_to_feagi(
                    message_to_feagi, feagi_ipu_channel, agent_settings, feagi_settings
                )
                # Sleep for the burst duration specified in the settings
                sleep(feagi_settings["burst_duration"])
            blank_image()  # reset the image or during gap
            sleep(capabilities["input"]["image_reader"]["0"]["image_gap_duration"])
            previous_frame_data = temporary_previous.copy()
            start_timer = 0.0
            message_to_feagi.clear()
        # Sleep for the burst duration before the next iteration
        sleep(feagi_settings["burst_duration"])
        continue_loop = capabilities["input"]["image_reader"]["0"]["loop"]
