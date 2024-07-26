#!/usr/bin/env python3
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

import time
import asyncio
import threading
from feagi_connector import router
from feagi_connector import retina
from feagi_connector import feagi_interface as feagi

# Variable storage #
raw_aptr = -1
global_ID_cortical_size = None
full_list_dimension = []
full_template_information_corticals = []
resize_list = {}
previous_genome_timestamp = 0
genome_tracker = 0
message_from_feagi = {}
refresh_rate = 0.01
ver = "local"


def generate_feagi_data(rgb, message_to_feagi):
    """
    This function generates data for Feagi by combining RGB values, message counter, and date into
    the provided message.
    """
    try:
        if rgb:
            if "data" not in message_to_feagi:
                message_to_feagi["data"] = dict()
            if "sensory_data" not in message_to_feagi["data"]:
                message_to_feagi["data"]["sensory_data"] = dict()
            message_to_feagi["data"]["sensory_data"]['camera'] = rgb['camera']
    except Exception as e:
        print("ERROR: ", e)
    return message_to_feagi


def append_sensory_data_for_feagi(sensory_category, sensory_data, message_to_feagi):
    """
    :param sensory_category: A name such as training, camera, IR, ultrasonic and so on.
    :param sensory_data: The data of dict only
    :param message_to_feagi: Use the existing dict to append
    :return: the updated dict called `message_to_feagi`
    """
    if "data" not in message_to_feagi:
        message_to_feagi["data"] = {}
    if "sensory_data" not in message_to_feagi["data"]:
        message_to_feagi["data"]["sensory_data"] = {}
    if sensory_category not in message_to_feagi["data"]["sensory_data"]:
        message_to_feagi["data"]["sensory_data"][sensory_category] = sensory_data
    else:
        for key in sensory_data:
            if key in message_to_feagi["data"]["sensory_data"][sensory_category]:
                message_to_feagi["data"]["sensory_data"][sensory_category][key].update(sensory_data[key])
            else:
                message_to_feagi["data"]["sensory_data"][sensory_category].update(sensory_data)
    return message_to_feagi


def signals_from_feagi(feagi_opu_channel):
    """ get OPU from FEAGI """
    return router.fetch_feagi(feagi_opu_channel)


def signals_to_feagi(message_to_feagi, feagi_ipu_channel, agent_settings, feagi_settings):
    """
    Sends data to FEAGI through the router.py
    """
    message_to_feagi['ver'] = ver
    message_to_feagi['sent_by_rounter'] = time.time()
    message_to_feagi['seqID'] = router.msg_counter
    if 'magic_link' not in feagi_settings:
        router.send_feagi(message_to_feagi, feagi_ipu_channel, agent_settings)
    else:
        router.websocket_send(message_to_feagi)
    router.msg_counter += 1

def grab_geometry():
    """
    To get the size of vision cortical areas (e.g., C, TL, TM...)
    """
    return router.fetch_geometry()


def fetch_servo_position_size_and_return_percentage(capabilities, current_position, servo_id,
                                                    flip=False):
    if full_list_dimension:
        if 'i_spos' in full_list_dimension:
            max_input = full_list_dimension['i_spos']['cortical_dimensions'][2]
            min_output = capabilities['servo']['servo_range'][str(servo_id)][0]
            max_output = capabilities['servo']['servo_range'][str(servo_id)][1]
            total_range = max_output - min_output
            encoder_position = (current_position / total_range) * max_input
            if flip:
                data = str(servo_id) + '-0-' + str(int(round(max_input / 2) + encoder_position))
            else:
                data = str(servo_id) + '-0-' + str(int(round(max_input / 2) - encoder_position))
            return data
    return None  # Just return none to confirm that it's unable to fetch genome information


def fetch_servo_motion_sensor_size_and_return_percentage(current_speed, servo_id, max_speed):
    if full_list_dimension:
        if 'i_smot' in full_list_dimension:
            max_input = full_list_dimension['i_smot']['cortical_dimensions'][2]
            min_output = 0
            total_range = max_speed - min_output
            encoder_position = ((current_speed / total_range) * max_input)
            if encoder_position < 0:
                data = str(servo_id + 1) + '-0-' + str(int(abs(encoder_position)))
            else:
                data = str(servo_id) + '-0-' + str(int(abs(encoder_position)))
            return data
    return None  # Just return none to confirm that it's unable to fetch genome information


def obtain_opu_data(message_from_feagi):
    """
    It retrieves raw data from FEAGI and then passes the data to the opu_processor, located in
    feagi_interface.py, which translates the ID of the cortical area into a human-readable name,
    such as "o__mot" to "motor." Once this translation is complete, it will streamline the
    process for actuators, retinas, and sensors to access the data from here, making it simpler
    for you.
    """
    opu_signal_dict = {}
    opu_data = feagi.opu_processor(message_from_feagi)
    for i in opu_data:
        if opu_data[i]:
            for x in opu_data[i]:
                if i not in opu_signal_dict:
                    opu_signal_dict[i] = {}
                opu_signal_dict[i][x] = opu_data[i][x]
    return opu_signal_dict


def obtain_data_type(data):
    """
    It displays the data type of the image it receives, which is useful for troubleshooting and
    verifying the image data type.

    - If it returns "ImagingCore," it means it's using PILLOW.
    - If it returns "ndarray," it's either using cv2 or a similar process.
    - If it returns "list," it's likely custom code. Verify if it's RGB or RGBA as well.
    """
    if type(data).__name__ == "ImagingCore":
        print("ImagingCore")
        return "ImagingCore"
    elif type(data).__name__ == "ndarray":
        print("numpy.ndarray")
        return "ndarray"
    elif type(data).__name__ == "list":
        print("list")
        return "list"
    else:
        print("Couldn't find: ", type(data).__name__, " and full name of the class: ", type(data))
        return "Unknown"


def obtain_blink_data(raw_frame, message_from_feagi, capabilities):
    """
    It will update based on the blink opu.
    """
    if "o_blnk" in message_from_feagi["opu_data"]:
        if message_from_feagi["opu_data"]["o_blnk"]:
            capabilities['camera']['blink'] = raw_frame
    return capabilities


def obtain_genome_number(genome_tracker, message_from_feagi):
    """
    Update when the genome modified.
    """
    if 'genome_num' in message_from_feagi:
        if message_from_feagi['genome_num'] != genome_tracker:
            return message_from_feagi['genome_num']
    return genome_tracker


def monitor_switch(message_from_feagi, capabilities):
    """
    Update when "o__mon" or the monitor_opu has changed.
    It is currently used for local screen recording.
    """
    if "o__mon" in message_from_feagi["opu_data"]:
        if message_from_feagi["opu_data"]["o__mon"]:
            for i in message_from_feagi["opu_data"]["o__mon"]:
                monitor_update = feagi.block_to_array(i)
                capabilities['camera']['monitor'] = monitor_update[0]
    return capabilities


def eccentricity_control_update(message_from_feagi, capabilities):
    """
    Update the gaze from the gaze opu cortical area
    """
    if 'ov_ecc' in message_from_feagi["opu_data"]:
        for data_point in message_from_feagi["opu_data"]['ov_ecc']:
            device_id = data_point.split('-')[0]
            if str(device_id) in ['0', '1']:
                intensity_select = (int(data_point.split('-')[-1]))
                max_depth_of_cortical = full_list_dimension['ov_ecc']['cortical_dimensions'][2] - 1
                max_range = capabilities['camera']['vision_range'][1]
                min_range = capabilities['camera']['vision_range'][0]
                capabilities['camera']["eccentricity_control"][str(device_id)] = int(((intensity_select / max_depth_of_cortical) * (max_range - min_range)) + min_range)
    return capabilities


def modulation_control_update(message_from_feagi, capabilities):
    """
    Update pupil size from the pupil opu cortical area
    """
    if 'ov_mod' in message_from_feagi["opu_data"]:
        for data_point in message_from_feagi["opu_data"]['ov_mod']:
            device_id = data_point.split('-')[0]
            if str(device_id) in ['0', '1']:
                intensity_select = (int(data_point.split('-')[-1]))
                max_depth_of_cortical = full_list_dimension['ov_mod']['cortical_dimensions'][2] - 1
                max_range = capabilities['camera']['vision_range'][1]
                min_range = capabilities['camera']['vision_range'][0]
                capabilities['camera']["modulation_control"][str(device_id)] = int(((intensity_select / max_depth_of_cortical) * (max_range - min_range)) + min_range)
    return capabilities


def detect_ID_data(message_from_feagi):
    """
    :param message_from_feagi: Should be a dict from FEAGI data only
    :return: Return the data that given by FEAGI
    """
    if "o___id" in message_from_feagi["opu_data"]:
        if message_from_feagi["opu_data"]["o___id"]:
            return message_from_feagi["opu_data"]["o___id"]
    return {}

def recognize_location_data(message_from_feagi):
    """
    :param message_from_feagi: Should be a dict from FEAGI data only
    :return: Return the data that given by FEAGI
    """
    if 'opu_data' in message_from_feagi:
        if "o__loc" in message_from_feagi["opu_data"]:
            if message_from_feagi["opu_data"]["o__loc"]:
                return message_from_feagi["opu_data"]["o__loc"]
    return {}


def detect_genome_change(message_from_feagi):
    """
    Update when the genome is loaded or using the different genome.
    """
    if "genome_changed" in message_from_feagi:
        if message_from_feagi["genome_changed"]:
            return message_from_feagi["genome_changed"]


def check_refresh_rate(message_from_feagi, current_second):
    """
    Update the current second when the feagi's refresh rate changed.
    """
    if message_from_feagi is not None:
        return message_from_feagi['burst_frequency']
    return current_second


def fetch_full_dimensions():
    """
    List of the full size and names of every cortical area. It does not include properties such as
    neurons or details.
    """
    return router.fetch_geometry()

def fetch_full_template_information():
    """
    List of the full size and names of every cortical area. It does not include properties such as
    neurons or details.
    """
    return router.fetch_template()


def check_genome_status(message_from_feagi, capabilities):
    """
    Verify if full_list_dimension is empty, size list for vision is empty, if genome has been
    changed, or genome modified in real time.
    """
    global previous_genome_timestamp, genome_tracker, full_list_dimension, resize_list, full_template_information_corticals
    if message_from_feagi['genome_changed'] is not None:
        if full_list_dimension is None:
            full_list_dimension = []
        if len(full_list_dimension) == 0:
            full_list_dimension = fetch_full_dimensions()
        if len(full_template_information_corticals) == 0:
            full_template_information_corticals = fetch_full_template_information()
        genome_changed = detect_genome_change(message_from_feagi)
        if genome_changed != previous_genome_timestamp:
            full_list_dimension = fetch_full_dimensions()
            response = full_list_dimension
            resize_list = retina.obtain_cortical_vision_size(capabilities['camera']["index"], response)
            previous_genome_timestamp = message_from_feagi["genome_changed"]
        current_tracker = obtain_genome_number(genome_tracker, message_from_feagi)
        if len(resize_list) == 0:
            response = full_list_dimension
            resize_list = retina.obtain_cortical_vision_size(capabilities['camera']["index"], response)
        if genome_tracker != current_tracker:
            full_list_dimension = fetch_full_dimensions()
            genome_tracker = current_tracker


def check_genome_status_no_vision(message_from_feagi):
    """
    Verify if full_list_dimension is empty, size list for vision is empty, if genome has been
    changed, or genome modified in real time.
    """
    global previous_genome_timestamp, genome_tracker, full_list_dimension, resize_list
    if message_from_feagi['genome_changed'] is not None:
        if full_list_dimension is None:
            full_list_dimension = []
        if len(full_list_dimension) == 0:
            full_list_dimension = fetch_full_dimensions()
        genome_changed = detect_genome_change(message_from_feagi)
        if genome_changed != previous_genome_timestamp:
            full_list_dimension = fetch_full_dimensions()
            previous_genome_timestamp = message_from_feagi["genome_changed"]
        current_tracker = obtain_genome_number(genome_tracker, message_from_feagi)
        if genome_tracker != current_tracker:
            full_list_dimension = fetch_full_dimensions()
            genome_tracker = current_tracker


def fetch_vision_turner(message_from_feagi, capabilities):
    """
    Update the threshold from the threshold OPU in BV. The current default values are 50, 255.
    """
    if full_list_dimension:
        if "ovtune" in message_from_feagi["opu_data"]:
            if message_from_feagi["opu_data"]["ovtune"]:
                for data_point in message_from_feagi["opu_data"]['ovtune']:
                    device_id = data_point.split('-')[0]
                    intensity_select = (int(data_point.split('-')[-1]))
                    max_depth_of_cortical = full_list_dimension['ovtune']['cortical_dimensions'][2] - 1
                    max_range = capabilities['camera']["threshold_range"][1]
                    min_range = capabilities['camera']["threshold_range"][0]
                    if int(device_id) == 1:
                        capabilities['camera']["percentage_to_allow_data"] = int(((intensity_select / max_depth_of_cortical) * (10 - 1)) + 1) / 10
                    else:
                        capabilities['camera']["threshold_default"][int(device_id)] = int(((intensity_select / max_depth_of_cortical) * (max_range - min_range)) + min_range)
    return capabilities


def fetch_threshold_type(message_from_feagi, capabilities):
    if "ov_thr" in message_from_feagi["opu_data"]:
        if message_from_feagi["opu_data"]["ov_thr"]:
            for data_point in message_from_feagi["opu_data"]["ov_thr"]:
                device_id = int(data_point.split('-')[0])
                capabilities['camera']["threshold_type"][int(device_id)] = True
    return capabilities


def fetch_mirror_opu(message_from_feagi, capabilities):
    if "ovflph" in message_from_feagi["opu_data"]:
        if message_from_feagi["opu_data"]["ovflph"]:
            if capabilities['camera']["mirror"]:
                capabilities['camera']["mirror"] = False
            else:
                capabilities['camera']["mirror"] = True
    return capabilities


def fetch_enhancement_data(message_from_feagi, capabilities):
    if full_list_dimension:
        if "ov_enh" in message_from_feagi["opu_data"]:
            if message_from_feagi["opu_data"]["ov_enh"]:
                for data_point in message_from_feagi["opu_data"]['ov_enh']:
                    device_id = int(data_point.split('-')[0])
                    if device_id == 1:
                        intensity_select = (int(data_point.split('-')[-1]))
                        max_depth_of_cortical = full_list_dimension['ov_enh']['cortical_dimensions'][
                                                 2] - 1
                        max_range = 1.4
                        min_range = 0.5
                        capabilities['camera']["enhancement"][int(device_id)] = float(((intensity_select / max_depth_of_cortical) * (max_range - min_range)) + min_range)
                    if device_id == 2:
                        intensity_select = (int(data_point.split('-')[-1]))
                        max_depth_of_cortical = full_list_dimension['ov_enh']['cortical_dimensions'][
                                                 2] - 1
                        max_range = 2.0
                        min_range = 0.8
                        capabilities['camera']["enhancement"][int(device_id)] = float(((intensity_select/ max_depth_of_cortical) * (max_range - min_range)) + min_range)
                    if device_id == 0:
                        intensity_select = (int(data_point.split('-')[-1]))
                        max_depth_of_cortical = full_list_dimension['ov_enh']['cortical_dimensions'][2]
                        max_range = 100
                        min_range = -100
                        capabilities['camera']["enhancement"][int(device_id)] = float(((intensity_select/ max_depth_of_cortical) * (max_range - min_range)) + min_range)
    return capabilities


def create_runtime_default_list(list, capabilities):
    """
    Generate the default capabilities for vision. Add a key in your configuration to overwrite this
    default list; otherwise, it will use the current default.

    Extra information:
        "threshold_default" # min #1, max #1, min #2, max #2,
        "threshold_type" # More detail below
        ^ simple thresholding types. see the retina.threshold_detect function. See below:
        [THRESH_BINARY, THRESH_BINARY_INV, THRESH_TRUNC, THRESH_TOZERO,  THRESH_TOZERO_INV, THRESH_OTSU]
        "threshold_name"  # Binary_threshold as a default
        "mirror" # flip the image
        "blink" # cv2 ndarray raw data of an image. Controlled by blink OPU in genome
        "eccentricity_control" # Controlled by eccentricity_control in genome
        "modulation_control" # Controlled by modulation_control in genome
        "vision_range" # min, max
        "size_list" # To get the size in real time based on genome's change/update
        "enhancement" # Controlled by enhancement OPU on inside the genome
    """
    if not list:
        list = {
            "input":{
                "camera": {
                    "0": {
                        "type": "ipu",
                        "disabled": False,
                        "index": "00",
                        "threshold_default": [50, 255, 130, 51],  # min #1, max #1, min #2, max #2,
                        "threshold_range": [1, 255],
                        "threshold_type": {},
                        # simple thresholding types. see the retina.threshold_detect function
                        "threshold_name": 0,  # Binary_threshold as a default
                        "mirror": True,  # flip the image
                        "blink": [],
                        # cv2 ndarray raw data of an image. Controlled by blink OPU in genome
                        "eccentricity_control": {'0': 1, '1': 1},
                        # Controlled by eccentricity_control in genome
                        "modulation_control": {'0': 99, '1': 99},
                        # Controlled by modulation_control in genome
                        "vision_range": [1, 99],  # min, max
                        "size_list": [],
                        # To get the size in real time based on genome's change/update
                        "enhancement": {},  # Enable ov_enh OPU on inside the genome
                        "percentage_to_allow_data": 1.0,
                        # this will be percentage for the full data.,
                        "dev_index": 0}
                }
            }
        }
        list = camera_config_update(list, capabilities)
    return list


def update_dict(d, u):
    for k, v in u.items():
        if isinstance(v, dict):
            d[k] = update_dict(d.get(k, {}), v)
        else:
            d[k] = v
    return d

def camera_config_update(list, capabilities):
    """
    Update the capabilities to overwrite the default generated capabilities.
    """
    if 'input' in capabilities and 'camera' in capabilities['input']:
        for device_id, device_config in capabilities['input']['camera'].items():
            if device_id not in list['input']['camera']:
                list['input']['camera'][device_id] = {}
            list['input']['camera'][device_id] = update_dict(list['input']['camera'][device_id], device_config)
    return list



def feagi_listener(feagi_opu_channel):
    """
    thread for listening FEAGI.
    """
    asyncio.run(router.fetch_feagi(feagi_opu_channel))


def start_websocket_in_threads(function, ip, port, ws_operation, ws, feagi_setting):
    threading.Thread(target=router.websocket_operation, args=(function, ip, port),
                     daemon=True).start()
    threading.Thread(target=router.bridge_operation, args=(ws_operation, ws, feagi_setting),
                     daemon=True).start()


def get_map_value(val, min1, max1, min2, max2):
    return feagi.map_value(val, min1, max1, min2, max2)
