#!/usr/bin/env python
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
import pycozmo
import asyncio
import traceback
import threading
import numpy as np
from PIL import Image
from time import sleep
import motor_functions
import facial_expression
from collections import deque
from datetime import datetime
from version import __version__
from feagi_connector import sensors
from feagi_connector import actuators
from feagi_connector import retina as retina
from feagi_connector import pns_gateway as pns
from feagi_connector import PIL_retina as pitina
from feagi_connector import feagi_interface as FEAGI

runtime_data = {
    "current_burst_id": 0,
    "feagi_state": None,
    "cortical_list": (),
    "battery_charge_level": 1,
    "host_network": {},
    'motor_status': {},
    'servo_status': {}
}

previous_frame_data = {}
rgb = {'camera': {}}
robot = {'accelerator': {}, "proximity": [], "gyro": [], 'servo_head': [], "battery": [],
         'lift_height': []}
camera_data = {"vision": []}
feagi.validate_requirements('requirements.txt')  # you should get it from the boilerplate generator


def window_average(sequence):
    return sum(sequence) // len(sequence)


def on_robot_state(cli, pkt: pycozmo.protocol_encoder.RobotState):
    """
    timestamp: The timestamp associated with the robot state.
    pose_frame_id: The ID of the frame of reference for the robot's pose.
    pose_origin_id: The ID of the origin for the robot's pose.
    pose_x, pose_y, pose_z: The x, y, and z coordinates of the robot's pose.
    pose_angle_rad: The angle of the robot's pose in radians.
    pose_pitch_rad: The pitch angle of the robot's pose in radians.
    lwheel_speed_mmps: Speed of the left wheel in millimeters per second.
    rwheel_speed_mmps: Speed of the right wheel in millimeters per second.
    head_angle_rad: The angle of the robot's head in radians.
    lift_height_mm: The height of the lift in millimeters.
    accel_x, accel_y, accel_z: Acceleration values along the x, y, and z axes.
    gyro_x, gyro_y, gyro_z: Gyroscopic values along the x, y, and z axes.
    battery_voltage: The voltage of the robot's battery.
    status: A status code associated with the robot.
    cliff_data_raw: Raw data related to cliff sensors.
    backpack_touch_sensor_raw: Raw data from the robot's backpack touch sensor.
    curr_path_segment: The ID of the current path segment.
    """
    robot['accelerator'] = {0: pkt.accel_x, 1: pkt.accel_y, 2: pkt.accel_z}
    robot['proximity'] = pkt.cliff_data_raw
    robot["gyro"] = [pkt.gyro_x, pkt.gyro_y, pkt.gyro_z]
    robot['servo_head'] = pkt.head_angle_rad
    robot['battery'] = pkt.battery_voltage
    robot['lift_height'] = pkt.lift_height_mm


async def expressions():
    expressions_array = [
        facial_expression.Neutral(),
        facial_expression.Excitement2(),
        facial_expression.Anger(),
        facial_expression.Sadness(),
        facial_expression.Happiness(),
        facial_expression.Surprise(),
        facial_expression.Disgust(),
        facial_expression.Fear(),
        facial_expression.Pleading(),
        facial_expression.Vulnerability(),
        facial_expression.Despair(),
        facial_expression.Guilt(),
        facial_expression.Disappointment(),
        facial_expression.Embarrassment(),
        facial_expression.Horror(),
        facial_expression.Skepticism(),
        facial_expression.Annoyance(),
        facial_expression.Fury(),
        facial_expression.Suspicion(),
        facial_expression.Rejection(),
        facial_expression.Boredom(),
        facial_expression.Tiredness(),
        facial_expression.Asleep(),
        facial_expression.Confusion(),
        facial_expression.Amazement(),
        facial_expression.Excitement()
    ]
    face_ignor_threshold = 1
    last_face_expression_time = time.time()
    while True:
        if face_selected:
            if time.time() - last_face_expression_time > face_ignor_threshold:
                last_face_expression_time = time.time()
                face_generator = pycozmo.procedural_face.interpolate(
                    facial_expression.Neutral(), expressions_array[face_selected[0]],
                    pycozmo.robot.FRAME_RATE * 2)
                for face in face_generator:
                    # expressions_array[0].eyes[0].lids[1].y -= 0.1
                    # expressions_array[0].eyes[0].lids[1].bend -= 0.1
                    # expressions_array[0].eyes[0].lids[0].angle += 25.0
                    # expressions_array[0].eyes[1].upper_inner_radius_x += 1.0
                    # expressions_array[0].eyes[0].upper_inner_radius_x += 1.0
                    # expressions_array[0].eyes[0].scale_x += 1.25
                    # expressions_array[0].eyes[1].upper_outer_radius_x = 1.0
                    if eye_one_location:
                        expressions_array[0].eyes[0].center_x = eye_one_location[0][0]
                        expressions_array[0].eyes[0].center_y = eye_one_location[0][1]
                        eye_one_location.pop()
                    if eye_two_location:
                        expressions_array[0].eyes[1].center_x = eye_two_location[0][0]
                        expressions_array[0].eyes[1].center_y = eye_two_location[0][1]
                        eye_two_location.pop()
                    # Render face image.
                    im = face.render()
                    # The Cozmo protocol expects a 128x32 image, so take only the even lines.
                    np_im = np.array(im)
                    np_im2 = np_im[::2]
                    im2 = Image.fromarray(np_im2)
                    # Display face image.
                    cli.display_image(im2)
            face_selected.pop()
            if len(face_selected) > 2:
                temp = face_selected.pop()
                face_selected.clear()
                face_selected.append(temp)
        else:
            time.sleep(0.05)


def face_starter():
    asyncio.run(expressions())


def on_body_info(cli, pkt: pycozmo.protocol_encoder.BodyInfo):
    print("pkt: ", pkt)


def on_camera_image(cli, image):
    global default_capabilities, previous_frame_data, rgb
    # Obtain the size automatically which will be needed in next line after the next line
    size = pitina.obtain_size(image)
    # Convert into ndarray based on the size it gets
    new_rgb = retina.RGB_list_to_ndarray(image.getdata(), size)
    # update astype to work well with retina and cv2
    raw_frame = retina.update_astype(new_rgb)
    camera_data['vision'] = raw_frame
    time.sleep(0.01)

def vision_initalization(cli):
    cli.add_handler(pycozmo.event.EvtNewRawCameraImage, on_camera_image)


def robot_status(cli):
    cli.add_handler(pycozmo.protocol_encoder.RobotState, on_robot_state)


def move_head(cli, angle, max, min):
    if min <= angle <= max:
        cli.set_head_angle(angle)  # move head
        return True
    else:
        print("reached to limit")
        return False


def lift_arms(cli, angle, max, min):
    if min <= angle <= max:
        cli.set_lift_height(angle)  # move head
        return True
    else:
        face_selected.append(4)
        return False


def action(obtained_data, arms_angle, head_angle, motor_data):
    recieve_motor_data = actuators.get_motor_data(obtained_data, motor_data)
    if recieve_motor_data:
        for motor_id in recieve_motor_data:
            if str(motor_id) in capabilities['output']['motor']:
                if not capabilities['output']['motor'][str(motor_id)]['disable']:
                    actuators.pass_the_power_to_motor(capabilities['output']['motor'][str(motor_id)]['max_power'],
                                                      recieve_motor_data[motor_id],
                                                      motor_id,
                                                      motor_data)
    else:
        motor_data = actuators.rolling_window_update(motor_data)


    recieve_servo_data = actuators.get_servo_data(obtained_data)
    rwheel_speed = motor_data[0][0]
    lwheel_speed = motor_data[1][0]
    motor_functions.drive_wheels(cli,
                                 lwheel_speed=lwheel_speed,
                                 rwheel_speed=rwheel_speed,
                                 duration=feagi_settings['feagi_burst_speed'] / 2)

    for id in recieve_servo_data:  # example output: {0: 100, 2: 100}
        device_id = actuators.feagi_id_converter(id)
        if not capabilities['output']['servo'][str(device_id)]['disable']:
            servo_power = actuators.servo_generate_power(capabilities['output']["servo"][str(device_id)]["max_power"], recieve_servo_data[id], id)
            if device_id == 0:
                test_head_angle = head_angle
                test_head_angle += servo_power
                if move_head(cli, test_head_angle, max, min):
                    head_angle = test_head_angle
            if device_id == 1:
                test_arm_angle = arms_angle
                test_arm_angle += servo_power
                if lift_arms(cli, test_arm_angle, max_lift, min_lift):
                    arms_angle = test_arm_angle

    if "misc" in obtained_data:
        if obtained_data["misc"]:
            print("face: ", face_selected, " misc: ", obtained_data["misc"])
            for i in obtained_data["misc"]:
                face_selected.append(i)
        obtained_data['misc'].clear()
    return arms_angle, head_angle


if __name__ == '__main__':
    config = FEAGI.build_up_from_configuration()
    feagi_settings = config['feagi_settings'].copy()
    agent_settings = config['agent_settings'].copy()
    default_capabilities = config['default_capabilities'].copy()
    message_to_feagi = config['message_to_feagi'].copy()
    capabilities = config['capabilities'].copy()

    # # # FEAGI registration # # # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    # - - - - - - - - - - - - - - - - - - #
    feagi_settings, runtime_data, api_address, feagi_ipu_channel, feagi_opu_channel = \
        FEAGI.connect_to_feagi(feagi_settings, runtime_data, agent_settings, capabilities,
                               __version__)
    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    face_selected = deque()
    eye_one_location = deque()
    eye_two_location = deque()
    motor_data = dict()
    for motor_id in capabilities['output']['motor']:
        if 'rolling_window_len' in capabilities['output']['motor'][motor_id]:
            length_rolling_window = capabilities['output']['motor'][motor_id]['rolling_window_len']
        else:
            length_rolling_window = 0  # Default to 0 which will be extremely sensitive and stiff
        motor_data = actuators.create_motor_rolling_window_len(length_window=length_rolling_window,
                                                               current_rolling_window_dict=motor_data,
                                                               motor_id=motor_id)


    threading.Thread(target=face_starter, daemon=True).start()
    msg_counter = 0
    genome_tracker = 0
    # # Raise head.
    cli = pycozmo.Client()
    cli.start()
    cli.connect()
    cli.wait_for_robot()
    # print("max in rad: ", pycozmo.robot.MAX_HEAD_ANGLE.radians)  # 0.7766715171374767
    # print("min in rad: ", pycozmo.robot.MIN_HEAD_ANGLE.radians)  # -0.4363323129985824
    max = pycozmo.robot.MAX_HEAD_ANGLE.radians - 0.1
    min = pycozmo.robot.MIN_HEAD_ANGLE.radians + 0.1
    max_lift = pycozmo.MAX_LIFT_HEIGHT.mm - 5
    min_lift = pycozmo.MIN_LIFT_HEIGHT.mm + 5
    angle_of_head = \
        (pycozmo.robot.MAX_HEAD_ANGLE.radians - pycozmo.robot.MIN_HEAD_ANGLE.radians) / 2.0
    angle_of_arms = 50  # TODO: How to obtain the arms encoders in real time
    cli.set_head_angle(angle_of_head)  # move head

    # # vision capture
    cli.enable_camera(enable=True, color=True)
    threading.Thread(target=robot_status, args=(cli,), daemon=True).start()
    threading.Thread(target=vision_initalization, args=(cli,), daemon=True).start()
    threading.Thread(target=retina.vision_progress,
                     args=(default_capabilities,feagi_settings,
                           camera_data,), daemon=True).start()
    time.sleep(2)
    # vision ends

    while True:
        try:
            message_from_feagi = pns.message_from_feagi
            if message_from_feagi:
                obtained_signals = pns.obtain_opu_data(message_from_feagi)
                # action(obtained_signals, angle_of_arms, angle_of_head, motor_data)
                angle_of_arms, angle_of_head = action(obtained_signals, angle_of_arms, angle_of_head, motor_data)
                # OPU section ENDS
                if "o_eye1" in message_from_feagi["opu_data"]:
                    if message_from_feagi["opu_data"]["o_eye1"]:
                        for i in message_from_feagi["opu_data"]["o_eye1"]:
                            split_data = i.split("-")
                            y_array = [70, 40, -60]
                            if split_data[0] == '2':
                                eye_one_location.append([80, y_array[int(split_data[1])]])
                            if split_data[0] == '1':
                                eye_one_location.append([0, y_array[int(split_data[1])]])
                            if split_data[0] == '0':
                                eye_one_location.append([-30, y_array[int(split_data[1])]])
                        face_selected.append(0)
                if "o_eye2" in message_from_feagi["opu_data"]:
                    if message_from_feagi["opu_data"]["o_eye2"]:
                        for i in message_from_feagi["opu_data"]["o_eye2"]:
                            split_data = i.split("-")
                            y_array = [65, 40, -50]
                            if split_data[0] == '2':
                                eye_two_location.append([40, y_array[int(split_data[1])]])
                            if split_data[0] == '1':
                                eye_two_location.append([-10, y_array[int(split_data[1])]])
                            if split_data[0] == '0':
                                eye_two_location.append([-30, y_array[int(split_data[1])]])
                        if len(face_selected) == 0:
                            face_selected.append(0)
                # if "o_init" in message_from_feagi["opu_data"]:
                #     if message_from_feagi["opu_data"]["o_init"]:
                #         for i in message_from_feagi["opu_data"]["o_init"]:
                #             split_data = i.split("-")
                #             if split_data[0] == '0':
                #                 motor_functions.display_lines(cli)

            raw_frame = camera_data['vision']
            # print(camera_data['vision'])
            # default_capabilities['input']['camera']['0']['blink'] = []
            previous_frame_data, rgb, default_capabilities = retina.process_visual_stimuli(
                raw_frame,
                default_capabilities,
                previous_frame_data,
                rgb, capabilities)
            # cv2.imshow("test",   raw_frame)
            # cv2.waitKey(30)
            if rgb:
                message_to_feagi = pns.generate_feagi_data(rgb, message_to_feagi)
            if robot['battery']:
                for device_id in capabilities['input']['battery']:
                    if not capabilities['input']['battery'][device_id]['disable']:
                        cortical_id = capabilities['input']['battery'][device_id]["cortical_id"]
                        create_data_list = dict()
                        create_data_list[cortical_id] = dict()
                        start_point = capabilities['input']['battery'][device_id]["feagi_index"] * len(capabilities['input']['battery'])
                        feagi_data_position = start_point
                        capabilities['input']['battery']['0']['maximum_value'], capabilities['input']['battery']['0']['minimum_value'] = sensors.measuring_max_and_min_range(robot['battery'],
                                                            capabilities['input']['battery']['0']['maximum_value'],
                                                            capabilities['input']['battery']['0']['minimum_value'])

                        position_in_feagi_location = sensors.convert_sensor_to_ipu_data(
                                                            capabilities['input']['battery']['0']['minimum_value'],
                                                           capabilities['input']['battery']['0']['maximum_value'],
                                                           robot['battery'],
                                                           capabilities['input']['battery']['0']['feagi_index'],
                                                           cortical_id=cortical_id)
                        create_data_list[cortical_id][position_in_feagi_location] = 100
                        if create_data_list[cortical_id]:
                            message_to_feagi = sensors.add_generic_input_to_feagi_data(create_data_list, message_to_feagi)

            if robot['gyro']:
                for device_id in capabilities['input']['gyro']:
                    if not capabilities['input']['gyro'][device_id]['disable']:
                        cortical_id = capabilities['input']['gyro'][device_id]["cortical_id"]
                        create_data_list = dict()
                        create_data_list[cortical_id] = dict()
                        start_point = capabilities['input']['gyro'][device_id]["feagi_index"] * len(capabilities['input']['gyro'])
                        feagi_data_position = start_point
                        try:
                            for device_id in range(len(capabilities['input']['gyro'][device_id]['max_value'])):
                                capabilities['input']['gyro']['0']['max_value'][device_id], capabilities['input']['gyro']['0']['min_value'][device_id] = sensors.measuring_max_and_min_range(robot['gyro'][device_id],
                                                                    capabilities['input']['gyro']['0']['max_value'][device_id],
                                                                    capabilities['input']['gyro']['0']['min_value'][device_id])

                                position_in_feagi_location = sensors.convert_sensor_to_ipu_data(
                                    capabilities['input']['gyro']['0']['min_value'][device_id],
                                    capabilities['input']['gyro']['0']['max_value'][device_id],
                                    robot['gyro'][device_id],
                                    capabilities['input']['gyro']['0']['feagi_index'] + device_id,
                                    cortical_id=cortical_id,
                                    symmetric=True)
                                create_data_list[cortical_id][position_in_feagi_location] = 100
                            if create_data_list[cortical_id]:
                                message_to_feagi = sensors.add_generic_input_to_feagi_data(create_data_list, message_to_feagi)
                        except:
                            pass

            # # Add accelerator section
            if robot['accelerator']:
                for device_id in capabilities['input']['accelerator']:
                    if not capabilities['input']['accelerator'][device_id]['disable']:
                        cortical_id = capabilities['input']['accelerator'][device_id]["cortical_id"]
                        create_data_list = dict()
                        create_data_list[cortical_id] = dict()
                        start_point = capabilities['input']['accelerator'][device_id]["feagi_index"] * len(capabilities['input']['accelerator'])
                        feagi_data_position = start_point
                        try:
                            for device_id in range(len(capabilities['input']['accelerator']['0']['max_value'])):
                                capabilities['input']['accelerator']['0']['max_value'][device_id], capabilities['input']['accelerator']['0']['min_value'][device_id] = sensors.measuring_max_and_min_range(robot['accelerator'][device_id],
                                                                    capabilities['input']['accelerator']['0']['max_value'][device_id],
                                                                    capabilities['input']['accelerator']['0']['min_value'][device_id])

                                position_in_feagi_location = sensors.convert_sensor_to_ipu_data(
                                    capabilities['input']['accelerator']['0']['min_value'][device_id],
                                    capabilities['input']['accelerator']['0']['max_value'][device_id],
                                    robot['accelerator'][device_id],
                                    capabilities['input']['accelerator']['0']['feagi_index'] + device_id,
                                    cortical_id=cortical_id,
                                    symmetric=True)
                                create_data_list[cortical_id][position_in_feagi_location] = 100
                            if create_data_list[cortical_id]:
                                message_to_feagi = sensors.add_generic_input_to_feagi_data(create_data_list, message_to_feagi)
                        except:
                            pass

            if robot['proximity']:
                for device_id in capabilities['input']['proximity']:
                    if not capabilities['input']['proximity'][device_id]['disable']:
                        cortical_id = capabilities['input']['proximity'][device_id]["cortical_id"]
                        create_data_list = dict()
                        create_data_list[cortical_id] = dict()
                        start_point = capabilities['input']['proximity'][device_id]["feagi_index"] * len(capabilities['input']['proximity'])
                        feagi_data_position = start_point
                        capabilities['input']['proximity']['0']['proximity_max_distance'], capabilities['input']['proximity']['0']['proximity_min_distance'] = sensors.measuring_max_and_min_range(robot['proximity'][0],
                                                            capabilities['input']['proximity']['0']['proximity_max_distance'],
                                                            capabilities['input']['proximity']['0']['proximity_min_distance'])

                        position_in_feagi_location = sensors.convert_sensor_to_ipu_data(
                                                            capabilities['input']['proximity']['0']['proximity_min_distance'],
                                                           capabilities['input']['proximity']['0']['proximity_max_distance'],
                                                           robot['proximity'][0],
                                                           capabilities['input']['proximity']['0']['feagi_index'],
                                                           cortical_id=cortical_id)
                        create_data_list[cortical_id][position_in_feagi_location] = 100
                        if create_data_list[cortical_id]:
                            message_to_feagi = sensors.add_generic_input_to_feagi_data(create_data_list, message_to_feagi)
            sleep(feagi_settings['feagi_burst_speed'])  # bottleneck
            pns.signals_to_feagi(message_to_feagi, feagi_ipu_channel, agent_settings, feagi_settings)
            message_to_feagi.clear()

            if rgb:
                for i in rgb['camera']:
                    rgb['camera'][i].clear()
        except Exception as e:
            print("ERROR IN COZMO MAIN CODE: ", e)
            traceback.print_exc()
            break
