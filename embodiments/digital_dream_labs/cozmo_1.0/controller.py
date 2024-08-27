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
import asyncio
import pycozmo
import threading
import traceback
import cozmo_functions
from time import sleep
import facial_expression
from version import __version__
from feagi_connector import retina
from feagi_connector import sensors
from feagi_connector import actuators
from feagi_connector import pns_gateway as pns
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
FEAGI.validate_requirements('requirements.txt')  # you should get it from the boilerplate generator


def face_starter(cli):
    asyncio.run(facial_expression.expressions(cli))


def action(obtained_data, arms_angle, head_angle, motor_data, motor_mapped):
    recieve_motor_data = actuators.get_motor_data(obtained_data, motor_data)
    recieve_servo_data = actuators.get_servo_data(obtained_data)
    recieve_servo_position_data = actuators.get_servo_position_data(obtained_data)

    if recieve_servo_position_data:
        for feagi_id in recieve_servo_position_data:
            device_id_list = actuators.feagi_mapped_to_dev_index(dev_id='servo', feagi_index=feagi_id, mapped_dict=motor_mapped)
            for device_id in device_id_list:
                if not capabilities['output']['servo'][str(device_id)]['disabled']:
                    new_power = actuators.get_position_data(recieve_servo_position_data[device_id], capabilities['output']['servo'][str(device_id)][ 'min_value'], capabilities['output']['servo'][str(device_id)][ 'max_value'])
                    if device_id == 0:
                        if cozmo_functions.move_head(cli, new_power, max, min):
                            head_angle = new_power
                    if device_id == 1:
                        if cozmo_functions.lift_arms(cli, new_power, max_lift, min_lift):
                            arms_angle = new_power


    if recieve_motor_data:
        for motor_id in recieve_motor_data:
            if str(motor_id) in capabilities['output']['motor']:
                if not capabilities['output']['motor'][str(motor_id)]['disabled']:
                    actuators.pass_the_power_to_motor(capabilities['output']['motor'][str(motor_id)]['max_power'],
                                                      recieve_motor_data[motor_id],
                                                      motor_id,
                                                      motor_data)
    else:
        motor_data = actuators.rolling_window_update(motor_data)
    rwheel_speed = 0
    lwheel_speed = 0
    for motor_id in motor_mapped['motor']:
        device_id_list = actuators.feagi_mapped_to_dev_index(dev_id='motor', feagi_index=motor_id, mapped_dict=motor_mapped)
        for motor in device_id_list:
                data_power = motor_data[motor][0]  # negative is forward on freenove. So that way, FEAGI dont get confused
                if motor_id == 0:
                    rwheel_speed = data_power
                if motor_id == 1:
                 lwheel_speed = data_power
    cozmo_functions.drive_wheels(cli,
                                 lwheel_speed=lwheel_speed,
                                 rwheel_speed=rwheel_speed,
                                 duration=feagi_settings['feagi_burst_speed'] / 2)

    for feagi_id in recieve_servo_data:  # example output: {0: 100, 2: 100}
        device_id_list = actuators.feagi_mapped_to_dev_index(dev_id='servo', feagi_index=feagi_id, mapped_dict=motor_mapped)
        for device_id in device_id_list:
            if not capabilities['output']['servo'][str(device_id)]['disabled']:
                servo_power = actuators.servo_generate_power(capabilities['output']["servo"][str(device_id)]["max_power"], recieve_servo_data[feagi_id], feagi_id)
                if device_id == 0:
                    test_head_angle = head_angle
                    test_head_angle += servo_power
                    if cozmo_functions.move_head(cli, test_head_angle, max, min):
                        head_angle = test_head_angle
                if device_id == 1:
                    test_arm_angle = arms_angle
                    test_arm_angle += servo_power
                    if cozmo_functions.lift_arms(cli, test_arm_angle, max_lift, min_lift, facial_expression.face_selected):
                        arms_angle = test_arm_angle


    if "misc" in obtained_data:
        if obtained_data["misc"]:
            print("face: ", facial_expression.face_selected, " misc: ", obtained_data["misc"])
            for i in obtained_data["misc"]:
                facial_expression.face_selected.append(i)
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
    motor_data = dict()
    for motor_id in capabilities['output']['motor']:
        if 'rolling_window_len' in capabilities['output']['motor'][motor_id]:
            length_rolling_window = capabilities['output']['motor'][motor_id]['rolling_window_len']
        else:
            length_rolling_window = 0  # Default to 0 which will be extremely sensitive and stiff
        motor_data = actuators.create_motor_rolling_window_len(length_window=length_rolling_window,
                                                               current_rolling_window_dict=motor_data,
                                                               motor_id=motor_id)

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
    motor_mapped = actuators.actuator_to_feagi_map(capabilities)

    # # vision capture
    cli.enable_camera(enable=True, color=True)
    threading.Thread(target=face_starter, args=(cli,), daemon=True).start()
    threading.Thread(target=cozmo_functions.robot_status, args=(cli,), daemon=True).start()
    threading.Thread(target=cozmo_functions.vision_initalization, args=(cli,), daemon=True).start()
    threading.Thread(target=retina.vision_progress,
                     args=(default_capabilities,feagi_settings,
                           cozmo_functions.camera_data,), daemon=True).start()
    time.sleep(2)
    # vision ends

    while True:
        try:
            message_from_feagi = pns.message_from_feagi
            if message_from_feagi:
                obtained_signals = pns.obtain_opu_data(message_from_feagi)
                # action(obtained_signals, angle_of_arms, angle_of_head, motor_data)
                angle_of_arms, angle_of_head = action(obtained_signals, angle_of_arms, angle_of_head, motor_data, motor_mapped)
                # OPU section ENDS
                # if "o_eye1" in message_from_feagi["opu_data"]:
                #     if message_from_feagi["opu_data"]["o_eye1"]:
                #         for i in message_from_feagi["opu_data"]["o_eye1"]:
                #             split_data = i.split("-")
                #             y_array = [70, 40, -60]
                #             if split_data[0] == '2':
                #                 eye_one_location.append([80, y_array[int(split_data[1])]])
                #             if split_data[0] == '1':
                #                 eye_one_location.append([0, y_array[int(split_data[1])]])
                #             if split_data[0] == '0':
                #                 eye_one_location.append([-30, y_array[int(split_data[1])]])
                #         facial_expression.face_selected.append(0)
                # if "o_eye2" in message_from_feagi["opu_data"]:
                #     if message_from_feagi["opu_data"]["o_eye2"]:
                #         for i in message_from_feagi["opu_data"]["o_eye2"]:
                #             split_data = i.split("-")
                #             y_array = [65, 40, -50]
                #             if split_data[0] == '2':
                #                 eye_two_location.append([40, y_array[int(split_data[1])]])
                #             if split_data[0] == '1':
                #                 eye_two_location.append([-10, y_array[int(split_data[1])]])
                #             if split_data[0] == '0':
                #                 eye_two_location.append([-30, y_array[int(split_data[1])]])
                #         if len(facial_expression.face_selected) == 0:
                #             facial_expression.face_selected.append(0)
                # if "o_init" in message_from_feagi["opu_data"]:
                #     if message_from_feagi["opu_data"]["o_init"]:
                #         for i in message_from_feagi["opu_data"]["o_init"]:
                #             split_data = i.split("-")
                #             if split_data[0] == '0':
                #                 cozmo_functions.display_lines(cli)

            # Vision section START
            raw_frame = cozmo_functions.camera_data['vision']
            previous_frame_data, rgb, default_capabilities = retina.process_visual_stimuli(
                raw_frame,
                default_capabilities,
                previous_frame_data,
                rgb, capabilities)
            if rgb:
                message_to_feagi = pns.generate_feagi_data(rgb, message_to_feagi)
            # Vision section END

            if cozmo_functions.robot['battery']:
                cortical_id = pns.name_to_feagi_id(sensor_name='battery')
                current_battery = cozmo_functions.robot['battery']
                for device_id in capabilities['input']['battery']:
                    if not capabilities['input']['battery'][device_id]['disabled']:
                        create_data_list = dict()
                        create_data_list[cortical_id] = dict()
                        position_in_feagi_location = sensors.convert_sensor_to_ipu_data(
                            capabilities['input']['battery']['0']['minimum_value'],
                            capabilities['input']['battery']['0']['maximum_value'],
                            current_battery,
                            capabilities['input']['battery']['0']['feagi_index'],
                            sensor_name='battery')
                        create_data_list[cortical_id][position_in_feagi_location] = 100
                        if create_data_list[cortical_id]:
                            message_to_feagi = sensors.add_generic_input_to_feagi_data(create_data_list,
                                                                                       message_to_feagi)

            if cozmo_functions.robot['gyro']:
                for device_id in capabilities['input']['gyro']:
                    if not capabilities['input']['gyro'][device_id]['disabled']:
                        cortical_id = pns.name_to_feagi_id(sensor_name='gyro')
                        create_data_list = dict()
                        create_data_list[cortical_id] = dict()
                        try:
                            for inner_device_id in range(len(capabilities['input']['gyro'][device_id]['max_value'])):
                                capabilities['input']['gyro'][device_id]['max_value'][inner_device_id], \
                                capabilities['input']['gyro'][device_id]['min_value'][
                                    inner_device_id] = sensors.measuring_max_and_min_range(
                                    cozmo_functions.robot['gyro'][inner_device_id],
                                    capabilities['input']['gyro'][device_id]['max_value'][inner_device_id],
                                    capabilities['input']['gyro'][device_id]['min_value'][inner_device_id])
                                position_in_feagi_location = sensors.convert_sensor_to_ipu_data(
                                    capabilities['input']['gyro'][device_id]['min_value'][inner_device_id],
                                    capabilities['input']['gyro'][device_id]['max_value'][inner_device_id],
                                    cozmo_functions.robot['gyro'][inner_device_id],
                                    capabilities['input']['gyro'][device_id]['feagi_index'] + inner_device_id,
                                    sensor_name='gyro',
                                    symmetric=True)
                                create_data_list[cortical_id][position_in_feagi_location] = 100
                            if create_data_list[cortical_id]:
                                message_to_feagi = sensors.add_generic_input_to_feagi_data(
                                    create_data_list, message_to_feagi)
                        except Exception as e:
                            print("here: ", e)
                            traceback.print_exc()

            # # Add accelerator section
            if cozmo_functions.robot['accelerator']:
                if pns.full_template_information_corticals:
                    if cozmo_functions.robot['accelerator']:
                        for device_id in capabilities['input']['accelerometer']:
                            if not capabilities['input']['accelerometer'][device_id]['disabled']:
                                cortical_id = pns.name_to_feagi_id(sensor_name='accelerometer')
                                create_data_list = dict()
                                create_data_list[cortical_id] = dict()
                                try:
                                    for inner_device_id in range(
                                            len(capabilities['input']['accelerometer'][device_id]['max_value'])):
                                        capabilities['input']['accelerometer'][device_id]['max_value'][inner_device_id], \
                                            capabilities['input']['accelerometer'][device_id]['min_value'][
                                                inner_device_id] = sensors.measuring_max_and_min_range(
                                            cozmo_functions.robot['accelerator'][inner_device_id],
                                            capabilities['input']['accelerometer'][device_id]['max_value'][
                                                inner_device_id],
                                            capabilities['input']['accelerometer'][device_id]['min_value'][
                                                inner_device_id])
                                        position_in_feagi_location = sensors.convert_sensor_to_ipu_data(
                                            capabilities['input']['accelerometer'][device_id]['min_value'][
                                                inner_device_id],
                                            capabilities['input']['accelerometer'][device_id]['max_value'][
                                                inner_device_id],
                                            cozmo_functions.robot['accelerator'][inner_device_id],
                                            capabilities['input']['accelerometer'][device_id][
                                                'feagi_index'] + inner_device_id,
                                            sensor_name='accelerometer',
                                            symmetric=True)
                                        create_data_list[cortical_id][position_in_feagi_location] = 100
                                    if create_data_list[cortical_id]:
                                        message_to_feagi = sensors.add_generic_input_to_feagi_data(
                                            create_data_list, message_to_feagi)
                                except Exception as e:
                                    pass

            if cozmo_functions.robot['proximity']:
                if pns.full_template_information_corticals:
                    cortical_id = pns.name_to_feagi_id(sensor_name='proximity')
                    for device_id in capabilities['input']['proximity']:
                        if not capabilities['input']['proximity'][device_id]['disabled']:
                            create_data_list = dict()
                            create_data_list[cortical_id] = dict()
                            capabilities['input']['proximity'][device_id]['max_value'], \
                            capabilities['input']['proximity'][device_id][
                                'min_value'] = sensors.measuring_max_and_min_range(
                                cozmo_functions.robot['proximity'][int(device_id)],
                                capabilities['input']['proximity'][device_id]['max_value'],
                                capabilities['input']['proximity'][device_id]['min_value'])

                            position_in_feagi_location = sensors.convert_sensor_to_ipu_data(
                                capabilities['input']['proximity'][device_id]['min_value'],
                                capabilities['input']['proximity'][device_id]['max_value'],
                                cozmo_functions.robot['proximity'][int(device_id)],
                                capabilities['input']['proximity'][device_id]['feagi_index'],
                                sensor_name='proximity',
                                symmetric=True)
                            create_data_list[cortical_id][position_in_feagi_location] = 100
                            if create_data_list[cortical_id]:
                                message_to_feagi = sensors.add_generic_input_to_feagi_data(create_data_list,
                                                                                           message_to_feagi)

            if pns.full_template_information_corticals:
                cortical_id = pns.name_to_feagi_id(sensor_name='servo')
                for device_id in capabilities['input']['servo']:
                    raw_data = 0
                    if device_id == '0':
                        raw_data = angle_of_head
                    if device_id == '1':
                        raw_data = angle_of_arms
                    if not capabilities['input']['servo'][device_id]['disabled']:
                        create_data_list = dict()
                        create_data_list[cortical_id] = dict()
                        position_in_feagi_location = sensors.convert_sensor_to_ipu_data(
                            capabilities['input']['servo'][device_id]['min_value'],
                            capabilities['input']['servo'][device_id]['max_value'],
                            raw_data,
                            capabilities['input']['servo'][device_id]['feagi_index'],
                            sensor_name='servo',
                            symmetric=True)
                        create_data_list[cortical_id][position_in_feagi_location] = 100
                        if create_data_list[cortical_id]:
                            message_to_feagi = sensors.add_generic_input_to_feagi_data(create_data_list,
                                                                                       message_to_feagi)
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
