import os
import json
import time
import argparse
import threading
from djitellopy import Tello
from datetime import datetime
from version import __version__
from feagi_connector import router
from feagi_connector import retina
from feagi_connector import sensors
from feagi_connector import actuators
from feagi_connector import pns_gateway as pns
from feagi_connector import feagi_interface as FEAGI

previous_frame_data = dict()
flag = False
camera_data = {"vision": {}}
speed = {'0': 50}


def get_battery(full_data):
    """
    full data should be a raw data of get_current_state().
    This will return the battery using the raw full data
    """
    new_data = dict()
    new_data['battery_charge_level'] = full_data['bat']
    return new_data


def get_ultrasonic(full_data):
    """
    full data should be a raw data of get_current_state().
    This will return the battery using the raw full data
    """
    return full_data['tof'] * 0.01  # convert to meter unit


def get_gyro(full_data):
    """
        full data should be a raw data of get_current_state().
        This function will return gyro data only.
        This gyro is 3 axis gyro.
    """
    new_data = dict()
    try:
        new_data['0'] = convert_gyro_into_feagi(full_data['pitch'],
                                                capabilities['gyro']['resolution'],
                                                capabilities['acc']['range'])
        new_data['1'] = convert_gyro_into_feagi(full_data['roll'],
                                                capabilities['gyro']['resolution'],
                                                capabilities['acc']['range'])
        new_data['2'] = convert_gyro_into_feagi(full_data['yaw'],
                                                capabilities['gyro']['resolution'],
                                                capabilities['acc']['range'])
        return new_data
    except Exception as e:
        print("ERROR STARTS WITH: ", e)


def get_accelerator(full_data):
    """
    full data should be a raw data of get_current_state().
    This function will return acc data only.
    """
    new_data = dict()
    try:
        new_data['0'] = convert_gyro_into_feagi(full_data['agx'],
                                                capabilities['acc']['resolution'],
                                                capabilities['acc']['range'])
        new_data['1'] = convert_gyro_into_feagi(full_data['agy'],
                                                capabilities['acc']['resolution'],
                                                capabilities['acc']['range'])
        new_data['2'] = offset_z(full_data['agz'], capabilities['acc']['resolution'],
                                 capabilities['acc']['range'])
        return new_data
    except Exception as e:
        print("ERROR STARTS WITH: ", e)


def return_resolution(data):
    """
    try return_resolution(tello.get_frame_read()) in your main.
    data should be `tello.get_frame_read()`
    this will return height and width. Update your config with this numbers as well
    """
    frame_read = data
    height, width, _ = frame_read.frame.shape
    return height, width

def misc_control(self, data, battery_level):
    global flag
    if data == 0:
        print("flag: ", flag)
        try:
            if flag == False:
                print("takeoff!")
                self.send_command_without_return("takeoff")
                flag = True
        except Exception as e:
            print("ERROR AT: ", e)
    if data == 1:
        print("flag: ", flag)
        if flag:
            print("landed!")
            self.send_command_without_return("land")
            flag = False
    if data == 2:
        try:
            if battery_level >= 50:
                self.send_command_without_return("flip {}".format("f"))
            else:
                print(
                    "ERROR! The battery is low. It must be at least above than 51% to be able to "
                    "flip")
        except Exception as e:
            print("Error at: ", e)
    if data == 3:
        try:
            if battery_level >= 50:
                self.send_command_without_return("flip {}".format("b"))
            else:
                print(
                    "ERROR! The battery is low. It must be at least above than 51% to be able to "
                    "flip")
        except Exception as e:
            print("Error at: ", e)
    if data == 4:
        try:
            if battery_level >= 50:
                self.send_command_without_return("flip {}".format("r"))
            else:
                print(
                    "ERROR! The battery is low. It must be at least above than 51% to be able to "
                    "flip")
        except Exception as e:
            print("Error at: ", e)
    if data == 5:
        try:
            if battery_level >= 50:
                self.send_command_without_return("flip {}".format("l"))
            else:
                print(
                    "ERROR! The battery is low. It must be at least above than 51% to be able to "
                    "flip")
        except Exception as e:
            print("Error at: ", e)


def full_frame(self):
    frame_read = self.get_frame_read()
    return frame_read.frame


def start_camera(self):
    """
    self as instantiation only
    """
    self.streamon()


def navigate_to_xyz(self, x=0, y=0, z=0, s=0):
    cmd = 'go {} {} {} {}'.format(x, y, z, s)
    self.send_command_without_return(cmd)


def convert_gyro_into_feagi(value, resolution, range_number):
    new_value = value - (range_number[0])
    return (new_value * resolution) / (range_number[1] - range_number[0])


def offset_z(value, resolution, range_number):
    """"
    Gravity is 9.8 m/s^2 however when the drone is on the table, it should be at zero. This
    offset will keep it to zero if the value is between than 2 and -2, it will be zero.
    """
    new_value = value - (range_number[0])
    new_value = (new_value * resolution) / (range_number[1] - range_number[0])
    if new_value > 2:
        return new_value
    elif new_value < -2:
        return new_value
    else:
        return 0


def action(obtained_signals):
    global speed
    recieve_emergency_stop = actuators.check_emergency_stop(obtained_signals)
    if recieve_emergency_stop:
        tello.send_command_without_return("emergency")  # STOP EVERYTHING IMMEDIATELY
    recieve_motion_control_data = actuators.get_motion_control_data(obtained_signals)
    recieve_speed_data = actuators.check_new_speed(obtained_signals)
    if recieve_speed_data:
        for i in recieve_speed_data:
            speed['0'] = recieve_speed_data[i]
    if recieve_motion_control_data:
        for i in recieve_motion_control_data:
            if 'yaw_left' == i:
                tello.send_command_without_return("cw {}".format(recieve_motion_control_data[i] *
                                                                 100))
            if 'yaw_right' == i:
                tello.send_command_without_return("ccw {}".format(recieve_motion_control_data[i]
                                                                  * 100))
            if "move_left" == i:
                navigate_to_xyz(tello, 0, 100 * int(recieve_motion_control_data[i]), 0, speed['0'])
            if "move_right" == i:
                navigate_to_xyz(tello, 0, -100 * int(recieve_motion_control_data[i]), 0, speed['0'])
            if "move_up" == i:
                navigate_to_xyz(tello, 0, 0, 100 * int(recieve_motion_control_data[i]), speed['0'])
            if "move_down" == i:
                navigate_to_xyz(tello, 0, 0, -100 * int(recieve_motion_control_data[i]), speed['0'])
    if 'misc' in obtained_signals:
        for i in obtained_signals['misc']:
            misc_control(tello, i, battery)
    if 'navigation' in obtained_signals:
        if obtained_signals['navigation']:
            try:
                data0 = obtained_signals['navigation'][0] * 10
            except Exception as e:
                data0 = 0
            try:
                data1 = obtained_signals['navigation'][1] * 10
            except Exception as e:
                data1 = 0
            try:
                data2 = obtained_signals['navigation'][2] * 10
            except Exception as e:
                data2 = 0
            try:
                speed = obtained_signals['speed'][0] * 10
            except Exception as e:
                speed = 0
            navigate_to_xyz(tello, data0, data1, data2, speed)


if __name__ == '__main__':
    runtime_data = dict()
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

    # # # # # # # # # # # # Variables/Dictionaries section # # # # # # # # # # # # # # # - - - -
    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - #
    msg_counter = 0
    flag_counter = 0
    checkpoint_total = 5
    flying_flag = False
    rgb = dict()
    rgb['camera'] = dict()
    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    # - - - # Initializer section
    tello = Tello()
    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    # - - - #
    print("Connecting with Tello drone...")
    tello.connect()
    print("Connected with Tello drone.")
    start_camera(tello)

    # overwrite manual
    threading.Thread(target=retina.vision_progress, args=(default_capabilities, feagi_opu_channel, api_address, feagi_settings, camera_data['vision'],), daemon=True).start()

    while True:
        try:
            message_from_feagi = pns.message_from_feagi
            if message_from_feagi:
                obtained_signals = pns.obtain_opu_data(message_from_feagi)
                action(obtained_signals)

            # Gather all data from the robot to prepare for FEAGI
            data = tello.get_current_state()
            gyro = get_gyro(data)
            acc = get_accelerator(data)
            sonar = get_ultrasonic(data)
            bat = get_battery(data)
            battery = bat['battery_charge_level']
            raw_frame = full_frame(tello)
            camera_data['vision'] = raw_frame
            default_capabilities['camera']['blink'] = []
            if len(default_capabilities['camera']['blink']) > 0:
                raw_frame = default_capabilities['camera']['blink']
            # Post image into vision
            previous_frame_data, rgb, default_capabilities = retina.process_visual_stimuli(
                raw_frame,
                default_capabilities,
                previous_frame_data,
                rgb, capabilities)

            # INSERT SENSORS INTO the FEAGI DATA SECTION BEGIN
            message_to_feagi = pns.generate_feagi_data(rgb, msg_counter, datetime.now(),
                                                       message_to_feagi)
            # Add gyro data into feagi data
            message_to_feagi = sensors.add_gyro_to_feagi_data(gyro, message_to_feagi)
            # Add battery data into feagi data
            message_to_feagi = sensors.add_battery_to_feagi_data(battery, message_to_feagi)
            # Add accelerator data into feagi data
            message_to_feagi = sensors.add_acc_to_feagi_data(acc, message_to_feagi)
            # Add sonar data into feagi data. Leveraging the same process as ultrasonic.
            message_to_feagi = sensors.add_ultrasonic_to_feagi_data(sonar, message_to_feagi)

            # Sending data to FEAGI
            pns.signals_to_feagi(message_to_feagi, feagi_ipu_channel, agent_settings, feagi_settings)
            message_to_feagi.clear()
            time.sleep(feagi_settings['feagi_burst_speed'])
        except KeyboardInterrupt as ke:
            print("ERROR: ", ke)
            tello.end()
            break
