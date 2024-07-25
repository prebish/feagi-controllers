#!/usr/bin/env python3
import asyncio
from time import sleep
from collections import deque
from feagi_connector import pns_gateway as pns
from feagi_connector import feagi_interface as feagi

def window_average(sequence):
    return sum(sequence) // len(sequence)


def obtain_opu_data(device_list, message_from_feagi):
    opu_signal_dict = {}
    opu_data = feagi.opu_processor(message_from_feagi)
    for i in device_list:
        if i in opu_data and opu_data[i]:
            for x in opu_data[i]:
                if i not in opu_signal_dict:
                    opu_signal_dict[i] = {}
                opu_signal_dict[i][x] = opu_data[i][x]
    return opu_signal_dict


def motor_generate_power(power_maximum, feagi_power, id):
    z_depth = pns.full_list_dimension['o__mot']['cortical_dimensions'][2]
    if z_depth == 1:
        return power_maximum * (feagi_power / 100)
    else:
        return (feagi_power / (z_depth - 1)) * power_maximum


def servo_generate_power(power, feagi_power, id):
    z_depth = pns.full_list_dimension['o__ser']['cortical_dimensions'][2]
    if z_depth == 1:
        return power * (feagi_power / 100)
    else:
        return (feagi_power / (z_depth - 1)) * power


def servo_negative_or_positive(id, power):
    if id % 2 == 0:
        power = abs(power)
    else:
        power = -1 * power
    return power


def feagi_id_converter(id):
    """
    This function converts motor IDs from 1,3,5,7 to 0,1,2,3.
    so, if you select 0 and 1, it will end up 0. if you select 2 and 3, it will end up 1.
    """
    if id % 2 == 0:
        return id // 2
    else:
        return (id - 1) // 2


def power_convert(motor_id, power):
    if motor_id % 2 == 0:
        return abs(power)
    else:
        return -1 * power

# Motor section

def get_motor_data(obtained_data, moving_average):
    motor_data = dict()
    processed_obtained_data = dict()
    if 'motor' in obtained_data:
        if obtained_data['motor'] is not {}:
            sorted_keys = sorted(obtained_data['motor'].keys(), key=int)
            for data_point in sorted_keys:
                device_power = power_convert(data_point, obtained_data['motor'][data_point])
                device_id = feagi_id_converter(data_point)
                if device_id in motor_data:
                    motor_data[device_id] = motor_data[device_id] - obtained_data['motor'][data_point]
                else:
                    motor_data[device_id] = device_power
    return motor_data

def pass_the_power_to_motor(power_maximum, device_power, device_id, moving_average_len):
    device_power = int(motor_generate_power(power_maximum, device_power, device_id))
    if device_id in moving_average_len:
        moving_average_len = update_moving_average(moving_average_len, device_id, device_power)
    moving_average_len[device_id].append(window_average(moving_average_len[device_id]))
    moving_average_len[device_id].popleft()

def rolling_window_update(stored_rolling_window_dict):
    for _ in stored_rolling_window_dict:
        stored_rolling_window_dict[_].append(0)
        stored_rolling_window_dict[_].popleft()
    return stored_rolling_window_dict


def create_motor_rolling_window_len(length_window=0, current_rolling_window_dict={}, motor_id='0'):
    rolling_window_len = length_window
    rolling_window = current_rolling_window_dict.copy()
    motor_id = int(motor_id)
    if motor_id in rolling_window:
        rolling_window[motor_id].update([0] * (rolling_window_len))
    else:
        rolling_window[motor_id] = deque([0] * rolling_window_len)
    return rolling_window

def update_moving_average(moving_average, device_id, device_power):
    moving_average[device_id].append(device_power)
    moving_average[device_id].popleft()
    return moving_average

# Motor section ends


def get_servo_data(obtained_data, converter_id=False):
    servo_data = dict()
    if 'servo' in obtained_data:
        for data_point in obtained_data['servo']:
            device_power = servo_negative_or_positive(data_point, obtained_data['servo'][
                data_point])
            if converter_id:
                device_id = feagi_id_converter(data_point)
            else:
                device_id = data_point
            if device_id in servo_data:
                servo_data[device_id] += device_power
            else:
                servo_data[device_id] = device_power
    return servo_data

def get_servo_position_data(obtained_data):
    servo_position_data = dict()
    if 'servo_position' in obtained_data:
        for data_point in obtained_data['servo_position']:
            device_power = obtained_data['servo_position'][data_point]
            device_id = data_point
            if device_id in servo_position_data:
                servo_position_data[device_id] += device_power
            else:
                servo_position_data[device_id] = device_power
    return servo_position_data


def check_emergency_stop(obtained_data):
    emergency_data = dict()  # I don't think there's any required input on emergency stop at all
    if 'emergency' in obtained_data:
        for data_point in obtained_data['emergency']:
            device_id = data_point
            device_power = obtained_data['emergency'][data_point]
            emergency_data[device_id] = device_power
    return emergency_data


def check_new_speed(obtained_data):
    speed_data = dict()  # I don't think there's any required input on emergency stop at all
    if 'speed' in obtained_data:
        for data_point in obtained_data['speed']:
            device_id = data_point
            device_power = obtained_data['speed'][data_point]
            speed_data[device_id] = device_power
    return speed_data


def get_motion_control_data(obtained_data):
    motion_data = dict()
    if 'motion_control' in obtained_data:
        for data_point in obtained_data['motion_control']:
            device_id = data_point
            device_power = obtained_data['motion_control'][data_point]
            motion_data[device_id] = device_power
    return motion_data


def get_led_data(obtained_data):
    led_data = dict()
    if 'led' in obtained_data:
        for data_point in obtained_data['led']:
            led_data[data_point] = obtained_data['led'][data_point]
    return led_data


def servo_keep_boundaries(current_position, max=180, min=0):
    """
    Prevent Servo position to go beyond range
    """
    if current_position > max:
        adjusted_position = float(max)
    elif current_position < min:
        adjusted_position = float(min)
    else:
        adjusted_position = float(current_position)
    return adjusted_position


def get_gpio_data(obtained_data):
    gpio_data = dict()
    if 'gpio' in obtained_data:
        for data_point in obtained_data['gpio']:
            gpio_data[data_point] = obtained_data['gpio'][data_point]
    return gpio_data


def check_convert_gpio_to_input(obtained_data):
    input_gpio_data = dict()
    if 'gpio_input' in obtained_data:
        for data_point in obtained_data['gpio_input']:
            input_gpio_data[data_point] = obtained_data['gpio_input'][data_point]
    return input_gpio_data


def get_position_data(power, min_output, max_output):
    max_input = pns.full_list_dimension['o_spos']['cortical_dimensions'][2]
    return (power / max_input) * (max_output - min_output) + min_output
