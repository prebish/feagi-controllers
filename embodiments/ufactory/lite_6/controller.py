import threading
import traceback
from time import sleep
from collections import deque
from version import __version__
from feagi_connector import router
from feagi_connector import sensors
from feagi_connector import actuators
from feagi_connector import pns_gateway as pns
from feagi_connector import feagi_interface as FEAGI

from xArm.xarm.wrapper import XArmAPI


def pose_to_default(arm, count):
    for number_id in range(count):
        runtime_data['servo_status'][number_id] = 0
    arm.reset(wait=True)


def updating_encoder_position_in_bg(arm):
    global runtime_data, capabilities, feagi_settings
    # for i in range(capabilities['servo']['count']):
    #     runtime_data['i_spos'][i] = 0
    while True:
        new_degree_list_of_servo = arm.get_servo_angle()
        for number_of_servo in range(capabilities['servo']['count']):
            name = pns.fetch_servo_position_size_and_return_percentage(capabilities,
                                                                       new_degree_list_of_servo[
                                                                           1][number_of_servo],
                                                                       number_of_servo,
                                                                       flip=True)
            if name is not None:
                runtime_data['i_spos'][name] = 100
        sleep(0.01)


def move(encoder_id, power):
    max_range = capabilities['servo']['servo_range'][str(encoder_id)][1]
    min_range = capabilities['servo']['servo_range'][str(encoder_id)][0]
    pre_power = runtime_data['servo_status'][encoder_id] + power
    if max_range >= pre_power >= min_range:
        runtime_data['servo_status'][encoder_id] = pre_power


def move_encoder(encoder_id, degree):
    max_range = capabilities['servo']['servo_range'][str(encoder_id)][1]
    min_range = capabilities['servo']['servo_range'][str(encoder_id)][0]
    if max_range >= degree >= min_range:
        runtime_data['servo_status'][encoder_id] = degree


def action(obtained_data, arm, speed):
    if "misc" in obtained_data:
        if obtained_data["misc"]:
            for i in obtained_data["misc"]:
                if i == 0:
                    arm.set_vacuum_gripper(on=True)
                if i == 1:
                    arm.set_vacuum_gripper(on=False)
                if i==2:
                    arm.open_lite6_gripper()
                if i==3:
                    arm.close_lite6_gripper()
    if 'servo_position' in obtained_data:
        try:
            if obtained_data['servo_position']:
                for data_point in obtained_data['servo_position']:
                    device_id = data_point
                    encoder_position = actuators.get_position_data(
                        obtained_data['servo_position'][data_point], capabilities, device_id)
                    move_encoder(device_id, encoder_position)
                angle_list = []
                for i in runtime_data['servo_status']:
                    angle_list.append(runtime_data['servo_status'][i])
                angle_list.append(0)  # last one doesn't do anything so just add 0
                arm.set_servo_angle(angle=angle_list, speed=speed)
        except Exception as e:
            print("ERROR: ", e)
            traceback.print_exc()

    if 'servo' in obtained_data:
        try:
            if obtained_data['servo']:
                for data_point in obtained_data['servo']:
                    new_position = obtained_data['servo'][data_point]
                    if data_point % 2 != 0:
                        new_position *= -1
                    device_id = (data_point // 2)
                    power = new_position / 10
                    move(device_id, power)
                angle_list = []
                for i in runtime_data['servo_status']:
                    angle_list.append(runtime_data['servo_status'][i])
                angle_list.append(0)  # last one doesn't do anything so just add 0
                arm.set_servo_angle(angle=angle_list, speed=speed)
        except Exception as e:
            print("ERROR: ", e)
            traceback.print_exc()
    return speed


if __name__ == "__main__":
    runtime_data = \
        {
            "current_burst_id": 0,
            "feagi_state": None,
            "cortical_list": (),
            "battery_charge_level": 1,
            "host_network": {},
            'servo_status': {},
            'i_spos': {}
        }

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

    # UFACTORY SETTING
    ip = "192.168.1.156"
    arm = XArmAPI(ip)  # we don't need that complicated above.
    arm.motion_enable(enable=True)
    arm.set_mode(0)
    arm.set_state(state=0)
    speed = 100
    # UFACTORY ENDS
    pose_to_default(arm, capabilities['servo']['count'])

    threading.Thread(target=updating_encoder_position_in_bg, args=(arm,), daemon=True).start()
    while True:
        try:
            message_from_feagi = pns.message_from_feagi
            if message_from_feagi:
                pns.check_genome_status_no_vision(message_from_feagi)
                obtained_signals = pns.obtain_opu_data(message_from_feagi)
                speed = action(obtained_signals, arm, speed)

            # Encoder position
            encoder_for_feagi = dict()
            try:
                if runtime_data['i_spos']:
                    message_to_feagi = sensors.add_generic_input_to_feagi_data(runtime_data,
                                                                               message_to_feagi)  # message_to_feagi =
                runtime_data['i_spos'].clear()
            except Exception as e:
                print("error: ", e)

            sleep(feagi_settings['feagi_burst_speed'])
            pns.signals_to_feagi(message_to_feagi, feagi_ipu_channel, agent_settings,
                                 feagi_settings)
            message_to_feagi.clear()
        except KeyboardInterrupt as ke:  # Keyboard error
            arm.disconnect()
            break
        except Exception as e:
            arm.disconnect()
            print("ERROR! : ", e)
            traceback.print_exc()
            break
