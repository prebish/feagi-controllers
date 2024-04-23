import threading
import traceback
from time import sleep
from collections import deque
from version import __version__
from feagi_connector import router
from feagi_connector import sensors
from pymycobot.mycobot import MyCobot
from feagi_connector import pns_gateway as pns
from feagi_connector import feagi_interface as FEAGI


previous_data_frame = dict()


class Arm:
    @staticmethod
    def connection_initialize(port='/dev/ttyUSB0'):
        """
        :param port: The default would be '/dev/ttyUSB0'. If the port is different, put a different port.
        :return:
        """
        return MyCobot(port, 115200)

    @staticmethod
    def pose_to_default(arm, count):
        for number_id in range(1, count, 1):
            if number_id != 2:
                runtime_data['servo_status'][number_id] = 2048
                arm.set_encoder(number_id, 2048)

    @staticmethod
    def power_convert(encoder_id, power):
        if encoder_id % 2 == 0:
            return -1 * power
        else:
            return abs(power)

    @staticmethod
    def encoder_converter(encoder_id):
        """
        This will convert from godot to motor's id. Let's say, you have 8x10 (width x depth from static_genome).
        So, you click 4 to go forward. It will be like this:
        o__mot': {'1-0-9': 1, '5-0-9': 1, '3-0-9': 1, '7-0-9': 1}
        which is 1,3,5,7. So this code will convert from 1,3,5,7 to 0,1,2,3 on motor id.
        Since 0-1 is motor 1, 2-3 is motor 2 and so on. In this case, 0 is for forward and 1 is for backward.
        """
        if encoder_id <= 1:
            return 1
        elif encoder_id <= 3:
            return 2
        elif encoder_id <= 5:
            return 3
        elif encoder_id <= 7:
            return 4
        elif encoder_id <= 9:
            return 5
        elif encoder_id <= 11:
            return 6
        else:
            print("Input has been refused. Please put encoder ID.")


def updating_encoder_position_in_bg():
    global runtime_data, capabilities, feagi_settings
    for i in range(1, capabilities['servo']['count'], 1):
        runtime_data['actual_encoder_position'][i] = deque([0, 0, 0, 0, 0])
    while True:
        for i in range(1, capabilities['servo']['count'], 1):
            if i != 2:
                new_data = arm.get_encoder(i)
                if new_data != -1:
                    if runtime_data['actual_encoder_position'][i]:
                        runtime_data['actual_encoder_position'][i].append(new_data)
                        runtime_data['actual_encoder_position'][i].popleft()
        sleep(0.01)


def move(arm, encoder_id, power):
    max_range = capabilities['servo']['servo_range'][str(encoder_id)][1]
    min_range = capabilities['servo']['servo_range'][str(encoder_id)][0]
    pre_power = runtime_data['servo_status'][encoder_id] + power
    if max_range >= pre_power >= min_range:
        arm.set_encoder(encoder_id, pre_power)
        runtime_data['servo_status'][encoder_id] = pre_power


def move_encoder(arm, encoder_id, degree):
    max_range = capabilities['servo']['servo_range'][str(encoder_id)][1]
    min_range = capabilities['servo']['servo_range'][str(encoder_id)][0]
    if max_range >= degree >= min_range:
        arm.set_encoder(encoder_id, degree)
        runtime_data['servo_status'][encoder_id] = degree


def action(obtained_data, arm):
    if 'servo_position' in obtained_data:
        try:
            if obtained_data['servo_position']:
                for data_point in obtained_data['servo_position']:
                    device_id = data_point + 1
                    encoder_position = (obtained_data['servo_position'][data_point] / 20) * ((capabilities['servo']['servo_range'][str(device_id)][1] -capabilities['servo']['servo_range'][str(device_id)][0])+capabilities['servo']['servo_range'][str(device_id)][0])
                    move_encoder(arm, device_id, encoder_position)
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
                    device_id = (data_point // 2) + 1
                    power = new_position
                    move(arm, device_id, power)
        except Exception as e:
            print("ERROR: ", e)
            traceback.print_exc()


if __name__ == "__main__":
    runtime_data = \
        {
            "current_burst_id": 0,
            "feagi_state": None,
            "cortical_list": (),
            "battery_charge_level": 1,
            "host_network": {},
            'servo_status': {},
            'actual_encoder_position': {}
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


    # MYCOBOT SECTION
    mycobot = Arm()
    arm = mycobot.connection_initialize()
    arm.set_speed(100)
    mycobot.pose_to_default(arm, capabilities['servo']['count'])
    arm.release_servo(1)
    threading.Thread(target=updating_encoder_position_in_bg, daemon=True).start()

    while True:
        try:
            message_from_feagi = pns.message_from_feagi
            if message_from_feagi:
                pns.check_genome_status_no_vision(message_from_feagi)
                obtained_signals = pns.obtain_opu_data(message_from_feagi)
                action(obtained_signals, arm)

            # Encoder position
            encoder_for_feagi = dict()
            try:
                for encoder_data in runtime_data['actual_encoder_position']:
                    encoder_for_feagi[encoder_data] = runtime_data['actual_encoder_position'][encoder_data][4]
                message_to_feagi = sensors.add_encoder_to_feagi_data(encoder_for_feagi,message_to_feagi)
            except Exception as e:
                print("error: ", e)

            sleep(feagi_settings['feagi_burst_speed'])
            pns.signals_to_feagi(message_to_feagi, feagi_ipu_channel, agent_settings, feagi_settings)
            message_to_feagi.clear()
        except KeyboardInterrupt as ke:  # Keyboard error
            arm.release_all_servos()
            break
        except Exception as e:
            arm.release_all_servos()
            print("ERROR! : ", e)
            traceback.print_exc()
            break
