import threading
import traceback
from time import sleep
from collections import deque
from version import __version__
from feagi_connector import actuators
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
        for number_id in range(count):
            if not capabilities['output']['servo'][str(number_id)]['disable']:
                arm.set_encoder(number_id, 2048)
                runtime_data['servo_status'][number_id] = 2048
            else:
                runtime_data['servo_status'][number_id] = 'disabled'




def updating_encoder_position_in_bg():
    global runtime_data, capabilities, feagi_settings
    for i in range(len(capabilities['output']['servo'])):
        runtime_data['actual_encoder_position'][i] = deque([0, 0, 0, 0, 0])
        runtime_data['for_feagi_data'][i] = 0
    while True:
        for i in range(len(capabilities['output']['servo'])):
            new_data = arm.get_encoder(i+1)
            if new_data != -1:
                if runtime_data['actual_encoder_position'][i]:
                    runtime_data['actual_encoder_position'][i].append(new_data)
                    runtime_data['actual_encoder_position'][i].popleft()
                    runtime_data['for_feagi_data'][i] = new_data
        sleep(0.01)


def action(obtained_data, arm):
    recieve_servo_data = actuators.get_servo_data(obtained_data)
    recieve_servo_position_data = actuators.get_servo_position_data(obtained_data)

    if recieve_servo_position_data:
        for real_id in recieve_servo_position_data:
            if not capabilities['output']['servo'][str(real_id)]['disable']:
                new_power = actuators.get_position_data(recieve_servo_position_data[real_id], capabilities['output']['servo'][str(real_id)][ 'min_value'], capabilities['output']['servo'][str(real_id)][ 'max_value'])
                arm.set_encoder(real_id + 1, new_power)
                runtime_data['servo_status'][real_id] = new_power


    if recieve_servo_data:
        for real_id in recieve_servo_data:  # example output: {0: 100, 2: 100}
            device_id = actuators.feagi_id_converter(real_id) + 1 # Since mycobot runs 1 to 6 instead of 0 to 5
            converted_id = actuators.feagi_id_converter(real_id)
            if not capabilities['output']['servo'][str(converted_id)]['disable']:
                servo_power = actuators.servo_generate_power(capabilities['output']["servo"][str(converted_id)]["max_power"], recieve_servo_data[real_id], real_id)
                pre_power = runtime_data['servo_status'][converted_id] + servo_power
                new_power = actuators.servo_keep_boundaries(pre_power, capabilities['output']['servo'][str(converted_id)][ 'max_value'], capabilities['output']['servo'][str(converted_id)][ 'min_value'])
                arm.set_encoder(device_id, new_power)
                runtime_data['servo_status'][converted_id] = new_power


if __name__ == "__main__":
    runtime_data = \
        {
            "current_burst_id": 0,
            "feagi_state": None,
            "cortical_list": (),
            "battery_charge_level": 1,
            "host_network": {},
            'servo_status': {},
            'actual_encoder_position': {},
            'for_feagi_data': {}
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
    mycobot.pose_to_default(arm, len(capabilities['output']['servo']))
    arm.release_servo(1)
    threading.Thread(target=updating_encoder_position_in_bg, daemon=True).start()

    while True:
        try:
            message_from_feagi = pns.message_from_feagi
            if message_from_feagi:
                pns.check_genome_status_no_vision(message_from_feagi)
                obtained_signals = pns.obtain_opu_data(message_from_feagi)
                action(obtained_signals, arm)
                # print(runtime_data['servo_status'])


            for device_id in capabilities['input']['servo']:
                if not capabilities['input']['servo'][device_id]['disable']:
                    cortical_id = capabilities['input']['servo'][device_id]["cortical_id"]
                    create_data_list = dict()
                    create_data_list[cortical_id] = dict()
                    capabilities['input']['servo'][device_id]['max_value'], capabilities['input']['servo'][device_id]['min_value'] = sensors.measuring_max_and_min_range(runtime_data['for_feagi_data'][int(device_id)],capabilities['input']['servo'][device_id]['max_value'],capabilities['input']['servo'][device_id]['min_value'])
                    position_in_feagi_location = sensors.convert_sensor_to_ipu_data(capabilities['input']['servo'][device_id]['min_value'], capabilities['input']['servo'][device_id]['max_value'], runtime_data['for_feagi_data'][int(device_id)],  capabilities['input']['servo'][device_id]['feagi_index'],cortical_id=cortical_id, symmetric=True)
                    create_data_list[cortical_id][position_in_feagi_location] = 100
                    if create_data_list[cortical_id]:
                        message_to_feagi = sensors.add_generic_input_to_feagi_data(create_data_list, message_to_feagi)


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
