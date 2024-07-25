import os
import json
import socket
import argparse
import requests
import threading
import traceback
from time import sleep
from feagi_connector import router
from feagi_connector import pns_gateway as pns
from feagi_connector.version import __version__



def pub_initializer(ipu_address, bind=True):
    return router.Pub(address=ipu_address, bind=bind)


def sub_initializer(opu_address, flags=router.zmq.NOBLOCK):
    return router.Sub(address=opu_address, flags=flags)


def feagi_registration(feagi_auth_url, feagi_settings, agent_settings, capabilities,
                       controller_version):
    host_info = router.app_host_info()
    runtime_data = {
        "host_network": {},
        "feagi_state": None
    }
    runtime_data["host_network"]["host_name"] = host_info["host_name"]
    runtime_data["host_network"]["ip_address"] = host_info["ip_address"]
    agent_settings['agent_ip'] = host_info["ip_address"]

    while runtime_data["feagi_state"] is None:
        print("\nAwaiting registration with FEAGI...")
        try:
            runtime_data["feagi_state"] = \
                router.register_with_feagi(feagi_auth_url, feagi_settings, agent_settings,
                                           capabilities, controller_version, __version__)
        except Exception as e:
            print("ERROR__: ", e, traceback.print_exc())
            pass
        sleep(1)
    print("\nversion: ", controller_version, "\n")
    print("\nagent version: ", __version__, "\n")
    return runtime_data["feagi_state"]


def block_to_array(block_ref):
    block_id_str = block_ref.split('-')
    array = [int(x) for x in block_id_str]
    return array


def is_FEAGI_reachable(server_host, server_port):
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(3)
        sock.connect((server_host, server_port))
        return True
    except Exception as e:
        return False


def feagi_setting_for_registration(feagi_settings, agent_settings):
    """
    Generate all needed information and return the full data to make it easier to connect with
    FEAGI
    """
    feagi_ip_host = feagi_settings["feagi_host"]
    api_port = feagi_settings["feagi_api_port"]
    app_data_port = agent_settings["agent_data_port"]
    return feagi_ip_host, api_port, app_data_port


def feagi_api_burst_engine():
    return '/v1/burst_engine/stimulation_period'


def feagi_api_burst_counter():
    return '/v1/burst_engine/burst_counter'


def feagi_inbound(feagi_inbound_port):
    """
    Return the zmq address of inbound
    """
    return 'tcp://0.0.0.0:' + feagi_inbound_port


def feagi_outbound(feagi_ip_host, feagi_opu_port):
    """
    Return the zmq address of outbound
    """
    return 'tcp://' + feagi_ip_host + ':' + \
           feagi_opu_port


def msg_processor(self, msg, msg_type, capabilities):
    # TODO: give each subclass a specific msg processor method?
    # TODO: add an attribute that explicitly defines message type (instead of parsing topic name)?
    if 'ultrasonic' in msg_type and msg.ranges[1]:
        return {
            msg_type: {
                idx: val for idx, val in enumerate([msg.ranges[1]])
            }
        }
    elif 'IR' in msg_type:
        rgb_vals = list(msg.data)
        avg_intensity = sum(rgb_vals) // len(rgb_vals)

        sensor_topic = msg_type.split('/')[0]
        sensor_id = int(''.join(filter(str.isdigit, sensor_topic)))

        # print("\n***\nAverage Intensity = ", avg_intensity)
        if avg_intensity > capabilities["infrared"]["threshold"]:
            return {
                'ir': {
                    sensor_id: False
                }
            }
        else:
            return {
                'ir': {
                    sensor_id: True
                }
            }


def compose_message_to_feagi(original_message, data=None, battery=0):
    """
    accumulates multiple messages in a data structure that can be sent to feagi
    """
    if data is None:
        data = {}
    runtime_data = dict()
    runtime_data["battery_charge_level"] = battery
    message_to_feagi = data
    if "data" not in message_to_feagi:
        message_to_feagi["data"] = dict()
    if "sensory_data" not in message_to_feagi["data"]:
        message_to_feagi["data"]["sensory_data"] = dict()
    if original_message is not None:
        for sensor in original_message:
            if sensor not in message_to_feagi["data"]["sensory_data"]:
                message_to_feagi["data"]["sensory_data"][sensor] = dict()
            for sensor_data in original_message[sensor]:
                if sensor_data not in message_to_feagi["data"]["sensory_data"][sensor]:
                    message_to_feagi["data"]["sensory_data"][sensor][sensor_data] = \
                        original_message[sensor][
                            sensor_data]
        message_to_feagi["data"]["sensory_data"]["battery"] = {
            1: runtime_data["battery_charge_level"] / 100}
    return message_to_feagi, runtime_data["battery_charge_level"]


def opu_processor(data):
    try:
        processed_opu_data = {'motor': {}, 'servo': {}, 'battery': {},
                              'discharged_battery': {}, 'reset': {}, 'camera': {}, 'misc': {},
                              "motion_control": {}, 'navigation': {}, 'speed': {}, "led": {},
                              "vision_resolution": {}, "vision_acuity": {}, 'servo_position': {},
                              "emergency": {}, "gpio": {}, "gpio_input": {}}
        opu_data = data["opu_data"]
        if opu_data is not None:
            if "o__mot" in pns.full_list_dimension:
                if pns.full_list_dimension['o__mot']['cortical_dimensions'][2] == 1:
                    if 'o__mot' in opu_data:  # motor percentage
                        for data_point in opu_data['o__mot']:
                            processed_data_point = block_to_array(data_point)
                            device_id = processed_data_point[0]
                            device_power = opu_data['o__mot'][data_point]
                            processed_opu_data['motor'][device_id] = device_power
                else:
                    if 'o__mot' in opu_data:  # motor position
                        if opu_data['o__mot']:
                            for data_point in opu_data['o__mot']:
                                processed_data_point = block_to_array(data_point)
                                device_id = processed_data_point[0]
                                device_power = processed_data_point[2]
                                processed_opu_data['motor'][device_id] = device_power
            if "o__ser" in pns.full_list_dimension:
                if pns.full_list_dimension['o__ser']['cortical_dimensions'][2] == 1:
                    if 'o__ser' in opu_data:
                        if opu_data['o__ser']:
                            for data_point in opu_data['o__ser']:
                                processed_data_point = block_to_array(data_point)
                                device_id = processed_data_point[0]
                                device_power = opu_data['o__ser'][data_point]
                                processed_opu_data['servo'][device_id] = device_power
                else:
                    if 'o__ser' in opu_data:
                        if opu_data['o__ser']:
                            for data_point in opu_data['o__ser']:
                                processed_data_point = block_to_array(data_point)
                                device_id = processed_data_point[0]
                                device_power = processed_data_point[2]
                                processed_opu_data['servo'][device_id] = device_power
            if 'o_cbat' in opu_data:
                if opu_data['o__bat']:
                    for data_point in opu_data['o_cbat']:
                        intensity = data_point[2]
                        processed_opu_data['battery'] = intensity
            if 'o_stop' in opu_data:
                if opu_data['o_stop']:
                    for data_point in opu_data['o_stop']:
                        processed_data_point = block_to_array(data_point)
                        device_id = processed_data_point[0]
                        device_power = opu_data['o_stop'][data_point]
                        processed_opu_data['emergency'][device_id] = device_power
            if 'o_dbat' in opu_data:
                if opu_data['o__bat']:
                    for data_point in opu_data['o_dbat']:
                        intensity = data_point
                        processed_opu_data['battery'] = intensity
            if 'o_init' in opu_data:
                if opu_data['o_init']:
                    for data_point in opu_data['o_init']:
                        processed_data_point = block_to_array(data_point)
                        device_id = processed_data_point[0]
                        device_power = opu_data['o_init'][data_point]
                        processed_opu_data['reset'][device_id] = device_power
            if 'o_misc' in opu_data:
                if opu_data['o_misc']:
                    for data_point in opu_data['o_misc']:
                        processed_data_point = block_to_array(data_point)
                        device_id = processed_data_point[0]
                        device_power = opu_data['o_misc'][data_point]
                        processed_opu_data['misc'][device_id] = device_power
            if "o_mctl" in pns.full_list_dimension:
                if pns.full_list_dimension['o_mctl']['cortical_dimensions'][2] == 1:
                    if 'o_mctl' in opu_data:
                        for data_point in opu_data['o_mctl']:
                            processed_data_point = block_to_array(data_point)
                            device_power = opu_data['o_mctl'][data_point] / 100.0
                            device_id = build_up_from_mctl(processed_data_point)
                            processed_opu_data['motion_control'][device_id] = device_power
                else:
                    if 'o_mctl' in opu_data:
                        if opu_data['o_mctl']:
                            for data_point in opu_data['o_mctl']:
                                processed_data_point = block_to_array(data_point)
                                device_power = processed_data_point[2] / \
                                               float(pns.full_list_dimension['o_mctl']['cortical_dimensions'][2])
                                device_id = build_up_from_mctl(processed_data_point)
                                processed_opu_data['motion_control'][device_id] = device_power
            if 'o__led' in opu_data:
                if opu_data['o__led']:
                    for data_point in opu_data['o__led']:
                        processed_data_point = block_to_array(data_point)
                        device_id = processed_data_point[0]
                        device_power = opu_data['o__led'][data_point]
                        processed_opu_data['led'][device_id] = device_power
            if 'o__nav' in opu_data:
                if opu_data['o__nav']:
                    for data_point in opu_data['o__nav']:
                        data_point = block_to_array(data_point)
                        device_id = data_point[0]
                        device_power = data_point[2]
                        device_power = device_power - 10
                        processed_opu_data['navigation'][device_id] = device_power
            if 'o__spd' in opu_data:
                if opu_data['o__spd']:
                    for data_point in opu_data['o__spd']:
                        data_point = block_to_array(data_point)
                        device_id = data_point[0]
                        device_power = data_point[2]
                        processed_opu_data['speed'][device_id] = device_power
            if 'o_vres' in opu_data:
                if opu_data['o_vres']:
                    for data_point in opu_data['o_vres']:
                        data_point = block_to_array(data_point)
                        device_id = data_point[0]
                        device_power = data_point[2]
                        processed_opu_data['vision_resolution'][device_id] = device_power
            if 'o_spos' in opu_data:  # Currently used in mycobot only. Different kind of position
                if opu_data['o_spos']:
                    for data_point in opu_data['o_spos']:
                        processed_data_point = block_to_array(data_point)
                        device_id = processed_data_point[0]
                        device_power = processed_data_point[2]
                        processed_opu_data['servo_position'][device_id] = device_power
            if 'o_vact' in opu_data:
                if opu_data['o_vact']:
                    for data_point in opu_data['o_vact']:
                        data_point = block_to_array(data_point)
                        device_id = data_point[0]
                        device_power = data_point[2]
                        processed_opu_data['vision_acuity'][device_id] = device_power
            if 'odgpio' in opu_data:
                if opu_data['odgpio']:
                    for data_point in opu_data['odgpio']:
                        processed_data_point = block_to_array(data_point)
                        device_id = processed_data_point[0]
                        device_power = opu_data['odgpio'][data_point]
                        processed_opu_data['gpio'][device_id] = device_power
            if 'oigpio' in opu_data:
                if opu_data['oigpio']:
                    for data_point in opu_data['oigpio']:
                        processed_data_point = block_to_array(data_point)
                        device_id = processed_data_point[0]
                        device_power = opu_data['oigpio'][data_point]
                        processed_opu_data['gpio_input'][device_id] = device_power
            return processed_opu_data
    except Exception as error:
        print("error: ", error)
        traceback.print_exc()
        # pass


def control_data_processor(data):
    control_data = data['control_data']
    if control_data is not None:
        if 'motor_power_coefficient' in control_data:
            configuration.capabilities["motor"]["power_coefficient"] = float(
                control_data['motor_power_coefficient'])
        if 'robot_starting_position' in control_data:
            for position_index in control_data['robot_starting_position']:
                configuration.capabilities["position"][position_index]["x"] = \
                    float(control_data['robot_starting_position'][position_index][0])
                configuration.capabilities["position"][position_index]["y"] = \
                    float(control_data['robot_starting_position'][position_index][1])
                configuration.capabilities["position"][position_index]["z"] = \
                    float(control_data['robot_starting_position'][position_index][2])
        return configuration.capabilities["motor"]["power_coefficient"], \
               configuration.capabilities["position"]


def connect_to_feagi(feagi_settings, runtime_data, agent_settings, capabilities, current_version,
                     bind_flag=False):
    print("Connecting to FEAGI resources...")
    feagi_auth_url = feagi_settings.pop('feagi_auth_url', None)
    runtime_data["feagi_state"] = feagi_registration(feagi_auth_url=feagi_auth_url,
                                                     feagi_settings=feagi_settings,
                                                     agent_settings=agent_settings,
                                                     capabilities=capabilities,
                                                     controller_version=current_version)
    api_address = runtime_data['feagi_state']["feagi_url"]
    router.global_api_address = api_address
    agent_data_port = str(runtime_data["feagi_state"]['agent_state']['agent_data_port'])
    print("** **", runtime_data["feagi_state"])
    feagi_settings['feagi_burst_speed'] = float(runtime_data["feagi_state"]['burst_duration'])
    if 'magic_link' not in feagi_settings:
        if bind_flag:
            ipu_channel_address = "tcp://*:" + agent_data_port  # This is for godot to work due to
            # bind unable to use the dns.
        else:
            ipu_channel_address = feagi_outbound(feagi_settings['feagi_host'], agent_data_port)

        print("IPU_channel_address=", ipu_channel_address)
        opu_channel_address = feagi_outbound(feagi_settings['feagi_host'],
                                             runtime_data["feagi_state"]['feagi_opu_port'])

        # ip = '172.28.0.2'
        # opu_channel_address = 'tcp://' + str(ip) + ':3000'
        # ipu_channel_address = 'tcp://' + str(ip) + ':3000'
        feagi_ipu_channel = pub_initializer(ipu_channel_address, bind=bind_flag)
        feagi_opu_channel = sub_initializer(opu_address=opu_channel_address)
        router.global_feagi_opu_channel = feagi_opu_channel
        threading.Thread(target=pns.feagi_listener, args=(feagi_opu_channel,), daemon=True).start()
    else:
        feagi_ipu_channel = None
        feagi_opu_channel = None
        print("websocket testing")
        websocket_url = feagi_settings['feagi_dns'].replace("https", "wss") + str("/p9053")
        print(websocket_url)
        router.websocket_client_initalize('192.168.50.192', '9053', dns=websocket_url)
        threading.Thread(target=router.websocket_recieve, daemon=True).start()


    return feagi_settings, runtime_data, api_address, feagi_ipu_channel, feagi_opu_channel


def build_up_from_mctl(id):
    action_map = {
        (0, 0): "move_left",
        (0, 1): "roll_left",
        (0, 2): "yaw_left",
        (1, 0): "move_up",
        (1, 1): "pitch_forward",
        (2, 0): "move_down",
        (2, 1): "pitch_back",
        (3, 0): "move_right",
        (3, 1): "roll_right",
        (3, 2): "yaw_right"
    }

    # Get the action from the dictionary, return None if not found
    return action_map.get((id[0], id[1]))


def configuration_load(path='./'):
    # NEW JSON UPDATE
    f = open(path + 'configuration.json')
    configuration = json.load(f)
    feagi_settings = configuration["feagi_settings"]
    agent_settings = configuration['agent_settings']
    capabilities = configuration['capabilities']
    feagi_settings['feagi_host'] = os.environ.get('FEAGI_HOST_INTERNAL', "127.0.0.1")
    feagi_settings['feagi_api_port'] = os.environ.get('FEAGI_API_PORT', "8000")
    message_to_feagi = {"data": {}}
    if 'description' in configuration:
        pns.ver = configuration['description']
    f.close()
    return feagi_settings, agent_settings, capabilities, message_to_feagi, configuration
    # END JSON UPDATE

def reading_parameters_to_confirm_communication(feagi_settings, configuration, path="."):
    # Check if feagi_connector has arg
    parser = argparse.ArgumentParser(description='enable to use magic link')
    parser.add_argument('-magic_link', '--magic_link', help='to use magic link', required=False)
    parser.add_argument('-magic-link', '--magic-link', help='to use magic link', required=False)
    parser.add_argument('-magic', '--magic', help='to use magic link', required=False)
    parser.add_argument('-ip', '--ip', help='to use feagi_ip', required=False)
    parser.add_argument('-port', '--port', help='to use feagi_port', required=False)
    args = vars(parser.parse_args())
    if args['port']:
        feagi_settings['feagi_opu_port'] = args['port']
    else:
        feagi_settings['feagi_opu_port'] = os.environ.get('FEAGI_OPU_PORT', "3000")

    if args['magic'] or args['magic_link']:
        if args['magic'] or args['magic_link']:
            for arg in args:
                if args[arg] is not None:
                    feagi_settings['magic_link'] = args[arg]
                    break
            configuration['feagi_settings']['feagi_url'] = feagi_settings['magic_link']
            with open(path+'configuration.json', 'w') as f:
                json.dump(configuration, f, indent=4)
        else:
            feagi_settings['magic_link'] = feagi_settings['feagi_url']
        url_response = json.loads(requests.get(feagi_settings['magic_link']).text)
        feagi_settings['feagi_dns'] = url_response['feagi_url']
        feagi_settings['feagi_api_port'] = url_response['feagi_api_port']
    elif args['ip']:
        # # FEAGI REACHABLE CHECKER # #
        feagi_flag = False
        print("retrying...")
        print("Waiting on FEAGI...")
        if args['ip']:
            feagi_settings['feagi_host'] = args['ip']
        if 'feagi_url' in configuration['feagi_settings']:
            del configuration['feagi_settings']['feagi_url']
        if 'feagi_dns' in feagi_settings:
            del feagi_settings['feagi_dns']
        if 'magic_link' in feagi_settings:
            del feagi_settings['magic_link']
            with open(path+'configuration.json', 'w') as f:
                json.dump(configuration, f, indent=4)
        while not feagi_flag:
            feagi_flag = is_FEAGI_reachable(os.environ.get('FEAGI_HOST_INTERNAL', feagi_settings["feagi_host"]),
                                            int(os.environ.get('FEAGI_OPU_PORT', feagi_settings['feagi_opu_port'])))
            sleep(2)
    elif feagi_settings['feagi_url']:
        if args['magic'] or args['magic_link']:
            for arg in args:
                if args[arg] is not None:
                    feagi_settings['magic_link'] = args[arg]
                    break
            configuration['feagi_settings']['feagi_url'] = feagi_settings['magic_link']
            with open(path+'configuration.json', 'w') as f:
                json.dump(configuration, f)
        else:
            feagi_settings['magic_link'] = feagi_settings['feagi_url']
        url_response = json.loads(requests.get(feagi_settings['magic_link']).text)
        feagi_settings['feagi_dns'] = url_response['feagi_url']
        feagi_settings['feagi_api_port'] = url_response['feagi_api_port']
    else:
        # # FEAGI REACHABLE CHECKER # #
        feagi_flag = False
        print("retrying...")
        print("Waiting on FEAGI...")
        if args['ip']:
            feagi_settings['feagi_host'] = args['ip']
        while not feagi_flag:
            feagi_flag = is_FEAGI_reachable(os.environ.get('FEAGI_HOST_INTERNAL', feagi_settings["feagi_host"]),int(os.environ.get('FEAGI_OPU_PORT', feagi_settings['feagi_opu_port'])))
            sleep(2)
    return feagi_settings, configuration

def build_up_from_configuration(path="./"):
    feagi_settings, agent_settings, capabilities, message_to_feagi, configuration = configuration_load(path)
    default_capabilities = {}  # It will be generated in process_visual_stimuli. See the
    # overwrite manual
    default_capabilities = pns.create_runtime_default_list(default_capabilities, capabilities)

    feagi_settings, configuration = reading_parameters_to_confirm_communication(feagi_settings, configuration,path)
    return {
        "feagi_settings": feagi_settings,
        "agent_settings": agent_settings,
        "default_capabilities": default_capabilities,
        "message_to_feagi": message_to_feagi,
        "capabilities": capabilities
    }

def map_value(val, min1, max1, min2, max2):
    """ Performs linear transformation to map value from
    range 1 [min1, max1] to a value in range 2 [min2, max2].

    :param val: value (int/float) being mapped
    :param min1: min of range 1
    :param max1: max of range 1
    :param min2: min of range 2
    :param max2: max of range 2
    :return: value mapped from range 1 to range 2
    """
    if val < min1:
        return min2
    if val > max1:
        return max2

    mapped_value = (val - min1) * ((max2 - min2) / (max1 - min1)) + min2

    if max2 >= mapped_value >= min2:
        return mapped_value