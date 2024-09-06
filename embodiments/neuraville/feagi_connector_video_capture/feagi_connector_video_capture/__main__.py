#!/usr/bin/env python3
import argparse
import requests
from time import sleep
import feagi_connector_video_capture
from feagi_connector import feagi_interface as feagi

if __name__ == '__main__':
    current_path = feagi_connector_video_capture.__path__
    feagi.validate_requirements(str(current_path[0]) + '/requirements.txt')  # you should get it from the boilerplate generator
    parser = argparse.ArgumentParser(description='configuration for any webcam')
    parser.add_argument('-loop', '--loop', help='Enable loop for the video', required=False)
    parser.add_argument('-ip', '--ip', help='Description for ip address argument', required=False)
    parser.add_argument('-device', '--device', help='To bind the location or index of webcam.',
                        required=False)
    parser.add_argument('-video', '--video', help='Use the path to video to read', required=False)
    parser.add_argument('-image', '--image', help='Use the path to image to read', required=False)
    parser.add_argument('-port', '--port', help='Change the port instead of default 8000.',
                        required=False)
    parser.add_argument('-magic_link', '--magic_link', help='Get the magic link from NRS button',
                        required=False)
    args = vars(parser.parse_args())

    # # Check if feagi_connector has arg
    feagi_settings, agent_settings, capabilities, message_to_feagi, configuration = feagi.configuration_load((str(current_path[0]) + '/'))
    # capabilities = retina.convert_new_json_to_old_json(capabilities)  # temporary
    # feagi_settings = config['feagi_settings'].copy()
    # agent_settings = config['agent_settings'].copy()
    # default_capabilities = config['default_capabilities'].copy()
    # message_to_feagi = config['message_to_feagi'].copy()
    # capabilities = config['capabilities'].copy()
    if args['ip']:
        feagi_settings["feagi_host"] = args['ip']
    if args['loop'] == "true" or args['loop'] == "True":
        capabilities['input']['camera']['0']["video_loop"] = bool(args['loop'])
    if args['device']:
        if args['device'] == "monitor":
            capabilities['input']['camera']['0']["video_device_index"] = "monitor"
        else:
            device_list = args['device'].split(',')
            if len(device_list) > 1:
                capabilities['input']['camera']['0']["video_device_index"] = [int(device) for device in device_list]
            else:
                capabilities['input']['camera']['0']["video_device_index"] = [int(device_list[0])]
    if args['video']:
        capabilities['input']['camera']['0']["video_device_index"] = args['video']
    if args['port']:
        feagi_settings["feagi_api_port"] = args['port']
    if args['image']:
        capabilities['input']['camera']['0']["image"] = args['image']
    if args['magic_link']:
        network_output = requests.get(args['magic_link']).json()
        capabilities['feagi_url'] = network_output['feagi_url']
        capabilities['feagi_api_port'] = network_output['feagi_api_port']
        # capabilities['feagi_opu_port'] = network_output['feagi_opu_port']

    inital_feagi_setting = feagi_settings.copy()
    inital_agent_settings = agent_settings.copy()
    inital_capabilities = capabilities.copy()
    inital_message_to_feagi = message_to_feagi.copy()
    while True:
        try:
            from feagi_connector_video_capture import controller as video_controller

            feagi_auth_url = feagi_settings.pop('feagi_auth_url', None)
            print("FEAGI AUTH URL ------- ", feagi_auth_url)
            video_controller.main(feagi_auth_url, feagi_settings, agent_settings,
                                  capabilities, message_to_feagi)
        except Exception as e:
            feagi_settings = inital_feagi_setting.copy()
            agent_settings = inital_agent_settings.copy()
            capabilities = inital_capabilities.copy()
            message_to_feagi = inital_message_to_feagi.copy()
            print(f"Controller run failed", e)
            # traceback.print_exc()
            sleep(2)
