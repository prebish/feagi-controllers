#!/usr/bin/env python3
import os
import json
import argparse
import requests
import traceback
from time import sleep
import feagi_connector_video_capture

if __name__ == '__main__':
    # NEW JSON UPDATE
    current_path = feagi_connector_video_capture.__path__
    f = open(str(current_path[0]) + '/configuration.json')
    configuration = json.load(f)
    feagi_settings = configuration["feagi_settings"]
    agent_settings = configuration['agent_settings']
    capabilities = configuration['capabilities']
    feagi_settings['feagi_host'] = os.environ.get('FEAGI_HOST_INTERNAL', "127.0.0.1")
    feagi_settings['feagi_api_port'] = os.environ.get('FEAGI_API_PORT', "8000")
    f.close()
    message_to_feagi = {"data": {}}
    # END JSON UPDATE

    # Check if feagi_connector has arg
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
    parser.add_argument('-magic_link', '--magic_link', help='to use magic link', required=False)
    parser.add_argument('-magic-link', '--magic-link', help='to use magic link', required=False)
    parser.add_argument('-magic', '--magic', help='to use magic link', required=False)
    args = vars(parser.parse_args())
    if args['ip']:
        feagi_settings["feagi_host"] = args['ip']
    if args['loop'] == "true" or args['loop'] == "True":
        capabilities["camera"]["video_loop"] = bool(args['loop'])
    if args['device']:
        if args['device'] == "monitor":
            capabilities["camera"]["video_device_index"] = "monitor"
        else:
            capabilities["camera"]["video_device_index"] = int(args['device'])
    else:
        capabilities["camera"]["video_device_index"] = 0
    if args['video']:
        capabilities["camera"]["video_device_index"] = args['video']
    if args['port']:
        feagi_settings["feagi_api_port"] = args['port']
    if args['image']:
        capabilities["camera"]["image"] = args['image']
    if feagi_settings['feagi_url'] or args['magic'] or args['magic_link']:
        if args['magic'] or args['magic_link']:
            for arg in args:
                if args[arg] is not None:
                    feagi_settings['magic_link'] = args[arg]
                    break
            configuration['feagi_settings']['feagi_url'] = feagi_settings['magic_link']
            with open('configuration.json', 'w') as f:
                json.dump(configuration, f)
        else:
            feagi_settings['magic_link'] = feagi_settings['feagi_url']
        url_response = json.loads(requests.get(feagi_settings['magic_link']).text)
        feagi_settings['feagi_dns'] = url_response['feagi_url']
        feagi_settings['feagi_api_port'] = url_response['feagi_api_port']

    if __name__ == '__main__':
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
