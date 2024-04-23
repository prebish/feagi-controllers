#!/usr/bin/env python3
import os
import json
import requests
import argparse
import traceback
import subprocess
from time import sleep
import feagi_connector_freenove


if __name__ == '__main__':
    # Check if feagi_connector has arg
    parser = argparse.ArgumentParser(description='configuration for any webcam')
    parser.add_argument('-ip', '--ip', help='to connect FEAGI, required=False')
    parser.add_argument('-setup', '--setup', help='first time setup only', required=False)
    parser.add_argument('-zmq_port', '--zmq_port', help='zmq port to connect with FEAGI thru zmq',
                        required=False)
    parser.add_argument('-api_port', '--api_port', help='api_port for FEAGI thru API',
                        required=False)
    parser.add_argument('-port_disabled', '--port_disabled', help='not include port',
                        required=False)
    parser.add_argument('-http_type', '--http_type', help='https:// or http://',
                        required=False)
    parser.add_argument('-magic_link', '--magic_link', help='to use magic link', required=False)
    parser.add_argument('-magic-link', '--magic-link', help='to use magic link', required=False)
    parser.add_argument('-magic', '--magic', help='to use magic link', required=False)
    parser.add_argument('-model', '--model', help='add `-model` and put type: 4wd, dog, '
                                                  'tank, or hexapod',
                        required=True)
    args = vars(parser.parse_args())

    # NEW JSON UPDATE
    current_path = feagi_connector_freenove.__path__
    f = open(str(current_path[0]) + '/configuration.json')
    configuration = json.load(f)
    feagi_settings =  configuration["feagi_settings"]
    agent_settings = configuration['agent_settings']
    capabilities = configuration['capabilities']
    feagi_settings['feagi_host'] = os.environ.get('FEAGI_HOST_INTERNAL', "127.0.0.1")
    feagi_settings['feagi_api_port'] = os.environ.get('FEAGI_API_PORT', "8000")
    f.close()
    message_to_feagi = {"data": {}}
    # END JSON UPDATE


    if args['ip']:
        feagi_settings["feagi_host"] = args['ip']
    if args['setup']:
        new_path = current_path[0] + "/setup.sh " + current_path[0]
        subprocess.run([new_path, "arguments"], shell=True)
    if args['zmq_port']:
        agent_settings["agent_data_port"] = args['zmq_port']
    if args['api_port']:
        feagi_settings["feagi_api_port"] = args['api_port']
    if args['model']:
        if args['model'] == '4wd':
            from feagi_connector_freenove import controller as freenove_smartcar_controller
        if args['model'] == 'dog':
            pass # Hasn't implemented yet
            print("NOT IMPLEMENTED YET")
        if args['model'] == 'tank':
            pass  # Hasn't implemented yet
            print("NOT IMPLEMENTED YET")
        if args['model'] == 'hexapod':
            pass  # Hasn't implemented yet
            print("NOT IMPLEMENTED YET")
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
    feagi_auth_url = feagi_settings.pop('feagi_auth_url', None)
    print("FEAGI AUTH URL ------- ", feagi_auth_url)
    while True:
        try:
            freenove_smartcar_controller.main(feagi_auth_url,
                                              feagi_settings,
                                              agent_settings,
                                              capabilities)
            sleep(5)
        except Exception as e:
            print(f"Controller run failed", e)
            traceback.print_exc()
            sleep(2)
