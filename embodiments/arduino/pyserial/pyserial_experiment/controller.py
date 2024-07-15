from time import sleep
import time
import json
from feagi_connector import feagi_interface as feagi
from feagi_connector import pns_gateway as pns
from feagi_connector.version import __version__
from feagi_connector import actuators
import serial


def initialization_port(port):
    return serial.Serial(port,
                         baudrate=9600,
                         timeout=2.5,
                         parity=serial.PARITY_NONE,
                         bytesize=serial.EIGHTBITS,
                         stopbits=serial.STOPBITS_ONE)


def convert_dict_to_json(data):
    data = json.dumps(data)
    return data


def send_serial(ser, data):
    ser.write(data.encode('ascii'))


def read_from_port(ser):
    data_received = ser.readline().decode('utf-8').rstrip()
    if data_received:
        print(data_received) # TODO: Needs to make this somewhat useful and scalable
        print("type: :", type(data_received))
        if data_received.isdigit():
            data_as_int = int(data_received)
            print(data_as_int)


if __name__ == "__main__":
    runtime_data = dict()
    config = feagi.build_up_from_configuration()
    feagi_settings = config['feagi_settings'].copy()
    agent_settings = config['agent_settings'].copy()
    default_capabilities = config['default_capabilities'].copy()
    message_to_feagi = config['message_to_feagi'].copy()
    capabilities = config['capabilities'].copy()

    # # # FEAGI registration # # # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    # - - - - - - - - - - - - - - - - - - #
    feagi_settings, runtime_data, api_address, feagi_ipu_channel, feagi_opu_channel = \
        feagi.connect_to_feagi(feagi_settings, runtime_data, agent_settings, capabilities,
                               __version__)
    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    msg_counter = runtime_data["feagi_state"]['burst_counter']

    ser = initialization_port(capabilities['arduino']['port'])

    # To give ardiuno some time to open port. It's required
    time.sleep(5)
    while True:
        message_from_feagi = pns.message_from_feagi
        # Fetch data such as motor, servo, etc and pass to a function (you make ur own action. 
        if message_from_feagi is not None:
            # Fetch data such as motor, servo, etc and pass to a function (you make ur own action.
            pns.check_genome_status_no_vision(message_from_feagi)
            feagi_settings['feagi_burst_speed'] = \
                pns.check_refresh_rate(message_from_feagi, feagi_settings['feagi_burst_speed'])
            obtained_signals = pns.obtain_opu_data(message_from_feagi)
            new_dict = dict()

            read_from_port(ser)

            if 'motor' in obtained_signals:
                if obtained_signals['motor']:
                    for x in obtained_signals['motor']:
                        if x in [0, 1, 2, 3]:
                            new_dict[x] = obtained_signals['motor'][x]
                    json_data = convert_dict_to_json(new_dict)  # convert dict to JSON
                    if ser.isOpen():
                        send_serial(ser, json_data)
