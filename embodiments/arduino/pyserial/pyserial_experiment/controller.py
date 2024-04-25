from time import sleep
import time
import json
from configuration import *
from feagi_connector import feagi_interface as feagi
from feagi_connector import pns_gateway as pns
from feagi_connector.version import __version__
from feagi_connector import actuators


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


if __name__ == "__main__":
    print("Ready...")
    ser = initialization_port(capabilities['arduino']['port'])
    feagi_flag = False
    print("Waiting on FEAGI...")
    while not feagi_flag:
        feagi_flag = feagi.is_FEAGI_reachable(os.environ.get('FEAGI_HOST_INTERNAL',
                                                             "127.0.0.1"), int(os.environ.get(
            'FEAGI_OPU_PORT', "3000")))
        sleep(2)
    print("DONE")
    previous_data_frame = {}
    runtime_data = {"cortical_data": {}, "current_burst_id": None,
                    "stimulation_period": 0.01, "feagi_state": None,
                    "feagi_network": None}

    # # # FEAGI registration # # # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    feagi_settings, runtime_data, api_address, feagi_ipu_channel, feagi_opu_channel = \
        feagi.connect_to_feagi(feagi_settings, runtime_data, agent_settings, capabilities,
                               __version__)
    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    msg_counter = runtime_data["feagi_state"]['burst_counter']

    # Experimenting...delete this
    data = dict()
    data[1] = 1
    data[2] = 1
    # Ends

    # To give ardiuno some time to open port. It's required
    time.sleep(5)
    while True:
        message_from_feagi = pns.signals_from_feagi(feagi_opu_channel)

        # Fetch data such as motor, servo, etc and pass to a function (you make ur own action. 
        if message_from_feagi is not None:
            obtained_signals = pns.obtain_opu_data(message_from_feagi)
            new_dict = dict()
            if 'motor' in obtained_signals:
                if obtained_signals['motor']:
                    for x in obtained_signals['motor']:
                        if x in [0, 1, 2, 3]:
                            new_dict[x] = obtained_signals['motor'][x]
                    json_data = convert_dict_to_json(new_dict) #convert dict to JSON
                    if ser.isOpen():
                        send_serial(ser, json_data)
