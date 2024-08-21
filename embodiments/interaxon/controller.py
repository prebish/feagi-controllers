import logging
import argparse
import time

import numpy as np
from time import sleep
from feagi_connector import pns_gateway as pns
from feagi_connector.version import __version__
from feagi_connector import sensors
from feagi_connector import feagi_interface as feagi
from brainflow.board_shim import BoardShim, BrainFlowInputParams, BoardIds

def convert_sensor_to_ipu_data(min_output, max_output, raw_data, pin_number):
    if pns.full_list_dimension:
        if 'i__bci' in pns.full_list_dimension:
            max_input = pns.full_list_dimension['i__bci']['cortical_dimensions'][2] - 1
            total_range = max_output - min_output
            position = (raw_data / total_range) * max_input
            data = str(pin_number) + '-0-' + str(abs(int(round(position, 2))))
            # print("max: ", max_output, " output: ", min_output, " raw data: ", raw_data, " name: ", pin_number, " completed data: ", data)
            return data
    return None


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

    # brainflow section #
    BoardShim.enable_dev_board_logger()
    logging.basicConfig(level=logging.DEBUG)
    parser = argparse.ArgumentParser()
    parser.add_argument('--timeout', type=int, help='timeout for device discovery or connection', required=False,
                        default=0)
    parser.add_argument('--ip-port', type=int, help='ip port', required=False, default=0)
    parser.add_argument('--ip-protocol', type=int, help='ip protocol, check IpProtocolType enum', required=False,
                        default=0)
    parser.add_argument('--ip-address', type=str, help='ip address', required=False, default='')
    parser.add_argument('--serial-port', type=str, help='serial port', required=False, default='')
    parser.add_argument('--mac-address', type=str, help='mac address', required=False, default='')
    parser.add_argument('--other-info', type=str, help='other info', required=False, default='')
    parser.add_argument('--streamer-params', type=str, help='streamer params', required=False, default='')
    parser.add_argument('--serial-number', type=str, help='serial number', required=False, default='')
    parser.add_argument('--board-id', type=int, help='board id, check docs to get a list of supported boards',
                        required=False, default=BoardIds.SYNTHETIC_BOARD)
    parser.add_argument('--file', type=str, help='file', required=False, default='')
    parser.add_argument('--master-board', type=int, help='master board id for streaming and playback boards',
                        required=False, default=BoardIds.NO_BOARD)
    args = parser.parse_args()

    params = BrainFlowInputParams()
    params.ip_port = args.ip_port
    params.serial_port = args.serial_port
    params.mac_address = args.mac_address
    params.other_info = args.other_info
    params.serial_number = args.serial_number
    params.ip_address = args.ip_address
    params.ip_protocol = args.ip_protocol
    params.timeout = args.timeout
    params.file = args.file
    params.master_board = args.master_board

    params = BrainFlowInputParams()
    board_shim = BoardShim(BoardIds.MUSE_2_BOARD, params)

    # debug code only
    max_value = []
    min_value = []
    for i in range(6):
        max_value.append(0)
        min_value.append(0)

    try:
        board_shim.prepare_session()
        board_shim.start_stream(450000, args.streamer_params)

        exg_channels = BoardShim.get_exg_channels(args.board_id)
        if len(exg_channels) < 4:
            print("The board does not have enough channels.")

        channels_to_print = exg_channels[:4]

        while True:
            message_from_feagi = pns.message_from_feagi

            if message_from_feagi:
                # Fetch data such as motor, servo, etc and pass to a function (you make ur own action.
                pns.check_genome_status_no_vision(message_from_feagi)
                feagi_settings['feagi_burst_speed'] = pns.check_refresh_rate(message_from_feagi, feagi_settings['feagi_burst_speed'])
                obtained_signals = pns.obtain_opu_data(message_from_feagi)
                new_dict = dict()

            data = board_shim.get_current_board_data(250)
            if data.any():
                convert_eeg_to_ipu = dict()
                create_analog_data_list = dict()
                create_analog_data_list['i__bci'] = dict()
                for number in channels_to_print:
                    channel = number - 1
                    convert_eeg_to_ipu[channel] = data[number][len(data[number])-1]
                    if data[number].max() > max_value[channel]:
                        max_value[channel] = data[number].max()
                    if data[number].min() < min_value[channel]:
                        min_value[channel] = data[number].min()
                    # position_of_analog = convert_sensor_to_ipu_data(min_value[channel], max_value[channel], convert_eeg_to_ipu[channel], channel)
                    position_of_analog = str(channel) + "-0-0"
                    create_analog_data_list['i__bci'][position_of_analog] = convert_eeg_to_ipu[channel] + 1000.0
                # print("min: ", min_value, " and max:", max_value)
                message_to_feagi = sensors.add_generic_input_to_feagi_data(create_analog_data_list,
                                                                           message_to_feagi)
                pns.signals_to_feagi(message_to_feagi, feagi_ipu_channel, agent_settings, feagi_settings)
                    # convert_eeg_to_ipu[channel] = np.average(data[channel].tolist())
                    # print(f"Channel {channel}: {np.average(data[channel].tolist())}")
                # print(convert_eeg_to_ipu)


            sleep(feagi_settings['feagi_burst_speed'])
    except BaseException:
        logging.warning('Exception', exc_info=True)
    finally:
        if board_shim.is_prepared():
            logging.info('Releasing session')
            board_shim.release_session()
