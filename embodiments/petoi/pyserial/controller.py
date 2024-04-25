from time import sleep
import time
import threading
import serial
from datetime import datetime
from configuration import *
from feagi_connector import feagi_interface as feagi
from feagi_connector import sensors
from feagi_connector import pns_gateway as pns
from feagi_connector.version import __version__
from feagi_connector import actuators

servo_status = dict()
gyro = {}


# Function to handle receiving data
def read_from_port(ser):
    global received_data, gyro
    full_data = ''
    # start_time = datetime.now()
    # counter = 0
    while True:
        # total_time = (datetime.now() - start_time).total_seconds()
        # if total_time > 1:
        #     start_time = datetime.now()
        #     print("data recieved: ", counter, " after 1 second", total_time)
        #     counter = 0
        reading = ser.readline().decode('utf-8').rstrip()
        received_data = reading
        try:
            if '#' in received_data:
                cleaned_data = received_data.replace('#', '')
                new_data = full_data + cleaned_data
                new_data = new_data.split(",")
                processed_data = []
                for i in new_data:
                    full_number = str()
                    for x in i:
                        if x in [".", "-"] or x.isdigit():
                            full_number += x
                    if full_number:
                        processed_data.append(float(full_number))
                # Add gyro data into feagi data
                gyro['gyro'] = {'0': processed_data[0], '1': processed_data[1],
                                '2': processed_data[2]}
            else:
                full_data = received_data
        except Exception as Error_case:
            pass
        # counter += 1

def feagi_to_petoi_id(device_id):
    mapping = {
        0: 0,
        1: 8,
        2: 12,
        3: 9,
        4: 13,
        5: 11,
        6: 15,
        7: 10,
        8: 14
    }
    return mapping.get(device_id, None)


def action(obtained_data):
    servo_data = actuators.get_servo_data(obtained_data, True)
    if 'servo_position' in obtained_data:
        servo_for_feagi = 'i '
        if obtained_data['servo_position'] is not {}:
            for data_point in obtained_data['servo_position']:
                device_id = feagi_to_petoi_id(data_point)
                encoder_position = (((180) / 20) * obtained_data['servo_position'][data_point]) - 90
                servo_for_feagi += str(device_id) + " " + str(encoder_position) + " "
            print(servo_for_feagi)
            ser.write(servo_for_feagi.encode())
    if servo_data:
        servo_for_feagi = 'i '
        for device_id in servo_data:
            servo_power = actuators.servo_generate_power(90, servo_data[device_id], device_id)
            if device_id not in servo_status:
                servo_status[device_id] = actuators.servo_keep_boundaries(servo_power)
                # pin_board[device_id].write(servo_status[device_id])
            else:
                servo_status[device_id] += servo_power / 10
                servo_status[device_id] = actuators.servo_keep_boundaries(servo_status[device_id])
                # pin_board[device_id].write(servo_status[device_id])
                token = feagi_to_petoi_id(device_id)
                task = servo_status[device_id] - 90  # white space
                servo_for_feagi += str(token) + " " + str(task) + " "
        print(servo_for_feagi)
        ser.write(servo_for_feagi.encode())



if __name__ == "__main__":
    ser = serial.Serial('/dev/ttyACM0', 115200)
    thread_read = threading.Thread(target=read_from_port, args=(ser,))
    # thread_write = threading.Thread(target=write_to_port, args=(ser,))

    thread_read.start()
    # thread_write.start()

    # thread_read.join()
    # thread_write.join()
    print("Ready...")
    feagi_flag = False
    print("Waiting on FEAGI...")
    while not feagi_flag:
        feagi_flag = feagi.is_FEAGI_reachable(os.environ.get('FEAGI_HOST_INTERNAL', "127.0.0.1"),
                                              int(os.environ.get('FEAGI_OPU_PORT', "3000")))
        sleep(2)
    print("DONE")
    previous_data_frame = {}
    runtime_data = {"cortical_data": {}, "current_burst_id": None, "stimulation_period": 0.01,
                    "feagi_state": None, "feagi_network": None}

    # # # FEAGI registration # # # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    feagi_settings, runtime_data, api_address, feagi_ipu_channel, feagi_opu_channel = \
        feagi.connect_to_feagi(feagi_settings, runtime_data, agent_settings, capabilities,
                               __version__)
    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    msg_counter = runtime_data["feagi_state"]['burst_counter']

    # To give ardiuno some time to open port. It's required
    threading.Thread(target=pns.feagi_listener, args=(feagi_opu_channel,), daemon=True).start()
    time.sleep(5)
    while True:
        message_from_feagi = pns.message_from_feagi

        # Fetch data such as motor, servo, etc and pass to a function (you make ur own action.
        if message_from_feagi is not None:
            pns.check_genome_status_no_vision(message_from_feagi)
            feagi_settings['feagi_burst_speed'] = pns.check_refresh_rate(message_from_feagi, feagi_settings['feagi_burst_speed'])
            obtained_signals = pns.obtain_opu_data(message_from_feagi)
            action(obtained_signals)
        if gyro:
            message_to_feagi = sensors.add_gyro_to_feagi_data(gyro['gyro'], message_to_feagi)

        message_to_feagi['timestamp'] = datetime.now()
        message_to_feagi['counter'] = msg_counter
        pns.signals_to_feagi(message_to_feagi, feagi_ipu_channel, agent_settings, feagi_settings)
        message_to_feagi.clear()
        sleep(feagi_settings['feagi_burst_speed'])
