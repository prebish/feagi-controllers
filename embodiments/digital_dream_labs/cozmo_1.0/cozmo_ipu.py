from feagi_connector import pns_gateway as pns
from feagi_connector import sensors
import traceback


def cozmo_ipu(robot, capabilities, angle_of_head, angle_of_arms, message_to_feagi):
    if robot['battery']:
        cortical_id = pns.name_to_feagi_id(sensor_name='battery')
        current_battery = robot['battery']
        for device_id in capabilities['input']['battery']:
            if not capabilities['input']['battery'][device_id]['disabled']:
                create_data_list = dict()
                create_data_list[cortical_id] = dict()
                position_in_feagi_location = sensors.convert_sensor_to_ipu_data(
                    capabilities['input']['battery']['0']['minimum_value'],
                    capabilities['input']['battery']['0']['maximum_value'],
                    current_battery,
                    capabilities['input']['battery']['0']['feagi_index'],
                    sensor_name='battery')
                create_data_list[cortical_id][position_in_feagi_location] = 100
                if create_data_list[cortical_id]:
                    message_to_feagi = sensors.add_generic_input_to_feagi_data(create_data_list,
                                                                               message_to_feagi)
    
    if robot['gyro']:
        for device_id in capabilities['input']['gyro']:
            if not capabilities['input']['gyro'][device_id]['disabled']:
                cortical_id = pns.name_to_feagi_id(sensor_name='gyro')
                create_data_list = dict()
                create_data_list[cortical_id] = dict()
                try:
                    for inner_device_id in range(len(capabilities['input']['gyro'][device_id]['max_value'])):
                        capabilities['input']['gyro'][device_id]['max_value'][inner_device_id], \
                            capabilities['input']['gyro'][device_id]['min_value'][
                                inner_device_id] = sensors.measuring_max_and_min_range(
                            robot['gyro'][inner_device_id],
                            capabilities['input']['gyro'][device_id]['max_value'][inner_device_id],
                            capabilities['input']['gyro'][device_id]['min_value'][inner_device_id])
                        position_in_feagi_location = sensors.convert_sensor_to_ipu_data(
                            capabilities['input']['gyro'][device_id]['min_value'][inner_device_id],
                            capabilities['input']['gyro'][device_id]['max_value'][inner_device_id],
                            robot['gyro'][inner_device_id],
                            capabilities['input']['gyro'][device_id]['feagi_index'] + inner_device_id,
                            sensor_name='gyro',
                            symmetric=True)
                        create_data_list[cortical_id][position_in_feagi_location] = 100
                    if create_data_list[cortical_id]:
                        message_to_feagi = sensors.add_generic_input_to_feagi_data(
                            create_data_list, message_to_feagi)
                except Exception as e:
                    print("here: ", e)
                    traceback.print_exc()
    
    # # Add accelerator section
    if robot['accelerator']:
        if pns.full_template_information_corticals:
            if robot['accelerator']:
                for device_id in capabilities['input']['accelerometer']:
                    if not capabilities['input']['accelerometer'][device_id]['disabled']:
                        cortical_id = pns.name_to_feagi_id(sensor_name='accelerometer')
                        create_data_list = dict()
                        create_data_list[cortical_id] = dict()
                        try:
                            for inner_device_id in range(
                                    len(capabilities['input']['accelerometer'][device_id]['max_value'])):
                                capabilities['input']['accelerometer'][device_id]['max_value'][inner_device_id], \
                                    capabilities['input']['accelerometer'][device_id]['min_value'][
                                        inner_device_id] = sensors.measuring_max_and_min_range(
                                    robot['accelerator'][inner_device_id],
                                    capabilities['input']['accelerometer'][device_id]['max_value'][
                                        inner_device_id],
                                    capabilities['input']['accelerometer'][device_id]['min_value'][
                                        inner_device_id])
                                position_in_feagi_location = sensors.convert_sensor_to_ipu_data(
                                    capabilities['input']['accelerometer'][device_id]['min_value'][
                                        inner_device_id],
                                    capabilities['input']['accelerometer'][device_id]['max_value'][
                                        inner_device_id],
                                    robot['accelerator'][inner_device_id],
                                    capabilities['input']['accelerometer'][device_id][
                                        'feagi_index'] + inner_device_id,
                                    sensor_name='accelerometer',
                                    symmetric=True)
                                create_data_list[cortical_id][position_in_feagi_location] = 100
                            if create_data_list[cortical_id]:
                                message_to_feagi = sensors.add_generic_input_to_feagi_data(
                                    create_data_list, message_to_feagi)
                        except Exception as e:
                            pass
    
    if robot['proximity']:
        if pns.full_template_information_corticals:
            cortical_id = pns.name_to_feagi_id(sensor_name='proximity')
            for device_id in capabilities['input']['proximity']:
                if not capabilities['input']['proximity'][device_id]['disabled']:
                    create_data_list = dict()
                    create_data_list[cortical_id] = dict()
                    capabilities['input']['proximity'][device_id]['max_value'], \
                        capabilities['input']['proximity'][device_id][
                            'min_value'] = sensors.measuring_max_and_min_range(
                        robot['proximity'][int(device_id)],
                        capabilities['input']['proximity'][device_id]['max_value'],
                        capabilities['input']['proximity'][device_id]['min_value'])
    
                    position_in_feagi_location = sensors.convert_sensor_to_ipu_data(
                        capabilities['input']['proximity'][device_id]['min_value'],
                        capabilities['input']['proximity'][device_id]['max_value'],
                        robot['proximity'][int(device_id)],
                        capabilities['input']['proximity'][device_id]['feagi_index'],
                        sensor_name='proximity',
                        symmetric=True)
                    create_data_list[cortical_id][position_in_feagi_location] = 100
                    if create_data_list[cortical_id]:
                        message_to_feagi = sensors.add_generic_input_to_feagi_data(create_data_list,
                                                                                   message_to_feagi)
    
    if pns.full_template_information_corticals:
        cortical_id = pns.name_to_feagi_id(sensor_name='servo')
        for device_id in capabilities['input']['servo']:
            raw_data = 0
            if device_id == '0':
                raw_data = angle_of_head
            if device_id == '1':
                raw_data = angle_of_arms
            if not capabilities['input']['servo'][device_id]['disabled']:
                create_data_list = dict()
                create_data_list[cortical_id] = dict()
                position_in_feagi_location = sensors.convert_sensor_to_ipu_data(
                    capabilities['input']['servo'][device_id]['min_value'],
                    capabilities['input']['servo'][device_id]['max_value'],
                    raw_data,
                    capabilities['input']['servo'][device_id]['feagi_index'],
                    sensor_name='servo',
                    symmetric=True)
                create_data_list[cortical_id][position_in_feagi_location] = 100
                if create_data_list[cortical_id]:
                    message_to_feagi = sensors.add_generic_input_to_feagi_data(create_data_list,
                                                                               message_to_feagi)
    return message_to_feagi