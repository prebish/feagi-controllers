from feagi_connector import pns_gateway as pns
from feagi_connector import sensors
import traceback


def cozmo_ipu(robot, capabilities, angle_of_head, angle_of_arms, message_to_feagi):
    if robot['battery']:
        message_to_feagi = sensors.create_data_for_feagi('battery', capabilities, message_to_feagi, robot['battery'],symmetric=False)
    
    if robot['gyro']:
        message_to_feagi = sensors.create_data_for_feagi('gyro', capabilities, message_to_feagi, robot['gyro'],symmetric=True)
    
    # # Add accelerator section
    if robot['accelerator']:
        message_to_feagi = sensors.create_data_for_feagi('accelerometer', capabilities, message_to_feagi, robot['accelerator'], symmetric=True, measure_enable=True)
    
    if robot['proximity']:
        message_to_feagi = sensors.create_data_for_feagi('proximity', capabilities, message_to_feagi, robot['proximity'][0], symmetric=True, measure_enable=True)
    
    if pns.full_template_information_corticals:
        # TODO: Figure how to fix this. Also I need to get the actual servo, not my own code tracker
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