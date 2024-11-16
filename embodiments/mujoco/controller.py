#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Copyright 2016-present Neuraville Inc. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================
"""

import threading
import copy
# import frame_data_test as cv2
from time import sleep
from feagi_connector import sensors
from feagi_connector import actuators
from feagi_connector import retina as retina
from feagi_connector import pns_gateway as pns
from feagi_connector.version import __version__
from feagi_connector import feagi_interface as feagi
import time, random
import mujoco, mujoco.viewer
import numpy as np

import json 

# Global variable section
camera_data = {"vision": []}  # This will be heavily relies for vision

RUNTIME = float('inf') # (seconds) timeout time 
SPEED   = 120 # simulation step speed

#this works but I'd think there's a better way to check this than opening the json again 
def get_servo_min_max(index):
    try:
        #get json 
        with open('capabilities.json', 'r') as file:
            data = json.load(file)
        # Access the servo data based on the index
        servo_data = data["capabilities"]["output"]["servo"][str(index)]
        min_value = servo_data["min_value"]
        max_value = servo_data["max_value"]
        return min_value, max_value
    except KeyError:
        return None, None  # Return None if the index is not found
    
def check_keypos(data, model):
    #check if current data.qpos matches a model.keypos
    # Loop over each keyframe
    for i in range(model.nkey):
        # Compare with current data.qpos using np.allclose
        if np.allclose(data.qpos, model.key_qpos[i], atol=1e-6):
            #print(f"Current qpos matches keyframe {i}")
            return i
    else:
        return -1

#Position number can be 1-5
def start_keypos(data, model, position_number):
    data.qpos = model.key_qpos[position_number] 
    #mujoco.mj_step(model, data) # single step to make sure data renders
    viewer.sync()
    return model.key_qpos[position_number]

#Better balance attempt using actual physics 
def balance_attempt_advanced(data, desired_qpos, desired_qvel):
    # Control gains
    Kp = np.array([100] * len(desired_qpos))  # Proportional gains
    Kd = np.array([10] * len(desired_qpos))   # Derivative gains
    # Current joint positions and velocities
    current_qpos = data.qpos[7:]
    current_qvel = data.qvel[6:]  # Exclude global velocities
    # Compute errors
    qpos_error = desired_qpos - current_qpos
    qvel_error = desired_qvel - current_qvel
    # PD control law
    control_torques = Kp * qpos_error + Kd * qvel_error
    # Apply control torques within actuator limits
    data.ctrl[:] = np.clip(control_torques, model.actuator_ctrlrange[:, 0], model.actuator_ctrlrange[:, 1])

#This does not work lol it stresses him out
def balance_attempt_basic(data, start_pos): 
    current_pos = data.qpos[7:]
    start_pos = start_pos[7:]
    for i, pos in enumerate(current_pos):
            if (pos < start_pos[i]):
                data.ctrl[i] += .01
            elif (pos > start_pos[i]):
                data.ctrl[i] -= .01

#Pauses the model until control is applied somewhere.
def pause_until_move(data, start_pos):
    for i, ctrl in enumerate(data.ctrl):
        if (ctrl != 0): #if we change the ctrl we "free" the limb
            return False # we are no longer paused
    
    data.qpos[:] = start_pos[:]
    return True


# Since we're ignoring the physics behind everything, moving multiple joints will create overflow in data.qacc 
# which briefly resets the model. Tried to fix this but solution is not simple. I notice the qpos positions go outside their bounds 
# when it happens so maybe hard bounds along with resetting the respective data.qvel and data.qacc values could fix it (just an idea) 
def pause_standing_unstable(data, start_pos, free_joints):
    for i, ctrl in enumerate(data.ctrl):
        if (data.ctrl[i] == 0):
            free_joints[i]=0
        if (ctrl != 0): #if we change the ctrl we "free" the limb
            free_joints[i] = -1 #find the limb to free and mark it
    for i, pos in enumerate(data.qpos[:7]):
            data.qpos[i] = start_pos[i]
            pass
    for i, pos in enumerate(data.qpos[7:]):
        if (free_joints[i] != -1):
            data.qpos[i+7] = start_pos[i+7]
    return free_joints    


def action(obtained_data, capabilities):
    """
    This is where you can make the robot do something based on FEAGI data. The variable
    obtained_data contains the data from FEAGI. The variable capabilities comes from
    the configuration.json file. It will need the capability to measure how much power it can control
    and calculate using the FEAGI data.

    obtained_data: dictionary.
    capabilities: dictionary.
    """
    recieve_servo_data = actuators.get_servo_data(obtained_data)
    recieve_servo_position_data = actuators.get_servo_position_data(obtained_data)
  
    if recieve_servo_position_data:
        # output like {0:0.50, 1:0.20, 2:0.30} # example but the data comes from your capabilities' servo range
        for real_id in recieve_servo_position_data:
            servo_number = real_id
            power = recieve_servo_position_data[real_id]
            data.ctrl[servo_number] = power
            
            
            """ #get min and max values from json (probably a better way to do this)
            min_val, max_val = get_servo_min_max(servo_number)

            if (old_power >= 0):
                new_power = old_power/max_val
            elif (old_power < 0):
                new_power = -old_power/min_val """
            
            #print("length: %d", length) #testing
            #print("new power: %d", new_power ," servo number: %d", servo_number, "old power: %d", old_power, "old power: %d", old_power, "old power: %d", old_power  ) #testing
            #also keep within bounds
            """ if (data.ctrl[servo_number] + new_power > 1):  
                data.ctrl[servo_number] = 1
            elif (data.ctrl[servo_number] + new_power < -1):  
                data.ctrl[servo_number] = -1
            else:
                data.ctrl[servo_number] = new_power """
            
            #data.qpos[servo_number+7] += .1
            #tolerance = .0001
            """ while abs(data.qpos[servo_number+7] - old_power) >tolerance:
                print("data.qpos[servo_number+7]: ", data.qpos[servo_number+7], "old_power: ", old_power) #testing
                if data.qpos[servo_number+7] > old_power:
                    data.qpos[servo_number+7] -= .00001
                elif data.qpos[servo_number+7] < old_power:
                    data.qpos[servo_number+7] += .00001
             """
    if recieve_servo_data:
        for real_id in recieve_servo_data:
            servo_number = real_id
            new_power = recieve_servo_data[real_id]
            data.ctrl[servo_number] = new_power


if __name__ == "__main__":
    # Generate runtime dictionary
    runtime_data = {"vision": [], "stimulation_period": None, "feagi_state": None,
                    "feagi_network": None}

    # This function will build the capabilities from your configuration.json and read the
    # args input. First, it will gather all details from your configuration.json. Once it's done,
    # it will read all input args, such as flags. Once it detects flags from the user, it will override
    # the configuration and use the input provided by the user.
    config = feagi.build_up_from_configuration()
    feagi_settings = config['feagi_settings'].copy()
    agent_settings = config['agent_settings'].copy()
    default_capabilities = config['default_capabilities'].copy()
    message_to_feagi = config['message_to_feagi'].copy()
    capabilities = config['capabilities'].copy()

    # Simply copying and pasting the code below will do the full work for you. It basically checks
    # and updates the network to ensure that it can connect with FEAGI. If it doesn't find FEAGI,
    # it will just wait and display "waiting on FEAGI...".
    # # # FEAGI registration # # # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    feagi_settings, runtime_data, api_address, feagi_ipu_channel, feagi_opu_channel = \
        feagi.connect_to_feagi(feagi_settings, runtime_data, agent_settings, capabilities,
                               __version__)
    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    # The function `create_runtime_default_list` will design and generate a complete JSON object
    # in the configuration, mainly for vision only. Once it's done, it will get the configuration JSON,
    # override all keys generated by this function, and store them into the same capabilities for
    # the rest of controller runtime.
    default_capabilities = pns.create_runtime_default_list(default_capabilities, capabilities)

    # This is for processing the data and updating in real-time based on the user's activity in BV,
    # such as cortical size, blink, reload genome, and other backend tasks.
    if "camera" in capabilities['input']:
        threading.Thread(target=retina.vision_progress,
                         args=(default_capabilities, feagi_settings, camera_data['vision'],),
                         daemon=True).start()
        
    # Model and Data objects which are used inside the simulation loop
    # make sure the model path is using the relative humanoid file located at ./humanoid.xml
    model = mujoco.MjModel.from_xml_path('./humanoid.xml')
    data  = mujoco.MjData(model)

    # Create a dict to store data
    force_list = {}
    for x in range(20):
        force_list[str(x)] = [0, 0, 0]

    #start_keypos(data, model, 0)
    #start_keypos(data, 4) #preset positions, 0: squat, 1: standing one leg, 2:...

    
    actuators.start_servos(capabilities) # inserted here. This is not something you should do on your end. I will fix it shortly
    with mujoco.viewer.launch_passive(model, data) as viewer:
        mujoco.mj_resetDataKeyframe(model, data, 4)
        start_time = time.time()
        free_joints = [0] * 21 #keep track of which joints to lock and free (for unstable pause method)
        #start_keypos(data, model, 1)
        start_pos = copy.copy(data.qpos)
        paused = True
        opt = mujoco.MjvOption()
        while viewer.is_running() and time.time() - start_time < RUNTIME:

            step_start = time.time()

            #check if key pos was manually changed (delete was pressed) and reset to our custom start instead (prob a better way to do this)
            if check_keypos(data, model) >= 0:
                #start_pos = start_keypos(data, model, check_keypos(data, model)) #if it was, update the start_pos
                pass

            ### PAUSING/BALANCE ### Only try one at a time.
            #balance_attempt_advanced(data, start_pos[7:], np.zeros_like(start_pos[7:])) #better attempt at balancing
            #if paused:
            #paused = pause_until_move(data, start_pos) #pauses the simulation until control is applied. Will lock positions if d.ctrl[:]==0
            #free_joints = pause_standing_unstable(data, start_pos, free_joints) #Unstable. Mainly for testing. lock the model but move joints freely. Helpful for seeing what controls actually do
            #balance_attempt_basic(data, start_pos) #Bad. tries to use ctrl to balance instead of hardcoding qpos.
            ###############

            #print("proximity data:" , data.sensordata) #test to print proximity data

            # steps the simulation forward 'tick' -- Only step if we aren't paused because it breaks the physics
            #if not paused:
            mujoco.mj_step(model, data)

            # The controller will grab the data from FEAGI in real-time
            message_from_feagi = pns.message_from_feagi
            if message_from_feagi:
                # Translate from feagi data to human readable data
                obtained_signals = pns.obtain_opu_data(message_from_feagi)
                pns.check_genome_status_no_vision(message_from_feagi)
                action(obtained_signals, data)

            #region READ POSITIONAL DATA HERE ###
            positions = data.qpos #all positions
            positions = positions[7:] #don't know what the first 7 positions are, but they're not joints so ignore them

            abdomen_positions = positions[:3] #first 3 are abdomen z,y,x
            abdomen_positions = abdomen_positions[::-1] #reverse it to x,y,z order
            #endregion

            #region READ FORCE DATA HERE ###
            # Loop through all degrees of freedom (DOFs) to access forces applied to each joint.
            # This loop gives internal force data related to actuators or controllers
            # and is useful for understanding forces acting directly on the joints.
            for i in range(model.nv):  # `nv` is the number of degrees of freedom (DOFs)
                force = data.qfrc_applied[i]
                # print(f"Joint DOF {i}, Applied Force: {force}")

            # Loop through all contacts in the simulation
            # between all bodies. This *should* include the xml model
            # and environmental bodies like ground/floor
            for i in range(data.ncon):
                # contact = data.contact[i]  # Access each contact # Not even sure how to address this. I was expecting something like true/false for each index - Kevin
                force = np.zeros(6)  # Use numpy to allocate blank array

                # Retrieve the contact force data
                mujoco.mj_contactForce(model, data, i, force)

                # Printout contact force data
                # print(f"Contact between body {contact.geom1} and body {contact.geom2}")
                # print(f"Force: {force[:3]}", " and id: ", i)
                obtained_data_from_force = force[:3]
                force_list[str(i)] = list((float(obtained_data_from_force[0]), float(obtained_data_from_force[1]), float(obtained_data_from_force[2])))
            # endregion

            # Pick up changes to the physics state, apply perturbations, update options from GUI.
            viewer.sync()

            # Tick Speed #
            time_until_next_step = (1/SPEED) - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)


            # Example to send data to FEAGI. This is basically reading the joint. R
            abdomen_gyro_data = {'0': [float(abdomen_positions[0]), float(abdomen_positions[1]), float(abdomen_positions[2])]}
            servo_data = {i: pos for i, pos in enumerate(positions[:21]) if
                          pns.full_template_information_corticals}
            sensor_data = {i: pos for i, pos in enumerate(data.sensordata) if
                          pns.full_template_information_corticals}

            #Creating message to send to FEAGI
            message_to_feagi_gyro = sensors.create_data_for_feagi('gyro',
                                                             capabilities,
                                                             message_to_feagi,
                                                             current_data=abdomen_gyro_data,
                                                             symmetric=True)
            message_to_feagi_servo = sensors.create_data_for_feagi('servo_position',
                                                             capabilities,
                                                             message_to_feagi,
                                                             current_data=servo_data,
                                                             symmetric=True)
            message_to_feagi_prox = sensors.create_data_for_feagi('proximity',
                                                             capabilities,
                                                             message_to_feagi,
                                                             current_data=sensor_data,
                                                             symmetric=True, measure_enable=True)
            message_to_feagi_force = sensors.create_data_for_feagi('pressure',
                                                                   capabilities,
                                                                   message_to_feagi,
                                                                   current_data=force_list,
                                                                   symmetric=True, measure_enable=False) # measure enable set to false so that way, it doesnt change 50/-50 in capabilities automatically


            # Sends to feagi data
            pns.signals_to_feagi(message_to_feagi_servo, feagi_ipu_channel, agent_settings, feagi_settings)
            pns.signals_to_feagi(message_to_feagi_gyro, feagi_ipu_channel, agent_settings, feagi_settings)
            pns.signals_to_feagi(message_to_feagi_prox, feagi_ipu_channel, agent_settings, feagi_settings) #confused why it still shows up in the bv when commented out
            pns.signals_to_feagi(message_to_feagi_force, feagi_ipu_channel, agent_settings, feagi_settings)

            # Clear data that is created by controller such as sensors
            message_to_feagi.clear()

            # Pick up changes to the physics state, apply perturbations, update options from GUI.
            viewer.sync()
            # cv2.get_frame(model=model, data=data, opt=opt)

            # Tick Speed #
            time_until_next_step = (1/SPEED) - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)
