import time
import mujoco
import mujoco.viewer
import threading 
import keyboard 

""" vfs = mujoco.mj_defaultVisual()

test = mujoco.mj_addFileVFS(vfs, '/Users/ctd/Downloads/', 'humanoid.xml') """



m = mujoco.MjModel.from_xml_path('/Users/ctd/Downloads/humanoid.xml')
d = mujoco.MjData(m)

#mujoco.mj_printModel(m, 'test.txt')
#m.opt.gravity = (0,0,-1)

def check_input():
  """ while True:
        keyboard.wait('space')
        print(f"does this work") """

input_thread = threading.Thread(target=check_input, daemon=True)
input_thread.start()

d.qpos = m.key_qpos[0]

with mujoco.viewer.launch_passive(m, d) as viewer:
  # Close the viewer automatically after 30 wall-seconds.
  start = time.time()
  while viewer.is_running():
    step_start = time.time()

    # mj_step can be replaced with code that also evaluates
    # a policy and applies a control signal before stepping the physics.
    mujoco.mj_step(m, d)

    # Example modification of a viewer option: toggle contact points every two seconds.
    with viewer.lock():
      viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(d.time % 2)

    # Pick up changes to the physics state, apply perturbations, update options from GUI.
    viewer.sync()

    #m.jnt('ankle_y_right').pos+=.01
    #m.jnt('abdomen_z').pos+=.01
   
    
    #0: global position left/right (x axis)
    #1: global position forward/back (y axis)
    #2: gloabl position up/down (z axis)
    #3: idk
    #4: idk
    #5: idk
    #6: idk
    #7: abdomen_z
    #8: abdomen_y
    #9: abdomen_x
    #10: hip_x_right
    #11: hip_z_right
    #12: hip_y_right
    #13: knee_right
    #14: ankle_y_right
    #15: ankle_x_right
    #16: hip_x_left
    #17: hip_z_left
    #18: hip_y_left
    #19: knee_left
    #20: ankle_y_left
    #21: ankle_x_left
    #22: shoulder1_right
    #23: shoulder_2_right
    #24: elbow_right
    #25: shoulder1_left
    #26: shoulder2_left
    #27: elbow_left

    print(f"{d.qpos[7]:.3f}")
    
    #d.qpos[7] += .01
    #d.qpos[2] +=.05
    #d.ctrl[0] += 1

    # Rudimentary time keeping, will drift relative to wall clock.
    time_until_next_step = m.opt.timestep - (time.time() - step_start)
    if time_until_next_step > 0:
      time.sleep(time_until_next_step)