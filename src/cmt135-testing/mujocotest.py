import time

import mujoco
import mujoco.viewer

m = mujoco.MjModel.from_xml_path('/Users/ctd/Downloads/humanoid.xml')
d = mujoco.MjData(m)

interval = 0
lastTime = time.time()

with mujoco.viewer.launch_passive(m, d) as viewer:
  while viewer.is_running():
    # mj_step can be replaced with code that also evaluates
    # a policy and applies a control signal before stepping the physics.
    mujoco.mj_step(m, d)
    
    # Get position data (joint positions)
    # qpos is the generalized position vector (contains joint positions)
    positions = d.qpos
    positions = positions[7:] #idk what the first 7 are
 
    currentTime = time.time()
    if currentTime - lastTime > interval:
        lastTime = currentTime
        for i, pos in enumerate(positions):
            print(i, pos)
    
    # Example modification of a viewer option: toggle contact points every two seconds.
    with viewer.lock():
        viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(d.time % 2)

    # Pick up changes to the physics state, apply perturbations, update options from GUI.
    viewer.sync()

    """ mujoco.mj_printModel(m, "test.txt")
    mujoco.mj_printData(m, d, "test2.txt") """
   