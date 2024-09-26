import time, random

import mujoco
import mujoco.viewer

m = mujoco.MjModel.from_xml_path('mujoco\model\humanoid\humanoid.xml')
d = mujoco.MjData(m)

inc_flag: bool = True # init flags for inc and dec in mujoco loop
dec_flag: bool = False
tolerance = 0.01  # Small tolerance for floating-point comparison

with mujoco.viewer.launch_passive(m, d) as viewer:
  # Close the viewer automatically after 30 wall-seconds.
  start = time.time()
  while viewer.is_running() and time.time() - start < 30:
    step_start = time.time()

    # make model spaz out
    for i in range(len(d.ctrl)):
      d.ctrl[i] += random.randint(0, 2) # increase pos of random joint
      d.ctrl[i] -= random.randint(0, 2) # decrease pos of random joint
      if d.ctrl[i] > 4: d.ctrl = 0  # reset value if too high

    # mj_step can be replaced with code that also evaluates
    # a policy and applies a control signal before stepping the physics.
    mujoco.mj_step(m, d)
    ##### - - - - - - -

    # Example modification of a viewer option: toggle contact points every two seconds.
    # with viewer.lock():
    #   viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(d.time % 2)

    # Pick up changes to the physics state, apply perturbations, update options from GUI.
    viewer.sync()

    # Rudimentary time keeping, will drift relative to wall clock.
    time_until_next_step = m.opt.timestep - (time.time() - step_start)
    if time_until_next_step > 0:
      time.sleep(time_until_next_step)



