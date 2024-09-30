import time, random
import mujoco, mujoco.viewer

m = mujoco.MjModel.from_xml_path('mujoco\model\humanoid\humanoid.xml')
d = mujoco.MjData(m)


def main():
  with mujoco.viewer.launch_passive(m, d) as viewer:
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

        # Example modification of a viewer option: toggle contact points every two seconds.
        # with viewer.lock():
        #   viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(d.time % 2)

        # Pick up changes to the physics state, apply perturbations, update options from GUI.
        viewer.sync()

        # Tick Speed # 
        time_until_next_step = 1#m.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0: time.sleep(time_until_next_step/8) # fabricates the sim 'FPS'

print(len(d.qpos))


if __name__ == "__main__":
  main()
