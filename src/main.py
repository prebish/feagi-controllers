import time, random
import mujoco, mujoco.viewer

#region CONSTANTS
RUNTIME = 60 # (seconds)
SPEED   = 120 # simulation step speed

#endregion CONSTANTS

#region data.qpos name references
abdomen_z   = 7
abdomen_y   = 8
abdomen_x   = 9
hip_x_right   = 10
hip_z_right   = 11
hip_y_right   = 12
knee_right    = 13
ankle_y_right = 14
ankle_x_right = 15
hip_x_left    = 16
hip_z_left    = 17
hip_y_left    = 18
knee_left     = 19
ankle_y_left  = 20
ankle_x_left  = 21
shoulder1_right = 22
shoulder2_right = 23
elbow_right     = 24
shoulder1_left  = 25
shoulder2_left  = 26
elbow_left      = 27
#endregion data.qpos name references

model = mujoco.MjModel.from_xml_path('mujoco/model/humanoid/humanoid.xml')
data  = mujoco.MjData(model)

def main():
  with mujoco.viewer.launch_passive(model, data) as viewer:
    start_time = time.time()

    while viewer.is_running() and time.time() - start_time < RUNTIME:
      step_start = time.time()

      # steps the simulation forward 'tick'
      mujoco.mj_step(model, data)

      # make model spaz out
      moving_jnt = random.randint(0, len(data.ctrl)-1)
      data.ctrl[moving_jnt] += random.randint(0, 2) # increase pos of random joint
      data.ctrl[moving_jnt] -= random.randint(0, 2) # decrease pos of random joint
      if data.ctrl[moving_jnt] > 4: 
        data.ctrl = 0  # reset value if too high


      ### READ POSITIONAL DATA HERE ###


      ###                           ###


      # Pick up changes to the physics state, apply perturbations, update options from GUI.
      viewer.sync()

      # Tick Speed # 
      time_until_next_step = (1/SPEED) - (time.time() - step_start)
      if time_until_next_step > 0:
        time.sleep(time_until_next_step)


if __name__ == "__main__":
  main()
