import time, random
import mujoco, mujoco.viewer

#region CONSTANTS
RUNTIME = 60 # (seconds)
SPEED   = 120 # simulation step speed

#endregion CONSTANTS

#region data.qpos name references
'''
POS INDICIES

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

'''

joints = [
  "abdomen_z",
  "abdomen_y",
  "abdomen_x",
  "hip_x_right",
  "hip_z_right",
  "hip_y_right",
  "knee_right",
  "ankle_y_right",
  "ankle_x_right",
  "hip_x_left",
  "hip_z_left",
  "hip_y_left",
  "knee_left",
  "ankle_y_left",
  "ankle_x_left",
  "shoulder1_right",
  "shoulder2_right",
  "elbow_right",
  "shoulder1_left",
  "shoulder2_left", 
  "elbow_left"
]
#endregion data.qpos name references


model = mujoco.MjModel.from_xml_path('/Users/ctd/Downloads/humanoid-1.xml')

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
      data.ctrl[moving_jnt] += random.randint(0, 2) # decrease pos of random joint
      # if data.ctrl[moving_jnt] > 4: 
      #   data.ctrl = 0  # reset value if too high


      ### READ POSITIONAL DATA HERE ###
      positions = data.qpos #all positions
      positions = positions[7:] #don't know what the first 7 positions are, but they're not joints so ignore them

      abdomen_positions = positions[:3]
      print("abdomen positions: %d", abdomen_positions)
      
      for i, pos in enumerate(positions):
        print("[", i, "]", joints[i] ,f": {pos:{.3}g}")


      # Pick up changes to the physics state, apply perturbations, update options from GUI.
      viewer.sync()

      # Tick Speed # 
      time_until_next_step = (1/SPEED) - (time.time() - step_start)
      if time_until_next_step > 0:
        time.sleep(time_until_next_step)


if __name__ == "__main__":
  main()
