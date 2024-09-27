import mujoco
import mediapy as media
import mujoco.viewer


xml = """
<mujoco>
  <worldbody>
    
    <geom name="red_box" type="box" size=".2 .2 .2" rgba="1 0 0 1"/>
    <geom name="green_sphere" pos=".2 .2 .2" size=".1" rgba="0 1 0 1"/>
  </worldbody>
</mujoco>
"""
model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)

print('id of "green_sphere": ', model.geom('green_sphere').id)
print('name of geom 1: ', model.geom(0).name)
print('name of body 0: ', model.body(0).name)
print(data.geom_xpos)
mujoco.mj_kinematics(model, data)
print('raw access:\n', data.geom_xpos)

# MjData also supports named access:
print('\nnamed access:\n', data.geom('green_sphere').xpos)

""" with mujoco.Renderer(model) as renderer:
  media.show_image(renderer.render()) """
""" with mujoco.Renderer(model) as renderer:
  mujoco.mj_forward(model, data)
  renderer.update_scene(data)
  media.show_image(renderer.render()) """

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
    # mj_step can be replaced with code that also evaluates
    # a policy and applies a control signal before stepping the physics.
    #mujoco.mj_step(m, d)

    # Get position data (joint positions)
    # qpos is the generalized position vector (contains joint positions)


        # Pick up changes to the physics state, apply perturbations, update options from GUI.
        viewer.sync()


   