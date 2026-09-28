import time

import numpy as np
import mujoco
import mujoco.viewer

model = mujoco.MjModel.from_xml_path("scene.xml")
data = mujoco.MjData(model)


def set_body_command(vx_b, vy_b, wz, dribbler):
    yaw = data.joint("yaw").qpos[0]
    c, s = np.cos(yaw), np.sin(yaw)
    data.actuator("vx").ctrl[0] = c * vx_b - s * vy_b
    data.actuator("vy").ctrl[0] = s * vx_b + c * vy_b
    data.actuator("wz").ctrl[0] = wz
    data.actuator("dribbler").ctrl[0] = dribbler


with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        t0 = time.time()

        # If the ball is pushed away instead of pulled in, flip the sign of the dribbler value.
        drive = 0.2 if data.joint("x").qpos[0] < 0.40 else 0.0
        set_body_command(drive, 0.0, 0.0, 600.0)

        mujoco.mj_step(model, data)
        viewer.sync()

        remaining = model.opt.timestep - (time.time() - t0)
        if remaining > 0:
            time.sleep(remaining)