import time
import mujoco
import mujoco.viewer
from threading import Thread
import threading

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py_bridge import UnitreeSdk2Bridge, ElasticBand

import config
import cv2
import zmq
import time
import logging_mp

logger_mp = logging_mp.get_logger(__name__, level=logging_mp.DEBUG)

locker = threading.Lock()
image = None

mj_model = mujoco.MjModel.from_xml_path(config.ROBOT_SCENE)
mj_data = mujoco.MjData(mj_model)
width = 640
height = 480

if config.ENABLE_ELASTIC_BAND:
    elastic_band = ElasticBand()
    if config.ROBOT == "h1" or config.ROBOT == "g1":
        band_attached_link = mj_model.body("torso_link").id
    else:
        band_attached_link = mj_model.body("base_link").id
    viewer = mujoco.viewer.launch_passive(
        mj_model, mj_data, key_callback=elastic_band.MujuocoKeyCallback
    )
else:
    viewer = mujoco.viewer.launch_passive(mj_model, mj_data)

mj_model.opt.timestep = config.SIMULATE_DT
num_motor_ = mj_model.nu
dim_motor_sensor_ = 3 * num_motor_

time.sleep(0.2)


def SimulationThread():
    global mj_data, mj_model, image

    ChannelFactoryInitialize(config.DOMAIN_ID, config.INTERFACE)
    unitree = UnitreeSdk2Bridge(mj_model, mj_data)

    if config.USE_JOYSTICK:
        unitree.SetupJoystick(device_id=0, js_type=config.JOYSTICK_TYPE)
    if config.PRINT_SCENE_INFORMATION:
        unitree.PrintSceneInformation()

    renderer = mujoco.Renderer(mj_model, height, width)
    step_count = 0

    while viewer.is_running():
        step_start = time.perf_counter()

        locker.acquire()

        if config.ENABLE_ELASTIC_BAND:
            if elastic_band.enable:
                mj_data.xfrc_applied[band_attached_link, :3] = elastic_band.Advance(
                    mj_data.qpos[:3], mj_data.qvel[:3]
                )
        mujoco.mj_step(mj_model, mj_data)
        # TODO: this will only update the image every 10 steps, regardless of the physics step rate
        # in the future ,we could make the image update rate constant and calculate the number of steps needed to update the image
        if step_count % 10 == 0:
            renderer.update_scene(mj_data, camera="head_cam")
            image = renderer.render()
    
        locker.release()

        time_until_next_step = mj_model.opt.timestep - (
            time.perf_counter() - step_start
        )
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
        step_count += 1


def PhysicsViewerThread():
    # Copied from image_server.py
    global image
    # Set ZeroMQ context and socket
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.bind(f"tcp://*:5555")

    while viewer.is_running():
        locker.acquire()
        viewer.sync()
        if image is not None:
            # Write image to shared memory similar to image_server.py
            ret, buffer = cv2.imencode('.jpg', image)
            if not ret:
                logger_mp.error("[Image Server] Frame imencode is failed.")
                continue

            jpg_bytes = buffer.tobytes()

            message = jpg_bytes

            socket.send(message)

        locker.release()
        time.sleep(config.VIEWER_DT)

    else:
        print("viewer is not running")

if __name__ == "__main__":
    viewer_thread = Thread(target=PhysicsViewerThread)
    sim_thread = Thread(target=SimulationThread)

    viewer_thread.start()
    sim_thread.start()
