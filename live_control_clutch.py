import os
import sys
import time
import asyncio  # Async loops for sensor + robot control
import websockets  # WebSocket client for device communication
import orjson  # Fast JSON parsing
from scipy.interpolate import interp1d

sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))

from xarm.wrapper import XArmAPI

IP = "192.168.1.154"

SPEED = 30
ACC = 20
SERVO_SPEED = 2
SERVO_ACC = 2

INITPOSE = [200, 0, 200, 180, 0, 0]

# Button index for clutch toggle
CLUTCH_BUTTON_INDEX = 2

sensoring_range = dict(
    x = [-0.031, -0.31],
    y = [0.05, -0.31],
    z = [-0.11, 0.4]
)

controlling_range = dict(
    x = [110, 350],
    y = [-150, 150],
    z = [20, 300]
)


# Map sensor space (inverse3) into robot space (xArm).
m_x = interp1d(sensoring_range["x"], controlling_range["x"])
m_y = interp1d(sensoring_range["y"], controlling_range["y"])
m_z = interp1d(sensoring_range["z"], controlling_range["z"])

# Connect to the robot arm and put it into position control to start.
arm = XArmAPI(IP)
arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(state=0)



# Move to a safe starting pose.
arm.set_position(*INITPOSE, wait=True)

async def main():

    queue = asyncio.Queue()
    exit_event = asyncio.Event()

    # Run sensor input and robot control concurrently.
    task_haply = asyncio.create_task(haply_loop(queue, exit_event))
    task_controller = asyncio.create_task(controller_loop(queue, exit_event))

    # while True:

    #     await asyncio.sleep(1)


    # Block until one of the loops requests shutdown.
    await exit_event.wait()
    print("Ending main program...")

    task_haply.cancel()
    task_controller.cancel()

    await asyncio.gather(task_haply, task_controller, return_exceptions=True)
    print("ALL Programm ended.")



# Main asynchronous loop

async def haply_loop(queue: asyncio.Queue=None, exit_event:asyncio.Event=None):

    uri = 'ws://localhost:10001'  # WebSocket port for Inverse Service 3.1 json format
    first_message = True
    inverse3_device_id = None
    force = {"x": 0, "y": 0, "z": 0}  # Forces to send to the Inverse3 device.

    # Read sensor data from HaptX/Haply and forward it to the controller.
    async with websockets.connect(uri) as ws:
        while True:
            # Receive data from the device
            response = await ws.recv()
            data = orjson.loads(response)

            # Get devices list from the data
            inverse3_devices = data.get("inverse3", [])
            verse_grip_devices = data.get("wireless_verse_grip", [])

            # Get the first device from the list
            inverse3_data = inverse3_devices[0] if inverse3_devices else {}
            verse_grip_data = verse_grip_devices[0] if verse_grip_devices else {}

            # Handle the first message to get device IDs and extra information
            if first_message:

                first_message = False

                if not inverse3_data:
                    print("No Inverse3 device found.")
                    exit_event.set()
                    break
                if not verse_grip_data:
                    print("No Wireless Verse Grip device found.")

                # Store device ID for sending forces
                inverse3_device_id = inverse3_data.get("device_id")

                # Get handedness from Inverse3 device config data (only available in the first message)
                handedness = inverse3_devices[0].get("config", {}).get("handedness")

                print(f"Inverse3 device ID: {inverse3_device_id}, Handedness: {handedness}")

                if verse_grip_data:
                    print(f"Wireless Verse Grip device ID: {verse_grip_data.get('device_id')}")

            # Extract position, velocity from Inverse3 device state
            position = inverse3_data["state"].get("cursor_position", {})
            velocity = inverse3_data["state"].get("cursor_velocity", {})

            # Extract buttons and orientation from Wireless Verse Grip device state (or default if not found)
            buttons = verse_grip_data.get("state", {}).get("buttons", {})
            orientation = verse_grip_data.get("state", {}).get("orientation", {})

            #print(f"Position: {position} Velocity: {velocity} Orientation: {orientation} Buttons: {buttons}")
            await queue.put([position['x'], position['y'], position['z'], 
                             orientation['x'], orientation['y'], orientation['z'], 
                             buttons['a'],  buttons['b'], buttons['c']])

            # Prepare the force command message to send
            # Must send forces to receive state updates (even if forces are 0)
            request_msg = {
                "inverse3": [
                    {
                        "device_id": inverse3_device_id,
                        "commands": {
                            "set_cursor_force": {
                                "values": force
                            }
                        }
                    }
                ]
            }

            # Send the force command message to the server
            await ws.send(orjson.dumps(request_msg))

            if exit_event.is_set():
                print("Robot controller stopped unexpectely, ending sensoring loop...")
                break
        print("Sensoring loop ended.")

async def controller_loop(queue:asyncio.Queue, exit_event:asyncio.Event=None):

    first_action = True
    clutch_active = False
    prev_clutch_pressed = False
    pose_offset = [0, 0, 0, 0, 0, 0]
    frozen_pose = None
    last_tar_pose = None

    # Pull sensor messages and drive the robot at servo speed.
    while arm.connected and arm.state != 4:
        sensor_pose = await queue.get()
        base_pose, btn = get_base_pose(sensor_pose)

        clutch_pressed = btn[CLUTCH_BUTTON_INDEX]
        if clutch_pressed and not prev_clutch_pressed:
            clutch_active = not clutch_active
            if clutch_active:
                frozen_pose = list(last_tar_pose) if last_tar_pose is not None else list(base_pose)
                print("Clutch engaged: holding current pose.")
            else:
                if frozen_pose is None:
                    frozen_pose = list(last_tar_pose) if last_tar_pose is not None else list(base_pose)
                pose_offset = [frozen_pose[i] - base_pose[i] for i in range(6)]
                print("Clutch released: resuming motion from new Haply pose.")
        prev_clutch_pressed = clutch_pressed

        if clutch_active:
            tar_pose = frozen_pose
        else:
            tar_pose = [base_pose[i] + pose_offset[i] for i in range(6)]
        last_tar_pose = tar_pose

        # targ_pose = [ct_pose[0]-cart_pose[0],
        #           ct_pose[1]-cart_pose[1],
        #           ct_pose[2]+cart_pose[2],
        #           ct_pose[3]+cart_pose[3],
        #           ct_pose[4]+cart_pose[4],
        #           ct_pose[5]-cart_pose[5]
        #           ]

        if btn[0] == True:
            code = arm.open_lite6_gripper()
            #code = arm.open_lite6_gripper()
        if btn[1] == True:
            code = arm.close_lite6_gripper()
        if exit_event.is_set():

            # Return to a safe pose before exiting.
            arm.set_mode(0)
            arm.set_state(state=0)

            arm.set_position(*INITPOSE, speed=SPEED, mvacc=ACC, wait=True)
            print("Manual stop detected, ending controlling loop...")
            break

        if first_action:

            first_action = False

            # First move to the target in position mode, then switch to servo mode.
            temp_pose = [tar_pose[0], tar_pose[1], tar_pose[2], 180, 5, 0]

            arm.set_mode(0)
            arm.set_state(state=0)

            arm.set_position(*temp_pose, speed=SPEED, mvacc=ACC, wait=True)

            time.sleep(0.5)

            arm.set_mode(1)
            arm.set_state(0)
            time.sleep(0.1)

        #await arm.set_servo_cartesian(mvpose=targ_pose, speed=SPEED, mvacc=ACC)
        code = arm.set_servo_cartesian(mvpose=tar_pose, speed=SERVO_SPEED, mvacc=SERVO_ACC)
        time.sleep(0.01)
        if code != 0:
            exit_event.set()
    print("Controlling loop ended.")

def get_base_pose(incoming_msg:list):
    # Map sensor position into robot coordinates.
    x = m_x(incoming_msg[0])
    y = m_y(incoming_msg[1])
    z = m_z(incoming_msg[2])

    # Convert orientation to degrees and re-map axes.
    rx = incoming_msg[5]*180
    ry = incoming_msg[3]*180
    rz = incoming_msg[4]*180

    base_pose = [x,y,z,rx+180,5-ry,-rz]
    sensor_btn = [incoming_msg[6], incoming_msg[7], incoming_msg[8]]

    return base_pose, sensor_btn

# Run the asynchronous main function
if __name__ == "__main__":
    asyncio.run(main())
