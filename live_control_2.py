import os
import sys
import time
import asyncio  # To run async loops
import websockets  # Required for device communication
import orjson  # JSON reader for fast processing
from scipy.interpolate import interp1d

sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))

from xarm.wrapper import XArmAPI

IP = "192.168.1.154"

SPEED = 30
ACC = 20
SERVO_SPEED = 2
SERVO_ACC = 2

INITPOSE = [200, 0, 200, 180, 0, 0]

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



m_x = interp1d(sensoring_range["x"], controlling_range["x"])
m_y = interp1d(sensoring_range["y"], controlling_range["y"])
m_z = interp1d(sensoring_range["z"], controlling_range["z"])

arm = XArmAPI(IP)
arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(state=0)



arm.set_position(*INITPOSE, wait=True)

async def main():

    queue = asyncio.Queue()
    exit_event = asyncio.Event()

    task_haply = asyncio.create_task(haply_loop(queue, exit_event))
    task_controller = asyncio.create_task(controller_loop(queue, exit_event))

    # while True:

    #     await asyncio.sleep(1)



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

    # Haptic loop

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

            if buttons['c'] == True:
                print("Manual stop detected, ending sensoring loop...")
                exit_event.set()
                break
            elif exit_event.is_set():
                print("Robot controller stopped unexpectely, ending sensoring loop...")
                break
        print("Sensoring loop ended.")

async def controller_loop(queue:asyncio.Queue, exit_event:asyncio.Event=None):

    first_action = True

    while arm.connected and arm.state != 4:
        sensor_pose = await queue.get()
        tar_pose, btn = get_tar_pose(sensor_pose)
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



            arm.set_mode(0)

            arm.set_state(state=0)



            arm.set_position(*INITPOSE, speed=SPEED, mvacc=ACC, wait=True)

            print("Manual stop detected, ending controlling loop...")

            break



        if first_action:



            first_action = False



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



def get_tar_pose(incoming_msg:list):

    x = m_x(incoming_msg[0])

    y = m_y(incoming_msg[1])

    z = m_z(incoming_msg[2])



    rx = incoming_msg[5]*180

    ry = incoming_msg[3]*180

    rz = incoming_msg[4]*180



    tar_pose = [x,y,z,rx+180,5-ry,-rz]

    sensor_btn = [incoming_msg[6], incoming_msg[7], incoming_msg[8]]



    return tar_pose, sensor_btn



# Run the asynchronous main function

if __name__ == "__main__":

    asyncio.run(main())