from pymodbus.client.sync import ModbusTcpClient
from pyRobotiqGripper import pyRobotiqGripper
import pygame
import numpy as np
import time

TRANSLATION_SPEED = 10
ROTATION_SPEED = 0
THRESHOLD = 2

def get_controller_input():
    pygame.event.pump()
    dx = int(-joystick.get_axis(1) * TRANSLATION_SPEED)  # left stick y axis
    dy = int(-joystick.get_axis(0) * TRANSLATION_SPEED)  # left stick x axis
    dz = int(((joystick.get_axis(2)+1)/2 - (joystick.get_axis(5)+1)/2) * TRANSLATION_SPEED)  # LT: Up, RT: Down
    rx = int(joystick.get_axis(3) * ROTATION_SPEED)
    ry = int(joystick.get_axis(4) * ROTATION_SPEED)
    rz = int(joystick.get_axis(4) * ROTATION_SPEED)
    gripper_open = joystick.get_button(0)
    gripper_close = joystick.get_button(1)
    return dx, dy, dz, rx, ry, rz, gripper_open, gripper_close

def angle_diff(a, b):
    """
    To consider the angle warping
    """
    diff = a - b
    diff = (diff + 180) % 360 - 180
    return diff

def update_pose_with_threshold(target, current):
    """
    target: [x, y, z, rx, ry, rz]
    current: [x, y, z, rx, ry, rz]
    resutn updated: [x, y, z, rx, ry, rz]
    """
    updated = current.copy()
    
    # position
    pos_diff = np.abs(target[:3] - current[:3])
    pos_mask = pos_diff > THRESHOLD
    updated[:3][pos_mask] = target[:3][pos_mask]

    # rotation
    rot_diff = np.abs(angle_diff(target[3:], current[3:]))
    rot_mask = rot_diff > THRESHOLD
    updated[3:][rot_mask] = target[3:][rot_mask]

    return updated

pygame.init()
pygame.joystick.init()
if pygame.joystick.get_count() < 1:
    raise RuntimeError("Can't find any joystick")
joystick = pygame.joystick.Joystick(0)
joystick.init()



grip = pyRobotiqGripper.RobotiqGripper("/dev/ttyUSB0")
grip.resetActivate()
grip.goTo(0, 255)

def uint16_to_int16(tbl):
    return [int(x) if x < 32768 else int(x - 65536) for x in tbl]

def int16_to_uint16(tbl):
    return [int(x) if x >= 0 else int(x + 65536) for x in tbl]

prev_gripper_state = None
initial_flag = True
while True:
    client = ModbusTcpClient("192.168.1.20", port=1502)

    if client.connect():
        print("Modbus TCP server connected!")
        try:
            while True:
                response = client.read_holding_registers(500, 6)
                if response is None or response.isError():
                    print("Error: No response from Modbus server.")
                    time.sleep(1)
                    continue

                robot_pose = np.array(uint16_to_int16(response.registers))
                print(f"Robot pose: {robot_pose}")
                
                dx, dy, dz, rx, ry, rz, gripper_open, gripper_close = get_controller_input()
                if initial_flag:
                    target_position = robot_pose + np.array([dx, dy, dz, rx, ry, rz])
                    initial_flag = False
                else:
                    target_position += np.array([dx, dy, dz, rx, ry, rz])
                print(f"delta: {[dx, dy, dz, rx, ry, rz]}")

                print(f"Target pose: {target_position}")
 
                target_pose = int16_to_uint16(target_position.tolist())

                client.write_registers(400, target_pose)
                
                if gripper_open and prev_gripper_state != "open":
                    grip.goTo(255, 255)
                    prev_gripper_state = "open"
                elif gripper_close and prev_gripper_state != "close":
                    grip.goTo(0, 255)
                    prev_gripper_state = "close"
                
                # 10 Hz
                time.sleep(0.2)
        except KeyboardInterrupt:
            print("User interrupted. Exiting...")
            break
    else:
        print("Modbus TCP server connection failed! Retrying in 1 second...")
        time.sleep(1)
    
    client.close()
