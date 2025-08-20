from pymodbus.client.sync import ModbusTcpClient
from pyRobotiqGripper import pyRobotiqGripper
import pygame
import numpy as np
import time

TRANSLATION_SPEED = 50
ROTATION_SPEED = 0.001

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
                
                target_position = robot_pose + np.array([dx, dy, dz, rx, ry, rz])

                target_pose = int16_to_uint16(target_position.tolist())

                print(f"Target pose: {target_position}")
                client.write_registers(400, target_pose)
                
                if gripper_open and prev_gripper_state != "open":
                    grip.goTo(255, 255)
                    prev_gripper_state = "open"
                elif gripper_close and prev_gripper_state != "close":
                    grip.goTo(0, 255)
                    prev_gripper_state = "close"
                
                # 10 Hrz
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("User interrupted. Exiting...")
            break
    else:
        print("Modbus TCP server connection failed! Retrying in 1 second...")
        time.sleep(1)
    
    client.close()
