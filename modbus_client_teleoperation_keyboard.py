from pymodbus.client.sync import ModbusTcpClient
from pyRobotiqGripper import pyRobotiqGripper
import pygame
import numpy as np
import time
from termcolor import cprint

# Set up speed parameters
TRANSLATION_SPEED = 5
ROTATION_SPEED = 0.001

def uint16_to_int16(tbl):
    return [int(x) if x < 32768 else int(x - 65536) for x in tbl]

def int16_to_uint16(tbl):
    return [int(x) if x >= 0 else int(x + 65536) for x in tbl]

# Initialize pygame for keyboard input
pygame.init()
pygame.display.set_mode((100, 100)) # Initialize a window for pygame

# Gripper initialization
grip = pyRobotiqGripper.RobotiqGripper("/dev/ttyUSB0")
grip.resetActivate()
grip.goTo(0, 255)

prev_gripper_state = None
target_position = np.array([0, 0, 0, 0, 0, 0])

while True:
    client = ModbusTcpClient("192.168.1.20", port=1502)

    if client.connect():
        print("Modbus TCP server connected!")
        try:
            while True:
                # Read current pose
                response = client.read_holding_registers(500, 6)
                if response.isError():
                    cprint("Failed to read registers", "yellow")
                    continue
                robot_pose = np.array(uint16_to_int16(response.registers))
                cprint(f"Robot pose: {robot_pose}", "green")

                # Get keyboard input
                dx, dy, dz, rx, ry, rz = 0, 0, 0, 0, 0, 0
                gripper_open = False
                gripper_close = False

                for event in pygame.event.get():
                    if event.type == pygame.KEYDOWN:
                        if event.key == pygame.K_UP:
                            dx += TRANSLATION_SPEED
                        elif event.key == pygame.K_DOWN:
                            dx -= TRANSLATION_SPEED
                        elif event.key == pygame.K_RIGHT:
                            dy += TRANSLATION_SPEED
                        elif event.key == pygame.K_LEFT:
                            dy -= TRANSLATION_SPEED
                        elif event.key == pygame.K_q:
                            dz += TRANSLATION_SPEED
                        elif event.key == pygame.K_e:
                            dz -= TRANSLATION_SPEED
                        elif event.key == pygame.K_w:
                            rx += ROTATION_SPEED
                        elif event.key == pygame.K_s:
                            rx -= ROTATION_SPEED
                        elif event.key == pygame.K_a:
                            ry += ROTATION_SPEED
                        elif event.key == pygame.K_d:
                            ry -= ROTATION_SPEED
                        elif event.key == pygame.K_z:
                            rz += ROTATION_SPEED
                        elif event.key == pygame.K_c:
                            rz -= ROTATION_SPEED
                        elif event.key == pygame.K_o:
                            gripper_open = True
                        elif event.key == pygame.K_p:
                            gripper_close = True

                # Update target pose
                delta = np.array([dx, dy, dz, rx, ry, rz])
                cprint(f"Delta: {delta}", "green")

                target_position = robot_pose + delta
                cprint(f"Target pose: {target_position}", "green")
                target_pose_uint16 = int16_to_uint16(target_position.astype(int).tolist())

                # Send target pose to robot
                client.write_registers(400, target_pose_uint16)

                # Gripper control
                if gripper_open and prev_gripper_state != "open":
                    grip.goTo(255, 255)
                    prev_gripper_state = "open"
                elif gripper_close and prev_gripper_state != "close":
                    grip.goTo(0, 255)
                    prev_gripper_state = "close"

                time.sleep(0.5)

        except KeyboardInterrupt:
            print("User interrupted. Exiting...")
            break
    else:
        print("Modbus TCP server connection failed! Retrying in 1 second...")
        time.sleep(1)

    client.close()
