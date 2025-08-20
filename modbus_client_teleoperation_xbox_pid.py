from pymodbus.client.sync import ModbusTcpClient
from pyRobotiqGripper import pyRobotiqGripper
import pygame
import numpy as np
import time

class SimplePIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0
    
    def compute(self, error):
        self.integral += error
        derivative = error - self.prev_error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output

# PID 컨트롤러 설정
pid_x = SimplePIDController(0.3, 0.01, 0.05)
pid_y = SimplePIDController(0.3, 0.01, 0.05)
pid_z = SimplePIDController(0.3, 0.01, 0.05)
pid_rx = SimplePIDController(0.3, 0.01, 0.05)
pid_ry = SimplePIDController(0.3, 0.01, 0.05)
pid_rz = SimplePIDController(0.3, 0.01, 0.05)

def get_controller_input():
    pygame.event.pump()
    dx = joystick.get_axis(0) * TRANSLATION_SPEED  
    dy = joystick.get_axis(1) * TRANSLATION_SPEED  
    dz = ((joystick.get_axis(2)+1)/2 - (joystick.get_axis(5)+1)/2) * TRANSLATION_SPEED  
    rx = joystick.get_axis(3) * ROTATION_SPEED  
    ry = joystick.get_axis(4) * ROTATION_SPEED  
    rz = joystick.get_axis(4) * ROTATION_SPEED  
    gripper_open = joystick.get_button(0)  
    gripper_close = joystick.get_button(1)  
    return dx, dy, dz, rx, ry, rz, gripper_open, gripper_close

pygame.init()
pygame.joystick.init()
if pygame.joystick.get_count() < 1:
    raise RuntimeError("Can't find any joystick")
joystick = pygame.joystick.Joystick(0)
joystick.init()

TRANSLATION_SPEED = 1
ROTATION_SPEED = 0.001

grip = pyRobotiqGripper.RobotiqGripper("/dev/ttyUSB0")
grip.resetActivate()
grip.goTo(0, 255)

def uint16_to_int16(tbl):
    return [int(x) if x < 32768 else int(x - 65536) for x in tbl]

def int16_to_uint16(tbl):
    return [int(x) if x >= 0 else int(x + 65536) for x in tbl]

prev_gripper_state = None

while True:
    client = ModbusTcpClient("192.168.1.21", port=1502)

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
                
                pid_dx_output = pid_x.compute(target_position[0] - robot_pose[0])
                pid_dy_output = pid_y.compute(target_position[1] - robot_pose[1])
                pid_dz_output = pid_z.compute(target_position[2] - robot_pose[2])
                pid_rx_output = pid_rx.compute(target_position[3] - robot_pose[3])
                pid_ry_output = pid_ry.compute(target_position[4] - robot_pose[4])
                pid_rz_output = pid_rz.compute(target_position[5] - robot_pose[5])

                # 제어된 위치 계산
                controlled_position = np.array(robot_pose) + np.array([
                    pid_dx_output, pid_dy_output, pid_dz_output, pid_rx_output, pid_ry_output, pid_rz_output
                ])
                target_pose = int16_to_uint16(controlled_position.tolist())

                print(f"Target pose: {controlled_position}")
                client.write_registers(400, target_pose)
                
                if gripper_open and prev_gripper_state != "open":
                    grip.goTo(255, 255)
                    prev_gripper_state = "open"
                elif gripper_close and prev_gripper_state != "close":
                    grip.goTo(0, 255)
                    prev_gripper_state = "close"
                
                # fps 60
                time.sleep(1/60)
        except KeyboardInterrupt:
            print("User interrupted. Exiting...")
            break
    else:
        print("Modbus TCP server connection failed! Retrying in 1 second...")
        time.sleep(1)
    
    client.close()
