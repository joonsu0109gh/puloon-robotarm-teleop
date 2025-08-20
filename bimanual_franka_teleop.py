#!/usr/bin/env python3

import numpy as np
import rospy
from geometry_msgs.msg import Point, Quaternion
from std_msgs.msg import Bool, Float32
import pybullet as p
import pybullet_data
from tf.transformations import quaternion_multiply, quaternion_inverse
from frankapy import FrankaArm, SensorDataMessageType
from frankapy.proto_utils import sensor_proto2ros_msg, make_sensor_group_msg
from frankapy.proto import JointPositionSensorMessage
from franka_interface_msgs.msg import SensorDataGroup
from scipy.spatial.transform import Rotation as R
from termcolor import cprint
import traceback
import yaml

"""
Simple PID controller class.

Adjust the following gains to control the arm:
- Proportional gain (Kp):
    - Determines response to the current error.
    - Higher values lead to faster response but may cause overshoot and oscillations.
- Integral gain (Ki):
    - Eliminates steady-state error by accumulating past errors.
    - Higher values reduce long-term error but may cause oscillation and instability.
- Derivative gain (Kd):
    - Predicts future error based on the rate of change of the current error.
    - Acts as a damping force to prevent overshoot.
    - Higher values smooth movement and prevent oscillations but can slow down the response.

References:
- DigiKey: https://www.digikey.com/en/maker/projects/introduction-to-pid-controllers/763a6dca352b4f2ba00adde46445ddeb
- UMich: https://ctms.engin.umich.edu/CTMS/index.php?example=Introduction&section=ControlPID
"""
class PIDControllerSimple:
    def __init__(self, Kp=0.3, Ki=0.001, Kd=0.01, output_limit=None):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.previous_error = np.zeros(3)
        self.integral = np.zeros(3)
        self.output_limit = output_limit

    def reset(self):
        """Reset the PID controller's integral and previous error."""
        self.previous_error = np.zeros(3)
        self.integral = np.zeros(3)

    def compute(self, error, dt):
        """
        Compute the PID output.

        Args:
            error (np.ndarray): The current error.
            dt (float): Time step.

        Returns:
            np.ndarray: PID controller output.
        """
        proportional = self.Kp * error

        self.integral += error * dt
        integral = self.Ki * self.integral

        derivative = self.Kd * (error - self.previous_error) / dt
        self.previous_error = error

        output = proportional + integral + derivative

        if self.output_limit is not None:
            output = np.clip(output, -self.output_limit, self.output_limit)

        return output

    def get_gains(self):
        """Return the current PID gains."""
        return self.Kp, self.Ki, self.Kd

"""
Bimanual VR Teleoperation Class.

Subscribes to VR controller data and controls Franka arms accordingly.

Execution Controls:
- Press A to start right arm teleop.
- Press X to start left arm teleop.
- Press B to pause both teleops.
- Press index trigger to lower P gain (slower response).
"""
class VRControllerFrankaTeleop:
    def __init__(self, config_path, robot_num=1):
        # Load configuration
        self.config_path = config_path
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)
        self.params = self.config["bimanualteleop"]
        
        self.robot_num = robot_num

        # Initialize PyBullet with Franka URDF
        p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        urdf_path = self.params["urdf_path"]
        self.robot_id_right = p.loadURDF(urdf_path, useFixedBase=True)
        self.robot_id_left = p.loadURDF(urdf_path, useFixedBase=True)

        # Franka impedance control parameters
        self.speed_linear = 0.0002
        self.speed_rot = 0.001
        self.linear_impedance = 6000
        self.rot_impedance = 15
        self.cartesian_impedances = [self.linear_impedance] * 3 + [self.rot_impedance] * 3

        # ROS Publishers
        if self.robot_num == 1:
            self.fa = FrankaArm(with_gripper=False, robot_num=1)
            self.fa_pub = rospy.Publisher("franka_ros_interface/sensor", SensorDataGroup, queue_size=1000)
        elif self.robot_num == 2:
            self.fa = FrankaArm(with_gripper=False, robot_num=2)
            self.fa_pub = rospy.Publisher("franka_ros_interface/sensor_2", SensorDataGroup, queue_size=1000)
        else:
            raise ValueError("Invalid robot number. Please specify 1 or 2.")
        

        # Teleoperation state variables
        self.teleop_position = None
        self.teleop_quat_right = [1, 0, 0, 0]
        self.teleop_quat_left = [1, 0, 0, 0]
        self.is_controlling_right = False
        self.is_controlling_left = False
        self.reference_position_right = None
        self.reference_position_left = None
        self.reference_rotation_right = None
        self.reference_rotation_left = None

        # Initialize PID controllers for position and rotation
        self.position_pid_right = PIDControllerSimple(Kp=0.1, Ki=0.0, Kd=0.01)
        self.rotation_pid_right = PIDControllerSimple(Kp=0.02, Ki=0.0, Kd=0.015)
        self.position_pid_left = PIDControllerSimple(Kp=0.1, Ki=0.0, Kd=0.01)
        self.rotation_pid_left = PIDControllerSimple(Kp=0.02, Ki=0.0, Kd=0.015)

        # Set safety boundaries
        self.min_pos = self.params["min_safe_box"]
        self.max_pos = self.params["max_safe_box"]

        # Joint smoothing parameter and control rate
        self.joint_smoothing = self.params["joint_smoothing"]
        pub_rate = self.params["pub_rate"]
        self.controlrate = rospy.Rate(pub_rate)

        # Initialize robots
        try:
            self.init_robots_right()
        except Exception as e:
            rospy.logerr(f"[Initialization] Right robot initialization error: {e}")

        self.init_robots_left()

        # ROS Subscribers for Right VR Controller
        rospy.Subscriber("/vr/right_controller/position", Point, self.update_right_position)
        rospy.Subscriber("/vr/right_controller/rotation", Quaternion, self.update_right_rotation)
        rospy.Subscriber("/vr/right_controller/a_button", Bool, self.update_right_a_button)
        rospy.Subscriber("/vr/right_controller/b_button", Bool, self.update_right_b_button)
        rospy.Subscriber("/vr/right_controller/index_trigger", Float32, self.update_right_index_trigger)

        # ROS Subscribers for Left VR Controller
        rospy.Subscriber("/vr/left_controller/position", Point, self.update_left_position)
        rospy.Subscriber("/vr/left_controller/rotation", Quaternion, self.update_left_rotation)
        rospy.Subscriber("/vr/left_controller/x_button", Bool, self.update_left_x_button)
        rospy.Subscriber("/vr/left_controller/y_button", Bool, self.update_left_y_button)
        rospy.Subscriber("/vr/left_controller/index_trigger", Float32, self.update_left_index_trigger)

        # ROS Subscribers for Headset Data
        rospy.Subscriber("/vr/headset/position", Point, self.update_headset_position)
        rospy.Subscriber("/vr/headset/rotation", Quaternion, self.update_headset_rotation)

        # Initialize controller and headset data
        self.right_controller_position = None
        self.right_controller_rotation = None
        self.right_a_button = False
        self.right_b_button = False
        self.right_index_trigger = 0.0

        self.left_controller_position = None
        self.left_controller_rotation = None
        self.left_x_button = False
        self.left_y_button = False
        self.left_index_trigger = 0.0

        self.headset_position = None
        self.headset_rotation = None

        # Transformation matrices
        self.fixed_headset_matrix = np.eye(4)
        self.right_controller_matrix = np.eye(4)
        self.left_controller_matrix = np.eye(4)

        self.headset_fixed = False
        self.fixed_right_controller_matrix = None
        self.fixed_right_controller_position = None
        self.fixed_right_controller_rotation = None
        self.fixed_left_controller_matrix = None
        self.fixed_left_controller_position = None
        self.fixed_left_controller_rotation = None

        # Time tracking for PID
        self.prev_time = rospy.Time.now()

        # Teleoperation flags
        self.teleop_start = False

        # Data collection flags
        self.start_data_collection = False
        self.end_data_collection = False

        # Robot reinitialization flags
        self.robot_1_center = self.params['reinitialize_robot_1_center']
        self.robot_2_center = self.params['reinitialize_robot_2_center']
        self.robot_reinitialize = False


    """
    Initialize the right robot arm.
    - Resets joints.
    - Sets safety boundaries.
    - Configures control parameters.
    """
    def init_robots_right(self):
        print("\n=======================================================================")
        cprint("Initializing right robot (If the initialization is not completed within 15 seconds, please restart the script)", "yellow")
        try:
            # Reset joints with a duration
            self.fa_right.reset_joints(duration=5)

            # Set initial pose
            self.robot_start_pose_right = self.fa_right.get_pose()
            self.teleop_position_right = np.array(self.robot_start_pose_right.translation)

            cprint("Right robot initialized", "green")
        except Exception as e:
            rospy.logerr(f"[Initialization] Robot initialization error: {e}")
            raise Exception(f"Robot initialization error: {e}")

    """
    Initialize the left robot arm.
    - Resets joints.
    - Sets safety boundaries.
    - Configures control parameters.
    """
    def init_robots_left(self):
        print("\n=======================================================================")
        cprint("Initializing left robot (If the initialization is not completed within 15 seconds, please restart the script)", "yellow")
        try:
            # Reset joints with a duration
            self.fa_left.reset_joints(duration=5)

            # Set initial pose
            self.robot_start_pose_left = self.fa_left.get_pose()
            self.teleop_position_left = np.array(self.robot_start_pose_left.translation)

            cprint("Left robot initialized", "green")
        except Exception as e:
            rospy.logerr(f"[Initialization] Robot initialization error: {e}")
            raise Exception(f"Robot initialization error: {e}")

    # ROS Subscriber Callback Functions

    def update_right_index_trigger(self, msg):
        """Update the right controller's index trigger value."""
        self.right_index_trigger = msg.data

    def update_left_index_trigger(self, msg):
        """Update the left controller's index trigger value."""
        self.left_index_trigger = msg.data

    def update_right_position(self, msg):
        """Update the right controller's position."""
        self.right_controller_position = np.array([msg.x, msg.y, msg.z])

    def update_right_rotation(self, msg):
        """Update the right controller's rotation."""
        self.right_controller_rotation = np.array([msg.x, msg.y, msg.z, msg.w])

    def update_right_a_button(self, msg):
        """Update the state of the right controller's A button."""
        self.right_a_button = msg.data

    def update_right_b_button(self, msg):
        """Update the state of the right controller's B button."""
        self.right_b_button = msg.data

    def update_left_position(self, msg):
        """Update the left controller's position."""
        self.left_controller_position = np.array([msg.x, msg.y, msg.z])

    def update_left_rotation(self, msg):
        """Update the left controller's rotation."""
        self.left_controller_rotation = np.array([msg.x, msg.y, msg.z, msg.w])

    def update_left_x_button(self, msg):
        """Update the state of the left controller's X button."""
        self.left_x_button = msg.data

    def update_left_y_button(self, msg):
        """Update the state of the left controller's Y button."""
        self.left_y_button = msg.data

    def update_headset_position(self, msg):
        """Update the headset's position."""
        self.headset_position = np.array([msg.x, msg.y, msg.z])

    def update_headset_rotation(self, msg):
        """Update the headset's rotation."""
        self.headset_rotation = np.array([msg.x, msg.y, msg.z, msg.w])

    """
    Calculate Inverse Kinematics (IK) using PyBullet and URDF.

    Args:
        pos (np.ndarray): Target position.
        quat (list): Target orientation as a quaternion.
        arm (FrankaArm): The FrankaArm instance (right or left).

    Returns:
        list: Joint positions calculated by IK.
    """
    def calculate_ik(self, pos, quat, arm):
        joints = arm.get_joints()
        robot_id = self.robot_id_right if arm == self.fa_right else self.robot_id_left
        for i in range(7):
            p.resetJointState(robot_id, i, joints[i])
        ik_solution = p.calculateInverseKinematics(
            bodyUniqueId=robot_id,
            endEffectorLinkIndex=9,
            targetPosition=pos,
            targetOrientation=quat
        )
        return ik_solution

    """
    Smooth joint movements to avoid jerky motions.

    Args:
        current_joints (list): Current joint positions.
        target_joints (list): Target joint positions.

    Returns:
        list: Smoothed joint positions.
    """
    def smooth_joints(self, current_joints, target_joints):
        return current_joints + self.joint_smoothing * (np.array(target_joints) - np.array(current_joints))

    """
    Calculate quaternion error between target and current orientations.

    Args:
        q_target (np.ndarray): Target quaternion.
        q_current (np.ndarray): Current quaternion.

    Returns:
        np.ndarray: Rotational vector representing the error.
    """
    def quaternion_error(self, q_target, q_current):
        q_error = quaternion_multiply(q_target, quaternion_inverse(q_current))
        return R.from_quat(q_error).as_rotvec()  # Convert to rotational vector (axis-angle)

    """
    Main teleoperation loop.

    Handles user inputs and controls the robot arms accordingly.
    """
    def do_teleop(self):
        try:
            print("=======================================================================")
            print("Turn on VR app and:")
            print("- Press Y to reinitialize robot arm and fix the viewpoint")
            print("- Press A to start robot arm control and collect data")
            print("- Press B to pause teleoperation")
            print("=======================================================================")

            # Initialize reference poses
            self.robot_start_pose_right = self.fa_right.get_pose()
            self.robot_start_pose_left = self.fa_left.get_pose()
            self.teleop_position_right = np.array(self.robot_start_pose_right.translation)
            self.teleop_position_left = np.array(self.robot_start_pose_left.translation)

            while not rospy.is_shutdown():
                try:
                    # Calculate time step for PID
                    current_time = rospy.Time.now()
                    dt = (current_time - self.prev_time).to_sec()
                    self.prev_time = current_time

                    # Handle Y button event (reinitialize robot)
                    if self.left_y_button and self.headset_position is not None and self.headset_rotation is not None:
                        if not self.headset_fixed:
                            # Fix headset transformation matrix
                            self.fixed_headset_matrix[:3, 3] = self.headset_position.copy()
                            self.fixed_headset_matrix[:3, :3] = R.from_quat(self.headset_rotation.copy()).as_matrix()
                            self.headset_fixed = True

                        if not self.robot_reinitialize:
                            # Update controller matrices
                            self.right_controller_matrix[:3, 3] = self.right_controller_position
                            self.right_controller_matrix[:3, :3] = R.from_quat(self.right_controller_rotation).as_matrix()
                            self.left_controller_matrix[:3, 3] = self.left_controller_position
                            self.left_controller_matrix[:3, :3] = R.from_quat(self.left_controller_rotation).as_matrix()

                            # Calculate fixed controller matrices relative to the headset
                            self.fixed_right_controller_matrix = np.linalg.inv(self.fixed_headset_matrix) @ self.right_controller_matrix
                            self.fixed_left_controller_matrix = np.linalg.inv(self.fixed_headset_matrix) @ self.left_controller_matrix

                            # Extract fixed positions and rotations
                            self.fixed_right_controller_position = self.fixed_right_controller_matrix[:3, 3]
                            self.fixed_right_controller_rotation = R.from_matrix(self.fixed_right_controller_matrix[:3, :3]).as_quat()

                            self.fixed_left_controller_position = self.fixed_left_controller_matrix[:3, 3]
                            self.fixed_left_controller_rotation = R.from_matrix(self.fixed_left_controller_matrix[:3, :3]).as_quat()

                            cprint("Reinitializing robot", "yellow")

                            # Set target poses for both robots
                            robot_1_pose = self.fa_right.get_pose()
                            robot_1_pose.translation = self.robot_1_center
                            print(f"Robot 1 target pose: {self.robot_1_center}")

                            robot_2_pose = self.fa_left.get_pose()
                            robot_2_pose.translation = self.robot_2_center
                            print(f"Robot 2 target pose: {self.robot_2_center}")

                            # Move robots to target poses
                            cprint("Moving to controller pose", "yellow")
                            self.fa_left.goto_pose(robot_2_pose, 3)
                            self.fa_left.goto_joints(
                                self.fa_left.get_joints(),
                                duration=10000,
                                dynamic=True,
                                buffer_time=10,
                                cartesian_impedances=self.cartesian_impedances
                            )

                            self.fa_right.goto_pose(robot_1_pose, 3)
                            self.fa_right.goto_joints(
                                self.fa_right.get_joints(),
                                duration=10000,
                                dynamic=True,
                                buffer_time=10,
                                cartesian_impedances=self.cartesian_impedances
                            )

                            # Update reference poses after reinitialization
                            self.robot_start_pose_right = self.fa_right.get_pose()
                            self.robot_start_pose_left = self.fa_left.get_pose()
                            self.teleop_position_right = np.array(self.robot_start_pose_right.translation)
                            self.teleop_position_left = np.array(self.robot_start_pose_left.translation)

                            self.robot_reinitialize = True
                            cprint("Robot reinitialized", "green")

                    # Update controller matrices if headset is fixed
                    if self.headset_fixed:
                        self.right_controller_matrix[:3, 3] = self.right_controller_position
                        self.right_controller_matrix[:3, :3] = R.from_quat(self.right_controller_rotation).as_matrix()
                        self.left_controller_matrix[:3, 3] = self.left_controller_position
                        self.left_controller_matrix[:3, :3] = R.from_quat(self.left_controller_rotation).as_matrix()

                        self.fixed_right_controller_matrix = np.linalg.inv(self.fixed_headset_matrix) @ self.right_controller_matrix
                        self.fixed_left_controller_matrix = np.linalg.inv(self.fixed_headset_matrix) @ self.left_controller_matrix

                        self.fixed_right_controller_position = self.fixed_right_controller_matrix[:3, 3]
                        self.fixed_right_controller_rotation = R.from_matrix(self.fixed_right_controller_matrix[:3, :3]).as_quat()
                        self.fixed_left_controller_position = self.fixed_left_controller_matrix[:3, 3]
                        self.fixed_left_controller_rotation = R.from_matrix(self.fixed_left_controller_matrix[:3, :3]).as_quat()

                    # Handle A button event (start arm control)
                    if self.right_a_button and not self.is_controlling_right and not self.is_controlling_left:
                        print("A pressed, starting arm control")

                        # Start data collection
                        if not self.start_data_collection:
                            with open(self.config_path, 'r') as f:
                                config = yaml.safe_load(f)
                            config['data_collecting'] = True
                            with open(self.config_path, 'w') as f:
                                yaml.safe_dump(config, f)
                            self.start_data_collection = True

                        # Initialize reference poses for both arms
                        self.is_controlling_right = True
                        self.reference_position_right = self.fixed_right_controller_position.copy()
                        self.reference_rotation_right = self.fixed_right_controller_rotation.copy()

                        self.is_controlling_left = True
                        self.reference_position_left = self.fixed_left_controller_position.copy()
                        self.reference_rotation_left = self.fixed_left_controller_rotation.copy()

                    # Handle B button event (pause control)
                    if self.right_b_button:
                        print("B pressed, pausing control")
                        self.is_controlling_right = False
                        self.is_controlling_left = False

                        # Stop data collection
                        if not self.end_data_collection:
                            with open(self.config_path, 'r') as f:
                                config = yaml.safe_load(f)
                            config['data_collecting'] = False
                            with open(self.config_path, 'w') as f:
                                yaml.safe_dump(config, f)
                            print("B pressed, stopping data collection")
                            self.end_data_collection = True

                    # Control Right Arm
                    if self.is_controlling_right and self.fixed_right_controller_position is not None and self.reference_position_right is not None:
                        # Position PID control
                        pos_error_right = self.fixed_right_controller_position - self.reference_position_right
                        pos_pid_output_right = self.position_pid_right.compute(pos_error_right, dt)

                        # Update reference position with PID output
                        self.reference_position_right += pos_pid_output_right
                        pos_pid_output_right = np.array([-pos_pid_output_right[2], pos_pid_output_right[0], pos_pid_output_right[1]])

                        # Rotation PID control
                        quat_error_right = self.quaternion_error(self.fixed_right_controller_rotation, self.reference_rotation_right)
                        quat_pid_output_right = self.rotation_pid_right.compute(quat_error_right, dt)

                        delta_rotation_right = R.from_rotvec(quat_pid_output_right).as_quat()
                        self.reference_rotation_right = quaternion_multiply(delta_rotation_right, self.reference_rotation_right)


                        # Adjust delta rotation for coordinate system
                        delta_rotation_right = np.array([
                            delta_rotation_right[2],
                            -delta_rotation_right[0],
                            -delta_rotation_right[1],
                            delta_rotation_right[3]
                        ])

                        # Update end-effector pose with PID output
                        self.teleop_position_right = np.clip(
                            self.teleop_position_right + pos_pid_output_right,
                            self.min_pos,
                            self.max_pos
                        )
                        self.teleop_quat_right = quaternion_multiply(delta_rotation_right, self.teleop_quat_right)

                        # Calculate IK and smooth joints
                        target_joints_right = self.calculate_ik(self.teleop_position_right, self.teleop_quat_right, self.fa_right)
                        smoothed_joints_right = self.smooth_joints(self.fa_right.get_joints(), target_joints_right)

                        # Publish joint values to right arm
                        traj_gen_proto_msg_right = JointPositionSensorMessage(id=0, timestamp=0, joints=smoothed_joints_right)
                        ros_msg_right = make_sensor_group_msg(
                            sensor_proto2ros_msg(traj_gen_proto_msg_right, SensorDataMessageType.JOINT_POSITION)
                        )
                        self.fa_right_pub.publish(ros_msg_right)

                    # Control Left Arm
                    if self.is_controlling_left and self.fixed_left_controller_position is not None and self.reference_position_left is not None:
                        # Position PID control
                        pos_error_left = self.fixed_left_controller_position - self.reference_position_left
                        pos_pid_output_left = self.position_pid_left.compute(pos_error_left, dt)


                        # Update reference position with PID output
                        self.reference_position_left += pos_pid_output_left
                        pos_pid_output_left = np.array([-pos_pid_output_left[2], pos_pid_output_left[0], pos_pid_output_left[1]])

                        # Rotation PID control
                        quat_error_left = self.quaternion_error(self.fixed_left_controller_rotation, self.reference_rotation_left)
                        quat_pid_output_left = self.rotation_pid_left.compute(quat_error_left, dt)

                        delta_rotation_left = R.from_rotvec(quat_pid_output_left).as_quat()
                        self.reference_rotation_left = quaternion_multiply(delta_rotation_left, self.reference_rotation_left)

                        # Adjust delta rotation for coordinate system
                        delta_rotation_left = np.array([
                            delta_rotation_left[2],
                            -delta_rotation_left[0],
                            -delta_rotation_left[1],
                            delta_rotation_left[3]
                        ])

                        # Update end-effector pose with PID outputs
                        self.teleop_position_left = np.clip(
                            self.teleop_position_left + pos_pid_output_left,
                            self.min_pos,
                            self.max_pos
                        )
                        self.teleop_quat_left = quaternion_multiply(delta_rotation_left, self.teleop_quat_left)

                        # Calculate IK and smooth joints
                        target_joints_left = self.calculate_ik(self.teleop_position_left, self.teleop_quat_left, self.fa_left)
                        smoothed_joints_left = self.smooth_joints(self.fa_left.get_joints(), target_joints_left)

                        # Publish joint values to left arm
                        traj_gen_proto_msg_left = JointPositionSensorMessage(id=0, timestamp=0, joints=smoothed_joints_left)
                        ros_msg_left = make_sensor_group_msg(
                            sensor_proto2ros_msg(traj_gen_proto_msg_left, SensorDataMessageType.JOINT_POSITION)
                        )
                        self.fa_left_pub.publish(ros_msg_left)

                    # Control loop rate
                    self.controlrate.sleep()

                except Exception as e:
                    rospy.logerr(f"[Teleop] Error in main loop: {e}")
                    cprint(f"[Teleop] Error in main loop: {e}", "red")
                    traceback_str = traceback.format_exc()
                    rospy.logerr(f"Traceback: {traceback_str}")
                    cprint(f"Traceback:\n{traceback_str}", "red")

        except Exception as e:
            rospy.logerr(f"[Teleop] Error in main execution: {e}")
            cprint(f"[Teleop] Error in main execution: {e}", "red")

        finally:
            cprint("[Teleop] Shutting down...", "yellow")
            if self.fa_right:
                self.fa_right.stop_skill()
            if self.fa_left:
                self.fa_left.stop_skill()
            rospy.signal_shutdown("Shutting down")

if __name__ == "__main__":
    try:
        # Path to the configuration file
        config_path = "/home/frida/catkin_ws/src/frankavr/scripts/vr_teleop/config.yaml"
        teleop = VRControllerFrankaTeleop(config_path=config_path, robot_num=1)
        teleop.do_teleop()
    except Exception as e:
        rospy.logerr(f"[Teleop] Error in main execution: {e}")