from PyQt5.QtWidgets import (QApplication, QMainWindow, QLabel, QPushButton, QVBoxLayout, QHBoxLayout, QWidget, QLineEdit, QComboBox, QMessageBox, QSizePolicy, QFormLayout, QDialog, QAction, QFrame,QSlider)
from PyQt5.QtGui import QPixmap, QImage
from PyQt5.QtCore import Qt, QTimer
import sys
import os
from PIL import Image
import logging
import os
import signal
import sys
import threading
import time
import numpy as np
import cv2
import hashlib
import sqlite3
import bosdyn.client
from bosdyn.client.frame_helpers import ODOM_FRAME_NAME
import bosdyn.client.util
from bosdyn.client.lease import *
from bosdyn.client.robot_command import RobotCommandClient, RobotCommandBuilder
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.image import ImageClient
from bosdyn.client import ResponseError, RpcError, create_standard_sdk
from bosdyn.client.lease import Error as LeaseBaseError
from bosdyn.client.time_sync import TimeSyncError
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive
from bosdyn.client.estop import EstopClient, EstopEndpoint, EstopKeepAlive
from bosdyn.client.power import PowerClient
from bosdyn.client.async_tasks import AsyncPeriodicQuery
from bosdyn.client.robot_command import RobotCommandBuilder
from bosdyn.api import basic_command_pb2
import bosdyn.api.basic_command_pb2 as basic_command_pb2
import bosdyn.api.power_pb2 as PowerServiceProto
import bosdyn.api.robot_state_pb2 as robot_state_proto
import bosdyn.api.spot.robot_command_pb2 as spot_command_pb2
import bosdyn.api.power_pb2 as power_pb2
from bosdyn.api.robot_state_pb2 import PowerState
from bosdyn.api.image_pb2 import Image as BosdynImageFormat
from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2
from bosdyn.util import duration_str
import bosdyn.geometry
from bosdyn.geometry import EulerZXY

# Global variables
streaming = False
current_camera_index = 0
fisheye_cameras = [("frontleft_fisheye_image", "Front Left"),("frontright_fisheye_image", "Front Right"),("left_fisheye_image", "Left Side"),("right_fisheye_image", "Right Side"),("back_fisheye_image", "Back")]
image_client = None  # You must initialize this properly
current_process = None
lease_client = None
lease_keep_alive = None
robot = None
command_client = None
image_client = None
wasd_interface = None
camera_label = None
status_label = None
battery_label = None
canvas = None
root = None
conn = sqlite3.connect('users.db')
cursor = conn.cursor()

# Define valid credentials and IP for demonstration purposes
VALID_USERNAME = "user2"
VALID_PASSWORD = "simplepassword"
VALID_IP = "192.168.80.3"
IP = ""
VELOCITY_CMD_DURATION = 0.6  # seconds
MAX_LINEAR_VELOCITY = 0.5  # m/s
MAX_ANGULAR_VELOCITY = 0.8  # rad/s
LOGGER = logging.getLogger()
VELOCITY_BASE_SPEED = 0.5  # m/s
VELOCITY_BASE_ANGULAR = 0.8  # rad/sec
VELOCITY_CMD_DURATION = 0.6  # seconds
COMMAND_INPUT_RATE = 0.1
BODY_HEIGHT_MIN = -0.2
BODY_HEIGHT_MAX = 0.2
BODY_HEIGHT_STEP = 0.02
PITCH_MIN = -0.5
PITCH_MAX = 0.5
PITCH_STEP = 0.05
ROLL_MIN = -0.5
ROLL_MAX = 0.5
ROLL_STEP = 0.05
YAW_MIN = -1.0
YAW_MAX = 1.0
YAW_STEP = 0.1

# from the wasd.py file
class ExitCheck(object):
    """A class to help exiting a loop, also capturing SIGTERM to exit the loop."""
    def __init__(self):
        self._kill_now = False
        signal.signal(signal.SIGTERM, self._sigterm_handler)
        signal.signal(signal.SIGINT, self._sigterm_handler)

    def __enter__(self):
        return self

    def __exit__(self, _type, _value, _traceback):
        return False

    def _sigterm_handler(self, _signum, _frame):
        self._kill_now = True

    def request_exit(self):
        """Manually trigger an exit (rather than sigterm/sigint)."""
        self._kill_now = True

    @property
    def kill_now(self):
        """Return the status of the exit checker indicating if it should exit."""
        return self._kill_now

class AsyncRobotState(AsyncPeriodicQuery):
    """Grab robot state."""
    def __init__(self, robot_state_client):
        super(AsyncRobotState, self).__init__('robot_state', robot_state_client, LOGGER, period_sec=0.2)

    def _start_query(self):
        return self._client.get_robot_state_async()

class WasdInterface(object):
    """A curses interface for driving the robot."""
    def __init__(self, robot):
        self._robot = robot
        self._lease_client = robot.ensure_client(LeaseClient.default_service_name)
        try:
            self._estop_client = self._robot.ensure_client(EstopClient.default_service_name)
            self._estop_endpoint = EstopEndpoint(self._estop_client, 'GNClient', 9.0)
        except:
            self._estop_client = None
            self._estop_endpoint = None
        self._power_client = robot.ensure_client(PowerClient.default_service_name)
        self._robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)
        self._robot_command_client = robot.ensure_client(RobotCommandClient.default_service_name)
        self._robot_state_task = AsyncRobotState(self._robot_state_client)
        self._lock = threading.Lock()
        self._body_height = 0.0             # NEW: Body height support
        self._body_pitch = 0.0              # NEW: Pitch support
        self._body_roll = 0.0               # NEW: Roll support
        self._body_yaw = 0.0                # NEW: Yaw support
        self._command_dictionary = {
            27: self._stop,  # ESC key
            '\t': self._quit_program,
            't': self._toggle_time_sync,
            ' ': self._toggle_estop,
            'r': self._self_right,
            'o': self._toggle_power,
            'p': self._toggle_power,
            'v': self._sit,
            'b': self._battery_change_pose,
            'f': self._stand,
            'w': self._move_forward,
            's': self._move_backward,
            'a': self._strafe_left,
            'd': self._strafe_right,
            'q': self._turn_left,
            'e': self._turn_right,
            'u': self._yaw_right,            # NEW
            'y': self._yaw_left,             # NEW
            'n': self._roll_left,            # NEW
            'm': self._roll_right,           # NEW
            'g': self._tilt_backward,        # NEW
            'h': self._tilt_forward,         # NEW
            'z': self._increase_body_height, # NEW
            'x': self._decrease_body_height, # NEW
            'c': self._reset_posture         # NEW
        }
        self._locked_messages = ['', '', '']
        self._estop_keepalive = None
        self._exit_check = None
        self._robot_id = None
        self._lease_keepalive = None

    def _tilt_forward(self):
        self._body_pitch += PITCH_STEP
        self.send_pitch_command(self._body_pitch)

    def _tilt_backward(self):
        self._body_pitch -= PITCH_STEP
        self.send_pitch_command(self._body_pitch)

    def _roll_left(self):
        self._body_roll += ROLL_STEP
        self.send_roll_command(self._body_roll)

    def _roll_right(self):
        self._body_roll -= ROLL_STEP
        self.send_roll_command(self._body_roll)
        self._body_roll = max(ROLL_MIN, self._body_roll - ROLL_STEP)
        self._send_orientation_command()

    def _yaw_left(self):
        self._body_yaw = max(YAW_MIN, self._body_yaw - YAW_STEP)
        self.send_yaw_command(self._body_yaw)

    def _yaw_right(self):
        self._body_yaw = min(YAW_MAX, self._body_yaw + YAW_STEP)
        self.send_yaw_command(self._body_yaw)

    def _increase_body_height(self):
        self._body_height = min(BODY_HEIGHT_MAX, self._body_height + BODY_HEIGHT_STEP)
        self.send_body_height_command()

    def _decrease_body_height(self):
        self._body_height = max(BODY_HEIGHT_MIN, self._body_height - BODY_HEIGHT_STEP)
        self.send_body_height_command()

    def _reset_posture(self):
        self._body_pitch = 0.0
        self._body_roll = 0.0
        self._body_yaw = 0.0
        self._body_height = 0.0
        self._send_orientation_command()
    
    def _send_orientation_command(self):
        # Send the orientation command to the robot
        footprint_R_body = EulerZXY(yaw=self._body_yaw, roll=self._body_roll, pitch=self._body_pitch)
        command = RobotCommandBuilder.synchro_stand_command(footprint_R_body=footprint_R_body)
        self._start_robot_command('adjust_body_height', command, end_time_secs=time.time() + VELOCITY_CMD_DURATION)

    def send_yaw_command(self, yaw_rad):
        roll = 0.0
        pitch = 0.0
        footprint_R_body = EulerZXY(yaw=yaw_rad, roll=roll, pitch=pitch)
        command = RobotCommandBuilder.synchro_stand_command(footprint_R_body=footprint_R_body)
        self._start_robot_command('adjust_body_height', command, end_time_secs=time.time() + VELOCITY_CMD_DURATION)

    def send_pitch_command(self, pitch_rad):
        roll = 0.0
        yaw = 0.0
        footprint_R_body = EulerZXY(yaw=yaw, roll=roll, pitch=pitch_rad)
        command = RobotCommandBuilder.synchro_stand_command(footprint_R_body=footprint_R_body)
        self._start_robot_command('adjust_body_height', command, end_time_secs=time.time() + VELOCITY_CMD_DURATION)

    def send_roll_command(self, roll_rad):
        pitch = 0.0
        yaw = 0.0
        footprint_R_body = EulerZXY(yaw=yaw, roll=roll_rad, pitch=pitch)
        command = RobotCommandBuilder.synchro_stand_command(footprint_R_body=footprint_R_body)
        self._start_robot_command('adjust_body_height', command, end_time_secs=time.time() + VELOCITY_CMD_DURATION)

    def send_body_height_command(self):
        command = RobotCommandBuilder.synchro_velocity_command(
            v_x=0.0, v_y=0.0, v_rot=0.0,
            body_height=self._body_height
        )
        self._start_robot_command('adjust_body_height', command, end_time_secs=time.time() + VELOCITY_CMD_DURATION)
        self.add_message(f'Height: {self._body_height:.2f} m')

    def start(self):
        """Begin communication with the robot."""
        # Construct our lease keep-alive object, which begins RetainLease calls in a thread.
        self._lease_keepalive = LeaseKeepAlive(self._lease_client, must_acquire=True, return_at_exit=True)

        self._robot_id = self._robot.get_id()
        # if self._estop_endpoint is not None:
        #     self._estop_endpoint.force_simple_setup()  # Set this endpoint as the robot's sole estop.

    def shutdown(self):
        """Release control of robot as gracefully as possible."""
        LOGGER.info('Shutting down WasdInterface.')
        if self._estop_keepalive:
            # This stops the check-in thread but does not stop the robot.
            self._estop_keepalive.shutdown()
        if self._lease_keepalive:
            self._lease_keepalive.shutdown()

    def flush_and_estop_buffer(self, stdscr):
        """Manually flush the curses input buffer but trigger any estop requests (space)"""
        key = ''
        while key != -1:
            key = stdscr.getch()
            if key == (' '):
                self._toggle_estop()

    def add_message(self, msg_text):
        """Display the given message string to the user in the curses interface."""
        with self._lock:
            self._locked_messages = [msg_text] + self._locked_messages[:-1]

    def message(self, idx):
        """Grab one of the 3 last messages added."""
        with self._lock:
            return self._locked_messages[idx]

    @property
    def robot_state(self):
        """Get latest robot state proto."""
        return self._robot_state_task.proto

    def _drive_cmd(self, key):
        """Run user commands at each update."""
        try:
            cmd_function = self._command_dictionary[key]
            cmd_function()

        except KeyError:
            if key and key != -1 and key < 256:
                self.add_message(f'Unrecognized keyboard command: \'{chr(key)}\'')

    def _try_grpc(self, desc, thunk):
        try:
            return thunk()
        except (ResponseError, RpcError, LeaseBaseError) as err:
            self.add_message(f'Failed {desc}: {err}')
            return None

    def _try_grpc_async(self, desc, thunk):

        def on_future_done(fut):
            try:
                fut.result()
            except (ResponseError, RpcError, LeaseBaseError) as err:
                self.add_message(f'Failed {desc}: {err}')
                return None

        future = thunk()
        future.add_done_callback(on_future_done)

    def _quit_program(self):
        self._sit()
        if self._exit_check is not None:
            self._exit_check.request_exit()

    def _toggle_time_sync(self):
        if self._robot.time_sync.stopped:
            self._robot.start_time_sync()
        else:
            self._robot.time_sync.stop()

    def _toggle_estop(self):
        """toggle estop on/off. Initial state is ON"""
        if self._estop_client is not None and self._estop_endpoint is not None:
            if not self._estop_keepalive:
                self._estop_keepalive = EstopKeepAlive(self._estop_endpoint)
            else:
                self._try_grpc('stopping estop', self._estop_keepalive.stop)
                self._estop_keepalive.shutdown()
                self._estop_keepalive = None

    def _start_robot_command(self, desc, command_proto, end_time_secs=None):

        def _start_command():
            self._robot_command_client.robot_command(command=command_proto, end_time_secs=end_time_secs)

        self._try_grpc(desc, _start_command)

    def _self_right(self):
        self._start_robot_command('self_right', RobotCommandBuilder.selfright_command())

    def _battery_change_pose(self):
        # Default HINT_RIGHT, maybe add option to choose direction?
        self._start_robot_command(
            'battery_change_pose',
            RobotCommandBuilder.battery_change_pose_command(
                dir_hint=basic_command_pb2.BatteryChangePoseCommand.Request.HINT_RIGHT))

    def _sit(self):
        self._start_robot_command('sit', RobotCommandBuilder.synchro_sit_command())

    def _stand(self):
        self._start_robot_command('stand', RobotCommandBuilder.synchro_stand_command())

    def _move_forward(self):
        self._velocity_cmd_helper('move_forward', v_x=VELOCITY_BASE_SPEED)

    def _move_backward(self):
        self._velocity_cmd_helper('move_backward', v_x=-VELOCITY_BASE_SPEED)

    def _strafe_left(self):
        self._velocity_cmd_helper('strafe_left', v_y=VELOCITY_BASE_SPEED)

    def _strafe_right(self):
        self._velocity_cmd_helper('strafe_right', v_y=-VELOCITY_BASE_SPEED)

    def _turn_left(self):
        self._velocity_cmd_helper('turn_left', v_rot=VELOCITY_BASE_ANGULAR)

    def _turn_right(self):
        self._velocity_cmd_helper('turn_right', v_rot=-VELOCITY_BASE_ANGULAR)

    def _stop(self):
        self._start_robot_command('stop', RobotCommandBuilder.stop_command())

    def _velocity_cmd_helper(self, desc='', v_x=0.0, v_y=0.0, v_rot=0.0):
        self._start_robot_command(desc, RobotCommandBuilder.synchro_velocity_command(v_x=v_x, v_y=v_y, v_rot=v_rot), end_time_secs=time.time() + VELOCITY_CMD_DURATION)

    def _return_to_origin(self):
        self._start_robot_command(
            'fwd_and_rotate',
            RobotCommandBuilder.synchro_se2_trajectory_point_command(goal_x=0.0, goal_y=0.0, goal_heading=0.0, frame_name=ODOM_FRAME_NAME, params=None, body_height=0.0, locomotion_hint=spot_command_pb2.HINT_SPEED_SELECT_TROT), end_time_secs=time.time() + 20)

    def _toggle_power(self):
        power_state = self._power_state()
        if power_state is None:
            self.add_message('Could not toggle power because power state is unknown')
            return

        if power_state == robot_state_proto.PowerState.STATE_OFF:
            self._try_grpc_async('powering-on', self._request_power_on)
        else:
            self._try_grpc('powering-off', self._safe_power_off)

    def _request_power_on(self):
        request = PowerServiceProto.PowerCommandRequest.REQUEST_ON
        return self._power_client.power_command_async(request)

    def _safe_power_off(self):
        self._start_robot_command('safe_power_off', RobotCommandBuilder.safe_power_off_command())

    def _power_state(self):
        state = self.robot_state
        if not state:
            return None
        return state.power_state.motor_power_state

    def _lease_str(self, lease_keep_alive):
        if lease_keep_alive is None:
            alive = 'STOPPED'
            lease = 'RETURNED'
        else:
            try:
                _lease = lease_keep_alive.lease_wallet.get_lease()
                lease = f'{_lease.lease_proto.resource}:{_lease.lease_proto.sequence}'
            except bosdyn.client.lease.Error:
                lease = '...'
            if lease_keep_alive.is_alive():
                alive = 'RUNNING'
            else:
                alive = 'STOPPED'
        return f'Lease {lease} THREAD:{alive}'

    def _power_state_str(self):
        power_state = self._power_state()
        if power_state is None:
            return ''
        state_str = robot_state_proto.PowerState.MotorPowerState.Name(power_state)
        return f'Power: {state_str[6:]}'  # get rid of STATE_ prefix

    def _estop_str(self):
        if not self._estop_client:
            thread_status = 'NOT ESTOP'
        else:
            thread_status = 'RUNNING' if self._estop_keepalive else 'STOPPED'
        estop_status = '??'
        state = self.robot_state
        if state:
            for estop_state in state.estop_states:
                if estop_state.type == estop_state.TYPE_SOFTWARE:
                    estop_status = estop_state.State.Name(estop_state.state)[6:]  # s/STATE_//
                    break
        return f'Estop {estop_status} (thread: {thread_status})'

    def _time_sync_str(self):
        if not self._robot.time_sync:
            return 'Time sync: (none)'
        if self._robot.time_sync.stopped:
            status = 'STOPPED'
            exception = self._robot.time_sync.thread_exception
            if exception:
                status = f'{status} Exception: {exception}'
        else:
            status = 'RUNNING'
        try:
            skew = self._robot.time_sync.get_robot_clock_skew()
            if skew:
                skew_str = f'offset={duration_str(skew)}'
            else:
                skew_str = '(Skew undetermined)'
        except (TimeSyncError, RpcError) as err:
            skew_str = f'({err})'
        return f'Time sync: {status} {skew_str}'

def check_battery_status(widget):
    global battery_label, VALID_IP, robot

    username = os.environ.get('BOSDYN_CLIENT_USERNAME')
    password = os.environ.get('BOSDYN_CLIENT_PASSWORD')

    def retry_later():
        QTimer.singleShot(60000, lambda: check_battery_status(widget))  # retry in 60 sec

    if not username or not password or not VALID_IP:
        battery_label.setText("Invalid Log-In")
        retry_later()
        return

    try:
        if not robot:
            sdk = bosdyn.client.create_standard_sdk("Battery Check")
            robot = sdk.create_robot(IP)
            robot.authenticate(username, password)

        state_client = robot.ensure_client(RobotStateClient.default_service_name)
        robot_state = state_client.get_robot_state()

        battery_states = robot_state.battery_states

        if battery_states:
            battery_percentage = battery_states[0].charge_percentage.value
            battery_label.setText(f"Battery: {battery_percentage:.1f}%")

            # Color change based on percentage
            if battery_percentage >= 60:
                color = "green"
            elif battery_percentage >= 30:
                color = "orange"
            else:
                color = "red"

            battery_label.setStyleSheet(f"color: {color}; font-size: 20px; font-weight:bold;")
        else:
            battery_label.setText("Battery info unavailable")

    except Exception as e:
        battery_label.setText(f"Error: {str(e)}")

    retry_later()

def wait_for_power_on(robot, timeout_sec=20):
    #Polls the robot's state until it is powered on or times out
    state_client = robot.ensure_client(RobotStateClient.default_service_name)

    for _ in range(timeout_sec):
        state = state_client.get_robot_state()
        if state.power_state.motor_power_state == PowerState.STATE_ON:
            return True
        time.sleep(1)

    raise TimeoutError("Timed out waiting for robot to power.")

def take_lease():
    global lease_client, lease_keep_alive, robot, command_client, image_client, wasd_interface, lease
    try:
        sdk = create_standard_sdk("SpotController")
        robot = sdk.create_robot(IP)
        robot.authenticate(os.environ['BOSDYN_CLIENT_USERNAME'], os.environ['BOSDYN_CLIENT_PASSWORD'])

        # Start time synchronization before taking lease
        robot.start_time_sync()
        robot.time_sync.wait_for_sync()

        lease_client = robot.ensure_client(LeaseClient.default_service_name)
        lease = lease_client.take() #Take the lease explicitly 
        lease_keep_alive = LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True)

        command_client = robot.ensure_client(RobotCommandClient.default_service_name)
        image_client = robot.ensure_client(ImageClient.default_service_name)

        #Power on the robot
        power_client = robot.ensure_client(PowerClient.default_service_name)

        #Estop Setup
        estop_client = robot.ensure_client(EstopClient.default_service_name)
        estop_endpoint = EstopEndpoint(estop_client, 'GUI_Estop', 9.0)
        estop_endpoint.force_simple_setup()  # become the sole E-Stop source
        estop_keep_alive = EstopKeepAlive(estop_endpoint)  # start sending heartbeats
        
        power_client.power_command(power_pb2.PowerCommandRequest.REQUEST_ON)
        wait_for_power_on(robot)

        global wasd_interface
        wasd_interface = WasdInterface(robot)
        wasd_interface.start()

        # Update global lease_keep_alive to wasd_interface's lease keepalive
        # global lease_keep_alive
        lease_keep_alive = wasd_interface._lease_keepalive

        status_label.setText("Lease acquired - Ready for commands")
        return True
    except Exception as e:
        status_label.setText(f"Lease error: {str(e)}")
        return False

def release_lease():
    global lease_keep_alive, lease_client, lease
    try:
        if lease_keep_alive:
            lease_keep_alive.shutdown()
        if lease_client and lease:
            lease_client.return_lease(lease)
            lease = None  # Clear the lease
        status_label.setText("Lease released")
    except Exception as e:
        status_label.setText(f"Error releasing lease: {str(e)}")

def stopProgram():
    release_lease()
    status_label.setText("Stopped - Lease released")

def startEStop():
    try:
        status_label.setText("E-Stop engaged (mock)")
    except Exception as e:
        status_label.setText(f"E-Stop error: {str(e)}")

def move_robot(v_x, v_y, v_rot):
    global robot, command_client, status_label

    try:
        if not robot or not command_client:
            if not take_lease():
                return False

        if not robot.is_powered_on():
            robot.power_on(timeout_sec=20)

        cmd = RobotCommandBuilder.synchro_velocity_command(
            v_x=v_x, v_y=v_y, v_rot=v_rot, duration=VELOCITY_CMD_DURATION)
        command_client.robot_command(cmd)

        status_label.setText("Move command sent")
        return True

    except Exception as e:
        status_label.setText(f"Move error: {str(e)}")
        print(f"Move error: {e}")
        return False

def on_key_press(event):
    global wasd_interface, status_label

    if not wasd_interface:
        return

    try:
        char = event.text().lower()
        command_map = wasd_interface._command_dictionary

        if char in command_map:
            command_map[char]()  # Trigger the associated method
            status_label.setText(f"Command: {char.upper()}")
        else:
            status_label.setText(f"Unrecognized key: {char}")
    except Exception as e:
        status_label.setText(f"Key error: {str(e)}")
        print(f"Key press error: {e}")

def run_with_inputs(username_input, password_input, ip_input, login_window, error_label):
    username = username_input.text()
    password = password_input.text()
    ip = ip_input.text()

    if not username or not password or not ip:
        QMessageBox.critical(None, "Error", "All fields are required!")
        return

    # Set environment variables (optional, depending on your use case)
    os.environ['BOSDYN_CLIENT_USERNAME'] = username
    os.environ['BOSDYN_CLIENT_PASSWORD'] = password

    global IP
    IP = ip

    # Validate the credentials and IP address
    if username == VALID_USERNAME and password == VALID_PASSWORD and ip == VALID_IP:
        login_window.close()  # Close the login window
        mainInterface()  # Open the main interface
    else:
        error_label.setText("Invalid credentials or IP address!")
        error_label.setStyleSheet("color: red;")
        # Clear the input fields for re-entry
        username_input.clear()
        password_input.clear()
        ip_input.clear()

def next_camera():
    global current_camera_index
    current_camera_index = (current_camera_index + 1) % len(fisheye_cameras)
    camera_selector.setCurrentIndex(current_camera_index)
    status_label.setText(f"Switched to: {fisheye_cameras[current_camera_index]}")

def start_video_stream():
    global streaming, video_timer
    if not streaming:
        streaming = True
        video_timer.start(100)  # Start the timer
        status_label.setText("Camera stream started")

def stop_video_stream():
    global streaming, video_timer
    streaming = False
    if video_timer.isActive():
        video_timer.stop()  # Stop the timer

    # Create a black QPixmap and set it to the camera label
    black_pixmap = QPixmap(camera_label.width(), camera_label.height())
    black_pixmap.fill(Qt.black)
    camera_label.setPixmap(black_pixmap)

    status_label.setText("Camera stream stopped")

def update_video_frame():
    global streaming, image_client, camera_label, current_camera_index
    if not streaming:
        return

    try:
        camera_name = fisheye_cameras[current_camera_index][0]
        response = image_client.get_image_from_sources([camera_name])[0]
        img_data = response.shot.image.data
        width = response.shot.image.cols
        height = response.shot.image.rows
        pixel_format = response.shot.image.pixel_format

        # Decode image
        if response.shot.image.format == BosdynImageFormat.FORMAT_JPEG:
            image_np = np.frombuffer(img_data, np.uint8)
            decoded_image = cv2.imdecode(image_np, cv2.IMREAD_COLOR)
            decoded_image = cv2.cvtColor(decoded_image, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(decoded_image).rotate(-90, expand=True)
        else:
            if pixel_format == BosdynImageFormat.PIXEL_FORMAT_GREYSCALE_U8:
                mode = 'L'
            elif pixel_format == BosdynImageFormat.PIXEL_FORMAT_RGB_U8:
                mode = 'RGB'
            elif pixel_format == BosdynImageFormat.PIXEL_FORMAT_RGBA_U8:
                mode = 'RGBA'
            else:
                camera_label.setText(f"Unsupported pixel format: {pixel_format}")
                return

            expected_bytes = width * height * len(mode)
            if len(img_data) < expected_bytes:
                camera_label.setText("Not enough image data")
                return

            img = Image.frombytes(mode, (width, height), img_data).rotate(-90, expand=True)

            # Convert grayscale to RGB for display
            if mode == 'L':
                img = img.convert("RGB")

        # Resize for display
        img = img.resize((640, 480))  # Make it bigger

        # Convert to QImage and set in QLabel
        qimg = QImage(img.tobytes(), img.width, img.height, QImage.Format_RGB888)
        camera_label.setPixmap(QPixmap.fromImage(qimg))
        camera_label.setText("")  # Clear any old error text
        status_label.setText(f"Streaming: {camera_name}")

    except Exception as e:
        camera_label.setText(f" ")

def mainInterface():
    global status_label, battery_label, camera_label, main_window, camera_selector, video_timer

    class SpotMainWindow(QMainWindow):
        def keyPressEvent(self, event):
            on_key_press(event)

    main_window = SpotMainWindow()
    main_window.setWindowTitle("Spot Control Panel")
    main_window.setGeometry(100, 100, 800, 900)

    # -- Central Widget --
    central_widget = QWidget()
    main_window.setCentralWidget(central_widget)

    side_menu = QFrame()
    side_menu.setFixedWidth(200)
    side_menu.setStyleSheet("background-color: #2c3e50; color: white;")
    side_menu.setVisible(False)  # Initially hidden

    side_layout = QVBoxLayout()
    side_layout.setAlignment(Qt.AlignTop)

    # Add some buttons or labels to side menu
    side_layout.addWidget(QLabel("Settings", alignment=Qt.AlignCenter))

    # --- Speed Slider Section ---
    speed_slider_layout = QVBoxLayout()
    speed_slider_layout.setAlignment(Qt.AlignHCenter)

    speed_label = QLabel("Speed: 0.5 m/s")
    speed_label.setStyleSheet("font-size: 16px; color: white;")
    speed_label.setAlignment(Qt.AlignCenter)

    speed_slider = QSlider(Qt.Horizontal)
    speed_slider.setMinimum(1)     # Represents 0.1 m/s
    speed_slider.setMaximum(10)    # Represents 1.0 m/s
    speed_slider.setValue(5)       # Default value = 0.5 m/s
    speed_slider.setTickInterval(1)
    speed_slider.setTickPosition(QSlider.TicksBelow)
    speed_slider.setFixedWidth(200)

    def update_speed(value):
        global VELOCITY_BASE_SPEED
        VELOCITY_BASE_SPEED = value / 10.0
        speed_label.setText(f"Speed: {VELOCITY_BASE_SPEED:.1f} m/s")

    speed_slider.valueChanged.connect(update_speed)

    speed_slider_layout.addWidget(speed_label)
    speed_slider_layout.addWidget(speed_slider)

    # --- Angular Speed Slider Section ---
    angular_slider_layout = QVBoxLayout()
    angular_slider_layout.setAlignment(Qt.AlignHCenter)

    angular_label = QLabel("Angular Speed: 0.8 rad/s")
    angular_label.setStyleSheet("font-size: 16px; color: white;")
    angular_label.setAlignment(Qt.AlignCenter)

    angular_slider = QSlider(Qt.Horizontal)
    angular_slider.setMinimum(1)     # Represents 0.1 rad/s
    angular_slider.setMaximum(15)    # Represents 1.5 rad/s
    angular_slider.setValue(8)       # Default value = 0.8 rad/s
    angular_slider.setTickInterval(1)
    angular_slider.setTickPosition(QSlider.TicksBelow)
    angular_slider.setFixedWidth(200)

    def update_angular_speed(value):
        global VELOCITY_BASE_ANGULAR
        VELOCITY_BASE_ANGULAR = value / 10.0
        angular_label.setText(f"Angular Speed: {VELOCITY_BASE_ANGULAR:.1f} rad/s")

    angular_slider.valueChanged.connect(update_angular_speed)

    side_layout.addWidget(speed_label)
    side_layout.addWidget(speed_slider)
    side_layout.addWidget(angular_label)
    side_layout.addWidget(angular_slider)

    side_menu.setLayout(side_layout)

    # --- Background Image Label (scales with window) ---
    class ResizableBackground(QLabel):
        def __init__(self, image_path):
            super().__init__()
            self.image_path = image_path
            self.setAlignment(Qt.AlignCenter)
            self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        def resizeEvent(self, event):
            pixmap = QPixmap(self.image_path)
            if not pixmap.isNull():
                self.setPixmap(pixmap.scaled(self.size(), Qt.KeepAspectRatioByExpanding, Qt.SmoothTransformation))

    bg_label = ResizableBackground("./Gold-Brayer2.png")
    bg_label.setParent(central_widget)
    bg_label.setGeometry(central_widget.rect())
    bg_label.lower()  # Ensure it's behind the overlay

    # --- Overlay widget ---
    overlay_widget = QWidget(central_widget)  # Set central_widget as parent
    overlay_widget.setStyleSheet("background: transparent;")
    overlay_layout = QVBoxLayout()
    main_layout = QHBoxLayout(central_widget)
    overlay_layout.setAlignment(Qt.AlignCenter)
    overlay_widget.setLayout(overlay_layout)
    overlay_widget.setGeometry(central_widget.rect())  # Initially fill the central widget

    main_layout.addWidget(side_menu)
    main_layout.addWidget(overlay_widget)

    # -- Container with buttons/labels --
    container = QWidget()
    container_layout = QVBoxLayout()
    container_layout.setAlignment(Qt.AlignHCenter)

    # Battery label
    battery_label = QLabel("Battery: N/A")
    battery_label.setStyleSheet("color: green; font-size: 20px; font-weight:bold;")
    battery_label.setAlignment(Qt.AlignCenter)
    battery_timer = QTimer()
    battery_timer.timeout.connect(lambda: check_battery_status(main_window))
    battery_timer.start(5000)  # update every 5 seconds
    check_battery_status(main_window)  # initial call
    container_layout.addWidget(battery_label)

    toggle_menu_button = QPushButton("☰ Menu")
    toggle_menu_button.setFixedSize(100, 30)
    toggle_menu_button.setStyleSheet("background-color: #34495e; color: white; font-size: 14px;")
    toggle_menu_button.clicked.connect(lambda: side_menu.setVisible(not side_menu.isVisible()))
    container_layout.insertWidget(0, toggle_menu_button)

    # Camera selector
    cam_layout = QHBoxLayout()
    cam_layout.setAlignment(Qt.AlignLeft)
    camera_selector = QComboBox()
    camera_selector.setFixedWidth(160)
    camera_selector.setFixedHeight(30)
    camera_selector.setStyleSheet(""" QComboBox { background-color: #2c3e50; color: white; font-size: 14px; border: 1px solid #34495e; border-radius: 5px; padding: 5px; }
        QComboBox::drop-down { background-color: #2980b9; }
        QComboBox::indicator { width: 20px; height: 20px; }
        QComboBox::item { background-color: #34495e; color: white; }
        QComboBox::item:selected { background-color: #2980b9; color: white; }""")
    camera_selector.addItems([label for _, label in fisheye_cameras])

    video_timer = QTimer()
    video_timer.timeout.connect(update_video_frame)
    next_cam_button = QPushButton("Next Camera")
    next_cam_button.setStyleSheet("background-color: #1abc9c; color: white; font-size: 14px;")
    next_cam_button.setFixedSize(120, 30)
    next_cam_button.clicked.connect(next_camera)

    # Start and Stop buttons for video stream
    start_button = QPushButton("Start Camera")
    start_button.setStyleSheet("background-color: darkgreen; color: white; font-size: 14px;")
    start_button.setFixedSize(120, 30)
    start_button.clicked.connect(start_video_stream)

    stop_button = QPushButton("Stop Camera")
    stop_button.setStyleSheet("background-color: darkred; color: white; font-size: 14px;")
    stop_button.setFixedSize(120, 30)
    stop_button.clicked.connect(stop_video_stream)

    # Add buttons to a horizontal layout
    cam_layout.addWidget(camera_selector)
    cam_layout.addWidget(next_cam_button)
    cam_layout.addWidget(start_button)
    cam_layout.addWidget(stop_button)
    container_layout.addLayout(cam_layout)

    # Sync selector to index
    def camera_changed(index):
        global current_camera_index
        current_camera_index = index
        status_label.setText(f"Switched to: {fisheye_cameras[index]}")
    camera_selector.currentIndexChanged.connect(camera_changed)

    # Camera display
    camera_label = QLabel("Take Lease then Press Start Camera to Turn on Camera")
    camera_label.setStyleSheet("background-color: black; color: white; font-size: 16px; padding: 5px; border-radius: 5px;")
    camera_label.setAlignment(Qt.AlignCenter)
    camera_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
    camera_label.setMinimumSize(640, 480)
    camera_label.setScaledContents(True)
    container_layout.addWidget(camera_label)
    
    def safe_stand():
        if 'wasd_interface' in globals() and wasd_interface:
            wasd_interface._stand()
            status_label.setText("Command: Stand")
        else:
            status_label.setText("Cannot Stand: Lease not acquired")

    def safe_sit():
        if 'wasd_interface' in globals() and wasd_interface:
            wasd_interface._sit()
            status_label.setText("Command: Sit")
        else:
            status_label.setText("Cannot Sit: Lease not acquired")
    
    # Status label
    status_label = QLabel("Ready")
    status_label.setStyleSheet("color: blue; font-size: 14px;")
    # container_layout.addWidget(status_label)

    # --- Spacer ---
    spacer = QLabel("")
    spacer.setFixedHeight(10)
    container_layout.addWidget(spacer)

    # Command buttons in a horizontal layout
    button_layout = QHBoxLayout()  # Create a horizontal layout for buttons
    button_layout.setAlignment(Qt.AlignCenter)

    # Define button styles
    button_styles = {
        "Stand": "background-color: green; color: white; font-size: 14px;",
        "Sit": "background-color: blue; color: white; font-size: 14px;",
        "Take Lease": "background-color: orange; color: white; font-size: 14px;",
        "Release Lease": "background-color: purple; color: white; font-size: 14px;",
        "E-Stop": "background-color: red; color: white; font-size: 14px;",
        "Stop": "background-color: yellow; color: black; font-size: 14px;",
        "Quit": "background-color: teal; color: white; font-size: 14px;"
    }

    # Create buttons with styles
    for button_text, style in button_styles.items():
        button = QPushButton(button_text)
        button.setStyleSheet(style)
        button.setFixedSize(100, 40)
        
        # Connect the button to its respective function
        if button_text == "Stand":
            button.clicked.connect(safe_stand)
        elif button_text == "Sit":
            button.clicked.connect(safe_sit)
        elif button_text == "Take Lease":
            button.clicked.connect(take_lease)
        elif button_text == "Release Lease":
            button.clicked.connect(release_lease)
        elif button_text == "E-Stop":
            button.clicked.connect(startEStop)
        elif button_text == "Stop":
            button.clicked.connect(stopProgram)
        elif button_text == "Quit":
            button.clicked.connect(main_window.close)

        button_layout.addWidget(button)  # Add the button to the layout

    # Add the button layout to the container layout
    container_layout.addLayout(button_layout)  # Add the button layout to the container layout

    # Set the container layout
    container.setLayout(container_layout)
    overlay_layout.addWidget(container)

    # Resize event to adjust background and overlay
    def resize_overlay(event):
        bg_label.setGeometry(central_widget.rect())
        overlay_widget.setGeometry(central_widget.rect())

    central_widget.resizeEvent = resize_overlay

    main_window.show()

class RobotControlApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.create_menu()

        # login_window = QWidget()
        self.setWindowTitle("Spot Login")
        self.setGeometry(200, 200, 400, 300)

        login_form = QFormLayout()

        username_input = QLineEdit()
        password_input = QLineEdit()
        ip_input = QLineEdit()

        password_input.setEchoMode(QLineEdit.Password)

        self.login_button = QPushButton("Login")
        self.login_button.setStyleSheet("background-color: black; color: white; font-size: 14px;")
        self.login_button.clicked.connect(lambda: run_with_inputs(username_input, password_input, ip_input, self, error_label))

        # --- Spacer ---
        spacer = QLabel("")
        spacer.setFixedHeight(10)

        user_label = QLabel("Username:")
        user_label.setStyleSheet("color: black; font-size: 16px; font-weight:bold;")
        username_input.setPlaceholderText("Enter username")
        username_input.setToolTip("Username must be unique")
        # username_input.setMaxLength(20)
        username_input.setStyleSheet("background-color: #f0f0f0; color: black; font-size: 16px; border: 1px solid #ccc; border-radius: 5px;")
        
        password_label = QLabel("Password:")
        password_label.setStyleSheet("color: black; font-size: 16px; font-weight:bold;")
        password_input.setPlaceholderText("Enter password")
        password_input.setToolTip("Password must be at least 6 characters long")
        password_input.setMaxLength(20)
        password_input.setStyleSheet("background-color: #f0f0f0; color: black; font-size: 16px; border: 1px solid #ccc; border-radius: 5px;")

        ip_label = QLabel("IP Address:")
        ip_label.setStyleSheet("color: black; font-size: 16px; font-weight:bold;")
        ip_input.setPlaceholderText("Enter IP Address")
        ip_input.setToolTip("Enter the IP address of the robot")
        ip_input.setMaxLength(15)
        ip_input.setStyleSheet("background-color: #f0f0f0; color: black; font-size: 16px; border: 1px solid #ccc; border-radius: 5px;")

        login_form.addRow(user_label, username_input)
        login_form.addRow(password_label, password_input)
        login_form.addRow(ip_label,ip_input)
        login_form.addRow(spacer)
        login_form.addWidget(self.login_button)

        form_widget = QWidget()
        form_widget.setLayout(login_form)
        form_widget.setStyleSheet("background-color: white; border-radius: 10px; padding: 20px;")

        self.dev_button = QPushButton("")
        self.dev_button.setStyleSheet("""
                QPushButton {
                    background-color: transparent;
                    border: none;
                    padding: 0;
                }
                QPushButton:hover {
                    background-color: #27ae60;
                    border: 1px solid white;
                    border-radius: 5px;
                }
            """)
        self.dev_button.setFixedSize(30, 10)
        self.dev_button.clicked.connect(self.dev_login)
        corner_button = QWidget()
        corner_layout = QHBoxLayout()
        corner_layout.addWidget(self.dev_button)
        corner_button.setLayout(corner_layout)

        center_layout = QHBoxLayout()
        center_layout.addStretch(1)  # Add flexible space before the form
        center_layout.addWidget(corner_button)
        center_layout.addWidget(form_widget)
        center_layout.addStretch(1)  # Add flexible space after the form
        center_layout.setAlignment(Qt.AlignCenter)  # Center the layout

        # --- Main Layout ---
        layout = QVBoxLayout()
        error_label = QLabel("")  # Label for error messages
        error_label.setFixedHeight(10)

        layout.addWidget(error_label)
        # layout.addWidget(corner_button)
        layout.addLayout(center_layout)  # Add the centered layout to the main layout

        # Set central widget
        container = QWidget()
        container.setLayout(layout)
        self.setCentralWidget(container)

        bg_label = ResizableBackground("./Gold-Brayer.png")
        bg_label.setParent(container)
        bg_label.setGeometry(container.rect())
        bg_label.lower()  # Ensure it's behind the overlay

    def dev_login(self):
        # Hardcoded developer credentials and IP
        os.environ['BOSDYN_CLIENT_USERNAME'] = "user2"
        os.environ['BOSDYN_CLIENT_PASSWORD'] = "simplepassword"
        global IP
        IP = "192.168.80.3"  # or use your actual dev IP

        self.close()  # close the login window
        mainInterface()  # launch the main interface directly

    def create_menu(self):
        menu_bar = self.menuBar()
        file_menu = menu_bar.addMenu("File")
        exit_action = QAction("Exit", self)
        exit_action.triggered.connect(self.close)
        file_menu.addAction(exit_action)

    def closeEvent(self, event):
        # Ensure everything is cleaned up before the app closes
        event.accept()  # Accept the event and close the application

class ResizableBackground(QLabel):
    def __init__(self, image_path):
        super().__init__()
        self.image_path = image_path
        self.setAlignment(Qt.AlignCenter)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def resizeEvent(self, event):
        pixmap = QPixmap(self.image_path)
        if not pixmap.isNull():
            self.setPixmap(pixmap.scaled(self.size(), Qt.KeepAspectRatioByExpanding, Qt.SmoothTransformation))

class RegisterDialog(QDialog):
    def __init__(self, conn, cursor, parent=None):
        super().__init__(parent)
        self.conn = conn
        self.cursor = cursor
        self.setWindowTitle("Register")
        self.setGeometry(100, 100, 400, 300)

        # --- Background Image ---
        bg_label = ResizableBackground("./Gold-Brayer.png")
        bg_label.setParent(self)
        bg_label.lower()  # Ensure it's behind all the other widgets

        # --- Form Layout ---
        login_form = QFormLayout()

        self.username_input = QLineEdit(self)
        self.password_input = QLineEdit(self)
        self.password_input.setEchoMode(QLineEdit.Password)

        user_label = QLabel("Username:")
        user_label.setStyleSheet("color: black; font-size: 16px; font-weight:bold;")
        self.username_input.setPlaceholderText("Enter username")
        self.username_input.setToolTip("Username must be unique")
        self.username_input.setMaxLength(20)
        self.username_input.setStyleSheet("background-color: #f0f0f0; color: black; font-size: 16px; border: 1px solid #ccc; border-radius: 5px;")
        
        password_label = QLabel("Password:")
        password_label.setStyleSheet("color: black; font-size: 16px; font-weight:bold;")
        self.password_input.setPlaceholderText("Enter password")
        self.password_input.setToolTip("Password must be at least 6 characters long")
        self.password_input.setMaxLength(20)
        self.password_input.setStyleSheet("background-color: #f0f0f0; color: black; font-size: 16px; border: 1px solid #ccc; border-radius: 5px;")

        login_form.addRow(user_label, self.username_input)
        login_form.addRow(password_label, self.password_input)

        # --- Spacer ---
        spacer = QLabel("")
        spacer.setFixedHeight(10)
        login_form.addRow(spacer)

        # --- Buttons ---
        self.register_button = QPushButton("Register", self)
        self.register_button.setStyleSheet("background-color: black; color: white; font-size: 14px;")
        self.register_button.clicked.connect(self.register)
        login_form.addRow(self.register_button)

        self.back_button = QPushButton("Go Back", self)
        self.back_button.setStyleSheet("background-color: lightGray; color: white; font-size: 14px;")
        self.back_button.clicked.connect(self.go_back)
        login_form.addRow(self.back_button)

        # --- Form Widget ---
        form_widget = QWidget()
        form_widget.setLayout(login_form)
        form_widget.setStyleSheet("background-color: white; border-radius: 10px; padding: 20px;")

        # --- Centering the Form ---
        center_layout = QHBoxLayout()
        center_layout.addStretch(1)  # Add flexible space before the form
        center_layout.addWidget(form_widget)
        center_layout.addStretch(1)  # Add flexible space after the form
        center_layout.setAlignment(Qt.AlignCenter)  # Center the layout

        # --- Main Layout ---
        layout = QVBoxLayout()
        error_label = QLabel("")  # Label for error messages
        error_label.setFixedHeight(10)
        layout.addWidget(error_label)
        layout.addLayout(center_layout)  # Add the centered layout to the main layout

        self.setLayout(layout)  # Set the main layout to the dialog

    def register(self):
        username = self.username_input.text()
        password = self.password_input.text()

        if not username or not password:
            QMessageBox.warning(self, "Input Error", "Please enter both username and password.")
            return

        if len(password) < 6:
            QMessageBox.warning(self, "Password Error", "Password must be at least 6 characters long.")
            return

        if self.user_exists(username):
            QMessageBox.warning(self, "Registration Error", "User  already exists.")
            return

        hashed_password = self.hash_password(password)
        cursor.execute('INSERT INTO users (username, password) VALUES (?, ?)', (username, hashed_password))
        conn.commit()
        QMessageBox.information(self, "Registration", "User  registered successfully.")
        self.accept()

    def user_exists(self, username):
        cursor.execute('SELECT * FROM users WHERE username = ?', (username,))
        return cursor.fetchone() is not None

    def hash_password(self, password):
        return hashlib.sha256(password.encode()).hexdigest()

    def go_back(self):
        self.close()

    def closeEvent(self, event):
        self.parent().show()  # Show LoginDialog again when RegisterDialog is closed
        super().closeEvent(event)

class LoginDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Login")
        self.setGeometry(100, 100, 400, 300)

        # --- Background Image ---
        bg_label = ResizableBackground("./Gold-Brayer.png")
        bg_label.setParent(self)
        bg_label.lower()  # Ensure it's behind all the other widgets

        # --- Form Layout ---
        login_form = QFormLayout()

        self.username_input = QLineEdit(self)
        self.password_input = QLineEdit(self)
        self.password_input.setEchoMode(QLineEdit.Password)

        user_label = QLabel("Username:")
        user_label.setStyleSheet("color: black; font-size: 16px; font-weight:bold;")
        self.username_input.setPlaceholderText("Enter username")
        self.username_input.setToolTip("Username must be unique")
        self.username_input.setMaxLength(20)
        self.username_input.setStyleSheet("background-color: white; color: black; font-size: 16px; border: 1px solid black; border-radius: 5px; padding: 15px;")
        
        password_label = QLabel("Password:")
        password_label.setStyleSheet("color: black; font-size: 16px; font-weight:bold;")
        self.password_input.setPlaceholderText("Enter password")
        self.password_input.setToolTip("Password must be at least 6 characters long")
        self.password_input.setMaxLength(20)
        self.password_input.setStyleSheet("background-color: white; color: black; font-size: 16px; border: 1px solid black; border-radius: 5px; padding: 15px;")

        login_form.addRow(user_label, self.username_input)
        login_form.addRow(password_label, self.password_input)

        # --- Spacer ---
        spacer = QLabel("")
        spacer.setFixedHeight(10)
        login_form.addRow(spacer)

        # --- Button layout ---
        button_layout = QVBoxLayout()

        # --- Buttons ---
        self.login_button = QPushButton("Login", self)
        self.login_button.setStyleSheet("background-color: black; color: white; font-size: 14px;")
        self.login_button.clicked.connect(self.login)
        button_layout.addWidget(self.login_button)

        self.register_button = QPushButton("Register", self)
        self.register_button.setStyleSheet("background-color: lightGray; color: black; font-size: 14px;")
        self.register_button.clicked.connect(self.register)
        button_layout.addWidget(self.register_button)

        # self.dev_button = QPushButton("")
        # self.dev_button.setStyleSheet("""
        #         QPushButton {
        #             background-color: transparent;
        #             border: none;
        #         }
        #         QPushButton:hover {
        #             background-color: #27ae60;
        #             border: 1px solid white;
        #             border-radius: 5px;
        #         }
        #     """)
        # self.dev_button.setFixedSize(30, 30)  # Small clickable area
        # self.dev_button.setToolTip("Developer Login")
        # self.dev_button.clicked.connect(self.dev_login)
        # corner_button = QWidget()
        # corner_layout = QHBoxLayout()
        # corner_layout.addStretch()
        # corner_layout.addWidget(self.dev_button)
        # corner_button.setLayout(corner_layout)

        # --- Add button layout to form ---
        login_form.addRow(button_layout)
        # login_form.addRow(corner_button)

        # --- Form Widget ---
        form_widget = QWidget()
        form_widget.setLayout(login_form)
        form_widget.setStyleSheet("background-color: white; border-radius: 10px; padding: 20px;")

        # --- Centering the Form ---
        center_layout = QHBoxLayout()
        center_layout.addStretch(1)  # Add flexible space before the form
        center_layout.addWidget(form_widget)
        center_layout.addStretch(1)  # Add flexible space after the form
        center_layout.setAlignment(Qt.AlignCenter)  # Center the layout

        # --- Main Layout ---
        layout = QVBoxLayout()
        self.error_label = QLabel("")
        self.error_label.setFixedHeight(10)
        layout.addWidget(self.error_label)
        # layout.addWidget(corner_button)
        layout.setAlignment(Qt.AlignCenter)
        layout.addLayout(center_layout)  # Add the centered layout to the main layout

        self.setLayout(layout)  # Set the main layout to the dialog

        # Initialize database
        self.init_db()

    # def dev_login(self):
    #     # Hardcoded developer credentials and IP
    #     os.environ['BOSDYN_CLIENT_USERNAME'] = "user2"
    #     os.environ['BOSDYN_CLIENT_PASSWORD'] = "simplepassword"
    #     global IP
    #     IP = "192.168.80.3"  # or use your actual dev IP

    #     self.close()  # close the login window
    #     mainInterface()  # launch the main interface directly

    def login(self):
        username = self.username_input.text()
        password = self.password_input.text()
        if self.user_exists(username) and self.verify_password(username, password):
            self.accept()
        else:
            self.error_label.setText("Invalid credentials")
            self.error_label.setStyleSheet("color: red; font-weight: bold; font-size: 14px;")

    def init_db(self):
        self.conn = sqlite3.connect('users.db')
        self.cursor = self.conn.cursor()
        self.cursor.execute('''CREATE TABLE IF NOT EXISTS users (username TEXT PRIMARY KEY, password TEXT NOT NULL)''')
        self.conn.commit()

    def user_exists(self, username):
        self.cursor.execute('SELECT * FROM users WHERE username = ?', (username,))
        return self.cursor.fetchone() is not None

    def verify_password(self, username, password):
        cursor.execute('SELECT password FROM users WHERE username = ?', (username,))
        stored_password = cursor.fetchone()
        if stored_password:
            return stored_password[0] == self.hash_password(password)
        return False

    def hash_password(self, password):
        return hashlib.sha256(password.encode()).hexdigest()

    def register(self):
        self.setEnabled(False)
        dialog = RegisterDialog(self.conn, self.cursor, self)
        dialog.exec_()
        self.setEnabled(True)

    def closeEvent(self, event):
        conn.close()  # Close the database connection
        sys.exit()
        print("Login dialog closed")  # Optional logging
        super().closeEvent(event)  # ✅ Just call the parent handler

    def reject(self):
        # User hit the X or pressed Esc
        super().reject()

if __name__ == "__main__":
    app = QApplication(sys.argv)

    # Create the main window but don't show it yet
    main_window = RobotControlApp()
    
    # Show login dialog BEFORE showing main window
    login_dialog = LoginDialog(main_window)
    if login_dialog.exec_() == QDialog.Accepted:
        main_window.statusBar().showMessage("User Login successful")
        main_window.show()  # Now show it
        sys.exit(app.exec_())
    elif login_dialog.exec_() == QDialog.Rejected:
        conn.close()
        sys.exit()  # Exit if login canceled
