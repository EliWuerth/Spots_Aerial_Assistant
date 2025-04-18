import tkinter as tk
from tkinter import messagebox
from PyQt5 import QtWidgets, QtGui, QtCore
from PyQt5.QtWidgets import (QApplication, QMainWindow, QLabel, QPushButton, QVBoxLayout, QHBoxLayout, QWidget, QLineEdit, QComboBox, QMessageBox, QStackedLayout,QSizePolicy)
from PyQt5.QtGui import QPalette, QPixmap, QBrush, QImage, QIcon
from PyQt5.QtCore import Qt, QTimer
import sys
import os
from PIL import Image, ImageTk, ImageEnhance
import curses
import io
import logging
import math
import os
import signal
import sys
import threading
import time
import bosdyn.client
from bosdyn.client.frame_helpers import ODOM_FRAME_NAME
import bosdyn.client.util
from bosdyn.client.lease import *
from bosdyn.client.robot_command import RobotCommandClient, RobotCommandBuilder, blocking_stand
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.image import ImageClient
from bosdyn.client.robot import Robot
from bosdyn.api import geometry_pb2,robot_state_pb2, basic_command_pb2
from bosdyn.util import seconds_to_duration
from collections import OrderedDict
from bosdyn.client import Robot
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive
from bosdyn.client.estop import EstopClient, EstopEndpoint, EstopKeepAlive
from bosdyn.client.power import PowerClient
from bosdyn.client.async_tasks import AsyncTasks, AsyncGRPCTask, AsyncPeriodicQuery
import bosdyn.api.basic_command_pb2 as basic_command_pb2
import bosdyn.api.power_pb2 as PowerServiceProto
import bosdyn.api.robot_state_pb2 as robot_state_proto
import bosdyn.api.spot.robot_command_pb2 as spot_command_pb2
from bosdyn.client import ResponseError, RpcError, create_standard_sdk
from bosdyn.client.lease import Error as LeaseBaseError
from bosdyn.client.time_sync import TimeSyncError
from bosdyn.util import duration_str, format_metric, secs_to_hms
import bosdyn.api.power_pb2 as power_pb2
from bosdyn.client import power
from bosdyn.api.robot_state_pb2 import PowerState

current_process = None
lease_client = None
lease_keep_alive = None
robot = None
command_client = None
image_client = None
wasd_interface = None
IP = ""
VELOCITY_CMD_DURATION = 0.6  # seconds
MAX_LINEAR_VELOCITY = 0.5  # m/s
MAX_ANGULAR_VELOCITY = 0.8  # rad/s

LOGGER = logging.getLogger()

VELOCITY_BASE_SPEED = 0.5  # m/s
VELOCITY_BASE_ANGULAR = 0.8  # rad/sec
VELOCITY_CMD_DURATION = 0.6  # seconds
COMMAND_INPUT_RATE = 0.1

camera_label = None
status_label = None
battery_label = None
canvas = None
root = None

# from the wasd.py file
def _image_to_ascii(image, new_width):
    """Convert an rgb image to an ASCII 'image' that can be displayed in a terminal."""
    ASCII_CHARS = '@#S%?*+;:,.'

    enhancer = ImageEnhance.Contrast(image)
    image = enhancer.enhance(0.8)

    # Scaling image before rotation by 90 deg.
    scaled_rot_height = new_width
    original_rot_width, original_rot_height = image.size
    scaled_rot_width = (original_rot_width * scaled_rot_height) // original_rot_height
    # Scaling rotated width (height, after rotation) by half because ASCII chars
    #  in terminal seem about 2x as tall as wide.
    image = image.resize((scaled_rot_width // 2, scaled_rot_height))

    # Rotate image 90 degrees, then convert to grayscale.
    image = image.transpose(Image.ROTATE_270)
    image = image.convert('L')

    def _pixel_char(pixel_val):
        return ASCII_CHARS[pixel_val * len(ASCII_CHARS) // 256]

    img = []
    row = [' '] * new_width
    last_col = new_width - 1
    for idx, pixel_char in enumerate(_pixel_char(val) for val in image.getdata()):
        idx_row = idx % new_width
        row[idx_row] = pixel_char
        if idx_row == last_col:
            img.append(''.join(row))
    return img

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

class CursesHandler(logging.Handler):
    """logging handler which puts messages into the curses interface"""
    def __init__(self, wasd_interface):
        super(CursesHandler, self).__init__()
        self._wasd_interface = wasd_interface

    def emit(self, record):
        msg = record.getMessage()
        msg = msg.replace('\n', ' ').replace('\r', '')
        self._wasd_interface.add_message(f'{record.levelname:s} {msg:s}')

class AsyncRobotState(AsyncPeriodicQuery):
    """Grab robot state."""
    def __init__(self, robot_state_client):
        super(AsyncRobotState, self).__init__('robot_state', robot_state_client, LOGGER, period_sec=0.2)

    def _start_query(self):
        return self._client.get_robot_state_async()

class AsyncImageCapture(AsyncGRPCTask):
    """Grab camera images from the robot."""
    def __init__(self, robot):
        super(AsyncImageCapture, self).__init__()
        self._image_client = robot.ensure_client(ImageClient.default_service_name)
        self._ascii_image = None
        self._video_mode = False
        self._should_take_image = False

    @property
    def ascii_image(self):
        """Return the latest captured image as ascii."""
        return self._ascii_image

    def toggle_video_mode(self):
        """Toggle whether doing continuous image capture."""
        self._video_mode = not self._video_mode

    def take_image(self):
        """Request a one-shot image."""
        self._should_take_image = True

    def _start_query(self):
        self._should_take_image = False
        source_name = 'frontright_fisheye_image'
        return self._image_client.get_image_from_sources_async([source_name])

    def _should_query(self, now_sec):  # pylint: disable=unused-argument
        return self._video_mode or self._should_take_image

    def _handle_result(self, result):
        import io
        image = Image.open(io.BytesIO(result[0].shot.image.data))
        self._ascii_image = _image_to_ascii(image, new_width=70)

    def _handle_error(self, exception):
        LOGGER.exception('Failure getting image: %s', exception)

class WasdInterface(object):
    """A curses interface for driving the robot."""
    def __init__(self, robot):
        self._robot = robot
        print("WasdInterface: __init__ called")  # Added debug log
        # Create clients -- do not use the for communication yet.
        self._lease_client = robot.ensure_client(LeaseClient.default_service_name)
        try:
            self._estop_client = self._robot.ensure_client(EstopClient.default_service_name)
            self._estop_endpoint = EstopEndpoint(self._estop_client, 'GNClient', 9.0)
        except:
            # Not the estop.
            self._estop_client = None
            self._estop_endpoint = None
        self._power_client = robot.ensure_client(PowerClient.default_service_name)
        self._robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)
        self._robot_command_client = robot.ensure_client(RobotCommandClient.default_service_name)
        self._robot_state_task = AsyncRobotState(self._robot_state_client)
        self._image_task = AsyncImageCapture(robot)
        self._async_tasks = AsyncTasks([self._robot_state_task, self._image_task])
        self._lock = threading.Lock()
        self._command_dictionary = {
            27: self._stop,  # ESC key
            '\t': self._quit_program,
            't': self._toggle_time_sync,
            ' ': self._toggle_estop,
            'r': self._self_right,
            'P': self._toggle_power,
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
            'i': self._image_task.take_image,
            'O': self._image_task.toggle_video_mode,
            'u': self._unstow,
            'j': self._stow,
            'l': self._toggle_lease
        }
        self._locked_messages = ['', '', '']  # string: displayed message for user
        self._estop_keepalive = None
        self._exit_check = None

        # Stuff that is set in start()
        self._robot_id = None
        self._lease_keepalive = None

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

    def drive(self, stdscr):
        """User interface to control the robot via the passed-in curses screen interface object."""
        with ExitCheck() as self._exit_check:
            curses_handler = CursesHandler(self)
            curses_handler.setLevel(logging.INFO)
            LOGGER.addHandler(curses_handler)

            stdscr.nodelay(True)  # Don't block for user input.
            stdscr.resize(26, 140)
            stdscr.refresh()

            # for debug
            curses.echo()

            try:
                while not self._exit_check.kill_now:
                    self._async_tasks.update()
                    self._drive_draw(stdscr, self._lease_keepalive)

                    try:
                        cmd = stdscr.getch()
                        # Do not queue up commands on client
                        self.flush_and_estop_buffer(stdscr)
                        self._drive_cmd(cmd)
                        time.sleep(COMMAND_INPUT_RATE)
                    except Exception:
                        # On robot command fault, sit down safely before killing the program.
                        self._safe_power_off()
                        time.sleep(2.0)
                        raise

            finally:
                LOGGER.removeHandler(curses_handler)

    def _drive_draw(self, stdscr, lease_keep_alive):
        """Draw the interface screen at each update."""
        stdscr.clear()  # clear screen
        stdscr.resize(26, 140)
        stdscr.addstr(0, 0, f'{self._robot_id.nickname:20s} {self._robot_id.serial_number}')
        stdscr.addstr(1, 0, self._lease_str(lease_keep_alive))
        stdscr.addstr(2, 0, self._battery_str())
        stdscr.addstr(3, 0, self._estop_str())
        stdscr.addstr(4, 0, self._power_state_str())
        stdscr.addstr(5, 0, self._time_sync_str())
        for i in range(3):
            stdscr.addstr(7 + i, 2, self.message(i))
        stdscr.addstr(10, 0, 'Commands: [TAB]: quit                               ')
        stdscr.addstr(11, 0, '          [T]: Time-sync, [SPACE]: Estop, [P]: Power')
        stdscr.addstr(12, 0, '          [I]: Take image, [O]: Video mode          ')
        stdscr.addstr(13, 0, '          [f]: Stand, [r]: Self-right               ')
        stdscr.addstr(14, 0, '          [v]: Sit, [b]: Battery-change             ')
        stdscr.addstr(15, 0, '          [wasd]: Directional strafing              ')
        stdscr.addstr(16, 0, '          [qe]: Turning, [ESC]: Stop                ')
        stdscr.addstr(17, 0, '          [l]: Return/Acquire lease                 ')
        stdscr.addstr(18, 0, '')

        # print as many lines of the image as will fit on the curses screen
        if self._image_task.ascii_image is not None:
            max_y, _max_x = stdscr.getmaxyx()
            for y_i, img_line in enumerate(self._image_task.ascii_image):
                if y_i + 17 >= max_y:
                    break

                stdscr.addstr(y_i + 17, 0, img_line)

        stdscr.refresh()

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

    def _toggle_lease(self):
        """toggle lease acquisition. Initial state is acquired"""
        if self._lease_client is not None:
            if self._lease_keepalive is None:
                self._lease_keepalive = LeaseKeepAlive(self._lease_client, must_acquire=True, return_at_exit=True)
            else:
                self._lease_keepalive.shutdown()
                self._lease_keepalive = None

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

    def _stow(self):
        self._start_robot_command('stow', RobotCommandBuilder.arm_stow_command())

    def _unstow(self):
        self._start_robot_command('stow', RobotCommandBuilder.arm_ready_command())

    def _return_to_origin(self):
        self._start_robot_command(
            'fwd_and_rotate',
            RobotCommandBuilder.synchro_se2_trajectory_point_command(goal_x=0.0, goal_y=0.0, goal_heading=0.0, frame_name=ODOM_FRAME_NAME, params=None, body_height=0.0, locomotion_hint=spot_command_pb2.HINT_SPEED_SELECT_TROT), end_time_secs=time.time() + 20)

    def _take_ascii_image(self):
        source_name = 'frontright_fisheye_image'
        image_response = self._image_client.get_image_from_sources([source_name])
        image = Image.open(io.BytesIO(image_response[0].shot.image.data))
        ascii_image = self._ascii_converter.convert_to_ascii(image, new_width=70)
        self._last_image_ascii = ascii_image

    def _toggle_ascii_video(self):
        if self._video_mode:
            self._video_mode = False
        else:
            self._video_mode = True

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

    def _battery_str(self):
        if not self.robot_state:
            return ''
        battery_state = self.robot_state.battery_states[0]
        status = battery_state.Status.Name(battery_state.status)
        status = status[7:]  # get rid of STATUS_ prefix
        if battery_state.charge_percentage.value:
            bar_len = int(battery_state.charge_percentage.value) // 10
            bat_bar = f'|{"=" * bar_len}{" " * (10 - bar_len)}|'
        else:
            bat_bar = ''
        time_left = ''
        if battery_state.estimated_runtime:
            time_left = f'({secs_to_hms(battery_state.estimated_runtime.seconds)})'
        return f'Battery: {status}{bat_bar} {time_left}'

def _setup_logging(verbose):
    """Log to file at debug level, and log to console at INFO or DEBUG (if verbose).

    Returns the stream/console logger so that it can be removed when in curses mode.
    """
    LOGGER.setLevel(logging.DEBUG)
    log_formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')

    # Save log messages to file wasd.log for later debugging.
    file_handler = logging.FileHandler('wasd.log')
    file_handler.setLevel(logging.DEBUG)
    file_handler.setFormatter(log_formatter)
    LOGGER.addHandler(file_handler)

    # The stream handler is useful before and after the application is in curses-mode.
    if verbose:
        stream_level = logging.DEBUG
    else:
        stream_level = logging.INFO

    stream_handler = logging.StreamHandler()
    stream_handler.setLevel(stream_level)
    stream_handler.setFormatter(log_formatter)
    LOGGER.addHandler(stream_handler)
    return stream_handler

def check_battery_status(widget):
    global battery_label, IP, robot

    username = os.environ.get('BOSDYN_CLIENT_USERNAME')
    password = os.environ.get('BOSDYN_CLIENT_PASSWORD')

    def retry_later():
        QTimer.singleShot(60000, lambda: check_battery_status(widget))  # retry in 60 sec

    if not username or not password or not IP:
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
    global lease_client, lease_keep_alive, robot, command_client, image_client, wasd_interface
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
    global lease_keep_alive, lease_client
    if lease_keep_alive:
        lease_keep_alive.shutdown()
    if lease_client:
        lease_client.return_lease()
    status_label.config(text="Lease released")

def stopProgram():
    release_lease()
    status_label.config(text="Stopped - Lease released")

def startEStop():
    try:
        status_label.config(text="E-Stop engaged (mock)")
    except Exception as e:
        status_label.config(text=f"E-Stop error: {str(e)}")

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

def update_camera_feed():
    global image_client
    try:
        selected_camera = camera_selector.currentText()
        image_response = image_client.get_image_from_sources([selected_camera])[0]
        img_data = image_response.shot.image.data
        cols = image_response.shot.image.cols
        rows = image_response.shot.image.rows

        if not img_data:
            camera_label.setText("No image data")
            return

        q_image = QImage(img_data, cols, rows, QImage.Format_Grayscale8)
        pixmap = QPixmap.fromImage(q_image).scaled(300, 200)
        camera_label.setPixmap(pixmap)
    except Exception as e:
        camera_label.setText(f"Camera Error:\n{str(e)}")
        camera_label.setPixmap(QPixmap())

    QTimer.singleShot(100, update_camera_feed)

def run_with_inputs(username, password, ip):
    if not username or not password or not ip:
        QMessageBox.critical(None, "Error", "All fields are required!")
        return

    os.environ['BOSDYN_CLIENT_USERNAME'] = username
    os.environ['BOSDYN_CLIENT_PASSWORD'] = password

    global IP
    IP = ip

    login_window.close()
    mainInterface()

def mainInterface():
    global status_label, battery_label, camera_label, main_window, camera_selector

    class SpotMainWindow(QMainWindow):
        def keyPressEvent(self, event):
            on_key_press(event)

    main_window = SpotMainWindow()
    main_window.setWindowTitle("Spot Control Panel")
    main_window.setGeometry(100, 100, 800, 600)

    # -- Central Widget --
    central_widget = QWidget()
    main_window.setCentralWidget(central_widget)

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
    overlay_layout.setAlignment(Qt.AlignCenter)
    overlay_widget.setLayout(overlay_layout)
    overlay_widget.setGeometry(central_widget.rect())  # Initially fill the central widget

    # -- Container with buttons/labels --
    container = QWidget()
    container_layout = QVBoxLayout()
    container_layout.setAlignment(Qt.AlignHCenter)

    # Battery label
    battery_label = QLabel("Battery: N/A")
    battery_label.setStyleSheet("color: green; font-size: 20px; font-style:bold;")
    battery_label.setAlignment(Qt.AlignCenter)
    container_layout.addWidget(battery_label)

    # Camera selector
    cam_layout = QHBoxLayout()
    cam_layout.addWidget(QLabel("Select Camera:"))
    cam_layout.setAlignment(Qt.AlignLeft)
    camera_selector = QComboBox()
    camera_selector.setFixedWidth(150)
    camera_selector.setFixedHeight(30)
    camera_selector.setStyleSheet(""" QComboBox { background-color: #2c3e50; color: white; font-size: 14px; border: 1px solid #34495e; border-radius: 5px; padding: 5px; }
        QComboBox::drop-down { background-color: #2980b9; }
        QComboBox::indicator { width: 20px; height: 20px; }
        QComboBox::item { background-color: #34495e; color: white; }
        QComboBox::item:selected { background-color: #2980b9; color: white; }""")
    camera_selector.addItems(["frontleft", "frontright", "rear", "depth", "hand", "spot-cam"])
    cam_layout.addWidget(camera_selector)
    container_layout.addLayout(cam_layout)

    # Camera status label
    camera_label = QLabel("Connecting to camera...")
    camera_label.setStyleSheet("background-color: black; color: white; font-size: 16px; padding: 5px; border-radius: 5px;")
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
    container_layout.addWidget(status_label)

    # --- Spacer ---
    spacer = QLabel("")
    spacer.setFixedHeight(10)
    container_layout.addWidget(spacer)

    # Command buttons in a horizontal layout
    button_layout = QHBoxLayout()  # Create a horizontal layout for buttons
    button_layout.setAlignment(Qt.AlignCenter)

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

def createLoginWindow():
    global login_window
    login_window = QWidget()
    login_window.setWindowTitle("Spot Login")
    login_window.setGeometry(200, 200, 400, 300)

    layout = QVBoxLayout()

    username_input = QLineEdit()
    password_input = QLineEdit()
    ip_input = QLineEdit()

    password_input.setEchoMode(QLineEdit.Password)

    login_button = QPushButton("Login")
    login_button.clicked.connect(lambda: run_with_inputs(username_input.text(), password_input.text(), ip_input.text()))
    
    # --- Spacer ---
    spacer = QLabel("")
    spacer.setFixedHeight(10)

    layout.addWidget(QLabel("Username:"))
    layout.addWidget(username_input)
    layout.addWidget(QLabel("Password:"))
    layout.addWidget(password_input)
    layout.addWidget(QLabel("IP Address:"))
    layout.addWidget(ip_input)
    layout.addWidget(spacer)
    layout.addWidget(login_button)

    login_window.setLayout(layout)
    login_window.show()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    createLoginWindow()
    sys.exit(app.exec_())
