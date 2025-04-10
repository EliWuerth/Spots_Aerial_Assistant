import tkinter as tk
from tkinter import messagebox
import bosdyn.client
from bosdyn.client.frame_helpers import ODOM_FRAME_NAME
import bosdyn.client.util
import time
from bosdyn.client.lease import *
from bosdyn.client.robot_command import RobotCommandClient, RobotCommandBuilder, blocking_stand
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.image import ImageClient
import examples.wasd
from bosdyn.client import create_standard_sdk
from bosdyn.client.robot import Robot
from bosdyn.api import geometry_pb2
from bosdyn.util import seconds_to_duration
from PyQt5.QtWidgets import (QApplication, QMainWindow, QLabel, QPushButton, QVBoxLayout, QHBoxLayout, QWidget)
from PyQt5.QtGui import QPalette, QPixmap, QBrush
from PyQt5.QtCore import Qt
import os
from PIL import Image, ImageTk


import curses
import io
import logging
import math
import os
import signal
import sys
import threading
import time
from collections import OrderedDict
from bosdyn.client import Robot
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive
from bosdyn.client.estop import EstopClient, EstopEndpoint, EstopKeepAlive
from bosdyn.client.power import PowerClient, PowerServiceProto
from bosdyn.client.robot_command import RobotCommandClient, RobotCommandBuilder
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.image import ImageClient
from bosdyn.client.async_tasks import AsyncTasks, AsyncRobotState, AsyncImageCapture
from bosdyn.api import robot_state_pb2, basic_command_pb2, spot_command_pb2
import io

import bosdyn.api.basic_command_pb2 as basic_command_pb2
import bosdyn.api.power_pb2 as PowerServiceProto
# import bosdyn.api.robot_command_pb2 as robot_command_pb2
import bosdyn.api.robot_state_pb2 as robot_state_proto
import bosdyn.api.spot.robot_command_pb2 as spot_command_pb2
import bosdyn.client.util
from bosdyn.api import geometry_pb2
from bosdyn.client import ResponseError, RpcError, create_standard_sdk
from bosdyn.client.async_tasks import AsyncGRPCTask, AsyncPeriodicQuery, AsyncTasks
from bosdyn.client.estop import EstopClient, EstopEndpoint, EstopKeepAlive
from bosdyn.client.frame_helpers import ODOM_FRAME_NAME
from bosdyn.client.image import ImageClient
from bosdyn.client.lease import Error as LeaseBaseError
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive
from bosdyn.client.power import PowerClient
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.time_sync import TimeSyncError
from bosdyn.util import duration_str, format_metric, secs_to_hms

current_process = None
lease_client = None
lease_keep_alive = None
robot = None
command_client = None
image_client = None
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
        super(AsyncRobotState, self).__init__('robot_state', robot_state_client, LOGGER,
                                              period_sec=0.2)

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
            ord('\t'): self._quit_program,
            ord('T'): self._toggle_time_sync,
            ord(' '): self._toggle_estop,
            ord('r'): self._self_right,
            ord('P'): self._toggle_power,
            ord('p'): self._toggle_power,
            ord('v'): self._sit,
            ord('b'): self._battery_change_pose,
            ord('f'): self._stand,
            ord('w'): self._move_forward,
            ord('s'): self._move_backward,
            ord('a'): self._strafe_left,
            ord('d'): self._strafe_right,
            ord('q'): self._turn_left,
            ord('e'): self._turn_right,
            ord('I'): self._image_task.take_image,
            ord('O'): self._image_task.toggle_video_mode,
            ord('u'): self._unstow,
            ord('j'): self._stow,
            ord('l'): self._toggle_lease
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
        self._lease_keepalive = LeaseKeepAlive(self._lease_client, must_acquire=True,
                                               return_at_exit=True)

        self._robot_id = self._robot.get_id()
        if self._estop_endpoint is not None:
            self._estop_endpoint.force_simple_setup(
            )  # Set this endpoint as the robot's sole estop.

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
            if key == ord(' '):
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
                self._lease_keepalive = LeaseKeepAlive(self._lease_client, must_acquire=True,
                                                       return_at_exit=True)
            else:
                self._lease_keepalive.shutdown()
                self._lease_keepalive = None

    def _start_robot_command(self, desc, command_proto, end_time_secs=None):

        def _start_command():
            self._robot_command_client.robot_command(command=command_proto,
                                                     end_time_secs=end_time_secs)

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
        self._start_robot_command(
            desc, RobotCommandBuilder.synchro_velocity_command(v_x=v_x, v_y=v_y, v_rot=v_rot),
            end_time_secs=time.time() + VELOCITY_CMD_DURATION)

    def _stow(self):
        self._start_robot_command('stow', RobotCommandBuilder.arm_stow_command())

    def _unstow(self):
        self._start_robot_command('stow', RobotCommandBuilder.arm_ready_command())

    def _return_to_origin(self):
        self._start_robot_command(
            'fwd_and_rotate',
            RobotCommandBuilder.synchro_se2_trajectory_point_command(
                goal_x=0.0, goal_y=0.0, goal_heading=0.0, frame_name=ODOM_FRAME_NAME, params=None,
                body_height=0.0, locomotion_hint=spot_command_pb2.HINT_SPEED_SELECT_TROT),
            end_time_secs=time.time() + 20)

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

def check_battery_status(root):
    global battery_label, IP, robot

    username = os.environ.get('BOSDYN_CLIENT_USERNAME')
    password = os.environ.get('BOSDYN_CLIENT_PASSWORD')

    if not username or not password or not IP:
        battery_label.config(text="Invalid Log-In")
        root.after(60000, check_battery_status, root)  
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
            battery_label.config(text=f"Battery: {battery_percentage}%")
        else:
            battery_label.config(text="Battery info unavailable")

    except Exception as e:
        battery_label.config(text=f"Error: {str(e)}")

    root.after(60000, check_battery_status, root)

def take_lease():
    global lease_client, lease_keep_alive, robot, command_client, image_client
    try:
        sdk = create_standard_sdk("SpotController")
        robot = sdk.create_robot(IP)
        robot.authenticate(os.environ['BOSDYN_CLIENT_USERNAME'], os.environ['BOSDYN_CLIENT_PASSWORD'])

        lease_client = robot.ensure_client(LeaseClient.default_service_name)
        lease = lease_client.take()
        lease_keep_alive = LeaseKeepAlive(lease_client)

        command_client = robot.ensure_client(RobotCommandClient.default_service_name)
        image_client = robot.ensure_client(ImageClient.default_service_name)

        blocking_stand(command_client)

        status_label.config(text="Lease acquired - Ready for commands")
        return True
    except Exception as e:
        status_label.config(text=f"Lease error: {str(e)}")
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
    global robot, command_client

    if not robot.is_powered_on():
        robot.power_on(timeout_sec=20)

    if not robot or not command_client:
        if not take_lease():
            return False

    try:
        cmd = RobotCommandBuilder.synchro_velocity_command(v_x=v_x, v_y=v_y, v_rot=v_rot, duration=VELOCITY_CMD_DURATION)
        command_client.robot_command(cmd)

        return True
    except Exception as e:
        status_label.config(text=f"Move error: {str(e)}")
        print({e})

        return False

def on_key_press(event):
    key = event.char.lower()
    if key == 'w':
        move_robot(MAX_LINEAR_VELOCITY, 0, 0)
    elif key == 's':
        move_robot(-MAX_LINEAR_VELOCITY, 0, 0)
    elif key == 'a':
        move_robot(0, MAX_LINEAR_VELOCITY, 0)
    elif key == 'd':
        move_robot(0, -MAX_LINEAR_VELOCITY, 0)
    elif key == 'q':
        move_robot(0, 0, MAX_ANGULAR_VELOCITY)
    elif key == 'e':
        move_robot(0, 0, -MAX_ANGULAR_VELOCITY)
    elif key == ' ':
        try:
            blocking_stand(command_client)
            status_label.config(text="Standing")
        except Exception as e:
            status_label.config(text=f"Stand error: {str(e)}")
            print({e})

    elif key == 'x':
        try:
            # Fixing sit command usage
            cmd = RobotCommandBuilder.synchro_stand_command()  # Command to sit or transition to sit
            command_client.robot_command(cmd)
            status_label.config(text="Sitting")
        except Exception as e:
            status_label.config(text=f"Sitting error: {str(e)}")
            print({e})

def update_camera_feed():
    global image_client, camera_label, root, camera_selector
    try:
        # Get the selected camera name from the dropdown
        selected_camera = camera_selector.get()

        # Attempt to get a valid image response from the selected camera
        image_response = image_client.get_image_from_sources([selected_camera])[0]
        img_data = image_response.shot.image.data

        if not img_data:  # Check if image data is available
            camera_label.config(text="No image data")
            return

        # Convert image data to displayable format
        img = Image.frombytes('L', (image_response.shot.image.cols, image_response.shot.image.rows), img_data)
        img = img.resize((300, 200))
        img = ImageTk.PhotoImage(img)

        # Update the camera label with the new image
        camera_label.configure(image=img)
        camera_label.image = img
    except Exception as e:
        camera_label.configure(text=f"Camera Error:\n{str(e)}", image='')
        print({e})
    
    # Continue updating the feed every 100ms
    root.after(100, update_camera_feed)

def run_with_inputs(username, password, ip):
    if not username or not password or not ip:
        messagebox.showerror("Error", "All fields required!")
        return

    os.environ['BOSDYN_CLIENT_USERNAME'] = username
    os.environ['BOSDYN_CLIENT_PASSWORD'] = password

    global IP
    IP = ip

    login_window.destroy()
    mainInterface()
    
def mainInterface():
    global status_label, battery_label, camera_label, root, camera_selector

    root = tk.Tk()
    root.geometry("600x600")
    root.title("Spot Control Panel")

    try:
        bg_image = Image.open("Gold-Brayer2.png")
        bg_image = bg_image.resize((600, 600))
        bg_image = ImageTk.PhotoImage(bg_image)
        canvas = tk.Canvas(root, width=600, height=600)
        canvas.pack(fill="both", expand=True)
        canvas.create_image(0, 0, image=bg_image, anchor="nw")
    except:
        canvas = tk.Canvas(root, width=600, height=600, bg='gray')
        canvas.pack(fill="both", expand=True)

    # Initialize status_label early
    status_label = tk.Label(root, text="Ready", font=('Arial', 12), fg="blue", bg="white")
    status_label.place(x=250, y=400)

    # Initialize battery_label early
    battery_label = tk.Label(root, text="Battery: N/A", font=('Arial', 12), fg="green", bg="white")
    battery_label.place(x=50, y=50)  # Position the battery status label

    # Set up camera selector dropdown
    camera_selector = tk.StringVar()
    camera_selector.set("frontleft")  # Default camera

    tk.Label(root, text="Select Camera:").place(x=50, y=100)
    camera_dropdown = tk.OptionMenu(root, camera_selector, "frontleft", "frontright", "rear", "depth", "hand", "spot-cam")
    camera_dropdown.place(x=150, y=100)

    # Buttons for controlling the robot, etc.
    tk.Button(root, text="Stand", font=('arial', 12), command=lambda: on_key_press(type('Event', (object,), {'char': ' '})())).place(x=50, y=150)
    tk.Button(root, text="Sit", font=('arial', 12), command=lambda: on_key_press(type('Event', (object,), {'char': 'x'})())).place(x=50, y=200)

    # Setup the camera feed label
    camera_label = tk.Label(root, text="Connecting to camera...", bg='black', fg='white')
    camera_label.place(x=150, y=430)

    # Update the camera feed when a new camera is selected
    camera_selector.trace("w", lambda *args: update_camera_feed())

    # Start checking the battery status periodically
    check_battery_status(root)

    root.mainloop()

def createLoginWindow():
    global login_window
    login_window = tk.Tk()
    login_window.geometry("400x300")
    login_window.title("Spot Login")

    tk.Label(login_window, text="Username:").pack(pady=10)
    username_entry = tk.Entry(login_window)
    username_entry.pack(pady=5)

    tk.Label(login_window, text="Password:").pack(pady=10)
    password_entry = tk.Entry(login_window, show="*")
    password_entry.pack(pady=5)

    tk.Label(login_window, text="IP Address:").pack(pady=10)
    ip_entry = tk.Entry(login_window)
    ip_entry.pack(pady=5)

    login_button = tk.Button(login_window, text="Login", command=lambda: run_with_inputs(username_entry.get(), password_entry.get(), ip_entry.get()))
    login_button.pack(pady=20)

    login_window.mainloop()

if __name__ == "__main__":
    createLoginWindow()
