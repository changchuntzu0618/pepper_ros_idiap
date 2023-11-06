#!/usr/bin/env python

"""
Created on Mon Jun 27 09:18:12 2016

@author: Christian Dondrup
"""

import time
import configargparse
import almath
from naoqi import ALProxy


class _Getch:
    """
    Gets a single character from standard input.  Does not echo to the
    screen.
    """
    def __init__(self):
        try:
            self.impl = _GetchWindows()
        except ImportError:
            self.impl = _GetchUnix()

    def __call__(self): return self.impl()


class _GetchUnix:
    def __init__(self):
        import tty, sys

    def __call__(self):
        import sys, tty, termios
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch


class _GetchWindows:
    def __init__(self):
        import msvcrt

    def __call__(self):
        import msvcrt
        return msvcrt.getch()


class KeyBoardTeleOp(object):
    def __init__(self, robot_ip, port):
        self.robot_ip = robot_ip
        self.port = port
        self.connect(robot_ip, port)
        self.vels = {
            'x': {'current': .0, 'min': -.55, 'max': .55},
            'y': {'current': .0, 'min': .0, 'max': .0},
            't': {'current': .0, 'min': -1., 'max': 1.}
        }

        # Variable for head movement
        self.yaw_angle = 10.0*almath.TO_RAD
        self.yaw_time = 0.5
        self.pitch_angle = 5.0*almath.TO_RAD
        self.pitch_time = 0.7

    def connect(self, robot_ip, port):
        try:
            self.motion_proxy  = ALProxy("ALMotion", robot_ip, port)
            print "Connected to %s:%s" % (robot_ip, str(port))
        except RuntimeError:
            print "Cannot connect to %s:%s. Retrying in 1 second." % (robot_ip, str(port))
            time.sleep(1)
            self.connect(robot_ip, port)

    def spin(self):
        getch = _Getch()
        ch = None
        print "Waiting for key"
        while ch != 'q':
            ch = getch()
            if ch == 'w':
                if self.vels['x']['current'] + 0.05 < self.vels['x']['max']:
                    self.vels['x']['current'] += 0.05
                else:
                    self.vels['x']['current'] = self.vels['x']['max']
            elif ch == 's':
                if self.vels['x']['current'] - 0.05 > self.vels['x']['min']:
                    self.vels['x']['current'] -= 0.05
                else:
                    self.vels['x']['current'] = self.vels['x']['min']
            elif ch == 'a':
                if self.vels['t']['current'] + 0.1 < self.vels['t']['max']:
                    self.vels['t']['current'] += 0.1
                else:
                    self.vels['t']['current'] = self.vels['t']['max']
            elif ch == 'd':
                if self.vels['t']['current'] - 0.1 > self.vels['t']['min']:
                    self.vels['t']['current'] -= 0.1
                else:
                    self.vels['t']['current'] = self.vels['t']['min']
            elif ch == 'j':
                self.motion_proxy.angleInterpolation("HeadYaw", +self.yaw_angle,
                                                     self.yaw_time, False)
            elif ch == 'l':
                self.motion_proxy.angleInterpolation("HeadYaw", -self.yaw_angle,
                                                     self.yaw_time, False)
            elif ch == 'i':
                self.motion_proxy.angleInterpolation("HeadPitch", -self.pitch_angle,
                                                     self.pitch_time, False)
            elif ch == 'k':
                self.motion_proxy.angleInterpolation("HeadPitch", +self.pitch_angle,
                                                     self.pitch_time, False)
            elif ch == ' ':
                self.vels['x']['current'] = 0.0
                self.vels['t']['current'] = 0.0

            command = {}
            for k,v in self.vels.items():
                command[k] = round(v['current'],2)
            self._move(**command)
            if ch != 'q': print command

        print "Good-bye"
        self._stop()

    def _stop(self):
        self._move()

    def _move(self, x=.0, y=.0, t=.0):
        self.motion_proxy.post.move(x, y, t)

if __name__ == "__main__":
    """
    """
    parser = configargparse.ArgumentParser()
    parser.add_argument("--ip",
                        type=str,
                        env_var="NAO_IP",
                        required=True,
                        help="Robot ip address")
    parser.add_argument("--port",
                        type=int,
                        default=9559,
                        help="Robot port number")
    args = parser.parse_args()

    k = KeyBoardTeleOp(args.ip, args.port)
    k.spin()
