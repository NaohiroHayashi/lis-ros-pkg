#!/usr/bin/env python
# -*- coding: utf-8 -*-
# This module provides a class that controls the serial servo motor manufactured by Futaba Corp.
# ver1.30622
# This module has been tested on python ver.2.6.6
# It need pySerial(http://pyserial.sourceforge.net/)
# (C) 2015 Hiroaki Matsuda and Naohiro Hayashi

import pymx
import rospy
import sys
from std_msgs.msg import String

class Hook(object):

        def __init__(self, port , baudrate, timeout = 1):
                self.mx = pymx.Mx()
                self.mx.open_port(port, baudrate, timeout)
                self._sub_hook = rospy.Subscriber('hook', String, self.callback)

        def initialize_gripper(self, speed = 10):
                self.mx.speed(1, speed)
                self.mx.speed(2, speed)

                self.servo_gripper(1, 'all')
                self.move_gripper(10, 'all')

        def finalize_gripper(self):
                print("clean gripper!")
                #~ self.servo_gripper(180, 'all')
                self.mx.close_port()
                return True

        def servo_gripper(self, enable = 0, gripper = 'all'):
                if gripper is 'left':
                        self.mx.reg_servo([1],
                                          [enable])
                        self.mx.action(254)

                elif gripper is 'right':
                        self.mx.reg_servo([2],
                                          [enable])
                        self.mx.action(254)

                elif gripper is 'all':
                        self.mx.reg_servo([1, 2],
                                          [enable, enable])
                        self.mx.action(254)

        def open_gripper(self, width):
                self.mx.reg_move([1, 2], [width, width])
                self.mx.action(254)

        def move_gripper(self, width = 0, gripper = 'all'):
                if gripper is 'left':
                        self.mx.reg_move([1],
                                         [180 + width])
                        self.mx.action(254)

                elif gripper is 'right':
                        self.mx.reg_move([2],
                                         [180 - width])
                        self.mx.action(254)

                elif gripper is 'all':
                        self.mx.reg_move([1, 2],
                                         [180 + width,
                                          180 - width])
                        self.mx.action(254)

        def callback(self, cmd_string):
                if cmd_string.data is 'a':
                        self.move_gripper(10, 'left')

                elif cmd_string.data is 'b':
                        self.move_gripper(0, 'left')

                elif cmd_string.data is 'c':
                        self.move_gripper(10, 'right')

                elif cmd_string.data is 'd':
                        self.move_gripper(0, 'right')

                elif cmd_string.data is 'e':
                        self.move_gripper(10, 'all')

                elif cmd_string.data is 'f':
                        self.move_gripper(0, 'all')
                else:
                    rospy.loginfo(rospy.get_name() + ": hook_node heard %s. Its msg is false!" % cmd_string.data)

def main():
    import time

    rospy.init_node('Hook_node', anonymous=False)

    hook = Hook('/dev/ttyUSB0', 3000000, 1)
    print("initialize gripper")
    hook.initialize_gripper(1)
    print("wait command")
    rospy.spin()
    rospy.on_shutdown(hook.finalize_gripper())


if __name__ == '__main__':
    sys.exit(main())
