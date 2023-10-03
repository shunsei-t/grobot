#!/usr/bin/env python
# -*- coding: utf-8 -*-

################################################################################
# Copyright 2017 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################

import os
import struct

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
    def kbhit():
        return msvcrt.kbhit()
else:
    import termios, fcntl, sys, os
    from select import select
    fd = sys.stdin.fileno()
    old_term = termios.tcgetattr(fd)
    new_term = termios.tcgetattr(fd)

    def getch():
        new_term[3] = (new_term[3] & ~termios.ICANON & ~termios.ECHO)
        termios.tcsetattr(fd, termios.TCSANOW, new_term)
        try:
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_term)
        return ch

    def kbhit():
        new_term[3] = (new_term[3] & ~(termios.ICANON | termios.ECHO))
        termios.tcsetattr(fd, termios.TCSANOW, new_term)
        try:
            dr,dw,de = select([sys.stdin], [], [], 0)
            if dr != []:
                return 1
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_term)
            sys.stdout.flush()

        return 0

from dynamixel_sdk import *                    # Uses Dynamixel SDK library

class DxlClass:
    # x シリーズのみ対応
    def __init__(self, device_port) -> None:
        PROTOCOL_VERSION            = 2.0            # See which protocol version is used in the Dynamixel
        DEVICENAME                  = device_port
        BAUDRATE                    = 57600

        self.ADDR_OPERATING_MODE         = 11
        self.ADDR_TORQUE_ENABLE          = 64
        self.ADDR_LED                    = 65
        self.ADDR_POSITION_D_GAIN        = 80
        self.ADDR_POSITION_I_GAIN        = 82
        self.ADDR_POSITION_P_GAIN        = 84
        self.ADDR_PROFILE_ACCELERATION   = 108
        self.ADDR_PROFILE_VELOCITY       = 112
        self.ADDR_MOVING                 = 122
        self.ADDR_MOVING_STATUS          = 124
        self.ADDR_GOAL_PWM               = 100
        self.ADDR_GOAL_CURRENT           = 102
        self.ADDR_GOAL_VELOCITY          = 104
        self.ADDR_GOAL_POSITION          = 116
        self.ADDR_PRESENT_VELOCITY       = 128
        self.ADDR_PRESENT_POSITION       = 132

        self.MAX_POSITION_VALUE          = 1048575           # Of MX with firmware 2.0 and X-Series the revolution on Extended Position Control Mode is 256 rev

        self.portHandler = PortHandler(DEVICENAME)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)

        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            quit()

        if self.portHandler.setBaudRate(BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            quit()


    def write_operateing_mode(self, id, mode):
        """
        OPERATING MODE<br>
        0  : current control<br>
        1  : velocity control<br>
        3  : position control<br>
        4  : extended position control<br>
        5  : current-base position control<br>
        16 : pwm control<br>

        *** Please set oprating mode before torque_state setting***
        """
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, id, self.ADDR_OPERATING_MODE, mode)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Operating mode changed to control mode.")


    def write_torque_state(self, id, state):
        """
        1=on, 0=off
        """
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, id, self.ADDR_TORQUE_ENABLE, state)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel has been successfully connected")


    def read_moving(self, id):
        """
        Present velocity <= Moving threshould  0
        Present velocity > Moving threshould   1
        """
        dxl_moving, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler, id, self.ADDR_MOVING)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        return dxl_moving


    def read_moving_status(self, id):
        """
        in-Position 0
        """
        dxl_moving_status, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler, id, self.ADDR_MOVING_STATUS)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        return dxl_moving_status


    def write_position_pid_gain(self, id, pid):
        """
        Args:
            id (int): _description_
            pid (list): [p_gain, i_gain, d_gain]
        """
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, id, self.ADDR_POSITION_P_GAIN, pid[0])
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, id, self.ADDR_POSITION_I_GAIN, pid[1])
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, id, self.ADDR_POSITION_D_GAIN, pid[2])
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))


    def write_profiles(self, id, profiles):
        """
        Args:
            id (int): _description_
            profiles (list): [acceleration, velocity]
        """
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, id, self.ADDR_PROFILE_ACCELERATION, profiles[0])
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, id, self.ADDR_PROFILE_VELOCITY, profiles[1])
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))


    def write_goal_pwm(self, id, goal_pwm):
        """
        Duty = goal_pwm * 100 / 855
        """
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, id, self.ADDR_GOAL_PWM, goal_pwm)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))


    def write_goal_current(self, id, goal_current):
        """
        scaling factorを参照
        """
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, id, self.ADDR_GOAL_CURRENT, goal_current)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))


    def write_goal_velocity(self, id, goal_velocity):
        """
        Velocity = goal_velocity * 0.229 [rpm]
        -1023~ ~1023
        """
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, id, self.ADDR_GOAL_VELOCITY, goal_velocity)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))


    def write_goal_position(self, id, goal_position):
        """
        Position = goal_position * 360 / 4096 [deg]
        """
        if abs(goal_position) > self.MAX_POSITION_VALUE:
            print("Goal position > max position")
            return 0
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, id, self.ADDR_GOAL_POSITION, goal_position)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))


    def read_present_velocity(self, id):
        """
        Velocity [rpm] = returnValue * 0.229 [rpm]
        """
        dxl_present_velocity, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, id, self.ADDR_PRESENT_VELOCITY)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        # print(type(dxl_present_velocity))
        tmp = struct.pack("I", dxl_present_velocity)
        return struct.unpack("i", tmp)[0]


    def read_present_position(self, id):
        """
        Position [deg] = returnValue * 360 [deg] / 4096
        """
        dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, id, self.ADDR_PRESENT_POSITION)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        return dxl_present_position


    def clear_multi_turn(self, id):
        dxl_comm_result, dxl_error = self.packetHandler.clearMultiTurn(self.portHandler, id)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

