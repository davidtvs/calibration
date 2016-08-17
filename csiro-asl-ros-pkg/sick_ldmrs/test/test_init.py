#!/usr/bin/env python

#***********************************************************
#* Software License Agreement (BSD License)
#*
#*  Copyright (c) 2010, CSIRO Autonomous Systems Laboratory
#*  All rights reserved.
#*
#*  Redistribution and use in source and binary forms, with or without
#*  modification, are permitted provided that the following conditions
#*  are met:
#*
#*   * Redistributions of source code must retain the above copyright
#*     notice, this list of conditions and the following disclaimer.
#*   * Redistributions in binary form must reproduce the above
#*     copyright notice, this list of conditions and the following
#*     disclaimer in the documentation and/or other materials provided
#*     with the distribution.
#*   * Neither the name of the CSIRO nor the names of its
#*     contributors may be used to endorse or promote products derived
#*     from this software without specific prior written permission.
#*
#*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#*  POSSIBILITY OF SUCH DAMAGE.
#***********************************************************

# Author: Brett Grandbois
#$Id: $

PKG = 'sick_ldmrs'

import roslib; roslib.load_manifest(PKG)
import rostest

import unittest
import sys
import struct
import socket
import time

from sick_ldmrs.cfg import *
from sick_ldmrs.utils import *
from sick_ldmrs.params import *

from testhelpers import *

class TestInitSeq(TestListenerHelper):

    expected_params = { 0x1100  :  'start_angle',
                        0x1101  :  'end_angle',
                        0x1102  :  'scan_frequency',
                        0x1104  :  'constant_angular_res'}


    def test_first_cmd(self):
        self.get_first_cmd()

    def test_error_restart(self):
        self.get_first_cmd()
        err = Error.pack(0,  0,  0,  0x0010)
        self.conn.sendall(err)
        time.sleep(1)
        data = self.conn.recv(1,  socket.MSG_WAITALL)
        # closing the socket at the peer will cause a return of nothing
        self.assertEqual(data,  '')
        self.conn.close()
        time.sleep(0.5)
        self.setUp()
        self.get_first_cmd()

    def test_error_reset_dsp(self):
        self.get_first_cmd()
        err = Error.pack(0,  0,  0,  0x0040)
        self.conn.sendall(err)
        time.sleep(1)
        data = self.conn.recv(1,  socket.MSG_WAITALL)
        # closing the socket at the peer will cause a return of nothing
        self.assertEqual(data,  '')
        self.conn.close()
        time.sleep(0.5)
        self.setUp()
        (cmd,  resrv,  index,  param) = self.recv_cmd()
        self.assertEqual(cmd,  0x0000,  'expected reset DSP for first cmd in sequence but got: %x' % cmd )

    def test_init_seq(self):

        params = LDMRSParams()
        params.update_config(ldmrsConfig.defaults,  0)
        self.get_first_cmd()
        self.conn.sendall(Reply.pack(0x0021))
        cmd = ''
        recv_params = {}
        while cmd != 0x0020:
            (cmd,  resrv,  index,  param) = self.recv_cmd()
            self.assertTrue(cmd in [0x0020,  0x0010],  'expecting a set param or start measure but got: %x' % cmd)
            if index:
                self.assertTrue(index in self.expected_params.keys(),  'got an invalid set parameter index %x' % index)
                recv_params[index] = param
            self.conn.sendall(Reply.pack(cmd))

        count = 0
        for k in recv_params.keys():
            got = recv_params[k]
            expected = params.ldmrs_params[self.expected_params[k]]
            self.assertEqual(got,  expected,  'unexpected parameter update for index %x, expected %d got %d' % (k,  expected,  got))
            count += 1

        self.assertEqual(count,  len(self.expected_params),  'did not get all the expected parameter updates')


if __name__ == '__main__':
    rospy.init_node('ldmrs_init',  disable_rostime=True)
    rostest.rosrun(PKG, 'ldmrs_init', TestInitSeq)
    listener.close()
