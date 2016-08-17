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
#$Id:$

PKG = 'sick_ldmrs'

import roslib; roslib.load_manifest(PKG)
import rostest

import unittest
import math
import time

from sick_ldmrs.utils import *


class TestUtils(unittest.TestCase):

    def setUp(self):
        now = time.time()
        self.frac,  self.fsecs = math.modf(now)
        self.nsecs = int(round(self.frac * (10**9)))
        self.frac32 = int(round(self.frac * (2**32)))
        self.secs = int(self.fsecs)

    def test_ntp_to_ros(self):
        ntp = (self.secs  * (2**32) )+ self.frac32 # should be equal to (secs << 32) | frac32
        r = NTP64_to_ROStime(ntp)
        self.assertEqual(r.secs,  self.secs)
        # seems to be some rounding imprecision with the division,
        # it's irrelevent at this level so just account for it
        diff = abs(r.nsecs - self.nsecs)
        self.assertTrue(diff < 2)

    def test_ros_to_ntp(self):
        r = rospy.Time(secs=self.secs,  nsecs=self.nsecs)
        ntp = ROStime_to_NTP64(r)
        self.assertEqual(ntp >> 32,  self.secs)
        # seems to be some rounding imprecision with the division,
        # it's irrelevent at this level so just account for it
        ntp32 = ntp & 0xffffffff
        diff = abs(ntp32 - self.frac32)
        self.assertTrue((diff < 4))

    def test_ntpsplit_to_ros(self):
        r = NTP64split_to_ROStime(self.secs,  self.frac32)
        self.assertEqual(r.secs,  self.secs)
        # seems to be some rounding imprecision with the division,
        # it's irrelevent at this level so just account for it
        diff = abs(r.nsecs - self.nsecs)
        self.assertTrue(diff < 2)

    def test_bitstring(self):
        self.assertEqual(bitString(6),  '00000110')
        self.assertEqual(bitString(0),  '00000000')
        self.assertEqual(bitString(0xa5),  '10100101')
        self.assertEqual(bitString(0xff),  '11111111')
        self.assertEqual(bitString(0xdeadbeef),  '11011110101011011011111011101111')
        self.assertEqual(bitString(0x1234),  '1001000110100')

if __name__ == '__main__':
    rospy.init_node('ldmrs_utils',  disable_rostime=True)
    rostest.rosrun(PKG, 'ldmrs_util', TestUtils)
