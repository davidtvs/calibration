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

NODE = 'ldmrs'

import roslib; roslib.load_manifest(PKG)
import rostest

import unittest
import sys
import struct
import socket
import time
import copy

import dynamic_reconfigure.client

from sick_ldmrs.cfg import *
from sick_ldmrs.utils import *
from sick_ldmrs.params import *

from testhelpers import *

class TestDynamicReconfigure(TestListenerHelper):

    # want access to init_seq so still derive from TestListenerHelper,
    # but need to override these to ensure we reset the default state
    # in the parameter server
    def setUp(self):
        global listener
        (self.conn,  addr) = listener.accept()
        self.init_seq()
        self.client = dynamic_reconfigure.client.Client(NODE)

    def tearDown(self):
        self.conn.close()
        self.client.update_configuration(ldmrsConfig.defaults)
        self.client.close()
        time.sleep(0.1)


    def test_update_norestart(self):
        self.conn.settimeout(2) # driver defaults to 5 so we just need to be less than that
        new_params = {'scan_frequency'  :   25,
                                    'start_angle'   :   20}

        self.client.update_configuration(new_params)
        # since we didnt set apply_changes this should go into the param object
        # but never get applied so we should time out waiting for a cmd
        self.assertRaises(socket.timeout,  self.conn.recv,  Header.size,  socket.MSG_WAITALL)

    def test_update_restart(self):
        global listener

        expected_params = { 0x1100  :  'start_angle',
                                            0x1101  :  'end_angle',
                                            0x1102  :   'scan_frequency',
                                            0x1104  :   'angular_res_type'}

        new_params = {'scan_frequency'  :   1,
                                    'start_angle'   :   20}

        defaults = copy.deepcopy(ldmrsConfig.defaults)

        params = LDMRSParams()
        params.update_config(defaults,  0)

        # verify the new values are different in case someone changes
        # the defaults
        for k, v in new_params.iteritems():
            self.assertNotEqual(v,  params.ldmrs_params[k])
            defaults[k] = v

        # now add in new params for use later
        params.update_config(defaults,  0)

        # add in the restart
        new_params.update({'apply_changes' : True})

        self.client.update_configuration(new_params)
        time.sleep(0.1)

        # the apply changes should force a xport restart so go through
        # the init sequence again verifying the values have changed

        # the restart will close the socket which will manifest in a data
        # return of ''
        hdr = self.conn.recv(Header.size,  socket.MSG_WAITALL)
        self.assertEqual(hdr,  '')

        (self.conn,  addr) = listener.accept()

        self.get_first_cmd()
        self.conn.sendall(Reply.pack(0x0021))
        cmd = ''
        recv_params = {}
        while cmd != 0x0020:
            (cmd,  resrv,  index,  param) = self.recv_cmd()
            self.assertTrue(cmd in [0x0020,  0x0010],  'expecting a set param or start measure but got: %x' % cmd)
            if index:
                self.assertTrue(index in expected_params.keys(),  'got an invalid set parameter index %x' % index)
                recv_params[expected_params[index]] = param
            self.conn.sendall(Reply.pack(cmd))

        new_params.pop('apply_changes')

        for k in new_params:
            self.assertEqual(recv_params[k],  params.get_ldmrs_param(k))


    def test_update_invalid_angles(self):
        # start angle must be greater than end angle
        # so just swap the defaults

        # note this test assumes fix_errors == True in the node
        # which means the angle values will not be updated

        new_params = dict()

        defaults = ldmrsConfig.defaults

        new_params['start_angle'] = defaults['end_angle']
        new_params['end_angle'] = defaults['start_angle']

        config = self.client.update_configuration(new_params)

        for k, v in config.iteritems():
            self.assertEqual(v,  defaults[k]),  'invalid angle key %s should be %s but got %s' % (k,  defaults[k],  v)

    def test_update_invalid_freq_type(self):
        # focussed is only available in 12.5Hz scan mode

        # note this test assumes fix_errors == True in the node
        # which means the resolution type will not be updated

        new_params = {'constant_angular_res'  :   False,
                                    'scan_frequency'   :   2}

        defaults = ldmrsConfig.defaults

        config = self.client.update_configuration(new_params)

        self.assertTrue(config['constant_angular_res'])
        self.assertEqual(config['scan_frequency'],  config['scan_frequency'])

if __name__ == '__main__':
    rospy.init_node('ldmrs_dyn',  disable_rostime=True)
    rostest.rosrun(PKG, 'ldmrs_dyn', TestDynamicReconfigure)
    listener.close()
