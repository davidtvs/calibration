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

# Author: Brett Grandbois and Fred Pauling
#$Id: test_ldmrs.py 321 2010-10-28 08:44:15Z pau386 $

PKG = 'sick_ldmrs'

NODE = 'ldmrs'

import roslib; roslib.load_manifest(PKG)
import rostest

import sensor_msgs
from sensor_msgs.msg import PointCloud2, LaserScan,  PointField
from rospy.numpy_msg import numpy_msg # for numpy arrays in the messages
import dynamic_reconfigure.client

import unittest
import sys
import struct
import socket
import time
import math
import threading
import os
import numpy as np

from testhelpers import *
from sick_ldmrs.cfg import *
from sick_ldmrs.utils import *
from sick_ldmrs.params import *


class TestMsg(TestListenerHelper):
    """ Test that ROS messages from the sick ldmrs node 
        are formatted correctly and contain the correct data.
        This is achieved by sending a message to the node via
        a socket and comparing the returned message to the original.
    """

    # stores the ros_message
    ros_msg = None
    ldmrs_msg = None
    timeout = 30.0 # seconds
    callback_event = threading.Event()

    # ROS subsriber callback
    def callback(self, data):
        self.ros_msg = data
        self.callback_event.set()


    # Test methods
    # would have like to use rospy.wait_for_message here
    # but need to subscribe to a topic before it will be published at all
    # so using traditional callbacks and thread events to achieve the same thing
    def test_cloud(self):
        cloud = rospy.Subscriber("/ldmrs/cloud", numpy_msg(PointCloud2), self.callback)
        self.get_msg()
        self.init_seq()
        self.send_ldmrs_msg()
        self.callback_event.wait(self.timeout)
        self.assertTrue(self.callback_event.isSet(),
                "Timeout waiting for cloud message")
        self.callback_event.clear()
        cloud.unregister()
        self.check_point_cloud()

    # 28 October 2010.
    # wanted to use numpy_msg deserialization for the scan messages 
    # but there is a bug in the numpy.frombuffer method which 
    # throws an exception when reading from a zero-length buffer 
    # (even when count=0). This occurs because our intensities 
    # field is empty. Using standard deserialization until
    # either numpy (ticket #1388) or ROS (ticket #4528) posts a fix.
    # Update 29 October: Looks like latest version of ROS fixes this error
    # but it is not available in the package repositories for Ubuntu yet

    def test_scan0(self):

        scan = rospy.Subscriber('/ldmrs/scan0', LaserScan, self.callback)
        #scan = rospy.Subscriber('/ldmrs/scan0', numpy_msg(LaserScan), self.callback)
        self.get_msg()
        self.init_seq()
        self.send_ldmrs_msg()
        self.callback_event.wait(self.timeout)
        self.assertTrue(self.callback_event.isSet(),
                "Timeout waiting for scan0 message")
        self.callback_event.clear()
        scan.unregister()
        self.check_scan(0)
        

    def test_scan1(self):

        scan = rospy.Subscriber('/ldmrs/scan1', LaserScan, self.callback)
        #scan = rospy.Subscriber('/ldmrs/scan1', numpy_msg(LaserScan), self.callback)
        self.get_msg()
        self.init_seq()
        self.send_ldmrs_msg()
        self.callback_event.wait(self.timeout)
        self.assertTrue(self.callback_event.isSet(),
                "Timeout waiting for scan1 message")
        self.callback_event.clear()
        scan.unregister()
        self.check_scan(1)
        
        
    def test_scan2(self):

        scan = rospy.Subscriber('/ldmrs/scan2', LaserScan, self.callback)
        #scan = rospy.Subscriber('/ldmrs/scan2', numpy_msg(LaserScan), self.callback)
        self.get_msg()
        self.init_seq()
        self.send_ldmrs_msg()
        self.callback_event.wait(self.timeout)
        self.assertTrue(self.callback_event.isSet(),
                "Timeout waiting for scan2 message")
        self.callback_event.clear()
        scan.unregister()
        self.check_scan(2)


    def test_scan3(self):
        
        scan = rospy.Subscriber('/ldmrs/scan3', LaserScan, self.callback)
        #scan = rospy.Subscriber('/ldmrs/scan3', numpy_msg(LaserScan), self.callback)
        self.get_msg()
        self.init_seq()
        self.send_ldmrs_msg()
        self.callback_event.wait(self.timeout)
        self.assertTrue(self.callback_event.isSet(),
                "Timeout waiting for scan3 message")
        self.callback_event.clear()
        scan.unregister()
        self.check_scan(3)

    @classmethod
    def get_msg(self):
        """ Get a single ldmrs scan data message from a file
            The file contains a single message only
        """
        msg_file = os.path.join(ldmrs_pkg_dir,'test_msg.ldmrs')
        f = open(msg_file,  'r')
        self.ldmrs_msg = f.read(-1)


    def send_ldmrs_msg(self):
        """ Send the test message to the LDMRS
        """
        self.conn.sendall(self.ldmrs_msg)


    def unpack_ldmrs_msg(self):        
        """ Unpack the raw ldmrs message into a format suitable for comparison
            against the returned ROS message
        """

        self.assertTrue(self.ros_msg is not None, "Subscriber callback did not produce a message")

        # Extract LDMRS message fields for comparison
        #===================================================================
        self.data_msg_header_ld = struct.unpack_from('<HHHQQHhhHhhhhhhH', self.ldmrs_msg,  offset = 24)
        uint16le = np.dtype(np.uint16, align='<') # force little endian
        ldmrs_data = np.frombuffer(self.ldmrs_msg, dtype=uint16le, offset = 24 + 44)
        self.start_time_ld = self.data_msg_header_ld[3]
        self.end_time_ld = self.data_msg_header_ld[4]
        self.start_angle_ld = self.data_msg_header_ld[6]
        self.end_angle_ld = self.data_msg_header_ld[7]


        # check that the data has the appropriate size
        self.assertTrue(ldmrs_data.size%5.0 == 0.0,  "Message does not contain a whole number of points")

        ldmrs_data = ldmrs_data.reshape(-1,  5)

        self.num_points_ld = ldmrs_data.shape[0]
        self.layer_echo_flags_ld16 = ldmrs_data[:, 0] & np.array([0b0000101111111111])
        self.layer_ld = (self.layer_echo_flags_ld16 & np.array([0b0000000000001111])).astype(np.uint8)
        self.echo_ld = ((self.layer_echo_flags_ld16 & np.array([0b0000000011110000])) >> 4).astype(np.uint8)
        self.h_angle_ticks_ld = ldmrs_data[:, 1].astype(np.int16)
        self.ticknum_ld = -(self.h_angle_ticks_ld - self.start_angle_ld)
        self.r_dist_ld = ldmrs_data[:, 2]
        self.echo_w_ld = ldmrs_data[:, 3]

        #convert h_angle_ticks_ld to time deltas for comparison to pc message
        self.delta_angle_ld = abs(self.end_angle_ld - self.start_angle_ld)

        self.start_time_ld = NTP64_to_ROStime(self.start_time_ld)
        self.end_time_ld = NTP64_to_ROStime(self.end_time_ld)
        self.duration_ld = self.end_time_ld - self.start_time_ld

        self.time_increment_ld = self.duration_ld.to_sec()/float(self.delta_angle_ld)
        self.tick_num_ld = np.abs(self.h_angle_ticks_ld - self.start_angle_ld)
        self.time_deltas_ld = self.tick_num_ld * self.time_increment_ld


    def check_point_cloud(self):
        """ Check that an ldmrs scan message and the resulting
            PointCloud2 are the same (within some tolerance)
        """

        self.unpack_ldmrs_msg()
        point_cloud = self.ros_msg

        # Extract fields from point_cloud message
        #=================================================================
        pc_data = np.frombuffer(point_cloud.data, dtype=np.uint8)

        time_stamp = point_cloud.header.stamp

        # check that the data has the appropriate size
        self.assertTrue(pc_data.size%19.0 == 0.0,  "Message does not contain a whole number of points")

        pc_data = pc_data.reshape(-1, 19)
        num_points_pc = pc_data.shape[0]
        x = pc_data[:,0:4].copy().view(np.float32).T
        y = pc_data[:,4:8].copy().view(np.float32).T
        z = pc_data[:,8:12].copy().view(np.float32).T
        time_deltas_pc = pc_data[:, 12:16].copy().view(np.float32).T
        echo_w_pc = pc_data[:, 16:18].copy().view(np.uint16).T
        layer_echo_flags_pc = pc_data[:, 18].copy().T


        # Convert PointCloudMessage fields to LDMRS message format
        # =================================================================
        r_dist_pc = np.round(((x**2 + y**2 + z**2)**0.5)*100).astype(np.int16)
        theta_pc = np.arctan2(y, x)
        rad2ticks = 11520/(2*np.pi)
        h_angle_ticks_pc = np.round(theta_pc*rad2ticks).astype(np.int16)

        # Pull out echo_pc layer_pc and flags_pc fields from echo_layer_flags_pc
        # need to copy so we can view as two byte arrays
        layer_pc = layer_echo_flags_pc & np.array([0b00000011], dtype=np.uint8)
        echo_pc  = (layer_echo_flags_pc & np.array([0b00001100], dtype=np.uint8)) >> 2
        layer_echo_pc = layer_pc | (echo_pc << 4)

        flags_pc = (layer_echo_flags_pc & np.array([0b01110000], dtype=np.uint8)) >> 4
        flags_pc = flags_pc | ((flags_pc & np.array([0b00000100], dtype=np.uint8)) << 1)
        flags_pc = flags_pc & np.array([0b11111011], dtype=np.uint8)

        layer_echo_flags_pc16 = layer_echo_pc.astype(np.uint16) | (flags_pc.astype(np.uint16) << 8)


        # Compare LDMRS-message and point cloud, these should be (practically) identical
        #====================================================================
       
        # check point fields are formatted correctly
        fields = point_cloud.fields
        field_names = [f.name for f in fields]
        expected_field_names = ['x', 'y', 'z', 'timedelta', 'echowidth', 'layerechoflags']
        self.assertTrue(field_names == expected_field_names,
            "PointFields: Expected: field names == %s, but got %s"%(str(expected_field_names),  str(field_names)))
        offsets = [f.offset for f in fields]
        expected_offsets = [0, 4, 8, 12, 16, 18]
        self.assertTrue(offsets == expected_offsets,
            "PointFields: Expected: offsets == %s, but got %s"%(str(expected_offsets),  str(offsets)))
        counts = [f.count for f in fields]
        expected_counts = [1, 1, 1, 1, 1, 1]
        self.assertTrue(counts == expected_counts,
            "PointFields: Expected: counts == %s, but got %s"%(str(expected_counts),  str(counts)))

        # check point_step is correct
        self.assertEqual(point_cloud.point_step, 19,
            "Expected point_step to be 19, but got %s"%str(point_cloud.point_step))
        # check row_step is correct
        self.assertEqual(point_cloud.row_step, num_points_pc * 19,
            "Expected point_step to be %s, but got %s"%(str(num_points_pc * 19), str(point_cloud.point_step)))
        # check is_dense is false
        self.assertFalse(point_cloud.is_dense,
            "Expected is_dense to be False")

        # check width
        self.assertEqual(point_cloud.width, num_points_pc,
            "Expected width to be %d, but got %s"%(num_points_pc, point_cloud.width))
        #check height
        self.assertEqual(point_cloud.height, 1,
            "Expected width to be 1, but got %s"%str(point_cloud.height))

        #TODO: need to check for big-endian / little endian

        # Check number of points is OK first
        self.assertTrue(self.num_points_ld == num_points_pc,
            "Expected number of points to be equal but got ldmrs: %d, point_cloud: %d"%(self.num_points_ld, num_points_pc))

        # Check radial distances are identical
        self.assertFalse(np.any(r_dist_pc - self.r_dist_ld),
            "Radial distances do not match between ldmrs message and PointCloud message")

        # Check layer_echo_flags are identical
        self.assertFalse(np.any(layer_echo_flags_pc16 - self.layer_echo_flags_ld16),
            "Layer/Echo/Flags do not match between ldmrs message and PointCloud message")

        # Check echo widths are identical
        self.assertFalse(np.any(echo_w_pc - self.echo_w_ld),
            "Echo widths do not match between ldmrs message and PointCloud message")

        # Check h_angle ticks are identical
        self.assertFalse(np.any(h_angle_ticks_pc - self.h_angle_ticks_ld),
            "Sample angles do not match between ldmrs message and PointCloud message")

        # Check time deltas are within tolerance of 1 microsecond
        self.assertFalse(np.any(np.abs(self.time_deltas_ld - time_deltas_pc) > 1e-6),
            "Time deltas do not match between ldmrs message and PointCloud message")

        #check frame_id
        self.assertEqual(point_cloud.header.frame_id,  '/ldmrs')


    def check_scan(self,  scan_num):
        """ Check that the scan is formatted correctly and contains correct data
            as compared to the original message.
            @param scan_num: the number of the scan plane to check, valid values are in {0,1,2,3}
            @type scan_num: int
        """
        self.unpack_ldmrs_msg()
        laser_scan = self.ros_msg
        
        # for 50Hz sampling rate:
        # adjust start angle to reflect offset of two samples for first beam
        scan_freq = rospy.get_param('/ldmrs/scan_frequency')
        if(scan_freq == 2):
            tick_adjust = 2
            start_time_adjust = 2.0/(50.0 * 11520)
        else: 
            tick_adjust = 0
            start_time_adjust = 0        
        
        start_angle_ld = self.data_msg_header_ld[6] - tick_adjust
        end_angle_ld = self.data_msg_header_ld[7] - tick_adjust

        # Need to convert ldmrs message closer to LaserScan format
        # since we lose so much information compared to the PointCloud2 format
        # note that this is a different implementation of the layer/echo extraction
        # process than the dataproc module in order to help prevent any systemic errors
        layer_inds = (self.layer_ld == scan_num).nonzero()[0]
        echoes = self.echo_ld[layer_inds]
        ticknums = self.ticknum_ld[layer_inds]
        ranges = self.r_dist_ld[layer_inds]

        #No need to sort further here since device already sorts by ticknum and echo

        # get indices of first/last echoes
        use_first_echo = rospy.get_param('/ldmrs/use_first_echo')
        breaks = np.diff(ticknums).nonzero()[0]
        if use_first_echo:
            inds = np.hstack((np.array([0]), breaks + 1))
        else:
            inds = np.hstack((breaks,  np.array(ranges.size -1)))

        # extract first/last echo samples
        ticknums = ticknums[inds]
        ranges = ranges[inds]

        # downsample ticknum to index in array based on frequency
        angle_range = abs(start_angle_ld - end_angle_ld)
        if scan_freq == 2:
            # Device quirk, need to shift ticknum down by 2 before downsampling
            tick_inds = (ticknums - 2)/16
            array_size = angle_range/16
        elif scan_freq == 1:
            tick_inds = ticknums/8
            array_size = angle_range/8
        else:  # scan_freq S== 12.5
            tick_inds = ticknums/4
            array_size = angle_range/4

        scan_ranges = np.zeros(array_size)
        scan_ranges[tick_inds] = ranges/100.0  # convert to m


        # Compare ldmrs message data with scan data
        #====================================================

        n_points = scan_ranges.size

        # check that the ranges are identical and are in the right location within the array
        # this will work regardless of the 
        ros_msg_ranges = np.array(laser_scan.ranges)
        self.assertFalse(np.any(np.abs(ros_msg_ranges - scan_ranges) > 1e-4),
                "LaserScan ranges do not match ranges from ldmrs for scan plane %d"%scan_num)

        # check that the intensity field  is empty (since the ldmrs does not have intensity values)
        self.assertEqual(np.array(laser_scan.intensities).size,  0,
                "Require intensities field to be empty, but got non_empty")

        # check frame_id
        expected_frame_id = "/ldmrs" + str(scan_num)
        self.assertEqual(laser_scan.header.frame_id,  expected_frame_id, 
                "Expected frame id to be: %s, but got %s"%(expected_frame_id, laser_scan.header.frame_id))

        # check angles match up
        rad2tick = 11520/(2*np.pi)

        start_angle = (laser_scan.angle_min)* rad2tick  # max angle is start
        delta_start_angle = abs(start_angle_ld - start_angle)
        start_angle_tol = 1
        self.assertTrue(delta_start_angle < start_angle_tol, 
                   "Start angles differ more than the allowed tolerance of %s ticks"%str(start_angle_tol))

        end_angle = laser_scan.angle_max * rad2tick  # min angle is end
        end_angle_tol = 1
        delta_end_angle = abs(end_angle_ld - end_angle)
        self.assertTrue(delta_end_angle < end_angle_tol, 
                   "End angles differ more than the allowed tolerance of %s ticks"%str(start_angle_tol))

        end_angle_incremental = start_angle + n_points*laser_scan.angle_increment*rad2tick
        inc_angle_tol = 1
        delta_end_angle_inc = abs(end_angle_incremental - end_angle)
        self.assertTrue(delta_end_angle_inc < inc_angle_tol, 
                    "Incrementally computed end angle differs more than the allowed tolerance of %s ticks from the true value"%str(inc_angle_tol))
        
        
if __name__ == '__main__':

    ldmrs_pkg_dir = os.path.join(roslib.packages.get_pkg_dir('sick_ldmrs'),  'test')
    rospy.init_node('ldmrs_test',  disable_rostime=True)
    rostest.rosrun(PKG, 'ldmrs_msg', TestMsg)
    listener.close()
