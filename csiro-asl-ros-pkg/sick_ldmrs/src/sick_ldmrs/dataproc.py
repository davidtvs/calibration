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

# Author: Fred Pauling
#$Id$

from struct import *
import numpy as np
import roslib
roslib.load_manifest('sick_ldmrs')
import rospy
from sensor_msgs.msg import *
import utils as util
from rospy.numpy_msg import numpy_msg
from params import LDMRSParams


class ProcessLDMRSData:
    """
    Class for processing and packaging Sick LD-MRS Laser Scan data messages into ROS messages.
    Converts raw scan data into ROS PointCloud2 and LaserScan messages and publishes them on request.
    The primary method you need to call is process_msg. This method processes the scan data
        and publishes LaserScan and/or PointCloud2 messages of the (transformed) data.
    You will also need to call set_timestamp_delta() to set the difference between the
    current time and the LD-MRS on-board time (which you can extract from the message header)
    """
    
    header_format = '<HHHQQHhhHhhhhhhH' # format string for data header
    header_struct = Struct(header_format)  # precompile for speed
    # setup structure for pulling out header components
    header_keys = ['ScanNumber', 'ScannerStatus', 
                   'SyncPhaseOffset', 'ScanStartTimeNTP', 
                   'ScanEndTimeNTP', 'AngleTicksPerRot', 
                   'StartAngle', 'EndAngle', 
                   'NumScanPoints', 'Reserved1', 
                   'Reserved2', 'Reserved3', 
                   'Reserved4', 'Reserved5', 
                   'Reserved6', 'Reserved7']

    num_header_bytes = 44  # 44 bytes for the header
    num_point_bytes = 10   # 10 bytes for each point
    # lookup table for beam elevation trig factors
    # taking centre of beam as elevation angle (actual limits are [-1.6, -0.8, 0, 0.8, 1.6])
    elev_angles = np.array([-1.2, -0.4, 0.4, 1.2]) * ((2*np.pi)/360.0) # rad
    v_sin_lut = np.sin(elev_angles)
    v_cos_lut = np.cos(elev_angles)

    def __init__(self,  topics,  params):
        """ Constructor
            @param topics: dictionary mapping topic names to publisher handles
                valid topic names are {"cloud", "scan0", "scan1", "scan2", "scan3"}
            @type topics: dict {topic_name:publisher_handle}
            @param params: dictionary mapping parameter names to values. 
                Where applicable, the parameters must be in device units (e.g. ticks)
            @type params: dict {ros_parameter_string:value}
        """
        self.params = params
        self.topics = topics

        # LaserScan messages (numbered 0-3, 0 is the lowest scan plane)
        # rewire for numpy serialization on publish
        self.scans = [numpy_msg(LaserScan)(),
                        numpy_msg(LaserScan)(),
                        numpy_msg(LaserScan)(),
                        numpy_msg(LaserScan)()]
        self._init_scans()

        # PointCloud2 message
        # rewire for numpy serialization on publish
        self.point_cloud = numpy_msg(PointCloud2)()
        self._init_point_cloud()

        # put variables into the namespace to prevent
        # attribute exceptions when the class is abused
        self.header = None
        self.x = None
        self.y = None
        self.z = None
        self.echo = None
        self.layer = None
        self.flags = None
        self.echo_w = None
        self.h_angle = None
        self.rads_per_tick = None
        self.pc_data = None
        self.last_start_time = None
        self.rads_per_tick = None
        self.seq_num = -1
        self.time_delta = rospy.Duration()
        self.n_points = 0
        # timestamp smoothing
        self.smoothtime = None
        self.smoothtime_prev = None
        self.recv_time_prev = None
        self.known_delta_t = rospy.Duration(256.0/self.params['scan_frequency'])
        self.time_smoothing_factor = self.params['time_smoothing_factor']
        self.time_error_threshold = self.params['time_error_threshold']
        self.total_err = 0.0  #integrate errors
        self.header = {}.fromkeys(self.header_keys)

    def process_msg(self,  msg, recv_time):
        """ Process an incoming data message. Convert to (and publish)
            PointCloud2 and LaserScan ROS messages depending
            on whether the associated topics
            ('cloud', 'scan0', 'scan1', 'scan2', 'scan3') resp.
            are subscribed.
            @param msg: the scan data message from the LD-MRS to be processed
            @type msg: read-only byte buffer (e.g. a python string)
        """
        # smooth the timestamp using the expected rate 
        self.smooth_timestamp(recv_time)
        
        self.msg = msg
        subscribers = self.num_subscribers()

        # any subscribers? if not just return, don't waste cpu cycles
        if any([n > 0 for n in subscribers.itervalues()]):
            self.unpack_data(msg)
            # is anyone subscribed to the cloud topic?
            if subscribers["cloud"]:
                self.make_point_cloud()
                self.publish_point_cloud()
            # is anyone subscribed to a scan topic?
            if any(["scan" in topic and num_subs > 0 for topic, num_subs in subscribers.iteritems()]):
                self.make_scans()
                self.publish_scans()
            
        return None


    def num_subscribers(self):
        """ Get the number of subscribers for each ROS topic defined in self.topics
        """
        subscribers = {}
        for topic, handle in self.topics.iteritems():
            subscribers[topic] = handle.get_num_connections()
        return subscribers


    def compute_tick_freq(self):
        """ Compute the tick frequency from the start/end angles and times.
            This is typically within 1% of the nominal values.
            Note: we are using the device supplied tick resolution of 1/32nd degree
            @return: tickfrequency - a floating point number (ticks per second)
        """
        delta_t = (self.header['ScanEndTimeNTP'] - self.header['ScanStartTimeNTP'])
        delta_ticks = self.header['StartAngle'] - self.header['EndAngle']
        tick_freq = (delta_ticks << 32)/ float(delta_t)
        return int(tick_freq)


    def set_timestamp_delta(self,  time_delta):
        """ Set the time delta to apply to the LD-MRS timestamp to bring
            it up to current ROS time.
            @param time_delta: the time delta to be applied to the timestamps from the LD-MRS
            @type time_delta: a rostime.Duration object
        """
        self.time_delta = time_delta
        
    def smooth_timestamp(self, recv_time):
        """ Smooth the timestamp to track the expected scan rate.
            Parameter time_smoothing_factor controls
            Small errors between rostime and smoothed time are corrected by
            applying a correction weighted by the time_smoothong_gain
            Large errors above the time_error_threshold are corrected to 
            recv_time by a step adjustment 
            @param recv_time: ros timestamp when the message was recieved
            @type ldmrs_time: rostime.Time object    
        """

        if not self.smoothtime_prev:
            # initialize to recv time
            self.smoothtime = recv_time
        else:
            self.smoothtime = self.smoothtime_prev + self.known_delta_t
            err = (recv_time - self.smoothtime).to_sec()
            if self.time_smoothing_factor > 0 and abs(err) < self.time_error_threshold:
                correction = rospy.Duration(err * (1-self.time_smoothing_factor))
                self.smoothtime += correction
            else:
                # error too high, or smoothing disabled - set smoothtime to last timestamp
                self.smoothtime = recv_time
            #print 'delta_smoothtime: %f, err: %f'%((self.smoothtime - self.smoothtime_prev).to_sec(), err)
        self.smoothtime_prev = self.smoothtime
          

    def unpack_data(self, msg):
        """ Unpack the data from an LD-MRS scan data message
            into instance attributes to be picked up by make_point_cloud  and/or
            make_scan.
            @param msg: a binary string in little-endian format encoding the
                message
            @type msg: read-only byte buffer (e.g. a python string)
        """

        # increment ROS message header sequence number for this scan
        # we are storing these internally rather than using the device supplied sequence number
        # since the device seq number is only uint16 and will quickly roll over
        #whereas the ROS message seq header field is uint32
        self.seq_num += 1

        # Parse the scan header into a dictionary
        header_tuple = self.header_struct.unpack_from(msg)
        for index, value in enumerate(header_tuple):
            self.header[self.header_keys[index]] = value
        
        self.rads_per_tick = (2.0 * np.pi) / self.header['AngleTicksPerRot']
        self.scan_start_time = self.smoothtime #util.NTP64_to_ROStime(self.header['ScanStartTimeNTP']) + self.time_delta

        # check that the input string has the correct number of bytes
        self.n_points = self.header['NumScanPoints']

        # Check we have enough bytes in the buffer
        n_data_bytes = len(msg) - self.num_header_bytes
        n_points_in_buffer = int(n_data_bytes/self.num_point_bytes)
        if n_points_in_buffer != self.n_points:
            rospy.logwarn("Number of point indicated by header (",  self.n_points,
                          ") is more than number of points in buffer (",  n_points_in_buffer,  ")")
            self.n_points = n_points_in_buffer

        # tick frequency (Hz*32)
        # Observed to vary by 1-2 percent from nominal frequency
        self.tick_freq = self.compute_tick_freq()

        if self.n_points is 0:
            # No data to unpack
            self.point_data = np.array([], dtype=uint16le)
        else:

            # Make a bytearray from the point data in the input string
            uint16le = np.dtype(np.uint16, align='<') # force little endian
            self.point_data = np.frombuffer(msg, dtype=uint16le,
                                            count =  self.n_points * self.num_point_bytes/2, #/2 accounts for 16bit view
                                            offset = self.num_header_bytes)

            # reshape array as 2D array of uint16 (n rows * num_point_bytes/2 cols)
            self.point_data = np.reshape(self.point_data, [self.n_points, -1])

            # now array has format:
            # Name:    Layer/Echo/Flags | H Angle | Rdist | Echo Width | Reserved |
            # Column:  0                | 1       | 2     | 3          | 4        |
            
            # Pull out data as 16bit fields using slices and views
            self.h_angle_ticks = self.point_data[:, 1].view(np.int16)
            # adjust the start time to account for the tick shift

            # compute number of scanner ticks since start angle tick for each sample
            self.ticknum = -(self.h_angle_ticks - self.header['StartAngle'])
            # radial distance to each point (in metres)
            self.r_dist = (self.point_data[:, 2]/100.0).astype(np.float32)
            # echo width (in cm!)
            self.echo_w = self.point_data[:, 3].copy()

            # Pull out echo layer and flags fields from echo_layer_flags
            # need to copy so we can view as two byte arrays
            layer_echo_flags = self.point_data.view(dtype=np.uint8) # view as 2D byte array
            self.layer = layer_echo_flags[:,0].copy()
            self.layer &= np.array([3], dtype=np.uint8) # extract bits 0-1
            self.echo = layer_echo_flags[:,0].copy()
            self.echo = (self.echo & np.array([48], dtype=np.uint8)) >> 4 # extract bits 4,5 and shift

            # Valid Flags bits are 0,1,3 -- want to move bit 3 to bit 2
            #   so we can put layer,echo and flags into one byte for point cloud later
            self.flags = layer_echo_flags[:,1].copy()
            self.flags |= ((self.flags & np.array([8], dtype=np.uint16)) >> 1) # copy bit 3 to bit 2
            self.flags &= np.array([7], dtype=np.uint8)  # mask off bit 3 keeping bits 0-2


    def publish_point_cloud(self):
        """ Publish the finished point cloud to the ROS network
            topic is /<node_name>/cloud
        """
        if self.point_cloud is not None:
            handle = self.topics["cloud"]
            handle.publish(self.point_cloud)
        else:
            rospy.logerr("Trying to publish empty point cloud to topic %s"%handle.name)


    def publish_scans(self):
        """ Publish scan topics on the ROS network.
            Publishes four topics (one for each scan plane of the LD-MRS):
                /<node_name>/scan0    -- the lowest scan plane
                /<node_name>/scan1
                /<node_name>/scan2
                /<node_name>/scan3    -- the highest scan plane
        """
        for i, scan in enumerate(self.scans):
            scan_id = "scan" + str(i)
            handle = self.topics[scan_id]
            if scan is not None and handle is not None:
                handle.publish(scan)
            else:
                rospy.logerr("Trying to publish empty scan to topic %s"%handle.name)


    def make_point_cloud(self):
        """Construct a point cloud from previously unpacked data (see unpack_data())
            The finished point cloud is stored as the instance variable 'point_cloud'
            Note you must call unpack_data before invoking this method
        """

        if self.n_points is 0:
            # No points
            self.pc_data = ""    
        else:
            # compute x,y,z coordinates in metres
            h_angle_rads = self.h_angle_ticks * self.rads_per_tick
            v_sines = self.v_sin_lut[self.layer]   # lookup cosines for elevation angle
            v_cosines = self.v_cos_lut[self.layer] # lookup cosines for elevation angle
    
            # x is in direction of travel
            # z is up
            # y is left
            # note that the scan direction is clockwise from y-axis toward x-axis
            #   this is the reverse of the expected scan direction for this coordinate system.
            # We account for this explicitly in the LaserScan messages (see _fill_laser_scans())
            self.x = (v_cosines * (np.cos(h_angle_rads) * self.r_dist)).astype(np.float32)
            self.y = (v_cosines * (np.sin(h_angle_rads) * self.r_dist)).astype(np.float32)
            self.z = (v_sines   * self.r_dist).astype(np.float32)
    
            # store delta from start time
            self.time_deltas = (self.ticknum * 1.0/self.tick_freq).astype(np.float32)
    
            # pack layer/echo/flags bytes into a single byte field
            # Bit:      0,1,  | 2,3  | 4,5,6 |
            # Meaning:  Layer | Echo | Flags |
            self.layer_echo_flags = self.layer | (self.echo << 2) | (self.flags << 4)
    
            # concatenate the numpy arrays in the correct order for the PointCloud2 message
            # need to pack as 2D array, then flatten since fields have varying byte widths
            data = np.hstack((self.x.view(np.uint8).T.reshape(-1, 4),
                              self.y.view(np.uint8).T.reshape(-1, 4),
                              self.z.view(np.uint8).T.reshape(-1, 4),
                              self.time_deltas.view(np.uint8).T.reshape(-1, 4),
                              self.echo_w.view(np.uint8).T.reshape(-1, 2),
                              self.layer_echo_flags.reshape(-1, 1)))   # already uint8
            data = data.reshape(-1) # 1D view with points correctly aligned
    
            # convert to byte string for serialization and transmission
            # The serialize_numpy method in _PointCloud2.py *should* (but doesn't)
            # take care of this when the message is published;
            # i.e. we should be able to leave this as a numpy array here.
            # This is (probably) an oversight by the developer in this instance
            self.pc_data = data.tostring()
    
        # finished computing data, now fill out the fields in the message ready for transmission
        self._fill_point_cloud()
    
    
    def make_scans(self):
        """ Generate laser scan messages from previously unpacked data
                the ROS parameter value of 'use_first_echo' in the ROS parameter
                server determines if first or last echo is used
        """

        if self.n_points is 0:
            # slot ranges into their correct location within the scan array
            # and separate out each layer using the layer breaks computed earlier
            self.scan_data = np.zeros([4,0], dtype=np.float32)
                
        else:
        
            # number of points to allocate per scan message
            # (depends on angular resolution and scan angle)
            self.npoints_scan = abs((self.header['StartAngle'] - self.header['EndAngle'])/self.ticknum2ind) + 1
    
            # Now get indices of first and last echos...
            # this is non-trivial since:
            # 1. each point has 0-3 echoes (no guarantee about order is given)
            # 2. points are interlaced by scan plane
            # ... so we need to lexical sort by: echo, then tick number, then layer
            # to get data in the right order.
            # Next we need to compute the indices for the
            # first/last echo for each sample using index differencing
    
            # do lexical sort to preserve order of previous sorts.
            # The echo sort may be redundant but SICK gives no guarantees on how
            #   the echoes are organized within the scan
            sort_inds = np.lexsort((self.echo, self.ticknum,  self.layer))
            # find where h_angle_ticks changes
            ind_diff = np.diff(self.h_angle_ticks[sort_inds])
            breaks = ind_diff.nonzero()[0]  # [0] due to singleton tuple return from nonzero()
    
            if self.params[LDMRSParams.use_first_echo]:
                # get indices of first echoes
                # breaks +1 indexes the first echo of each sample
                # prepend 0 since diff result has length n-1 and
                # first sample is always the first echo
                inds = np.hstack((np.array([0]),  breaks + 1)) # sorted indices of first echoes
            else:
                # get indices of last echoes
                # breaks indexes the last echo
                # append last index since diff result has length n-1 and
                # last sample is always the last echo
                last_ind = np.array([self.r_dist.size-1])
                inds = np.hstack((breaks, last_ind)) #sorted indices of last echoes
    
            # back out to unsorted inds so we can filter the required data
            inds = sort_inds[inds]
    
            #select first/last echo entries (elements remain sorted by ticknum and layer)
            layers = self.layer[inds]
            ranges = self.r_dist[inds]
            ticks = self.ticknum[inds]
    
            # Finally we need to find the layer breaks and store each layer separately
            # in a zero padded array of the correct size.
    
            # find the indices of the layer breaks
            layer_breaks = np.diff(layers).nonzero()[0]
            #index of first entry for each layer
            firsts = np.hstack((np.array([0]), layer_breaks + 1))
            #index of last entry for each layer
            lasts = np.hstack((layer_breaks,  np.array([ranges.size -1])))
    
            # !!! NOTE: When scan frequency is 50 Hz this shifts the scan clockwise by 1/16th degree
            # a better solution would be to move the start angle and start time correspondingly
            tick_inds = np.round(((ticks - self.tick_ind_adjust)/self.ticknum2ind)).astype(np.int16) # integer division
    
            # slot ranges into their correct location within the scan array
            # and separate out each layer using the layer breaks computed earlier
            self.scan_data = np.zeros((4,  self.npoints_scan),  dtype = np.float32)
    
            for i in range(0, 4):
                self.scan_data[i, tick_inds[firsts[i]:(lasts[i]+1)]] = ranges[firsts[i]:(lasts[i]+1)]
    
        # finished marsahlling the range data, now it's time to fill in the details
        self._fill_scans()
    
#--------------------------------------------------------------------------
# Private methods

    def _init_point_cloud(self):
        """ Set up the point cloud class as much as possible in advance
        """

        pc = self.point_cloud

        # set frame id (for the cloud topic this is just the frame_id_base)
        pc.header.frame_id = self.params[LDMRSParams.frame_id_prefix]

        # set up point fields
        # there are 6 fields:
        # x,y,z,timedelta,echowidth,layerechoflags
        fields = [['x', PointField.FLOAT32, 0],
                    ['y', PointField.FLOAT32, 4],
                    ['z', PointField.FLOAT32, 8],
                    ['timedelta', PointField.FLOAT32, 12],
                    ['echowidth', PointField.UINT16, 16],
                    ['layerechoflags', PointField.UINT8, 18]]

        # set up PointFields that describe the layout of the data
        # blob in the message
        point_fields = []
        for field in fields:
            pf = PointField()
            pf.name = field[0]
            pf.datatype = field[1]
            pf.offset = field[2]
            pf.count = 1
            point_fields.append(pf)
        pc.fields = point_fields

        # set up other attributes
        pc.height = 1       # data is 1D
        pc.is_dense = False # unordered points
        pc.point_step = 19  # bytes per point

        # endianness check and set
        if struct.pack("h", 1) == "\000\001":
            pc.is_bigendian = True
        else:
            pc.is_bigendian = False


    def _fill_point_cloud(self):
        """ Pack the point cloud data into the PointCloud2 message """

        pc = self.point_cloud

        # ------------------Header----------------------------
        pc.header.stamp = self.smoothtime
        pc.header.seq = self.seq_num
        
        # ----------------- Other ----------------------------
        num_points = len(self.x)
        pc.width = num_points
        pc.row_step = len(self.pc_data)  # number of bytes of data

        #------------------- Data -------------------
        pc.data = self.pc_data


    def _init_scans(self):
        """ Initial set up for the Scan messages
        """
        # conversion factors from tick num to scan index used in make_scans
        tick_freq = self.params[LDMRSParams.scan_frequency]
        self.ticknum2ind = tick_freq/800  # 4, 8, 16 which corresponds to 1/8, 1/4, and 1/2 degree for 12.5, 25, 50 Hz
        
        # convert ticks to indices with a coarser resolution base
        # to conserve space in the scan message. Original base is 1/32 degree.
        # Otherwise there would be a lot of zero padding in between points
        # and thus extra b/w overhead in the message
        # angle_ticks has a 2 tick offset when scan frequency is 50Hz
        # need to shift ticks down by 2 so we can divide ticks to get scan indices later
        if self.params['scan_frequency'] == 12800: # 50Hz
            self.tick_ind_adjust = 2
            time_per_tick = 1.0/(50.0 * 11520)
            self.start_time_adjust = rospy.Duration(2 * time_per_tick)
        else:
            self.tick_ind_adjust = 0
            self.start_time_adjust = rospy.Duration(0)

        frame_id_prefix = self.params[LDMRSParams.frame_id_prefix]

        for i, scan in enumerate(self.scans):
            scan.header.frame_id = frame_id_prefix + str(i)  # append the scan plane number to the frame id
            scan.intensities = np.array([]) # empty (must be numpy array for serialization)
            scan.range_min = 0.01   # using 1cm so that zero range points are not displayed in RVIZ
            scan.range_max = 10000  # No max range specified on spec sheet


    def _fill_scans(self):
        """Add scan data to the scans and set up per-message parameter values
        """
        # adjust start time and start angle if using 50 Hz to compensate 
        # for 2 sample offset of first sample
        #tick2rad = 2*np.pi/11520
        #start_angle_adjust = self.tick_ind_adjust*tick2rad
        #start_time_adjust = self.tick_ind_adjust/12800.0 
        #start_time = self.scan_start_time + rospy.Duration(start_time_adjust)
        
        # compute the time since the last scan started
        if self.smoothtime_prev is None:
            time_between_scans = 0.0
        else:
            time_between_scans = self.smoothtime - self.smoothtime_prev - self.start_time_adjust
            time_between_scans = time_between_scans.secs + time_between_scans.nsecs/1e9

        # Set up angles
        # NOTE!: the scanner rotates clockwise from above so
        # angle_min > angle_max and we use a NEGATIVE ANGLE INCREMENT
        # this works in RVIZ so we are going with it
        # the problem stems from the poorly named angle_min and angle_max fields in LaserScan.
        # These should probably have been named angle_start and angle_end (or similar) to prevent
        # assumptions about which one is greater, and hence assumptions about
        # the sign of the angle_increment.
        angle_min = (self.header['StartAngle'] - self.tick_ind_adjust)*self.rads_per_tick  # rad
        angle_max = (self.header['EndAngle'] - self.tick_ind_adjust)*self.rads_per_tick    # rad
        angle_increment = -self.ticknum2ind*self.rads_per_tick # angle between samples (rad)

        # Times are handled as you would expect (time increment is +ve):
        # time for each point = header.stamp + time_increment*<index>
        time_increment = float(self.ticknum2ind)/self.tick_freq #time between samples

        # set up each scan (only the range data differs)
        for i, scan in enumerate(self.scans):
            scan.header.stamp = self.scan_start_time + self.start_time_adjust
            scan.header.seq = self.seq_num
            scan.scan_time = time_between_scans
            scan.angle_min = angle_min
            scan.angle_max = angle_max
            scan.angle_increment = angle_increment
            scan.time_increment = time_increment
            scan.ranges = self.scan_data[i, :]

        # store the current time to generate the next scan.scan_time
        self.last_start_time = self.scan_start_time

#EOF
