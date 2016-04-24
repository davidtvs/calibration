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

# Author: David Haddon and Fred Pauling
#$Id: mdlslm.py 437 2011-07-14 04:17:07Z had067 $


from __future__ import division
import roslib; roslib.load_manifest("mdlslm")
import time
import numpy as np
from rospy.numpy_msg import numpy_msg # for numpy arrays in the messages
import struct
import rospy
import socket
import sys
from sensor_msgs.msg import LaserScan


class SLM:
  header_fmt=">cHHHB2s2s2s7s2s2s4sc"
  header_struct=struct.Struct(header_fmt)
  ls=numpy_msg(LaserScan)()
  pub=None

  def __init__(self,name="slm_laser", udp_addr="128.2.0.201", port=30, speed=10, frame="/slm", vis_spot=False, lo_res=False ):
    self.udp_addr=udp_addr
    self.udp_port=port
    #ros 
    rospy.init_node(name)
    self.pub=rospy.Publisher(name, numpy_msg(LaserScan))
    self.ls.range_max=655
    self.ls.range_min=0.01
    self.ls.header.frame_id=frame
    self.seq=0
    self.comms_fail=0
    
        
    #Open UDP port
    self.sock = socket.socket( socket.AF_INET, # Internet
                      socket.SOCK_DGRAM ) # UDP
    self.sock.settimeout(0.1)
    
    self.set_enable_reply(True)
    self.send_cmd("X1\n") # Set output to Binary
    self.set_speed(speed)
    self.set_visible(vis_spot)
    self.set_resolution(lo_res)
    self.send_cmd("S\n") # Start motor spinning
    self.set_laser_on(True)
  
  def shutdown(self):
    self.set_laser_on(False)
    self.set_visible(False)
    self.send_cmd("T\n") # Turn Motor off
    self.sock.close()
    print "\nLaser shutdown complete" 

  def send_cmd(self, cmd, recv_reply = True):
    tries_left=5
    self.sock.sendto( cmd, (self.udp_addr, self.udp_port) )    
    while (tries_left and recv_reply):
      try:
        data, addr = self.sock.recvfrom( 1500 ) # buffer size is 1500 bytes
        if len(data) > 0:
          self.comms_fail=0
          return data
      except socket.error as e:
        tries_left-=1
        self.comms_fail+=1
    if self.comms_fail > 20:
      raise Exception ('Comms Failed')
    return None
    
  def set_enable_reply(self, replies_on):
    self.send_cmd("H\n", recv_reply=False) # don't expect a response
    try:
      data, addr = self.sock.recvfrom( 1500 ) # buffer size is 1500 bytes
      if len(data) > 0:
        reply=True
    except socket.error as e:
      reply=False;  
    if reply==replies_on:
      return
    else:
      self.send_cmd("H\n")
      
    
  def set_laser_on(self, laser_on):
    if (laser_on):
      self.send_cmd("A\n")
      self.laser_on=laser_on
    else:
      self.send_cmd("B\n")
      self.laser_on=laser_on
  
  # Set Angular resolution to either 100th (fine) or 10th (course) degree
  def set_resolution(self, lo_res):
    if (lo_res):
      self.send_cmd("FT\n")
      self.fine_resolution=True
      print "Resolution set to course!"      
      
    else:
      self.send_cmd("FH\n")
      self.fine_resolution=False

      

  def set_visible(self, laser_on):
    if (laser_on):
      self.send_cmd("P\n")  # Turn Visible laser on
      self.visible_laser=True
    else:
      self.send_cmd("Q\n")  # Turn Visible laser off
      self.visible_laser=False
          
  # Set Head Speed to hz revolutions per second
  def set_speed (self, hz):
    cmd="I" + str(hz) + "\n" 
    self.speed=hz
    self.send_cmd(cmd)
    
  def recv_pkt(self):
    try:
      data, addr = self.sock.recvfrom( 1500 ) # buffer size is 1500 bytes
      if len(data) > 0:
        if ('$' in data):
          return data
    except socket.error as e:
      return "" 

  # Data reception stuff
  
  def parse_pkt(self, data):
       
    if len(data) > self.header_struct.size:
      
      hdr=self.header_struct.unpack_from(data)    
      if hdr[0] == '@':
        
        self.numpoints=hdr[1]
        self.angle_rate=hdr[2]/100
        self.angle_res=hdr[3]/1000
        self.num_points=hdr[4]
        self.hours=int(hdr[5])
        self.mins=int(hdr[6])
        self.secs=int(hdr[7])
        self.usecs=int(hdr[8])
        self.day=int(hdr[9])
        self.month=int(hdr[10])
        self.year=int(hdr[11])
        self.ls.time_increment=(1/((360*self.angle_rate)/self.angle_res))
        
        epoch_secs=int(time.mktime((self.year, self.month, self.day, self.hours,self.mins,self.secs,0,0,0)))
       
        if self.seq==0:
          # First time.. compute offset to allow for lack of real time on laser
          realtime=rospy.Time.now().to_sec()
          self.tm_offset=realtime-epoch_secs-self.usecs/1000000
          
        self.ls.header.stamp=rospy.Time(epoch_secs+self.tm_offset)    
          
      
        data_arr=np.frombuffer(data,offset=self.header_struct.size,dtype=np.uint8)
        
        if len(data_arr)==self.numpoints*8:
          
          data_arr=data_arr.reshape([-1,8])
          uint16be=np.dtype('>H')
          self.ranges=data_arr[:,1:3].copy().view(dtype=uint16be)
          self.signal=data_arr[:,3:5].copy().view(dtype=uint16be)
          self.angles=data_arr[:,5:7].copy().view(dtype=uint16be)
          return True
        else:
          print "Wrong size of data ", len(data_arr)
          return False
          
  def rosify_pkt(self):
    #everything need to be reversed as ROS expects counter clockwise scans
    
    self.ls.header.seq=self.seq
    self.ls.ranges=(self.ranges/100.0).astype(np.float32)
    self.ls.intensities=(self.signal).astype(np.float32)
    self.ls.angle_max=(360-self.angles[len(self.angles)-1]/100)/(180.0/np.pi)
    self.ls.angle_min=(360-self.angles[0]/100)/(180/np.pi)
    self.ls.angle_increment=-((self.angle_res)/(180.0/np.pi))
    
    self.pub.publish(self.ls)
    self.seq+=1
    
def is_arg_with_param(arg, prefix):
    if not arg.startswith(prefix):
        return False
    if not arg.startswith(prefix+"="):
        print "Expected '=' after "+prefix
        print
        usage(1)
    return True

def usage():
    print '''MDL SLM Driver

      CSIRO 2011

      David Haddon and Fred Pauling

      $Id: mdlslm.py 437 2011-07-14 04:17:07Z had067 $

      This is a quick hack driver.. do not use for production

      Usage:

      rosrun mdlslm mdlslm.py [-rate=<10>] [__name=<slm_laser>] [-port=<30>] [-ip=<128.2.0.201>] [-frame=</slm>] [-lo_res=0] [-visible=0]

      rate    :  Spin rate in Hz, should be between 1-20
      __name  :  Ros topic name
      port    :  Laser UDP port  (this will not change the laser settings!)
      ip      :  IP address of laser (once again.. will not change the laser settings)
      frame   :  Frame ID for LaserScan topic
      lo_res  :  set to 1 to use 10th degree increments, defaults to 100th degree
      visible :  Turn on the red visible laser'''
    
 
          
if __name__ == "__main__":
  udp_port=30
  udp_addr="128.2.0.201"
  slm=None
  name="slm_laser"
  speed=10
  frame="/slm"
  lo_res=0
  visible=0

  try:
    for arg in sys.argv[1:]: # Be very tolerant in case we are roslaunched.
      if arg == "--help":
        usage()
        exit(1)
      elif is_arg_with_param(arg, '-ip'):
        udp_addr = arg[4:]
      elif is_arg_with_param(arg, '-port'):
        udp_port = int(arg[6:])
      elif is_arg_with_param(arg, '-rate'):
        speed = int(arg[6:])
      elif is_arg_with_param(arg, '-frame'):
        frame= arg[7:]
      elif is_arg_with_param(arg, '-visible'):
        visible= int(arg[9:])        
      elif is_arg_with_param(arg, '-lo_res'):
        lo_res= int(arg[8:])        
      elif is_arg_with_param(arg, '__name:'):
        name = arg[8:]
        
    print 'Using topic name of %s' %(name)
    slm=SLM(name, udp_addr,udp_port, speed, frame, bool(visible), bool(lo_res))

    while not rospy.is_shutdown():
      data=slm.recv_pkt()
      #print "Len= " + str(len(data)) + "  Data=" +str(data)
      if slm.parse_pkt(data):
        slm.rosify_pkt()
        
  except rospy.ROSInterruptException: pass 
    
  except ValueError as parse_fail:
    print "Parse failed.. check your arguments\n : ", parse_fail
    usage()
  except Exception as ex:
    print "Caught an Error.. WTF? : ", ex
  if (slm != None):
    slm.shutdown()
  exit()

