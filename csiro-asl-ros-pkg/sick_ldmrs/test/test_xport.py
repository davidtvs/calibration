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
#$Id$

'''Unit tests for xport layer.  These are ROS-agnostic so run under unitrun.'''


PKG = 'sick_ldmrs'
import roslib; roslib.load_manifest(PKG)
import rostest
import unittest
import time
import threading

from sick_ldmrs.xport import *
from sick_ldmrs.exceptions import *

HOST='localhost'
PORT=12002

ADDR = (HOST, PORT)

class XPortConnectionTests(unittest.TestCase):

    # just a callable object creating the XPort object
    # used in the assertRaises calls
    @staticmethod
    def xport_create(host,  port):
        xport = XPort(host,  port)

    def test_gaierror(self):
        # an invalid hostname should fail the gethostbyname call
        self.assertRaises(FatalException,  self.xport_create,  'garbage',  PORT)


    def test_connrefused(self):
        # connecting to an unused port should trigger ECONNREFUSED
        self.assertRaises(InfoRestartException,   self.xport_create,  HOST,  22222)

    def test_timeout(self):
        # try to connect to a bogus address, should force a timeout
        self.assertRaises(TimeoutException,  self.xport_create,  '180.180.180.180',  PORT)

class XPortTestPeer(threading.Thread):
    ''' Remote network peer helper.
    
    Creates a remote network peer to enable send/recv of socket data for
    peer testing purposes.
    '''

    def __init__(self,  host,  port):
        ''' Construct the remote peer.
        
        Create the listening socket for the remote peer simulator.
        @param host: host/ip_addr to listen on
        @type host: str
        @param port: port to listen on
        @type port: int
        '''
        threading.Thread.__init__(self)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET,  socket.SO_REUSEADDR,  1)
        self.sock.setsockopt(socket.SOL_SOCKET,  socket.SO_RCVBUF,  256)
        self.sock.settimeout(3)
        self.sock.bind((host, port))
        self.sock.listen(1)
        self.ready_ev = threading.Event()


    def run(self):
        ''' Derived Thread.run method for executing remote peer calls'''
        (conn,  addr) = self.sock.accept()
        self.ready_ev.wait()
        self.cmd(conn)
        conn.close()

    def call(self,  cmd):
        ''' signal remote peer thread to execute callable object
        @param cmd: command to execute
        @type cmd: callable object
        '''
        self.cmd = cmd
        self.ready_ev.set()


    def close(self):
        '''close listening socket'''
        self.sock.close()

    def wait(self):
        ''' wait for remote peer thread to finish '''
        self.join()



class XPortTestHelper(unittest.TestCase):
    ''' common setUp and tearDown methods '''

    def setUp(self):
        ''' set up the remote listening thread '''
        self.peer = XPortTestPeer(HOST, PORT)
        self.peer.start()

    def tearDown(self):
        ''' wait for remote peer thread to exit '''
        self.peer.wait()
        self.peer.close()



class XPortSendTests(XPortTestHelper):

    largestr = 'testing!' * (1024 * 1024)

    def send_timeout(self,  conn):
        time.sleep(8)

    def test_timeout(self):
        xport = XPort(HOST,  PORT)
        self.peer.call(self.send_timeout)
        time.sleep(0.5)
        self.assertRaises(TimeoutException,  xport.send,  self.largestr * 20)

    def send_shutdown(self,  conn):
        self.xport.shutdown()

    def test_shutdown(self):
        self.xport = XPort(HOST,  PORT)
        self.peer.call(self.send_shutdown)
        self.assertRaises(InfoRestartException,  self.xport.send,  self.largestr)

    def send_close(self,  conn):
        pass

    def test_close(self):
        xport = XPort(HOST,  PORT)
        self.peer.call(self.send_close)
        time.sleep(5)
        self.assertRaises(InfoRestartException,  xport.send,  self.largestr)

class XPortRecvTests(XPortTestHelper):

    teststr = 'testing!'

    def recv_timeout(self,  conn):
        # make sure the sleep value here is greater than the xport timeout val
        time.sleep(15)

    def test_timeout(self):
        # delay accept in peer thread beyond the conn timeout val, forcing a 
        # timeout exception
        xport = XPort(HOST,  PORT)
        self.peer.call(self.recv_timeout)
        self.assertRaises(TimeoutException,  xport.recv,  len(self.teststr))

    def recv_shutdown(self,  conn):
        time.sleep(0.5)
        conn.shutdown(socket.SHUT_RDWR)

    def test_shutdown(self):
        # peer shutdown will cause a recv return of ''
        xport = XPort(HOST,  PORT)
        self.peer.call(self.recv_shutdown)
        self.assertRaises(InfoRestartException,  xport.recv,  len(self.teststr))

    def recv_recvall_timeout(self,  conn):
        # force the peer to send some but not all the requested data,
        # eventually the recv will time out
        conn.send(self.teststr[0:1])
        time.sleep(7)

    def test_recvall_timeout(self):
        xport = XPort(HOST,  PORT)
        self.peer.call(self.recv_recvall_timeout)
        self.assertRaises(TimeoutException,  xport.recv,  len(self.teststr))

    def recv_recvall_broken(self,  conn):
        half = len(self.teststr) / 2
        data1 = self.teststr[:half]
        data2 = self.teststr[half:]

        conn.send(data1)
        time.sleep(1)
        conn.send(data2)

    def test_recvall_broken(self):
        # ensure recv will wait for all requested data before returning
        xport = XPort(HOST,  PORT)
        self.peer.call(self.recv_recvall_broken)
        data = xport.recv(len(self.teststr))
        self.assertEqual(data,  self.teststr)

    def recv_close(self,  conn):
        pass

    def test_close(self):
        # detect a closed peer socket
        xport = XPort(HOST,  PORT)
        self.peer.call(self.recv_close)
        self.assertRaises(InfoRestartException,  xport.recv,  len(self.teststr))



if __name__=='__main__':
    rostest.unitrun(PKG,  'xport_conn',  XPortConnectionTests)
    rostest.unitrun(PKG,  'xport_recv',  XPortSendTests)
    rostest.unitrun(PKG,  'xport_send',  XPortRecvTests)



