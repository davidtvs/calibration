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
from sick_ldmrs.utils import *

import unittest
import struct
import socket
import time

HOST='localhost'
PORT=12002


listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
listener.setsockopt(socket.SOL_SOCKET,  socket.SO_REUSEADDR,  1)
listener.settimeout(60)
listener.bind((HOST,  PORT))
listener.listen(1)

class Header(object):
    magic = 0xaffec0c2
    hdr_struct = struct.Struct('>LLLBBHQ')
    size = hdr_struct.size

    @classmethod
    def hdr_pack(cls,  msg_type,  msg_size):
        return (cls.hdr_struct.pack(cls.magic,  0,  msg_size,  0,  0,  msg_type,  Now_to_NTP64()))

    @classmethod
    def hdr_unpack(cls,  msg):
        (magic_word,  prev_size,  size,  resv,  devid,  data_type,  ntp) = cls.hdr_struct.unpack(msg)
        if magic_word == cls.magic:
            valid = True
        else:
            valid = False

        return (valid,  size,  data_type,  ntp)


class Reply(Header):

    code = 0x2020

    reply = struct.Struct('<H')

    @classmethod
    def pack(cls,  cmd):
        return cls.hdr_pack(cls.code,  cls.reply.size) + cls.reply.pack(cmd)


class Command(object):

    code = 0x2010

    cmd_generic = struct.Struct('<HH')
    # A large assumption here.  The parameters can be either
    # signed or unsigned (thank you for that sick...) but our
    # expected values are all less than 32K so simplifying by
    # always unpacking to signed
    cmd_set = struct.Struct('<HHHhh')


    @classmethod
    def unpack(cls,  data):
        if len(data) == cls.cmd_generic.size:
            (cmd,  resrv) = cls.cmd_generic.unpack(data)
            index = None
            param = None
        elif len(data) == cls.cmd_set.size:
            (cmd,  resrv,  index,  param,  unused) = cls.cmd_set.unpack(data)
        else:
            raise NotImplementedError('received unexpected message type from laser driver: %s' % data)

        return(cmd,  resrv,  index,  param)

    @classmethod
    def pack(cls,  cmd):
        return(cls.cmd_generic.pack(cmd, 0))

class Error(Header):

    code = 0x2030

    format = struct.Struct('<HHHHQ')

    @classmethod
    def pack(cls,  err1,  err2,  warn1,  warn2):
        return cls.hdr_pack(cls.code,  cls.format.size) + cls.format.pack(err1,  err2,  warn1,  warn2,  0)


class TestListenerHelper(unittest.TestCase):

    def setUp(self):
        global listener
        (self.conn,  addr) = listener.accept()

    def tearDown(self):
        self.conn.close()
        time.sleep(0.1)

    def recv_cmd(self):
        hdr = self.conn.recv(Header.size,  socket.MSG_WAITALL)
        self.assertEqual(len(hdr),  Header.size,  'expected header of size %d but got %d' % (Header.size,  len(hdr)))
        (valid,  size,  data_type,  ntp) = Header.hdr_unpack(hdr)
        self.assertTrue(valid,  'invalid msg sync code')
        self.assertEqual(data_type,  Command.code,  'expecting a cmd but got type %x' % data_type)
        cmd = self.conn.recv(size,  socket.MSG_WAITALL)
        return Command.unpack(cmd)

    def init_seq(self):
        cmd = ''
        while cmd != 0x0020:
            (cmd,  resrv,  index,  param) = self.recv_cmd()
            self.conn.sendall(Reply.pack(cmd))

    def get_first_cmd(self):
        (cmd,  resrv,  index,  param) = self.recv_cmd()
        self.assertEqual(cmd,  0x0021,  'expected stop measure for first cmd in sequence but got: %x' % cmd )
