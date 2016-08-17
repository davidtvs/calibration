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

import socket
import errno

from exceptions import *

class XPort(object):
    '''Class representation of a socket transport thread.

    Simplification wrapper around socket.socket and related objects.
    The send and recv methods will wait for all requested data and/or
    provide a filtered exception (something good, bad, or ugly happened).
    '''

    def __init__ (self,  host,  port):
        ''' XPort constructor, connect to requested peer.

        Connect to requested network peer or raise reason why not.
        @param host: hostname to connect to.  can be name or IPv4 dot notation.
        @type host: str
        @param port: network port to connect to
        @type port: int
        @raise FatalExcetpion: Host/port does not exist or is unreachable.
        @raise TimeoutException: Network is reachable but timeout on connect.
        Likely host is not yet ready.
        @raise InfoRestartException: Non-fatal network problem, able to re-try.
        @raise ErrorRestartException: More serious network problem, possibly can
        re-try.
        '''
        try:
            self.__shutdown = False

            self.__sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            # don't want small messages lingering in the buffers
            self.__sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            # needed for just calling connect()?  doesn't hurt to set it...
            self.__sock.setsockopt(socket.SOL_SOCKET,  socket.SO_REUSEADDR,  1)

            # 5 second timeout should be more than sufficient
            # in any case of this timeout happening we want the exception to
            # force a restart
            self.__sock.settimeout(5.0)

            host_ip = socket.gethostbyname(host)

            self.__sock.connect((host_ip, port))

        # addressing error is fatal, it means a supplied host does not exist
        # or can not be found
        except (socket.herror,  socket.gaierror),  e:
           raise FatalException('address exception in XPort init:  %s' % e)

        # in init a timeout means restart the system, most likely the LD-MRS
        # isn't ready yet
        except socket.timeout,  e:
            raise TimeoutException('timeout exceeded in host connect: %s' % e)

        except socket.error,  e:
            fatals = [errno.ENETUNREACH, errno.EACCES, errno.EPERM, errno.EFAULT]
            not_found = [errno.ECONNREFUSED, errno.EHOSTUNREACH, errno.EHOSTDOWN,
                         errno.EALREADY]
            if e.errno in fatals:
                raise FatalException('fatal error in socket init sequence: %s'
                                     % e)
            elif e.errno in not_found:
                raise InfoRestartException('no host found at %s:%d...continuing to try'
                                           % (host,  port))
            else:
                raise ErrorRestartException('error in socket init sequence: %s'
                                            % e)

    def recv(self,  length):
        ''' Wait for data from the network peer.

        Wait until all requested data from the network peer has arrived.
        @param length: how much data is requested
        @type length: int
        @return: data from network peer
        @rtype: str
        @raise TimeoutException: no data has arrived in the timeout period
        @raise InfoRestartException: remote peer has closed the socket, the
        data request can not be fulfilled but able to reconnect to peer.
        @raise NextMsgException: signal interruption, current data request
        can not be fulfilled but new request can be issued
        @raise ErrorRestartException: network problem reported and data request
        can not be fulfilled.  should reconnect to peer.
        '''
        data_recv = 0
        return_string = ''
        next_msg = [errno.EINTR,  ]
        info = [errno.ECONNRESET, errno.EPIPE]
        while data_recv < length:
            try:
                data = self.__sock.recv(length - data_recv, socket.MSG_WAITALL)
            except socket.timeout:
                raise TimeoutException('timeout on socket.recv')
            except socket.error,  e:
                if e.errno in next_msg:
                    raise NextMsgException()
                elif e.errno in info:
                    raise InfoRestartException('socket.recv restarting xport: %s'
                                               % e)
                else:
                    raise ErrorRestartException('error in socket recv: %s' % e)

            if not data:
                raise InfoRestartException('socket.recv zero-length, likely a shutdown signal')
            else:
                return_string += data
                data_recv += len(data)

        return return_string


    def send(self,  cmd):
        '''Send a data message to the network peer.

        Send all requested message data.
        @param cmd: data to send
        @type cmd: str
        @raise TimeoutException: data could not be sent during the timeout
        period, likely the peer has gone down
        @raise InfoRestartException: peer has closed connection or local shutdown
        signal caught.  either way current send can not be fulfilled.
        @raise ErrorRestartException: network problem reported and data send
        can not be fulfilled.  should reconnect to peer.
        '''

        info = [errno.EPIPE,  errno.ECONNRESET,  errno.EINTR]
        try:
            self.__sock.sendall(cmd)
        except socket.timeout:
            raise TimeoutException('timeout on socket.sendall')
        except socket.error,  e:
            if e.errno in info:
                # EPIPE should be a shutdown signal
                raise InfoRestartException('socket.sendall: %s' % e)
            else:
                raise ErrorRestartExcetpion('error in socket.sendall: %s' % e)

    def shutdown(self):
        ''' Call socket.shutdown to pop out of any blocking send/recv '''
        # this can be called from external callbacks as well
        # as the main loop exit
        if not self.__shutdown:
            # call shutdown first to immediately terminate comms
            # close will linger trying to flush any pending data
            self.__shutdown = True
            try:
                self.__sock.shutdown(socket.SHUT_RDWR)
            except socket.error:
                # we're shutting down so don't care about socket level errors
                pass

    def close(self):
        '''close the xport socket'''
        try:
            self.__sock.close()
        except socket.error:
            # we're shutting down so don't care about socket-level errors
            pass













