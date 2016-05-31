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



import roslib; roslib.load_manifest("sick_ldmrs")

import rospy
from rospy.numpy_msg import numpy_msg # for numpy arrays in the messages
from sensor_msgs.msg import PointCloud2, LaserScan

from dynamic_reconfigure import server

from sick_ldmrs.xport import *
from sick_ldmrs.ldmrsmsgs import *
from sick_ldmrs.dataproc import *
from sick_ldmrs.utils import *
from sick_ldmrs.cfg import ldmrsConfig


topics = {}.fromkeys(( 'cloud',  'scan0',  'scan1',  'scan2',  'scan3'))

# these need to be global so the shutdown hook can access them
xport = None
scanning = False
param_handler = None

def stop_xport():
    ''' Signal the socket system to shutdown.'''
    global xport

    if xport:
        xport.shutdown()

def node_shutdown_hook():
    ''' Node pre-shutdown callback.

    When notified of impending node shutdown by master,
    force exit of transport loop by calling socket.shutdown.
    '''
    rospy.logdebug('node shutdown hook called')
    stop_xport()

def get_msg():
    '''Receive a message from the LD-MRS.

    Block until an entire message from the LD-MRS has been received.
    First look for a proper start code (or 'magic word' in Sick terminology)
    and then using the supplied message size, wait for the rest.

    @return: Tuple of:

    message type (data_type field from header)

    the raw message itself (minus the header)

    recv_time: current system time when message was received

    @rtype: int, str, int
    '''
    magic_word = ~MsgHeader.magic_word_int

    while magic_word != MsgHeader.magic_word_int:
            data_needed = MsgHeader.size
            header = ''
            while len(header)  < MsgHeader.size:
                data = xport.recv(data_needed)
                recv_time = rospy.get_time()
                if not header:
                    magic_idx = data.find(MsgHeader.magic_word_str)
                    if magic_idx > -1:
                        header = data[magic_idx:]
                        data_needed -= len(header)
                else:
                    # with WAITALL should never get here but need to account
                    # for any network issues
                    data_needed -= len(data)
                    header += data
            try:
                (magic_word,  size,  data_type,  ntp) = MsgHeader.parse(header)

            except Exception,  e:
                rospy.logdebug('recv header parse exception: %s' % e)
                magic_word =  ~MsgHeader.magic_word_int

            else:
                if not MsgType.valid_msg(data_type,  size):
                    rospy.logdebug('invalid message detected in header unpack:  data_type = %x  size = %d'
                                   % (data_type,  size))
                    magic_word = ~MsgHeader.magic_word_int

    # unfortunately python socket doesn't have an interface for SO_TIMESTAMP
    # which would be handy here, although that might have portability issues?

    # pull out rest of message
    msg =  xport.recv(size)

    return (data_type,  msg, recv_time)

def send_msg(msg):
    ''' Send message to device.

    Format and prepend the required message header to the message
    string and send to the device.
    @param msg: message to send device, expected to be properly formatted but not
    containing the message header
    @type msg: str
    '''
    global xport

    hdr = MsgHeader.gen_cmd(len(msg),  Now_to_NTP64())
    xport.send(hdr + msg)

def msg_loop(cmd,  err,  scan):
    ''' Loop forever receiving messages and processing messages.

    Main message loop.  Sit on a socket.recv waiting for a message from
    the device.  When one comes in parse the message type and pass the
    message data to the appropriate parsing object.

    The LD-MRS has an awkward time update protocol involving sending
    the second and fractional components in two distinct messages so
    rather than try to deal with a complicated synchronization with
    the system time, we just treat the device time as a constant offset
    (it starts at 0) and stamp the messages with deltas from the system time.
    The offset constant is averaged from the device replies from the initial
    parameter update.
    @param cmd: command/reply processor
    @type cmd: CommandHandler
    @param err: error message processor
    @type err: ErrorWarningMsgHandler
    @param scan: scan data processor
    @type scan: ProcessLDMRSData
    @raise FatalException: A Sick undocumented error message has been received.
    @raise ErrorRestartException: A number of consecutive device timeouts have
    occurred which most likely means the device has gone down.
    '''
    global xport,  scanning

    timeouts = 0
    # yes there are two loops polling this same method,
    # makes it simpler than having to do the shutdown
    # checks at various points within this loop
    while not rospy.is_shutdown():
        try:
            # this can (and will) block until a valid message is received
            (msg_type,  msg, recv_time) = get_msg()
            timeouts = 0
            # error processing is independent of cmd/scan state
            if msg_type == MsgType.error:
                    err.process_msg(msg)
            # This one was discovered during development and isn't listed in any
            # of the documentation we have from Sick.  Seems to be an error
            # description string based on invalid parameters.
            elif msg_type == MsgType.undocumented_error:
                    raise FatalException('undocumented LD-MRS error: %s' % msg)
            elif msg_type == MsgType.reply:
                    if scanning:
                        rospy.logdebug('Command reply received in scanning mode???  Msg: %s' % msg)
                        raise NextMsgException
                    else:
                        cmd_to_send = cmd.process_msg(msg)
                        if not cmd_to_send:
                            #seems to be a delay when LD-MRS transitions
                            #to start scanning
                            rospy.sleep(4.0)
                            scanning = True
                            rospy.logdebug('init sequence complete,  transitioning to scanning')
                        else:
                            send_msg(cmd_to_send)

            elif msg_type == MsgType.scan:
                if scanning:
                    scan.process_msg(msg,rospy.Time(recv_time))
                else:
                    rospy.logdebug('Dropped message since I am not in  scanning mode')
            else:
                rospy.loginfo('received invalid message type: %x ????'
                              % msg_type)

        # soak up this exception to go back into the loop
        except NextMsgException:
            pass

        except TimeoutException:
            timeouts += 1
            if timeouts > 10:
                raise ErrorRestartException('consecutive timeouts in message receive')

def init_ros():
    ''' ROS node initialization.

    Initialize the node, advertise topics, and any other ROS housekeeping.
    '''
    global topics, param_handler

    rospy.init_node('sickldmrs',  log_level=rospy.DEBUG)

    node_name = rospy.get_name().split('/')[-1]

    rospy.loginfo('node %s starting up.' % node_name)

    topics['cloud'] = rospy.Publisher('/%s/cloud' % node_name,
                                      numpy_msg(PointCloud2))

    topics['scan0']  = rospy.Publisher('/%s/scan0' % node_name,
                                       numpy_msg(LaserScan))
    topics['scan1']  = rospy.Publisher('/%s/scan1' % node_name,
                                       numpy_msg(LaserScan))
    topics['scan2']  = rospy.Publisher('/%s/scan2' % node_name,
                                       numpy_msg(LaserScan))
    topics['scan3']  = rospy.Publisher('/%s/scan3' % node_name,
                                       numpy_msg(LaserScan))


    rospy.on_shutdown(node_shutdown_hook)

def _update_config_callback(config,  level):
    ''' Callback for dynamic_reconfigure server.

    If apply_changes parameter is set to true restart
    and update the device parameters.
    @param config: the configuration dictionary
    @type config: dict
    @param level: the run level (not used)
    @type level: int
    @return: new configuration parameters.
    @rtype: dict
    '''
    global param_handler

    rospy.logdebug("Checking parameters are valid: %s ..." %str(config))
    config = param_handler.update_config(config,  level,  fix_errors=True)
    rospy.logdebug("Parameters are OK")

    if  ("apply_changes" in config) and config["apply_changes"]:
        config["apply_changes"] = False # make sure changes are only applied once
        # Restart the transport layer since we need to change the device
        # parameters,and this can only be done on restart with the present
        # implementation
        rospy.loginfo("Restarting with ROS parameters: %s, Equivalent to device parameters: %s"\
                                %(str(config),  str(param_handler.ldmrs_params)))
        stop_xport()

    return config

def ros_main():
    ''' Main ROS node loop.

    Main function of module, which is also the main node loop.
    The node will receive scan data from the device and then hand the scan
    to the processing object which will process and publish the data if there
    are any subscribers to the topic.
    To keep implementation simple, most state is contained in the existence or
    not of an object.  If there is an issue, recover by destroying the object
    and reinitializing.  Then we just filter on the exception type to determine
    how to proceed.
    @return: This is the module main() so the return here is the exit code.
            0 - Exit ok
            1 - Fatal internal problem caused exit
            2 - Fatal ROS problem caused exit
    @rtype: int
    '''
    global xport, scanning, topics, param_handler

    try:
        init_ros()
    except Exception,  e:
        print 'Fatal exception initializing ROS stack: %s...exiting' % e
        return 2

    # LD-MRS manual recommends 5 second wait for laser to become available
    # put in a get status monitor?
    rospy.sleep(5.0)

    rospy.logdebug('startup delay finished')

    reset_dsp = False

    timeouts = 0

    # needs to exist before creating the dynamic_reconfigure server
    param_handler = LDMRSParams()

    # dynamic_reconfigure server
    # Invokes callback on init and initializes param_handler.
    conf_server = server.Server(ldmrsConfig, _update_config_callback)

    # these have static state and dont have to be reconfigured on restart
    error_handler = ErrorWarningMsgHandler()
    cmd_handler  = CommandHandler(param_handler.ldmrs_params)

    while not rospy.is_shutdown():
        try:
            # these two are not in the dynamic config as they are expected to be
            # statically mapped via launch file
            host = rospy.get_param('~host',  '192.168.0.244')
            port = rospy.get_param('~port',  12002)
            xport = XPort(host,  port)

            rospy.loginfo('connected to %s:%d' % (host,  port))

            # respawn these on restart and pass in (possibly updated) parameters
            scan_handler = ProcessLDMRSData(topics, param_handler.ldmrs_params)

            if reset_dsp:
                # there is no reply for this command
                send_msg(cmd_handler.generate_cmd('Reset'))
                # TODO: need to verify this timeout with laser behavior
                rospy.sleep(5.0)
                reset_dsp = False

            # kick-start the init sequence
            send_msg(cmd_handler.first_message())

            msg_loop(cmd_handler, error_handler, scan_handler)

        # nothing to do here, the finally block will handle the restart
        # as named, just log the cause to info
        except InfoRestartException,  e:
            rospy.loginfo('restarting xport due to: %s' % e)

        # restart again, really the only difference between this and
        # InfoRestart is the logging path.  also a good spot for a breakpoint...
        except ErrorRestartException,  e:
            rospy.logerr('restarting xport due to: %s' % e)

        except FatalException,  e:
            rospy.logfatal('fatal exception: %s...exiting' % e)
            return 1

        # timeout at this level is either from connection or initial
        # message xmit, keep going but log the issue
        except TimeoutException,  e:
            timeouts += 1
            if timeouts > 50:
                rospy.logwarn('50 timeouts for %s...continuing' % e)
                timeouts = 0

        # silent restart, any logging needs to be done at the raising layer
        except ResetDSPException:
            reset_dsp = True

        # exception raised to signal shutdown from a sleep, just pop out
        except rospy.ROSInterruptException:
            pass

        except rospy.ROSException,  e:
            # assuming if we're in here then the logging was initialized ok
            rospy.logfatal('ROS stack exception %s...exiting' % e)
            return 2

        except InvalidParamterException,  e:
            rospy.logerr('Exiting due to invalid parameter settings: %s' % e)
            return 3

        except Exception,  e:
            rospy.logwarn('Caught unknown exception: %s...restarting transport.' % e)

        finally:
            # close immediately to kill the connection as we
            # don't know when the garbage collector will kick in
            # need an if here as we could hit this finally from a connection
            # error in xport.init
            if xport:
                xport.close()

            scanning = False
            # bit of a delay as the is_shutdown does not seem to be set immediately
            # and we're looping back to re-connect
            # probably because the shutdown hook is causing this exit and our
            # timeslice is allowing this execution before the main teardown code
            # kicks in
            rospy.sleep(0.5)

if __name__ == '__main__':
    ros_main()

