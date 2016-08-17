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

from exceptions import *
from dataproc import *
from utils import bitString
from params import LDMRSParams

class MsgHeader(object):
    format_struct = struct.Struct('!LLLBBHQ')
    # magic word is the sick terminology
    magic_word_str = '\xaf\xfe\xc0\xc2'
    magic_word_int = 0xaffec0c2
    size = 24

    @classmethod
    def gen_cmd(cls, length,  time):
        return cls.format_struct.pack(cls.magic_word_int,  0,  length,  0,  0,  MsgType.cmd,  time)

    @classmethod
    def parse(cls, header):
        (magic_word,  prev_size,  size,  resv,  devid,  data_type,  ntp) = cls.format_struct.unpack(header)
        return (magic_word,  size,  data_type,  ntp)


class MsgType(object):
    scan = 0x2202
    error = 0x2030
    cmd = 0x2010
    reply = 0x2020
    undocumented_error = 0x6430
    max_sizes = {
                    scan    :   96068,
                    error   :   16,
                    cmd     :   14,
                    reply   :   40,
                    undocumented_error  :   1024    # made up purposely large number until we get some info from sick
                 }

    max_size = max(max_sizes.values())

    @classmethod
    def valid_msg(cls, msg_code,  msg_size):
        return (msg_code in cls.max_sizes.keys()) and (msg_size <= cls.max_sizes[msg_code])



class ErrorWarningMsgHandler(object):
    """ Notifies the ROS network of error and warning conditions in the ldmrs
        and requests a restart of the appropriate driver layer if necessary
    """

    # Bit masks for error/warning detection.
    # Format is a dictionary mapping bitmasks to a list containing:
    # 1. the error/warning message in entry 0
    # 2. the action to take after the error/warning is transmitted (e.g. restart the node)
    #       no action is signalled by "ignore"

    # Mapping from bits to error messages for error register 1
                    # bits 0,1,4,10-13
    error_reg_1_masks = {0b0011110000010011:["Contact Support",  "fatal"],
                   # bit 2
                   0b0000000000000100:["Scan buffer transmitted incompletely, " +
                                    "decrease scan resolution/frequency/range;" +
                                    "contact support", "fatal"],
                    # bit 3
                   0b0000000000001000:["Scan buffer overflow, " +
                                    "decrease scan resolution/frequency/range; " +
                                    "contact support",  "fatal"]}

    # Mapping from bits to error messages for error register 2

                    # bits 0,1,2,7
    error_reg_2_masks = {0b0000000010000111:["Contact Support",  "fatal"],
                    # bits 4,5
                   0b0000000000110000:["Incorrect configuration data, " +
                                       "load correct configuration parameters",  "fatal"],
                    # bit 6
                   0b0000000001000000:["Data processing timeout, " +
                        "decrease scan resolution or scan frequency",  "fatal"]}

    # Mapping from bits to warning messages for warning register 1
                     # bit 0
    warning_reg_1_masks = {0b0000000000000001:["Internal communication error", "ignore"],
                     # bits 1,2,5,6
                     0b0000000001100110:["Internal warning", "ignore"],
                     # bit 3
                     0b0000000000001000:["Warning: temperature very low", "ignore"],
                     # bit 4
                     0b0000000000010000:["Warning: temperature very high",  "ignore"],
                     # bit 7
                     0b0000000010000000:["Sychronization error, " +
                        "check sychronization and scan frequency",  "restart"]}

    # Mapping from bits to warning messages for warning register 2
                     # bit 1
    warning_reg_2_masks = {0b0000000000000010:["Ethernet interface blocked, " +
                        "check Ethernet connection",  "info"],
                     # bit 3
                     0b0000000000001000:["Contact support", "ignore"],
                     # bit 4
                     0b0000000000010000:["Error recieving Ethernet data, " +
                        "check Ethernet connection/data", "restart"],
                     # bit 5
                     0b0000000000100000:["Incorrect or forbidden command recieved, " +
                        "check command", "ignore"],
                     # bit 6
                     0b0000000001000000:["Memory access failure, restart LD-MRS, " +
                        "contact support", "reset"]}

    bit_masks_list = [error_reg_1_masks,  error_reg_2_masks,
                      warning_reg_1_masks,  warning_reg_2_masks]
    # struct format string for parsing the message
    err_msg_struct = struct.Struct("<HHHHHHHH")

    def process_msg(self,  msg):
        # unpack as uint16 little endian words, words 0-3 are the registers
        bit_fields = self.err_msg_struct.unpack(msg)
        reset = False
        fatal = False
        restart = False
        info = False
        #reg_idx ranges over 0-3
        for reg_idx, bit_masks in enumerate(ErrorWarningMsgHandler.bit_masks_list):
            bit_field = bit_fields[reg_idx]  # actual register values
            # check for each error/warning in turn
            for bit_mask in bit_masks:
                set_bits = bit_field & bit_mask
                if set_bits:
                    # warning or error condition is present
                    if reg_idx < 2: #error registers are 0,1
                        rospy.logerr("ErrorRegister%d bits %s: "%(reg_idx+1, str(bitString(set_bits)))  + bit_masks[bit_mask][0])
                    else: # warning registers are 2,3
                        rospy.logwarn("WarningRegister%d bits %s: "%(reg_idx-1, str(bitString(set_bits)))  + bit_masks[bit_mask][0])
                    # ok, everything should be logged so now raise any exceptions if needed
                    if bit_masks[bit_mask][1] == "restart":
                        restart = True
                    elif  bit_masks[bit_mask][1] == "reset":
                        reset = True
                    elif bit_masks[bit_mask][1] == "fatal":
                        fatal = True
                    elif bit_masks[bit_mask][1] == "info":
                        info = True

            if fatal:
                raise FatalException('fatal LD-MRS error code recieved...can not continue')
            if reset:
                raise ResetDSPException('LD-MRS ResetDSP command required')
            if restart:
                raise ErrorRestartException('LD-MRS comms issue, restarting network')
            if info:
                raise InfoRestartException('LD-MRS comms issue, restarting network')




class CommandHandler:

    def __init__(self, params):
        """ Constructor
            @param params: ldmrs parameters 
            @type params: dict {ldmrs_parameter_name:value}
        """

        self.params = params

        # mapping from command names to command generator functions
        self.gen_cmd = {"Reset":self.gen_reset_cmd,
                        "GetStatus":self.gen_get_status_cmd,
                        "SaveConfig":self.gen_save_config_cmd,
                        "SetParameter":self.gen_set_parameter_cmd,
                        "GetParameter":self.gen_get_parameter_cmd,
                        "ResetDefaultParameters":self.gen_reset_default_params_cmd,
                        "StartMeasure":self.gen_start_measure_cmd,
                        "StopMeasure":self.gen_stop_measure_cmd,
                        "SetNTPTimestampSec":self.gen_set_NTP_timestamp_sec_cmd,
                        "SetNTPTimestampFracSec":self.gen_set_NTP_timestamp_frac_sec_cmd}

        # dictionary mapping command ids to command names
        self.cmd_name = {0x0000:"Reset",
                        0x0001: "GetStatus",
                        0x0004: "SaveConfig",
                        0x0010: "SetParameter",
                        0x0011: "GetParameter",
                        0x001A: "ResetDefaultParameters",
                        0x0020: "StartMeasure",
                        0x0021: "StopMeasure",
                        0x0030: "SetNTPTimestampSec",
                        0x0031: "SetNTPTimestampFracSec"}

        # inverted cmd_name dictionary maps command names to command ids
        self.name_cmd = dict([[v,k] for k,v in self.cmd_name.iteritems()])

        # precompiled data unpacking structs
        self.struct_uint32le = struct.Struct('<L')  # 32 bit litle endian word (unsigned)
        self.struct_uint16le = struct.Struct('<H')  # 16 bit litle endian word (unsigned)
        self.struct_int16le = struct.Struct('<h')  # 16 bit litle endian word (signed)

        # restart sequence vars
        self.state = 0      # current state in restart sequence
        self.next_state = 1   # next state in restart sequence

        # restart command sequence (a simple state machine)
        # format is {state_id:[action(), next_state], ... }
        self.restart_seq = {0 : [lambda: self.generate_cmd("StopMeasure"),  1],
                            # Set the scan_frequency to 12.5Hz (3200 ticks per second) initially
                            # so that we can set any angular_res_type. Otherwise we see errors
                            # with conflicting angular_res_type and scan_frequency settings.
                            # reversing the order of operations will not help this either
                            # since the parameters are set one at a time
                            1 : [lambda: self.generate_cmd("SetParameter", "scan_frequency",  3200), 2],
                            2 : [lambda: self.generate_cmd("SetParameter", "start_angle"), 3],
                            3 : [lambda: self.generate_cmd("SetParameter", "end_angle"), 4],
                            4 : [lambda: self.generate_cmd("SetParameter", "constant_angular_res"), 5],
                            5 : [lambda: self.generate_cmd("SetParameter", "scan_frequency"), 6],
                            6 : [lambda: self.generate_cmd("StartMeasure"), -1]} # -1 signals end of sequence


    def process_msg(self,  msg):
        """ Process an incoming command ack message.
            This should only occur during a restart sequence.
            The actual message is ignored and we assume that the reciept of a
            command ack message signals the completion of the previous command
            in the restart sequence and triggers the return of the next message.
            @param msg: the command ack message to be processed
            @type msg: read only buffer (e.g. string)
            @raise ErrorRestartException: Command reply has incorrect size
            @return the next command message to be sent to the device (a string),
                returns None if no more commands are to be sent
                
        """

        if len(msg) >self.struct_uint16le.size:
            raise ErrorRestartException('not expecting a command reply of length %d' % len(msg))
        else:
            status = self.struct_uint16le.unpack(msg)
            # this command sequence has been used over and over, an error here means something
            # is seriously wrong internally
            if status[0] & 0x8000:
                raise ErrorRestartException('error command reply code: %x' % status[0])

        return self.transition_states(msg)

    def transition_states(self,  msg):
        """ Move to the next state in the restart command sequence and
        return the command for that state
        @param msg: read only buffer contatining the ldmrs scan data message
        @type msg: read only buffer (e.g. string)
        @return the next command message in the sequence
        """
        if self.next_state >= 0:
            self.state = self.next_state
            self.next_state = self.restart_seq[self.state][1]
            action_fun = self.restart_seq[self.state][0]
            return action_fun()  # return the next message in the sequence
        else: # end of sequence reached
            return None

    def generate_cmd(self,  cmd_name,  *args):
        """ Generate a command byte string to send to the LD-MRS.
            The returned byte string needs to be encapsulated with
            a message header with data type of 0x2010.
            @param cmd_name: Name of the command to send
                (per. self.params.parameter_index.keys())
            @type cmd_name: string
            @param *args: additional arguments as required by the
                generator function referenced by self.gen_cmd[cmd_name]
            @type *args: varargs
            @return: string containing the next command to be sent to the ldmrs
        """
        return self.gen_cmd[cmd_name](*args)

    def first_message(self):
        """ This method is called once to initiate the restart command sequence.
        Subsequent commands are returned via the normal operation of process_msg()
        when command acknowledgement messages are recieved.
        @return: the first message in the restart sequence
        """
        self.state = 0
        self.next_state = self.restart_seq[self.state][1]
        return self.restart_seq[self.state][0]()



    #-------------------------------------------
    # command generators
    #       Refer to the LD-MRS manual for descriptions of each command
    #------------------------------------------
    def gen_reset_cmd(self):
        """ Generate byte string for the Reset DSP command
        """
        return '\x00\x00\x00\x00'

    def gen_get_status_cmd(self):
        """ Generate a "get_status" command string
            @return the command string for the device
                (must be wrapped in a header with data type 0x2010)
        """
        return '\x01\x00\x00\x00'

    def gen_set_parameter_cmd(self,  param_name,  param_value=None):
        """ Generate a "set_parameter" command message, if no value is passed,
            use the value in self.params
            @param param_name: the LD-MRD parameter name to be set
                (in self.params.parameter_index.keys())
            @type param_name: string
            @param value: value to set. If none, use value in self.params
            @type value: value required by corresponding parameter
            @return the command string for the device
                (must be wrapped in a header with data type 0x2010)
        """
        if param_value is None:
            if param_name in self.params:
                param_value = self.params[param_name]
            else:
                raise Exception('Error in gen_set_parameter_cmd: unknown parameter name: %s'%param_name)
        param_index_str = LDMRSParams.parameter_index[param_name][0]
        param_dtype = LDMRSParams.parameter_index[param_name][1]
        # pack the value appropriately for the corresponding data type
        if param_dtype == "uint32":
            param_val_str = self.struct_uint32le.pack(param_value)
        elif param_dtype == "uint16":
            param_val_str = self.struct_uint16le.pack(param_value) + '\x00\x00'
        elif param_dtype == "int16":
            param_val_str = self.struct_int16le.pack(param_value) + '\x00\x00'
        return '\x10\x00\x00\x00' + param_index_str + param_val_str

    def gen_get_parameter_cmd(self,  param_name):
        """ Generate a get parameter command string
            @param param_name: the LD-MRS parameter name to be set
                (in self.param.parameter_index.keys())
            @type param_name: string
            @return: the command string for the device
                (must be wrapped in a header with data type 0x2010)
        """
        return '\x11\x00\x00\x00' + LDMRSParams.parameter_index[param_name]

    def gen_save_config_cmd(self):
        """ Generate a save configuration command string
            This saves the current configuration permanently
            @return: save configuration command (must be wrapped in a header with data type 0x2010)
        """
        return '\x04\x00\x00\x00'

    def gen_reset_default_params_cmd(self):
        """ Generate reset default parameters command
            @return: reset default parameters command (must be wrapped in a header with data type 0x2010)
        """
        return '\x1a\x00\x00\x00'
        
    def gen_start_measure_cmd(self):
        """ Generate a start measure command
            @return: start measure command (must be wrapped in a header with data type 0x2010)
        """
        return '\x20\x00\x00\x00'

    def gen_stop_measure_cmd(self):
        """ Generate a stop measure command
            @return: stop measure command (must be wrapped in a header with data type 0x2010)
        """
        return '\x21\x00\x00\x00'

    def gen_set_NTP_timestamp_sec_cmd(self,  secs):
        """ Generate command string to set NTP time stamp
            seconds field on the device
            @param secs: 16bit seconds time value
            @type secs: uint16
            @return: the command string for the device
                (must be wrapped in a header with data type 0x2010)
        """
        return '\x30\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00' + \
                    self.struct_uint32le.pack(secs)

    def gen_set_NTP_timestamp_frac_sec_cmd(self,  frac_secs):
        """ Generate command string to set NTP time stamp
            fractional seconds (*2^-32 sec) field on the device
            @param frac_secs: 16bit fractional seconds time value
            @type frac_secs: uint16
            @return: the command string for the device
                (must be wrapped in a header with data type 0x2010)
        """
        return '\x31\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00' + \
                    self.struct_uint32le.pack(frac_secs)
