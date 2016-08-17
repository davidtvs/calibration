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

from __future__ import division

import roslib
roslib.load_manifest('sick_ldmrs')
import rospy

""" Some useful routines
"""

#--------------------------------------------------------

def NTP64_to_ROStime(NTPtime):
    """ Convert a 64bit NTP timestamp to rostime.Time

    @param NTPtime: 64bit NTP timestamp. high 32bits are seconds,
    low 32bits are fractional seconds (*2^-31)
    @type NTPtime: int
    @return: rospy.Time with seconds and nanoseconds
    fields initialised based on input
    @rtype: rospy.Time
    """
    seconds = int(NTPtime >> 32)
    frac_seconds = NTPtime & 0xFFFFFFFF
    # nanoseconds =  frac_seconds * 1x10^9      / 2^31
    nanoseconds   = int(round((frac_seconds * 0x3b9aca00) / 0x100000000))
    return rospy.Time(seconds,  nanoseconds)

def NTP64split_to_ROStime(seconds_part, fractional_part):
    """ Convert seconds and fractional seconds to a rospy.Time object

    @param seconds_part: high 32 bits of NTP64 timestamp
    @type seconds_part: int
    @param fractional_part: low 32 bits of NTP64 timestamp
    @type fractional_part: int
    @return: rospy.Time representation of the input
    @rtype: rospy.Time
    """
    NTPtime = seconds_part << 32 | fractional_part
    return NTP64_to_ROStime(NTPtime)

def ROStime_to_NTP64(ROStime):
    """ Convert rospy.Time object to a 64bit NTP timestamp

    @param ROStime: timestamp
    @type ROStime: rospy.Time
    @return: 64bit NTP representation of the input
    @rtype: int
    """
    NTPtime = ROStime.secs << 32
    # NTPtime |= nanoseconds * 2^31 / 1x10^9
    NTPtime |= int((ROStime.nsecs * 0x100000000) / 0x3b9aca00)
    return NTPtime

def Now_to_NTP64():
    """ Get 64bit NTP timestamp of current system time

    @return: NTP representation of current system time
    @rtype: int
    """
    return ROStime_to_NTP64(rospy.Time.now())

def bitString(integral_value, min_bits=8):
    """ Return a string representation of a binary value

    Return a string representation of a binary value, padded with zeros in the
    higher order bits if necessary
    @param integral_value: integer or long integer to convert to bit string
    @type integral_value: int
    @param min_bits: minimum number of bits to display in the result
    (pad zeros to the left if necessary)
    @type min_bits: int
    @return: ascii string containing the binary representation of the
    of the input value
    @rtype: str
    """
    bitstring = bin(integral_value).split('b')[1]
    pad_zeros = min_bits - len(bitstring)
    if pad_zeros > 0:
        bitstring = '0'*pad_zeros + bitstring
    return bitstring



