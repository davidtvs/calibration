/**************************************************************************************************
 Software License Agreement (BSD License)

 Copyright (c) 2014-2015, LAR toolkit developers - University of Aveiro - http://lars.mec.ua.pt
 All rights reserved.

 Redistribution and use in source and binary forms, with or without modification, are permitted
 provided that the following conditions are met:

  *Redistributions of source code must retain the above copyright notice, this list of
   conditions and the following disclaimer.
  *Redistributions in binary form must reproduce the above copyright notice, this list of
   conditions and the following disclaimer in the documentation and/or other materials provided
   with the distribution.
  *Neither the name of the University of Aveiro nor the names of its contributors may be used to
   endorse or promote products derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************************************************************************************************/
/**
 \file  camera_config.cpp
 \brief Configuration of the camera mode
 \details This configures the image format of the cameras. Thus, both cameras have the exactly the same configuration, allowing to use them as a stereo system
 \author Marcelo Pereira
*/

#include "calibration/camera_config.h"

/**
@brief Confifuration of the camera image format
@param[in] camera
@return bool
*/
bool SetConfiguration(Camera &camera)
{

    uint16_t roi_width;
    uint16_t roi_height;
    uint16_t roi_offset_x;
    uint16_t roi_offset_y;

    // Error for checking if functions went okay
    Error error;

    // Get Format7 information
    Format7Info fmt7Info;
    bool supported;
    //fmt7Info.mode = fmt7Mode;
    error = camera.GetFormat7Info(&fmt7Info, &supported);
    if ( error != PGRERROR_OK )
    {
        cout << "Failed to get camera info from camera" << endl;
        return false;
    }

    // Make Format7 Configuration
    Format7ImageSettings fmt7ImageSettings;
    fmt7ImageSettings.mode = MODE_1;
    fmt7ImageSettings.pixelFormat = PIXEL_FORMAT_RAW8;

    // Check Width
    roi_width = 964; // Locks the width into an appropriate multiple using an integer divide

    fmt7ImageSettings.width = roi_width;


    // Check Height
    roi_height = 724; // Locks the height into an appropriate multiple using an integer divide

    fmt7ImageSettings.height = roi_height;


    // Check OffsetX
    roi_offset_x = 0;  // Locks the X offset into an appropriate multiple using an integer divide

    fmt7ImageSettings.offsetX  = roi_offset_x;

    // Check OffsetY
    roi_offset_y = 0;  // Locks the X offset into an appropriate multiple using an integer divide

    fmt7ImageSettings.offsetY  = roi_offset_y;

    // Validate the settings to make sure that they are valid
    Format7PacketInfo fmt7PacketInfo;
    bool valid;
    error = camera.ValidateFormat7Settings(&fmt7ImageSettings, &valid, &fmt7PacketInfo);
    if ( error != PGRERROR_OK )
    {
        cout << "Failed to validate format" << endl;
        return false;
    }

    // Stop the camera to allow settings to change.
    error = camera.SetFormat7Configuration(&fmt7ImageSettings, fmt7PacketInfo.recommendedBytesPerPacket);
    if ( error != PGRERROR_OK )
    {
        cout << "Failed to set format" << endl;
        error.PrintErrorTrace();
        return false;
    }

    // Get camera info to check if camera is running in color or mono mode
    CameraInfo cInfo;
    error = camera.GetCameraInfo(&cInfo);
    if ( error != PGRERROR_OK )
    {
        cout << "Failed to get camera info2 from camera" << endl;
        return false;
    }


}
