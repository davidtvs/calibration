/**************************************************************************************************
 Software License Agreement (BSD License)

 Copyright (c) 2011-2013, LAR toolkit developers - University of Aveiro - http://lars.mec.ua.pt
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
 * \file
 * \brief Reconfigure the laser sensor

/*
 * setConfig.cpp
 *
 *  Created on: 09-09-2011
 *      Author: konradb3
 */

#include <iostream>

#include <boost/lexical_cast.hpp>

#include <lms1xx/lms1xx.h>

void print_usage() {

	std::cout << " Usage : " << std::endl;
	std::cout << " set_config ip resolution rate " << std::endl;
	std::cout << " Exqample : " << std::endl;
	std::cout << " set_config 192.168.1.2 0.25 50 " << std::endl;
}

int main(int argc, char** argv) {

	LMS1xx laser;
	scanCfg sCfg;

	if(argc < 4) {
		print_usage();
		return 0;
	}

	laser.connect(argv[1]);

	if(!laser.isConnected()) {
		std::cout << "Unable to connect to device at address : " << argv[1] << std::endl;
		return 0;
	}

	sCfg.angleResolution = (int)(boost::lexical_cast<double>(std::string(argv[2])) * 10000);
	sCfg.scaningFrequency = boost::lexical_cast<int>(std::string(argv[3])) * 100;

	laser.login();
	laser.setScanCfg(sCfg);
	laser.saveConfig();

	sCfg = laser.getScanCfg();

	std::cout << "Configuration set to : " << std::endl;
	std::cout << "resolution : " << (double)sCfg.angleResolution/10000.0 << std::endl;
	std::cout << "frequency : " << (double)sCfg.scaningFrequency/100.0 << std::endl;

	laser.disconnect();

	return 0;
}
