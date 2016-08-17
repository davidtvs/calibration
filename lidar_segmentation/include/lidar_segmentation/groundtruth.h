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
\file  groundtruth.h 
\brief Groundtruth related functions header
\author Daniel Coimbra
*/

#ifndef _COIMBRA_GROUNDTRUTH_H_
#define _COIMBRA_GROUNDTRUTH_H_

#include <vector>


/**
 * \class C_DataFromFile
 * Class to accumulate the data from the .txt file 
 * \date April 2013
 * \author Daniel Coimbra
 * 
 */

class C_DataFromFile
{

public:	
	
	int iteration; /**< Number of the iteration */
	
	vector<double> x_valuesf; /**< Vector with all the the xx values of one iteration */
	
	vector<double> y_valuesf; /**< Vector with all the the yy values of one iteration */

	vector<double> z_valuesf; /**< Vector with all the the zz values of one iteration */
	
	vector<int> labels;			/**< Vector with all the the labels of one iteration */
};

//Shared pointer to the C_DataFromFile class
typedef boost::shared_ptr<C_DataFromFile> C_DataFromFilePtr;


/**
@brief Reads from a file the x, y and labels values from all the laser points from one iteration
@param data_gts output x, y and labels from one iteration, it uses the C_DataFromFile class 
@param values_per_scan Number of values per scan
@return The x, y and labels values from all the laser points from one iteration
*/

int readDataFile( vector<C_DataFromFilePtr>&  data_gts , int values_per_scan);


/**
@brief Performs Segmentation operation with the Adaptative Breakpoint Detector
@param points incoming Laser Points
@param clusters_GT clusters output vector of clusters, these clusters use the Cluster class
@return Number of clusters resulted from the Adaptative Breakpoint Detector
*/

int convertPointsToCluster(vector<PointPtr>& points, vector<ClusterPtr>& clusters_GT);

#endif
