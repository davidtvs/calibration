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
\file  visualization_rviz.h 
\brief Visualization on rviz related functions header 
\author Daniel Coimbra
*/

#ifndef _COIMBRA_VISUALIZATION_RVIZ_H_
#define _COIMBRA_VISUALIZATION_RVIZ_H_

#include <vector>

/**
 * \class Markers
 * Class to handle the visualization markers 
 * \author Jorge Almeida in mtt
 * 
 */

class Markers
{
	public:		
		void update(visualization_msgs::Marker& marker)
		{
			for(uint i=0;i<markers.size();++i)
				if(markers[i].ns==marker.ns && markers[i].id==marker.id)//Marker found
				{
					markers.erase(markers.begin()+i);
					break;
				}
				
			markers.push_back(marker);
		}
		
		void decrement(void)
		{
			for(uint i=0;i<markers.size();++i)
			{
				switch(markers[i].action)
				{
					case visualization_msgs::Marker::ADD:
						markers[i].action = visualization_msgs::Marker::DELETE;
						break;
					case visualization_msgs::Marker::DELETE:
						markers[i].action = -1;
						break;
				}
			}
		}
		
		void clean(void)
		{
			vector<visualization_msgs::Marker> new_markers;

			for(uint i=0;i<markers.size();++i)
				if(markers[i].action!=-1)
					new_markers.push_back(markers[i]);
				
			markers=new_markers;
		}

		vector<visualization_msgs::Marker> getOutgoingMarkers(void)
		{
			vector<visualization_msgs::Marker> m_out(markers);
			return markers;
		}
		
	private:
		
		vector<visualization_msgs::Marker> markers;
};


/**
@brief Create marker form clusters
 * This function creates a set of markers for the current clusters from the sereral segmentation algorithms
 *this uses the Markers class to keep a up to date list of the current markers. 
@param clusters Cluster class clusters from the simpleClustering function
@param clusters_Premebida Cluster class clusters from the premebidaClustering function
@param clusters_Dietmayer Cluster class clusters from the dietmayerClustering function
@param clusters_Santos Cluster class clusters from the santosClustering function
@param clusters_ABD Cluster class clusters from the abdClustering function
@param clusters_nn Cluster class clusters from the nnClustering function
@param clusters_GT Cluster class clusters from the convertPointsToCluster function
@return markers to be published
@author Jorge Almeida inspired - createTargetMarkers in mtt
*/

vector<visualization_msgs::Marker> createTargetMarkers( vector<ClusterPtr>& clusters , vector<ClusterPtr>& clusters_Premebida ,  vector<ClusterPtr>& clusters_Dietmayer, vector<ClusterPtr>& clusters_Santos, vector<ClusterPtr>& clusters_ABD ,vector<ClusterPtr>& clusters_nn , vector<ClusterPtr>&  clusters_GT  );

#endif


