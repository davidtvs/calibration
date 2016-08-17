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
\file  groundtruth.cpp
\brief Groundtruth related functions.
\author Daniel Coimbra
*/

#include "lidar_segmentation/lidar_segmentation.h"
#include "lidar_segmentation/clustering.h"
#include "lidar_segmentation/groundtruth.h"

//read from the ground truth file
int readDataFile( vector<C_DataFromFilePtr>&  data_gts , int values_per_scan)
{

	string path = ros::package::getPath("roslib");
	ifstream source("src/gt_datas/GT_NEW_DIV.txt");  // build a read-Stream

	if (!source.is_open())
    {
		cout << "Couldn't open the GT_NEW file" << endl;
		return 1;
    }

	string line;

	while(source.good())    //read stream line by line
	{
		getline(source, line);      //Get iteration

		if(line[0]!='i')
			continue;

		int it;

		//New iteration
		sscanf(line.c_str(),"%*c %d",&it);

		C_DataFromFilePtr data_gt(new C_DataFromFile);

		data_gt->iteration = it;

		getline(source ,line); //get x values

		int c = 0;
		double x , y;
		int label;

		const char*ptr = line.c_str();

		while(c < values_per_scan)
		{
			sscanf(ptr,"%lf", &x); //scans values

			data_gt->x_valuesf.push_back(x);

			c++;

			ptr = strstr(ptr, " "); //seeks for " "

			if(ptr==NULL)
				break;

			ptr++;
		}

//----------------------------------------------------------------------------
// 		for(uint j=0;j< data_gt->x_valuesf.size();j++)
// 		{
// 			cout<<"x_valuesf: "<< data_gt->x_valuesf[j]<<endl;
// 		}
//----------------------------------------------------------------------------

		getline(source , line); //get y values
		const char*ptr2 = line.c_str();
		c = 0;

		while(c < values_per_scan)
		{
			sscanf(ptr2, "%lf" , &y);

			data_gt->y_valuesf.push_back(y);
			c++;

			ptr2 = strstr(ptr2, " "); //seeks for " "

			if(ptr2==NULL)
				break;

			ptr2++;
		}

//----------------------------------------------------------------------------
// 		for(uint j=0;j< data_gt->x_valuesf.size();j++)
// 		{
// 			cout<<"y_valuesf: "<< data_gt->y_valuesf[j]<<endl;
//  	}
//----------------------------------------------------------------------------

		getline(source , line); //labels
		const char*ptr3 = line.c_str();
		c = 0;

		while(c < values_per_scan)
		{
			sscanf(ptr3, "%d" , &label);

			data_gt->labels.push_back(label);
			c++;

			ptr3 = strstr(ptr3, " "); //seeks for " "

			if(ptr3==NULL)
				break;

			ptr3++;
		}

//----------------------------------------------------------------------------
// 		for(uint j=0; j< data_gt->x_valuesf.size(); j++)
// 		{
// 			cout<<"labels : "<< data_gt->labels[j] <<endl;
// 		}
//----------------------------------------------------------------------------

		data_gts.push_back(data_gt);

	} //end while

		cout<<"close file"<<endl;
		source.close();

	return 0;

} //end function


bool comparePoints(PointPtr p1,PointPtr p2)
{
	return p1->cluster_id < p2->cluster_id;  //ascending order
}

int convertPointsToCluster(vector<PointPtr>& points, vector<ClusterPtr>& clusters_GT)
{

 	ClusterPtr cluster_gt(new Cluster);

	//add the first value
	if( points[0]->label != 0 )
	{
		PointPtr p(new Point);
		p->x = points[0]->x;
		p->y = points[0]->y;
		p->label = points[0]->label;
		p->cluster_id = points[0]->cluster_id;

		cluster_gt->support_points.push_back(p);
		cluster_gt->centroid = calculateClusterCentroid( cluster_gt->support_points );
		cluster_gt->central_point = calculateClusterMedian(cluster_gt->support_points);
		cluster_gt->id = p->cluster_id;
	}


	//For every line in the file
	for(uint idx = 1; idx < points.size(); idx++  )
	{
			if(points[idx]->cluster_id== 0)
				continue;

			if( (points[idx]->cluster_id != points[idx-1]->cluster_id))
			{

				if(cluster_gt->support_points.size()>0)
					clusters_GT.push_back(cluster_gt);

				//make cluster point to a new Cluster.
				cluster_gt.reset(new Cluster);

				PointPtr p(new Point);
				p->x = points[idx]->x;
				p->y = points[idx]->y;
				p->label = points[idx]->label;
				p->cluster_id = points[idx]->cluster_id;

				cluster_gt->support_points.push_back(p);
				cluster_gt->centroid = calculateClusterCentroid( cluster_gt->support_points );
				cluster_gt->central_point = calculateClusterMedian(cluster_gt->support_points);
				cluster_gt->id = p->cluster_id;

			}else
			{
				PointPtr p(new Point);
				p->x = points[idx]->x;
				p->y = points[idx]->y;
				p->label = points[idx]->label;
				p->cluster_id = points[idx]->cluster_id;

				cluster_gt->support_points.push_back(p);

				cluster_gt->centroid = calculateClusterCentroid( cluster_gt->support_points );
				cluster_gt->central_point = calculateClusterMedian(cluster_gt->support_points);
				cluster_gt->id = p->cluster_id;
			}

	} //end for

 	if(cluster_gt->support_points.size()>0)
 		clusters_GT.push_back(cluster_gt);

	//cout<< "Clusters GT " << clusters_GT.size() << endl;

	return clusters_GT.size();
}
