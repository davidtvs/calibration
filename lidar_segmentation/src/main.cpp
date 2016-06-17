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
\file  main.cpp
\brief Main node source code, handlers and loop
\author Daniel Coimbra
*/

#include "lidar_segmentation/lidar_segmentation.h"
#include "lidar_segmentation/clustering.h"
#include "lidar_segmentation/groundtruth.h"
#include "lidar_segmentation/visualization_rviz.h"

//Marker's publisher
ros::Publisher markers_pub;
int scan_header;

bool correctClusterId (PointPtr p, int cluster_id)
{
	return (p->cluster_id==cluster_id);
}

void filterPoints(vector<PointPtr>& points_in , vector<PointPtr>& points_out, double min_range, double max_range)
{
	points_out.clear();

	for(uint i=0;i<points_in.size();i++)
	{
		PointPtr p = points_in[i];

		if(p->range < min_range || p->range > max_range || isnan(p->range) || p->cluster_id == 0)
		{
				continue;
		}else
		{
			points_out.push_back(p);
		}
	}

//----------------------------------------------------------------------------
// 	for(uint i=0 ; i<points_out.size();i++)
//   	cout<< "postion" << i <<"xfora: "<<points_out[i]->x<<" yfora: "<<points_out[i]->y<<" lfora: "<<points_out[i]->label <<endl;
//----------------------------------------------------------------------------

	return;
}

void removeInvalidPoints(vector<PointPtr>& laser_points, vector<ClusterPtr> clusters, uint min_points )
{

	for(uint i=0;i<clusters.size();i++)
	{
		if(clusters[i]->support_points.size()<=min_points)
		{
			int cluster_id = clusters[i]->id;

			laser_points.erase(std::remove_if(laser_points.begin(),laser_points.end(), boost::bind(correctClusterId,_1,cluster_id)), laser_points.end());
		}
	}
}


/**
@brief Handler for the incoming data
@param points incoming Laser Points
@param iteration iteration of the Laser Scan
@return void
*/

void dataFromFileHandler(vector<PointPtr>& groundtruth_points , int iteration)
{
	int write_results = 0.0;
	int write_results_no_small_clusters = 0.0;

	// cout << "Scan number: " << iteration << endl;

	vector<PointPtr> groundtruth_points_filtered;

	//Filter the laser points
	filterPoints(groundtruth_points,groundtruth_points_filtered,0.01,50.);

	//Sort them by label
	vector<PointPtr> groundtruth_points_filtered_sorted  =  groundtruth_points_filtered;
	sort(groundtruth_points_filtered_sorted.begin(),groundtruth_points_filtered_sorted.end(),comparePoints);

//-------------------------------------------------------------------------------------------------------------------
	//Make GT clusters

	vector<ClusterPtr> clusters_GT;
	convertPointsToCluster(groundtruth_points_filtered_sorted, clusters_GT);
	//Remove GT clusters with less than a certain size
	uint minimum_points = 3;
	vector<PointPtr> groundtruth_small_points_removed = groundtruth_points_filtered;
	removeInvalidPoints(groundtruth_small_points_removed, clusters_GT, minimum_points);

//-------------------------------------------------------------------------------------------------------------------

//  Wirte Segments centers and boundaries to a text file
	if(write_results >= 1.0)
	{
		writeResults_GT(iteration , groundtruth_points_filtered_sorted , 1);
		writeResults_GT(iteration , groundtruth_points_filtered_sorted , 2);

		writeResults_paths(groundtruth_points_filtered, 1, 0.5 , 0.135, 21 , iteration, 1);
		writeResults_paths(groundtruth_points_filtered, 1, 0.5 , 0.135, 21 , iteration, 2);

		writeResults_paths(groundtruth_points_filtered, 2, 0.55 , 0.021, 21 , iteration, 1);
		writeResults_paths(groundtruth_points_filtered, 2, 0.55 , 0.021, 21 , iteration, 2);

		writeResults_paths(groundtruth_points_filtered, 3, 0.5 , 0.125, 21 , iteration, 1);
		writeResults_paths(groundtruth_points_filtered, 3, 0.5 , 0.125, 21 , iteration, 2);

		writeResults_paths(groundtruth_points_filtered, 4, 4.0 , 0.9, 21 , iteration, 1);
		writeResults_paths(groundtruth_points_filtered, 4, 4.0 , 0.9, 21 , iteration, 2);

		writeResults_paths(groundtruth_points_filtered, 5, 0.5 , 0.125, 21 , iteration, 1);
		writeResults_paths(groundtruth_points_filtered, 5, 0.5 , 0.125, 21 , iteration, 2);

		writeResults_paths(groundtruth_points_filtered, 6, 0.5 , 0.125, 21 , iteration, 1);
		writeResults_paths(groundtruth_points_filtered, 6, 0.5 , 0.125, 21 , iteration, 2);

		writeResults_paths(groundtruth_points_filtered, 7, 5.0 , 1.75, 21 , iteration, 1);
		writeResults_paths(groundtruth_points_filtered, 7, 5.0 , 1.75, 21 , iteration, 2);
	}

// 	-------------------------------------------------------------------------------------------------------------------

	//  Wirte Segments centers and boundaries to a text file - no small clusters
	if( write_results_no_small_clusters   >= 1.0   )
	{

// 		Make GT clusters - sort second sort
		vector<PointPtr> groundtruth_small_points_removed_sorted  =  groundtruth_small_points_removed;
		sort(groundtruth_small_points_removed_sorted.begin(),groundtruth_small_points_removed_sorted.end(),comparePoints);

		writeResults_GT(iteration , groundtruth_small_points_removed_sorted, 1);
		writeResults_GT(iteration , groundtruth_small_points_removed_sorted, 2);

		writeResults_paths(groundtruth_small_points_removed, 1, 0.5 , 0.135, 21 , iteration, 1);
		writeResults_paths(groundtruth_small_points_removed, 1, 0.5 , 0.135, 21 , iteration, 2);

		writeResults_paths(groundtruth_small_points_removed, 2, 0.55 , 0.021, 21 , iteration, 1);
		writeResults_paths(groundtruth_small_points_removed, 2, 0.55 , 0.021, 21 , iteration, 2);

		writeResults_paths(groundtruth_small_points_removed, 3, 0.5 , 0.125, 21 , iteration, 1);
		writeResults_paths(groundtruth_small_points_removed, 3, 0.5 , 0.125, 21 , iteration, 2);

		writeResults_paths(groundtruth_small_points_removed, 4, 4.0 , 0.9, 21 , iteration, 1);
		writeResults_paths(groundtruth_small_points_removed, 4, 4.0 , 0.9, 21 , iteration, 2);

		writeResults_paths(groundtruth_small_points_removed, 5, 0.5 , 0.125, 21 , iteration, 1);
		writeResults_paths(groundtruth_small_points_removed, 5, 0.5 , 0.125, 21 , iteration, 2);

		writeResults_paths(groundtruth_small_points_removed, 6, 0.5 , 0.125, 21 , iteration, 1);
		writeResults_paths(groundtruth_small_points_removed, 6, 0.5 , 0.125, 21 , iteration, 2);

		writeResults_paths(groundtruth_small_points_removed, 7, 5.0 , 1.75, 21 , iteration, 1);
		writeResults_paths(groundtruth_small_points_removed, 7, 5.0 , 1.75, 21 , iteration, 2);
	}

// 	vector<ClusterPtr> clusters_GTs;
// 	convertPointsToCluster(groundtruth_points_filtered , clusters_GTs);

	vector<ClusterPtr> clusters;
 	double threshold = 2;  //[m]
 	simpleClustering(groundtruth_points_filtered ,threshold , clusters);

	vector<ClusterPtr> clusters_Premebida;
	double threshold_cosine = 0.70;
	premebidaClustering(groundtruth_points_filtered ,threshold_cosine , clusters_Premebida);

	vector<ClusterPtr>  clusters_Dietmayer;
	double C0 = 1.5;
	dietmayerClustering(groundtruth_points_filtered , C0, clusters_Dietmayer);

	vector<ClusterPtr> clusters_Santos;
	double Beta = deg2rad(30.0);
	santosClustering(groundtruth_points_filtered, C0 , Beta , clusters_Santos);

	vector<ClusterPtr> clusters_ABD;
	double lambda =  deg2rad(10.0);
	abdClustering(groundtruth_points_filtered ,lambda, clusters_ABD);

	vector<ClusterPtr> clusters_nn;
	double threshold_nn = 1.5;
	nnClustering( groundtruth_points_filtered, threshold_nn , clusters_nn);

// 	Vizualize the Segmentation results
	visualization_msgs::MarkerArray targets_markers;
   	targets_markers.markers = createTargetMarkers( clusters , clusters_Premebida , clusters_Dietmayer, clusters_Santos ,clusters_ABD, clusters_nn, clusters_GT );
   	markers_pub.publish(targets_markers);


	// cout<<"done all"<<endl;

} //end function

int writeResults_GT(uint iteration , vector<PointPtr>& points , int id_result)
{
		vector<ClusterPtr> clusters_c;
		convertPointsToCluster(points, clusters_c);

		if(iteration<414)
		{
			stringstream ss;
			if(id_result == 1)
				ss << "src/NO_boundaries/gt/gt_p1.txt";
			else
				ss << "src/NO_centers/gt/gt_p1.txt";

			ofstream fpc(ss.str().c_str(), ios::app);

			if (!fpc.is_open())
			{
				cout << "Couldn't open fpc file" << endl;
				return 1;
			}

			fpc<<"Inf "<< clusters_c.size() <<" "<< iteration << endl;

			if(id_result == 1)
			{
				for (uint k = 0; k < clusters_c.size(); ++k)
				{
					ClusterPtr cluster = clusters_c[k];
					double final_pos = (cluster->support_points.size() -1);
					fpc<< fixed << setprecision(4) << cluster->support_points[0]->x <<" "<< cluster->support_points[0]->y <<" "<< cluster->support_points[final_pos]->x <<" "<< cluster->support_points[final_pos]->y <<" "<< cluster->support_points.size()<< endl;
				}
			}else
			{
				for (uint k = 0; k < clusters_c.size(); ++k)
				{
					ClusterPtr cluster = clusters_c[k];
					fpc<< fixed << setprecision(4) << cluster->central_point->x <<" "<< cluster->central_point->y  <<" "<< cluster->support_points.size()<< endl;
				}
			}

			fpc.close();

			} //end if path1

			if(iteration >= 414 && iteration <= 521)
			{
				stringstream ss;
				if(id_result == 1)
					ss << "src/NO_boundaries/gt/gt_p2.txt";
				else
					ss << "src/NO_centers/gt/gt_p2.txt";

				ofstream fpc(ss.str().c_str(), ios::app);

				if (!fpc.is_open())
				{
					cout << "Couldn't open fpc file" << endl;
					return 1;
				}

				fpc<<"Inf "<< clusters_c.size() <<" "<< iteration << endl;

				if(id_result == 1)
				{
					for (uint k = 0; k < clusters_c.size(); ++k)
					{
						ClusterPtr cluster = clusters_c[k];
						double final_pos = (cluster->support_points.size() -1);
						fpc<< fixed << setprecision(4) << cluster->support_points[0]->x <<" "<< cluster->support_points[0]->y <<" "<< cluster->support_points[final_pos]->x <<" "<< cluster->support_points[final_pos]->y <<" "<< cluster->support_points.size()<< endl;
					}
				}else
				{
					for (uint k = 0; k < clusters_c.size(); ++k)
					{
						ClusterPtr cluster = clusters_c[k];
						fpc<< fixed << setprecision(4) << cluster->central_point->x <<" "<< cluster->central_point->y  <<" "<< cluster->support_points.size()<< endl;
					}
				}

			fpc.close();

			} //end if path2

			if(iteration >= 776)
			{
				stringstream ss;
				if(id_result == 1)
					ss << "src/NO_boundaries/gt/gt_p3.txt";
				else
					ss << "src/NO_centers/gt/gt_p3.txt";

				ofstream fpc(ss.str().c_str(), ios::app);

				if (!fpc.is_open())
				{
					cout << "Couldn't open fpc file" << endl;
					return 1;
				}

				fpc<<"Inf "<< clusters_c.size() <<" "<< iteration << endl;

				if(id_result == 1)
				{
					for (uint k = 0; k < clusters_c.size(); ++k)
					{
						ClusterPtr cluster = clusters_c[k];
						double final_pos = (cluster->support_points.size() -1);
						fpc<< fixed << setprecision(4) << cluster->support_points[0]->x <<" "<< cluster->support_points[0]->y <<" "<< cluster->support_points[final_pos]->x <<" "<< cluster->support_points[final_pos]->y <<" "<< cluster->support_points.size()<< endl;
					}
				}else
				{
					for (uint k = 0; k < clusters_c.size(); ++k)
					{
						ClusterPtr cluster = clusters_c[k];
						fpc<< fixed << setprecision(4) << cluster->central_point->x <<" "<< cluster->central_point->y  <<" "<< cluster->support_points.size()<< endl;
					}
				}

				fpc.close();

				} //end if path3

	return 0;

} // end function


int writeResults_paths(vector<PointPtr>& points, int algorithm_id, double initial_value, double increment, int number_of_iterations , uint iteration, int id_result)
{
	string path = ros::package::getPath("roslib");
// 	vector<geometry_msgs::Point> xy_data;
// 	convertToCartesian( scan_msg , xy_data ,1);

	switch (algorithm_id) {
		case 1: //SIMPLE_SEG

			for(int i = 0 ; i< number_of_iterations ; i++)
			{
				vector<ClusterPtr> clusters_c;

				double td = initial_value + (i * increment);
				simpleClustering(points, td , clusters_c);

				if(iteration<414)
				{
					stringstream ss;
					if(id_result == 1)
						ss << "src/NO_boundaries/ss/path1/ss_p1_" << i << ".txt";
					else
						ss << "src/NO_centers/ss/path1/ss_p1_" << i << ".txt";

					ofstream fpc(ss.str().c_str(), ios::app);

					if (!fpc.is_open())
					{
						cout << "Couldn't open fpc" << i << " file" << endl;
						return 1;
					}

					fpc<<"Inf "<< clusters_c.size() <<" "<< iteration << endl;

					if(id_result == 1)
					{
						for (uint k = 0; k < clusters_c.size(); ++k)
						{
							ClusterPtr cluster = clusters_c[k];
							double final_pos = (cluster->support_points.size() -1);
							fpc<< fixed << setprecision(4) << cluster->support_points[0]->x <<" "<< cluster->support_points[0]->y <<" "<< cluster->support_points[final_pos]->x <<" "<< cluster->support_points[final_pos]->y <<" "<< cluster->support_points.size()<< endl;
						}
					}else
					{
						for (uint k = 0; k < clusters_c.size(); ++k)
						{
							ClusterPtr cluster = clusters_c[k];
							fpc<< fixed << setprecision(4) << cluster->central_point->x <<" "<< cluster->central_point->y  <<" "<< cluster->support_points.size()<< endl;
						}
					}

					fpc.close();

				} //end if path1

				if(iteration >= 414 && iteration <= 521)
				{
					stringstream ss;
					if(id_result == 1)
						ss << "src/NO_boundaries/ss/path2/ss_p2_" << i << ".txt";
					else
						ss << "src/NO_centers/ss/path2/ss_p2_" << i << ".txt";

					ofstream fpc(ss.str().c_str(), ios::app);

					if (!fpc.is_open())
					{
						cout << "Couldn't open fpc" << i << " file" << endl;
						return 1;
					}

					fpc<<"Inf "<< clusters_c.size() <<" "<< iteration << endl;

					if(id_result == 1)
					{
						for (uint k = 0; k < clusters_c.size(); ++k)
						{
							ClusterPtr cluster = clusters_c[k];
							double final_pos = (cluster->support_points.size() -1);
							fpc<< fixed << setprecision(4) << cluster->support_points[0]->x <<" "<< cluster->support_points[0]->y <<" "<< cluster->support_points[final_pos]->x <<" "<< cluster->support_points[final_pos]->y <<" "<< cluster->support_points.size()<< endl;
						}
					}else
					{
						for (uint k = 0; k < clusters_c.size(); ++k)
						{
							ClusterPtr cluster = clusters_c[k];
							fpc<< fixed << setprecision(4) << cluster->central_point->x <<" "<< cluster->central_point->y  <<" "<< cluster->support_points.size()<< endl;
						}
					}

					fpc.close();

				} //end if path2

				if(iteration >= 776)
				{
					stringstream ss;
					if(id_result == 1)
						ss << "src/NO_boundaries/ss/path3/ss_p3_" << i << ".txt";
					else
						ss << "src/NO_centers/ss/path3/ss_p3_" << i << ".txt";

					ofstream fpc(ss.str().c_str(), ios::app);

					if (!fpc.is_open())
					{
					cout << "Couldn't open fpc" << i << " file" << endl;
					return 1;
					}

					fpc<<"Inf "<< clusters_c.size() <<" "<< iteration << endl;

					if(id_result == 1)
					{
						for (uint k = 0; k < clusters_c.size(); ++k)
						{
							ClusterPtr cluster = clusters_c[k];
							double final_pos = (cluster->support_points.size() -1);
							fpc<< fixed << setprecision(4) << cluster->support_points[0]->x <<" "<< cluster->support_points[0]->y <<" "<< cluster->support_points[final_pos]->x <<" "<< cluster->support_points[final_pos]->y <<" "<< cluster->support_points.size()<< endl;
						}
					}else
					{
						for (uint k = 0; k < clusters_c.size(); ++k)
						{
							ClusterPtr cluster = clusters_c[k];
							fpc<< fixed << setprecision(4) << cluster->central_point->x <<" "<< cluster->central_point->y  <<" "<< cluster->support_points.size()<< endl;
						}
					}

					fpc.close();

				} //end if path3

			} //end for

		break; // end SIMPLE_SEG

		case 2: //PREM_SEG

			for(int i = 0 ; i< number_of_iterations ; i++)
			{
				vector<ClusterPtr> clusters_c;
				double td = initial_value + (i * increment);
				premebidaClustering(points, td, clusters_c );

				if(iteration<414)
				{
					stringstream ss;
					if(id_result == 1)
						ss << "src/NO_boundaries/ms_p1_" << i << ".txt";
					else
						ss << "src/NO_centers/ms_p1_" << i << ".txt";

					ofstream fpc(ss.str().c_str(), ios::app);

					if (!fpc.is_open())
					{
						cout << "Couldn't open fpc" << i << " file" << endl;
						return 1;
					}

					fpc<<"Inf "<< clusters_c.size() <<" "<< iteration << endl;

					if(id_result == 1)
					{
					for (uint k = 0; k < clusters_c.size(); ++k)
					{
						ClusterPtr cluster = clusters_c[k];
						double final_pos = (cluster->support_points.size() -1);
						fpc<< fixed << setprecision(4) << cluster->support_points[0]->x <<" "<< cluster->support_points[0]->y <<" "<< cluster->support_points[final_pos]->x <<" "<< cluster->support_points[final_pos]->y <<" "<< cluster->support_points.size()<< endl;
					}
					}else
					{
						for (uint k = 0; k < clusters_c.size(); ++k)
						{
							ClusterPtr cluster = clusters_c[k];
							fpc<< fixed << setprecision(4) << cluster->central_point->x <<" "<< cluster->central_point->y  <<" "<< cluster->support_points.size()<< endl;
						}
					}

					fpc.close();

				} //end if path1

				if(iteration >= 414 && iteration <= 521)
				{
					stringstream ss;
					if(id_result == 1)
						ss << "src/NO_boundaries/ms_p2_" << i << ".txt";
					else
						ss << "src/NO_centers/ms_p2_" << i << ".txt";

					ofstream fpc(ss.str().c_str(), ios::app);

					if (!fpc.is_open())
					{
						cout << "Couldn't open fpc" << i << " file" << endl;
						return 1;
					}

					fpc<<"Inf "<< clusters_c.size() <<" "<< iteration << endl;

					if(id_result == 1)
					{
						for (uint k = 0; k < clusters_c.size(); ++k)
						{
							ClusterPtr cluster = clusters_c[k];
							double final_pos = (cluster->support_points.size() -1);
							fpc<< fixed << setprecision(4) << cluster->support_points[0]->x <<" "<< cluster->support_points[0]->y <<" "<< cluster->support_points[final_pos]->x <<" "<< cluster->support_points[final_pos]->y <<" "<< cluster->support_points.size()<< endl;
						}
					}else
					{
						for (uint k = 0; k < clusters_c.size(); ++k)
						{
							ClusterPtr cluster = clusters_c[k];
							fpc<< fixed << setprecision(4) << cluster->central_point->x <<" "<< cluster->central_point->y  <<" "<< cluster->support_points.size()<< endl;
						}
					}

					fpc.close();

				} //end if path2

				if(iteration >= 776)
				{
					stringstream ss;
					if(id_result == 1)
						ss << "src/NO_boundaries/ms_p3_" << i << ".txt";
					else
						ss << "src/NO_centers/ms_p3_" << i << ".txt";

					ofstream fpc(ss.str().c_str(), ios::app);

					if (!fpc.is_open())
					{
						cout << "Couldn't open fpc" << i << " file" << endl;
						return 1;
					}

					fpc<<"Inf "<< clusters_c.size() <<" "<< iteration << endl;

					if(id_result == 1)
					{
						for (uint k = 0; k < clusters_c.size(); ++k)
						{
							ClusterPtr cluster = clusters_c[k];
							double final_pos = (cluster->support_points.size() -1);
							fpc<< fixed << setprecision(4) << cluster->support_points[0]->x <<" "<< cluster->support_points[0]->y <<" "<< cluster->support_points[final_pos]->x <<" "<< cluster->support_points[final_pos]->y <<" "<< cluster->support_points.size()<< endl;
						}
					}else
					{
						for (uint k = 0; k < clusters_c.size(); ++k)
						{
							ClusterPtr cluster = clusters_c[k];
							fpc<< fixed << setprecision(4) << cluster->central_point->x <<" "<< cluster->central_point->y  <<" "<< cluster->support_points.size()<< endl;
						}
					}

					fpc.close();

				} //end if path3

			} //end for

		break;	// end PREM_SEG

		case 3:  //DIET_SEG

			for(int i = 0 ; i< number_of_iterations ; i++)
			{
				vector<ClusterPtr> clusters_c;

				double td = initial_value + (i * increment);
				dietmayerClustering( points, td , clusters_c );

				if(iteration<414)
				{
					stringstream ss;
					if(id_result == 1)
						ss << "src/ultimate_boundaries/ds/path1/ds_p1_" << i << ".txt";
					else
						ss << "src/ultimate_centers/ds/path1/ds_p1_" << i << ".txt";

					ofstream fpc(ss.str().c_str(), ios::app);

					if (!fpc.is_open())
					{
						cout << "Couldn't open fpc" << i << " file" << endl;
						return 1;
					}

					fpc<<"Inf "<< clusters_c.size() <<" "<< iteration << endl;

					if(id_result == 1)
					{
						for (uint k = 0; k < clusters_c.size(); ++k)
						{
							ClusterPtr cluster = clusters_c[k];
							double final_pos = (cluster->support_points.size() -1);
							fpc<< fixed << setprecision(4) << cluster->support_points[0]->x <<" "<< cluster->support_points[0]->y <<" "<< cluster->support_points[final_pos]->x <<" "<< cluster->support_points[final_pos]->y <<" "<< cluster->support_points.size()<< endl;
						}
					}else
					{
						for (uint k = 0; k < clusters_c.size(); ++k)
						{
							ClusterPtr cluster = clusters_c[k];
							fpc<< fixed << setprecision(4) << cluster->central_point->x <<" "<< cluster->central_point->y  <<" "<< cluster->support_points.size()<< endl;
						}
					}

					fpc.close();

				} //end if path1

				if(iteration >= 414 && iteration <= 521)
				{
					stringstream ss;
					if(id_result == 1)
						ss << "src/ultimate_boundaries/ds/path2/ds_p2_" << i << ".txt";
					else
						ss << "src/ultimate_centers/ds/path2/ds_p2_" << i << ".txt";

					ofstream fpc(ss.str().c_str(), ios::app);

					if (!fpc.is_open())
					{
						cout << "Couldn't open fpc" << i << " file" << endl;
						return 1;
					}

					fpc<<"Inf "<< clusters_c.size() <<" "<< iteration << endl;

					if(id_result == 1)
					{
						for (uint k = 0; k < clusters_c.size(); ++k)
						{
							ClusterPtr cluster = clusters_c[k];
							double final_pos = (cluster->support_points.size() -1);
							fpc<< fixed << setprecision(4) << cluster->support_points[0]->x <<" "<< cluster->support_points[0]->y <<" "<< cluster->support_points[final_pos]->x <<" "<< cluster->support_points[final_pos]->y <<" "<< cluster->support_points.size()<< endl;
						}
					}else
					{
						for (uint k = 0; k < clusters_c.size(); ++k)
						{
							ClusterPtr cluster = clusters_c[k];
							fpc<< fixed << setprecision(4) << cluster->central_point->x <<" "<< cluster->central_point->y  <<" "<< cluster->support_points.size()<< endl;
						}
					}

					fpc.close();

				} //end if path2

				if(iteration >= 776)
				{
					stringstream ss;
					if(id_result == 1)
						ss << "src/ultimate_boundaries/ds/path3/ds_p3_" << i << ".txt";
					else
						ss << "src/ultimate_centers/ds/path3/ds_p3_" << i << ".txt";

					ofstream fpc(ss.str().c_str(), ios::app);

					if (!fpc.is_open())
					{
						cout << "Couldn't open fpc" << i << " file" << endl;
						return 1;
					}

					fpc<<"Inf "<< clusters_c.size() <<" "<< iteration << endl;

					if(id_result == 1)
					{
						for (uint k = 0; k < clusters_c.size(); ++k)
						{
							ClusterPtr cluster = clusters_c[k];
							double final_pos = (cluster->support_points.size() -1);
							fpc<< fixed << setprecision(4) << cluster->support_points[0]->x <<" "<< cluster->support_points[0]->y <<" "<< cluster->support_points[final_pos]->x <<" "<< cluster->support_points[final_pos]->y <<" "<< cluster->support_points.size()<< endl;
						}
					}else
					{
						for (uint k = 0; k < clusters_c.size(); ++k)
						{
							ClusterPtr cluster = clusters_c[k];
							fpc<< fixed << setprecision(4) << cluster->central_point->x <<" "<< cluster->central_point->y  <<" "<< cluster->support_points.size()<< endl;
						}
					}

					fpc.close();

				} //end if path3

			} //end for

		break;	// end DIET_SEG

		case 4:	//ABD_SEG

			for(int i = 0 ; i< number_of_iterations ; i++)
			{
				vector<ClusterPtr> clusters_c;
				double td = deg2rad( initial_value + (i * increment) );
				abdClustering( points, td, clusters_c );

				if(iteration<414)
				{
					stringstream ss;
					if(id_result == 1)
						ss << "src/NO_boundaries/abd/path1/abd_p1_" << i << ".txt";
					else
						ss << "src/NO_centers/abd/path1/abd_p1_" << i << ".txt";

					ofstream fpc(ss.str().c_str(), ios::app);

					if (!fpc.is_open())
					{
						cout << "Couldn't open fpc" << i << " file" << endl;
						return 1;
					}

					fpc<<"Inf "<< clusters_c.size() <<" "<< iteration << endl;

					if(id_result == 1)
					{
						for (uint k = 0; k < clusters_c.size(); ++k)
						{
							ClusterPtr cluster = clusters_c[k];
							double final_pos = (cluster->support_points.size() -1);
							fpc<< fixed << setprecision(4) << cluster->support_points[0]->x <<" "<< cluster->support_points[0]->y <<" "<< cluster->support_points[final_pos]->x <<" "<< cluster->support_points[final_pos]->y <<" "<< cluster->support_points.size()<< endl;
						}
					}else
					{
						for (uint k = 0; k < clusters_c.size(); ++k)
						{
							ClusterPtr cluster = clusters_c[k];
							fpc<< fixed << setprecision(4) << cluster->central_point->x <<" "<< cluster->central_point->y  <<" "<< cluster->support_points.size()<< endl;
						}
					}

					fpc.close();

				} //end if path1

				if(iteration >= 414 && iteration <= 521)
				{
					stringstream ss;
					if(id_result == 1)
						ss << "src/NO_boundaries/abd/path2/abd_p2_" << i << ".txt";
					else
						ss << "src/NO_centers/abd/path2/abd_p2_" << i << ".txt";

					ofstream fpc(ss.str().c_str(), ios::app);

					if (!fpc.is_open())
					{
						cout << "Couldn't open fpc" << i << " file" << endl;
						return 1;
					}

					fpc<<"Inf "<< clusters_c.size() <<" "<< iteration << endl;

					if(id_result == 1)
					{
						for (uint k = 0; k < clusters_c.size(); ++k)
						{
							ClusterPtr cluster = clusters_c[k];
							double final_pos = (cluster->support_points.size() -1);
							fpc<< fixed << setprecision(4) << cluster->support_points[0]->x <<" "<< cluster->support_points[0]->y <<" "<< cluster->support_points[final_pos]->x <<" "<< cluster->support_points[final_pos]->y <<" "<< cluster->support_points.size()<< endl;
						}
					}else
					{
						for (uint k = 0; k < clusters_c.size(); ++k)
						{
							ClusterPtr cluster = clusters_c[k];
							fpc<< fixed << setprecision(4) << cluster->central_point->x <<" "<< cluster->central_point->y  <<" "<< cluster->support_points.size()<< endl;
						}
					}

					fpc.close();

				} //end if path2

				if(iteration >= 776)
				{
					stringstream ss;
					if(id_result == 1)
						ss << "src/NO_boundaries/abd/path3/abd_p3_" << i << ".txt";
					else
						ss << "src/NO_centers/abd/path3/abd_p3_" << i << ".txt";

					ofstream fpc(ss.str().c_str(), ios::app);

					if (!fpc.is_open())
					{
					cout << "Couldn't open fpc" << i << " file" << endl;
					return 1;
					}

					fpc<<"Inf "<< clusters_c.size() <<" "<< iteration << endl;

					if(id_result == 1)
					{
						for (uint k = 0; k < clusters_c.size(); ++k)
						{
							ClusterPtr cluster = clusters_c[k];
							double final_pos = (cluster->support_points.size() -1);
							fpc<< fixed << setprecision(4) << cluster->support_points[0]->x <<" "<< cluster->support_points[0]->y <<" "<< cluster->support_points[final_pos]->x <<" "<< cluster->support_points[final_pos]->y <<" "<< cluster->support_points.size()<< endl;
						}
					}else
					{
						for (uint k = 0; k < clusters_c.size(); ++k)
						{
							ClusterPtr cluster = clusters_c[k];
							fpc<< fixed << setprecision(4) << cluster->central_point->x <<" "<< cluster->central_point->y  <<" "<< cluster->support_points.size()<< endl;
						}
					}

					fpc.close();

				} //end if path3

			} //end for

		break; //end ABD_SEG

		case 5: //NN_SEG

			for(int i = 0 ; i< number_of_iterations ; i++)
			{
				vector<ClusterPtr> clusters_c;
				double td = initial_value + (i * increment);
				nnClustering(points , td, clusters_c );

				if(iteration<414)
				{
					stringstream ss;
					if(id_result == 1)
						ss << "src/ultimate_boundaries/snn/path1/snn_p1_" << i << ".txt";
					else
						ss << "src/ultimate_centers/snn/path1/snn_p1_" << i << ".txt";

					ofstream fpc(ss.str().c_str(), ios::app);

					if (!fpc.is_open())
					{
						cout << "Couldn't open fpc" << i << " file" << endl;
						return 1;
					}

					fpc<<"Inf "<< clusters_c.size() <<" "<< iteration << endl;

					if(id_result == 1)
					{
						for (uint k = 0; k < clusters_c.size(); ++k)
						{
							ClusterPtr cluster = clusters_c[k];
							double final_pos = (cluster->support_points.size() -1);
							fpc<< fixed << setprecision(4) << cluster->support_points[0]->x <<" "<< cluster->support_points[0]->y <<" "<< cluster->support_points[final_pos]->x <<" "<< cluster->support_points[final_pos]->y <<" "<< cluster->support_points.size()<< endl;
						}
					}else
					{
						for (uint k = 0; k < clusters_c.size(); ++k)
						{
							ClusterPtr cluster = clusters_c[k];
							fpc<< fixed << setprecision(4) << cluster->central_point->x <<" "<< cluster->central_point->y  <<" "<< cluster->support_points.size()<< endl;
						}
					}

					fpc.close();

				} //end if path1

				if(iteration >= 414 && iteration <= 521)
				{
					stringstream ss;
					if(id_result == 1)
						ss << "src/ultimate_boundaries/snn/path2/snn_p2_" << i << ".txt";
					else
						ss << "src/ultimate_centers/snn/path2/snn_p2_" << i << ".txt";

					ofstream fpc(ss.str().c_str(), ios::app);

					if (!fpc.is_open())
					{
						cout << "Couldn't open fpc" << i << " file" << endl;
						return 1;
					}

					fpc<<"Inf "<< clusters_c.size() <<" "<< iteration << endl;

					if(id_result == 1)
					{
						for (uint k = 0; k < clusters_c.size(); ++k)
						{
							ClusterPtr cluster = clusters_c[k];
							double final_pos = (cluster->support_points.size() -1);
							fpc<< fixed << setprecision(4) << cluster->support_points[0]->x <<" "<< cluster->support_points[0]->y <<" "<< cluster->support_points[final_pos]->x <<" "<< cluster->support_points[final_pos]->y <<" "<< cluster->support_points.size()<< endl;
						}
					}else
					{
						for (uint k = 0; k < clusters_c.size(); ++k)
						{
							ClusterPtr cluster = clusters_c[k];
							fpc<< fixed << setprecision(4) << cluster->central_point->x <<" "<< cluster->central_point->y  <<" "<< cluster->support_points.size()<< endl;
						}
					}

					fpc.close();

				} //end if path2

				if(iteration >= 776)
				{
					stringstream ss;
					if(id_result == 1)
						ss << "src/ultimate_boundaries/snn/path3/snn_p3_" << i << ".txt";
					else
						ss << "src/ultimate_centers/snn/path3/snn_p3_" << i << ".txt";

					ofstream fpc(ss.str().c_str(), ios::app);

					if (!fpc.is_open())
					{
						cout << "Couldn't open fpc" << i << " file" << endl;
						return 1;
					}

					fpc<<"Inf "<< clusters_c.size() <<" "<< iteration << endl;

					if(id_result == 1)
					{
						for (uint k = 0; k < clusters_c.size(); ++k)
						{
							ClusterPtr cluster = clusters_c[k];
							double final_pos = (cluster->support_points.size() -1);
							fpc<< fixed << setprecision(4) << cluster->support_points[0]->x <<" "<< cluster->support_points[0]->y <<" "<< cluster->support_points[final_pos]->x <<" "<< cluster->support_points[final_pos]->y <<" "<< cluster->support_points.size()<< endl;
						}
					}else
					{
						for (uint k = 0; k < clusters_c.size(); ++k)
						{
							ClusterPtr cluster = clusters_c[k];
							fpc<< fixed << setprecision(4) << cluster->central_point->x <<" "<< cluster->central_point->y  <<" "<< cluster->support_points.size()<< endl;
						}
					}

					fpc.close();

				} //end if path3

			} //end for

		break; //end NN_SEG

		case 6:	 //SANTOS_C_SEG

			for(int i = 0 ; i< number_of_iterations ; i++)
			{
				vector<ClusterPtr> clusters_c;
				double beta = deg2rad(15.0);
				double td = initial_value + (i * increment);

				santosClustering( points , td , beta , clusters_c );

				if(iteration<414)
				{
					stringstream ss;
					if(id_result == 1)
						ss << "src/ultimate_boundaries/sa_c/path1/sa_c_p1_" << i << ".txt";
					else
						ss << "src/ultimate_centers/sa_c/path1/sa_c_p1_" << i << ".txt";

					ofstream fpc(ss.str().c_str(), ios::app);

					if (!fpc.is_open())
					{
						cout << "Couldn't open fpc" << i << " file" << endl;
						return 1;
					}

					fpc<<"Inf "<< clusters_c.size() <<" "<< iteration << endl;

					if(id_result == 1)
					{
						for (uint k = 0; k < clusters_c.size(); ++k)
						{
							ClusterPtr cluster = clusters_c[k];
							double final_pos = (cluster->support_points.size() -1);
							fpc<< fixed << setprecision(4) << cluster->support_points[0]->x <<" "<< cluster->support_points[0]->y <<" "<< cluster->support_points[final_pos]->x <<" "<< cluster->support_points[final_pos]->y <<" "<< cluster->support_points.size()<< endl;
						}
					}else
					{
						for (uint k = 0; k < clusters_c.size(); ++k)
						{
							ClusterPtr cluster = clusters_c[k];
							fpc<< fixed << setprecision(4) << cluster->central_point->x <<" "<< cluster->central_point->y  <<" "<< cluster->support_points.size()<< endl;
						}
					}

					fpc.close();

				} //end if path1

				if(iteration >= 414 && iteration <= 521)
				{
					stringstream ss;
					if(id_result == 1)
						ss << "src/ultimate_boundaries/sa_c/path2/sa_c_p2_" << i << ".txt";
					else
						ss << "src/ultimate_centers/sa_c/path2/sa_c_p2_" << i << ".txt";

					ofstream fpc(ss.str().c_str(), ios::app);

					if (!fpc.is_open())
					{
						cout << "Couldn't open fpc" << i << " file" << endl;
						return 1;
					}

					fpc<<"Inf "<< clusters_c.size() <<" "<< iteration << endl;

					if(id_result == 1)
					{
						for (uint k = 0; k < clusters_c.size(); ++k)
						{
							ClusterPtr cluster = clusters_c[k];
							double final_pos = (cluster->support_points.size() -1);
							fpc<< fixed << setprecision(4) << cluster->support_points[0]->x <<" "<< cluster->support_points[0]->y <<" "<< cluster->support_points[final_pos]->x <<" "<< cluster->support_points[final_pos]->y <<" "<< cluster->support_points.size()<< endl;
						}
					}else
					{
						for (uint k = 0; k < clusters_c.size(); ++k)
						{
							ClusterPtr cluster = clusters_c[k];
							fpc<< fixed << setprecision(4) << cluster->central_point->x <<" "<< cluster->central_point->y  <<" "<< cluster->support_points.size()<< endl;
						}
					}

					fpc.close();

				} //end if path2

				if(iteration >= 776)
				{
					stringstream ss;
					if(id_result == 1)
						ss << "src/ultimate_boundaries/sa_c/path3/sa_c_p3_" << i << ".txt";
					else
						ss << "src/ultimate_centers/sa_c/path3/sa_c_p3_" << i << ".txt";

					ofstream fpc(ss.str().c_str(), ios::app);

					if (!fpc.is_open())
					{
						cout << "Couldn't open fpc" << i << " file" << endl;
						return 1;
					}

					fpc<<"Inf "<< clusters_c.size() <<" "<< iteration << endl;

					if(id_result == 1)
					{
						for (uint k = 0; k < clusters_c.size(); ++k)
						{
							ClusterPtr cluster = clusters_c[k];
							double final_pos = (cluster->support_points.size() -1);
							fpc<< fixed << setprecision(4) << cluster->support_points[0]->x <<" "<< cluster->support_points[0]->y <<" "<< cluster->support_points[final_pos]->x <<" "<< cluster->support_points[final_pos]->y <<" "<< cluster->support_points.size()<< endl;
						}
					}else
					{
						for (uint k = 0; k < clusters_c.size(); ++k)
						{
							ClusterPtr cluster = clusters_c[k];
							fpc<< fixed << setprecision(4) << cluster->central_point->x <<" "<< cluster->central_point->y  <<" "<< cluster->support_points.size()<< endl;
						}
					}

					fpc.close();

				} //end if path3

			} //end for

			break; //end SANTOS_C_SEG

		case 7: //SANTOS_B_SEG

			for(int i = 0 ; i< number_of_iterations ; i++)
			{
				vector<ClusterPtr> clusters_c;
				double C0 = 1.0;
				double td = deg2rad( initial_value + (i * increment) );
				santosClustering(points , C0, td , clusters_c );

				if(iteration<414)
				{
					stringstream ss;
					if(id_result == 1)
						ss << "src/NO_boundaries/sa_b/path1/sa_b_p1_" << i << ".txt";
					else
						ss << "src/NO_centers/sa_b/path1/sa_b_p1_" << i << ".txt";

					ofstream fpc(ss.str().c_str(), ios::app);

					if (!fpc.is_open())
					{
						cout << "Couldn't open fpc" << i << " file" << endl;
						return 1;
					}

					fpc<<"Inf "<< clusters_c.size() <<" "<< iteration << endl;

					if(id_result == 1)
					{
						for (uint k = 0; k < clusters_c.size(); ++k)
						{
							ClusterPtr cluster = clusters_c[k];
							double final_pos = (cluster->support_points.size() -1);
							fpc<< fixed << setprecision(4) << cluster->support_points[0]->x <<" "<< cluster->support_points[0]->y <<" "<< cluster->support_points[final_pos]->x <<" "<< cluster->support_points[final_pos]->y <<" "<< cluster->support_points.size()<< endl;
						}
					}else
					{
						for (uint k = 0; k < clusters_c.size(); ++k)
						{
							ClusterPtr cluster = clusters_c[k];
							fpc<< fixed << setprecision(4) << cluster->central_point->x <<" "<< cluster->central_point->y  <<" "<< cluster->support_points.size()<< endl;
						}
					}

					fpc.close();

				} //end if path1

				if(iteration >= 414 && iteration <= 521)
				{
					stringstream ss;
					if(id_result == 1)
						ss << "src/NO_boundaries/sa_b/path2/sa_b_p2_" << i << ".txt";
					else
						ss << "src/NO_centers/sa_b/path2/sa_b_p2_" << i << ".txt";

					ofstream fpc(ss.str().c_str(), ios::app);

					if (!fpc.is_open())
					{
						cout << "Couldn't open fpc" << i << " file" << endl;
						return 1;
					}

					fpc<<"Inf "<< clusters_c.size() <<" "<< iteration << endl;

					if(id_result == 1)
					{
						for (uint k = 0; k < clusters_c.size(); ++k)
						{
							ClusterPtr cluster = clusters_c[k];
							double final_pos = (cluster->support_points.size() -1);
							fpc<< fixed << setprecision(4) << cluster->support_points[0]->x <<" "<< cluster->support_points[0]->y <<" "<< cluster->support_points[final_pos]->x <<" "<< cluster->support_points[final_pos]->y <<" "<< cluster->support_points.size()<< endl;
						}
					}else
					{
						for (uint k = 0; k < clusters_c.size(); ++k)
						{
							ClusterPtr cluster = clusters_c[k];
							fpc<< fixed << setprecision(4) << cluster->central_point->x <<" "<< cluster->central_point->y  <<" "<< cluster->support_points.size()<< endl;
						}
					}

					fpc.close();

				} //end if path2

				if(iteration >= 776)
				{
					stringstream ss;
					if(id_result == 1)
						ss << "src/NO_boundaries/sa_b/path3/sa_b_p3_" << i << ".txt";
					else
						ss << "src/NO_centers/sa_b/path3/sa_b_p3_" << i << ".txt";

					ofstream fpc(ss.str().c_str(), ios::app);

					if (!fpc.is_open())
					{
						cout << "Couldn't open fpc" << i << " file" << endl;
						return 1;
					}

					fpc<<"Inf "<< clusters_c.size() <<" "<< iteration << endl;

					if(id_result == 1)
					{
						for (uint k = 0; k < clusters_c.size(); ++k)
						{
							ClusterPtr cluster = clusters_c[k];
							double final_pos = (cluster->support_points.size() -1);
							fpc<< fixed << setprecision(4) << cluster->support_points[0]->x <<" "<< cluster->support_points[0]->y <<" "<< cluster->support_points[final_pos]->x <<" "<< cluster->support_points[final_pos]->y <<" "<< cluster->support_points.size()<< endl;
						}
					}else
					{
						for (uint k = 0; k < clusters_c.size(); ++k)
						{
							ClusterPtr cluster = clusters_c[k];
							fpc<< fixed << setprecision(4) << cluster->central_point->x <<" "<< cluster->central_point->y  <<" "<< cluster->support_points.size()<< endl;
						}
					}

					fpc.close();

				} //end if path3

			} //end for

			break; //end SANTOS_B_SEG


	} //end switch

	return 0;

}


int createPointsFromFile( vector<PointPtr>& points , C_DataFromFilePtr data_gt )
{

	double theta;
	double r;

	for(uint j = 0; j < data_gt->x_valuesf.size() ; j++ )
	{
		theta = atan2(data_gt->y_valuesf[j],data_gt->x_valuesf[j]);
		r = sqrt(data_gt->y_valuesf[j]*data_gt->y_valuesf[j] + data_gt->x_valuesf[j]*data_gt->x_valuesf[j]);

		PointPtr p(new Point);

		p->x=data_gt->x_valuesf[j];
		p->y=data_gt->y_valuesf[j];
		p->z=data_gt->z_valuesf[j];
		p->theta=theta;
		p->range=r;
		p->label = j;
		p->iteration = j+1;
		p->cluster_id = data_gt->labels[j];

		points.push_back(p);
	}

	return 0;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "lidar_segmentation");
	ros::NodeHandle n;

	markers_pub = n.advertise<visualization_msgs::MarkerArray>( "/markers", 1000 );

	//Read the groud truth file
 	int values_per_scan = 541;
 	vector<C_DataFromFilePtr>  data_gts;

  	readDataFile( data_gts ,values_per_scan);
	ros::Rate loop_rate(1);


// 	for( uint it = 0; it < data_gts.size() ; it++ )
 	for(uint it = 0; it < data_gts.size() ; it++)
	{
		vector<PointPtr> points;

 		createPointsFromFile(points, data_gts[it] );
		scan_header	= data_gts[it]->iteration;

		dataFromFileHandler(points , scan_header);

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
