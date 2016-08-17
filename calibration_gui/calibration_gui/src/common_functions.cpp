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
 \file  common_functions.cpp
 \brief Functions required by the SICK LMS and SICK LD-MRS algorithms for the ball center detection
 \author Marcelo Pereira
 \date   December, 2015
*/

#include <lidar_segmentation/lidar_segmentation.h>
#include "calibration_gui/sick_ldmrs.h"
#include <lidar_segmentation/clustering.h>
#include <lidar_segmentation/groundtruth.h>
#include <cmath>
#include <algorithm>
#include <sensor_msgs/LaserScan.h>
#include <numeric>
#include <complex>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>


/**
@brief Calculation of the circle properties
@param[in] cluster corresponding cluster of the detected circle
@param[out] R radius of the circle
@param[out] Center coordinates of the circle center
@return void
*/
void CalculateCircle(ClusterPtr cluster, double& R, Point& Center)
{
    double Sx=0,Sy=0,x_m,y_m,u,v,Suu,Svv,Suv,Suuu,Svvv,Suvv,Svuu,uc,vc;
    for(int i=0;i<cluster->support_points.size();i++)
    {
        Sx+=cluster->support_points[i]->x;
        Sy+=cluster->support_points[i]->y;
    }
    x_m=Sx/cluster->support_points.size();
    y_m=Sy/cluster->support_points.size();

    for(int i=0;i<cluster->support_points.size();i++)
    {
        u=cluster->support_points[i]->x-x_m;
        v=cluster->support_points[i]->y-y_m;
        Suu+=pow(u,2);
        Svv+=pow(v,2);
        Suv+=u*v;
        Suuu+=pow(u,3);
        Svvv+=pow(v,3);
        Svuu+=v*pow(u,2);
        Suvv+=u*pow(v,2);
    }

    vc=(Suu*0.5*(Svvv+Svuu)-Suv*0.5*(Suuu+Suvv))/(Svv*Suu-Suv*Suv);
    uc=(0.5*(Suuu+Suvv)-vc*Suv)/Suu;
    vc=vc+y_m;
    uc=uc+x_m;

    Center.x=uc;
    Center.y=vc;

    for(int i=0;i<cluster->support_points.size();i++)
        R+=sqrt(pow((cluster->support_points[i]->x-uc),2) + pow((cluster->support_points[i]->y-vc),2));

    R=R/cluster->support_points.size();
}

/**
@brief Creation of several points that belong to a circle based on its properties
@param[out] circle_points points created
@param[in] radius radius of the circle
@param[in] centre coordinates of the circle center
@param[in] number_points number of points to be created
@return void
*/
void circlePoints(vector<ClusterPtr>& circle_points, double radius, double centre[3], int number_points)
{
    double increment = M_PI*2/number_points;
    ClusterPtr cPoints (new Cluster);
    for(int i=0;i<=number_points;i++)
    {
        double angle = i * increment;
        PointPtr coord (new Point);
        coord->x=centre[0] + sin(angle)*radius;
        coord->y=centre[1] + cos(angle)*radius;
        cPoints->support_points.push_back(coord);
    }
    circle_points.push_back(cPoints);
}

/**
@brief Convert data to XYZ
@param[in] scan laser scan
@param[out] data_gt converted data
@return void
*/
void convertDataToXY(sensor_msgs::LaserScan scan, C_DataFromFilePtr& data_gt)
{
    C_DataFromFilePtr data (new C_DataFromFile);
    int s=scan.ranges.size();
    int l=1;
    double X, Y;
    for(int n=0; n<s; n++)
    {
            double angle, d, x, y, z;
            d=scan.ranges[n];
            z=0;
            angle = scan.angle_min + n*scan.angle_increment;
            if(angle>=0)
            {
                x=d*cos(angle);
                y=d*sin(angle);
            }
            else
            {
                angle = abs(angle);
                x = d*cos(angle);
                y = -d*sin(angle);
            }

            X+=x;
            Y+=y;
            data->x_valuesf.push_back(x);
            data->y_valuesf.push_back(y);
            data->z_valuesf.push_back(z);
            data->labels.push_back(l);
    }
    data->iteration=s;
    data_gt=data;
}
