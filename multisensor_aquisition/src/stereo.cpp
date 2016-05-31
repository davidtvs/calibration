#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/thread.hpp>

#include <pcl/common/common.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

using namespace std;
using namespace cv;

class Viewer
{
    public:
        
        Viewer()
        {
            visualization_process = new boost::thread(&Viewer::visualization,this);
        }
        
        void drawCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,string id,int point_size)
        {
            mtx.lock();
            
            viewer->removePointCloud(id);
            pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
            viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb,id);
            viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, id);
            mtx.unlock();
        }
        
        void visualization()
        {
            viewer.reset(new pcl::visualization::PCLVisualizer ("3D Viewer"));
            viewer->setBackgroundColor (0.6, 0.6, 0.6);
            
            viewer->setWindowName("Viewer");
            
            vector<string> vicon_ids;
            vector<string> clouds_ids;
            vector<string> text_ids;
            
            viewer->setCameraFieldOfView(0.6);
            
            viewer->setShowFPS(true);       
            
            viewer->addCoordinateSystem(0.4);
 
            for(int i=0;i<3;i++)
            {
                boost::this_thread::sleep (boost::posix_time::microseconds (100000));
                viewer->spinOnce(10);
                viewer->setSize(600,600);
            }
            
            while(!viewer->wasStopped())
            {
                mtx.lock();
                viewer->spinOnce (10);
                mtx.unlock();
                boost::this_thread::sleep (boost::posix_time::microseconds (100000));
            }
            
            viewer->close();
            
            return;
        }
        
            
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    
        boost::thread* visualization_process;
        boost::mutex mtx;
};

class StereoProc
{
    public:
        
        struct
        {
            int min_disp;
            int disp;
            int SAD;
            int SGMp1;
            int SGMp2;
            int disp12_max_diff;
            int pre_filter_cap;
            int unique;
            int speckle_window;
            int speckle_range;
            bool fullDP;
            
            double cx;
            double cy;
            
            double fx;
            double fy;
            double B;
            
        }local_parameters;
        
        StereoProc(ros::NodeHandle& nh_, string parameters_):
        nh(nh_),
        it(nh_),
        parameters(parameters_.c_str(),FileStorage::READ),
        cloud(new pcl::PointCloud<pcl::PointXYZRGB>)
        {
            received_left = false;
            received_right = false;
            
            loadParameters();
            
            sub_left = it.subscribe("/rectified/left/image_rect_color", 1, &StereoProc::leftHandler,this);
            sub_right = it.subscribe("/rectified/right/image_rect_color", 1, &StereoProc::rightHandler,this);
        }
        
        void leftHandler(const sensor_msgs::ImageConstPtr& msg)
        {
            cv_bridge::CvImagePtr cv_ptr;
            
            try
            {
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            }
            catch (cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }
            
            cv_ptr->image.copyTo(left);
            received_left = true;
            
            if(received_left && received_right)
            {
                doStereo();
                
                received_left = false;
                received_right = false;
            }
        }

        void rightHandler(const sensor_msgs::ImageConstPtr& msg)
        {
            cv_bridge::CvImagePtr cv_ptr;
            
            try
            {
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            }
            catch (cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }
            
            cv_ptr->image.copyTo(right);
            received_right = true;
            
            if(received_left && received_right)
            {
                doStereo();
                
                received_left = false;
                received_right = false;
            }
        }
    
        bool doStereo()
        {
            cv::Mat disp_img;
            
            sgbm(left,right,disp_img,local_parameters.min_disp,local_parameters.disp*16,local_parameters.SAD,local_parameters.SGMp1,local_parameters.SGMp2,local_parameters.disp12_max_diff,local_parameters.pre_filter_cap,local_parameters.unique,local_parameters.speckle_window,local_parameters.speckle_range,local_parameters.fullDP);
            
            calculatePointCloud(disp_img,left,cloud);
            view.drawCloud(cloud,"cloud",3);
                
            return true;
        }
        
        void sgbm(const Mat& imgLeft,const Mat& imgRight,Mat& disparity,int minDisp, int nDisp, int sadWind, int p1, int p2,int disp12MaxDiff, int preFiltCap, int uinqRatio, int speckleWind, int speckleRange, bool fullDP)
        {
            try
            {
                Mat imgDisparity32F = Mat(imgLeft.rows, imgLeft.cols, CV_32FC1);
                Mat imgDisparity16S = Mat(imgLeft.rows, imgLeft.cols, CV_16SC1);

                StereoSGBM sgbm(minDisp, nDisp, sadWind, p1, p2, disp12MaxDiff, preFiltCap, uinqRatio, speckleWind, speckleRange, fullDP);

                //-- 3. Calculate the disparity image
                sgbm(imgLeft, imgRight, imgDisparity16S);
                imgDisparity16S.convertTo(imgDisparity32F, CV_32FC1, 1.0 / 16.0);

                disparity = imgDisparity32F; //m_images_stereo.vdisp_image;

            }
            catch (cv::Exception& exc){
                cout<<"Exception in function sgbm: "<<exc.what()<<endl;
            }
        }
        
        bool calculatePointCloud(const Mat& disp_img,const Mat& color_image,const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
        {
            // Fill in the cloud data
            cloud->clear();
            
            
            //For each pixel calculate the xyz position
            for(int row=0;row<disp_img.rows;row++)
            {
                for(int col=0;col<disp_img.cols;col++)
                {
                    double u = col - local_parameters.cx;
                    double v = row - local_parameters.cy;

                    pcl::PointXYZRGB p;
                     
                    cv::Vec3b pc = color_image.at<cv::Vec3b>(row,col);

                    p.r=pc.val[2];
                    p.g=pc.val[1];
                    p.b=pc.val[0];
                    
                    //p.data
                    double z = local_parameters.fx*local_parameters.B/disp_img.at<float>(row,col);

                    p.z = z;
                    p.x = -u*z/local_parameters.fx;
                    p.y = -v*z/local_parameters.fy;
                    
                    if(disp_img.at<float>(row,col)!=-1)
                        if(!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z) && !std::isinf(p.x) && !std::isinf(p.y) && !std::isinf(p.z))
                        {
                            cloud->push_back(p);
                        }
                }
            }
            
            return true;
        }
        
        void loadParameters()
        {
            local_parameters.min_disp = parameters["min_disparity"];
            local_parameters.disp = parameters["max_disparity"];
            local_parameters.SAD = parameters["SAD"];
            local_parameters.SGMp1 = parameters["SGMp1"];
            local_parameters.SGMp2 = parameters["SGMp2"];
            local_parameters.disp12_max_diff = parameters["disp12_max_diff"];
            local_parameters.pre_filter_cap = parameters["pre_filter_cap"];
            local_parameters.unique = parameters["unique"];
            local_parameters.speckle_window = parameters["speckle_window"];
            local_parameters.speckle_range = parameters["speckle_range"];
            local_parameters.fullDP =  (int)parameters["fullDP"];
            
            local_parameters.cx = parameters["camera_cx"];
            local_parameters.cy = parameters["camera_cy"];
            
            local_parameters.fx = parameters["camera_fx"];
            local_parameters.fy = parameters["camera_fy"];
            local_parameters.B = parameters["base_line"];
        }
        
        
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
        
        bool received_left;
        bool received_right;
        
        Mat left;
        Mat right;
        
        image_transport::Subscriber sub_left;
        image_transport::Subscriber sub_right;
        
        FileStorage parameters;
        ros::NodeHandle nh;
        image_transport::ImageTransport it;
        
        Viewer view;
};

int main(int argc,char**argv)
{
    ros::init(argc, argv, "save_frames");
    ros::NodeHandle node("~");

    if(argc!=2)
    {
        cout<<"Error!!, wrong number of input arguments"<<endl;
        cout<<"Usage: "<<argv[0]<<" parameters_file.yaml"<<endl;
        return 1;
    }
    
    StereoProc stereo(node,argv[1]);
    
    ros::spin();
    
    return 0;
}