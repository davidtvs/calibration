#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/filesystem.hpp>
#include <boost/format.hpp>

using namespace std;

class SaveFrames
{

    public:
        SaveFrames(ros::NodeHandle& nh_):
        nh(nh_),
        it(nh_)
        {
            ros::NodeHandle private_node("~");
            
            seq_number = 0;
            
            string path;
            private_node.param("output_path",path,string("no output path defined"));
            
            output_path = path;
            
            subscriber = it.subscribe("image_raw", 1000, &SaveFrames::imageHandler,this);
        }
        
        void imageHandler(const sensor_msgs::ImageConstPtr& msg)
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
            
            boost::filesystem::path file_url = output_path / (boost::format("%06.d.png") % seq_number).str();
            
            cv::imwrite(file_url.string(),cv_ptr->image);
            seq_number++;
        }

        uint seq_number;
        
        boost::filesystem::path output_path;
        
        sensor_msgs::Image image;
        
        image_transport::Subscriber subscriber;
        
        ros::NodeHandle nh;
        image_transport::ImageTransport it;
};

int main(int argc,char**argv)
{
    ros::init(argc, argv, "save_frames");
    ros::NodeHandle node;

    SaveFrames save_frames(node);
    
    ros::spin();
    
    
    return 0;
}