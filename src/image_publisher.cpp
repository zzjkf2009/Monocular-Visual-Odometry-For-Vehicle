#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <fstream>
#include <string>


int main(int argc, char** argv)
{
        ros::init(argc, argv, "image_publisher");
        ros::NodeHandle nh;
        image_transport::ImageTransport it(nh);
        image_transport::Publisher pub = it.advertise("camera/image", 1);
        int frame_index = 0;
        ros::Rate loop_rate(5);
        char filename[200];
        char filenameInput[200];
        if(argc > 1) {
                strcpy(filenameInput,argv[1]);
        }
        else {
                ROS_ERROR("No imge path");
                return -1;
        }

        while (nh.ok()) {
                sprintf(filename, filenameInput, frame_index);
                cv::Mat image = cv::imread(filename, CV_LOAD_IMAGE_COLOR);
                if(!image.data) {
                        ROS_ERROR("last Image");
                        return -1;
                }
                sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
                pub.publish(msg);
                frame_index++;
                ros::spinOnce();
                loop_rate.sleep();
        }
}
