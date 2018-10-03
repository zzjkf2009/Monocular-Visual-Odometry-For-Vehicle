/**
 * @Author: Zejiang Zeng <yzy>
 * @Date:   2018-10-01T21:42:18-04:00
 * @Email:  zzeng@terpmail.umd.edu
 * @Filename: visual_odometry.cpp
 * @Last modified by:   yzy
 * @Last modified time: 2018-10-01T21:46:29-04:00
 * @Copyright: (C) 2017 Zejiang Zeng - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the BSD license.
 */

 #include "vo_features.h"
 #include <ros/ros.h>
 #include <cv_bridge/cv_bridge.h>
 #include <image_transport/image_transport.h>
 #include <tf/transform_broadcaster.h>

using namespace cv;
using namespace std;

#define MIN_NUM_FEAT 2000

class Visual_Odometry {
public:
Visual_Odometry(double focal_length,cv::Point2d principle_point);
private:
ros::NodeHandle nh_;
image_transport::Subscriber sub_;
vector<Point2f> prevFeatures_; // Vectors to store the coordinates of the feature points
vector<Point2f> currFeatures_;
Mat prevImage_;
Mat currImage_;
Mat E_, R_, t_, mask_, R_f, t_f;
double focal_; // focal length of the camera
cv::Point2d pp_;  // principle point of the camera
double scale_ = 0.01;
bool initRT = true;
tf::TransformBroadcaster br_;
tf::Transform transform_;
void imageSubscriber();
void imageCallback(const sensor_msgs::ImageConstPtr& msg);
};



Visual_Odometry::Visual_Odometry(double focal_length,cv::Point2d principle_point) : focal_(focal_length),pp_(principle_point) {
        image_transport::ImageTransport it(nh_);
        sub_ = it.subscribe("camera/image", 1,
                            &Visual_Odometry::imageCallback, this);
}


void Visual_Odometry::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        try {
                if (prevImage_.empty()) {
                        ROS_INFO("prev image is empty");
                        Mat prevImage_color = cv_bridge::toCvShare(msg, "bgr8")->image; // Read image msg;
                        cvtColor(prevImage_color, prevImage_, COLOR_BGR2GRAY); // Convert color image to grayscale
                        featureDetection(prevImage_, prevFeatures_,20); // Detect features in inital image
                }
                else {
                        //  ROS_INFO("Let's do publish tf");
                        Mat currImage_color = cv_bridge::toCvShare(msg, "bgr8")->image;
                        cvtColor(currImage_color, currImage_, COLOR_BGR2GRAY);
                        vector<uchar> status;
                        featureTracking(prevImage_, currImage_, prevFeatures_, currFeatures_, status);
                        E_ = findEssentialMat(currFeatures_, prevFeatures_, focal_, pp_, RANSAC, 0.999, 1.0, mask_);
                        recoverPose(E_, currFeatures_, prevFeatures_, R_, t_, focal_, pp_, mask_);
                        Mat prevPts(2, prevFeatures_.size(), CV_64F), currPts(2, currFeatures_.size(), CV_64F);
                        for(int i=0; i < prevFeatures_.size(); i++) { //this (x,y) combination makes sense as observed from the source code of triangulatePoints on GitHub
                                prevPts.at<double>(0,i) = prevFeatures_.at(i).x;
                                prevPts.at<double>(1,i) = prevFeatures_.at(i).y;
                                currPts.at<double>(0,i) = currFeatures_.at(i).x;
                                currPts.at<double>(1,i) = currFeatures_.at(i).y;
                        }
                        if (initRT) {
                                initRT = false;
                                R_f = R_.clone();
                                t_f = t_.clone();
                        } else {
                                t_f = t_f + scale_*(R_f*t_);
                                R_f = R_*R_f;
                        }

                        if (prevFeatures_.size() < MIN_NUM_FEAT) {
                                featureDetection(prevImage_, prevFeatures_,20);
                                featureTracking(prevImage_,currImage_,prevFeatures_,currFeatures_, status);
                        }
                        prevImage_ = currImage_.clone();
                        prevFeatures_ = currFeatures_;
                        transform_.setOrigin(tf::Vector3(t_f.at<double>(0), t_f.at<double>(2), t_f.at<double>(1)));
                        transform_.setBasis(tf::Matrix3x3(R_f.at<double>(0),R_f.at<double>(1),R_f.at<double>(2),R_f.at<double>(3),R_f.at<double>(4),R_f.at<double>(5),R_f.at<double>(6),R_f.at<double>(7),R_f.at<double>(8)));
                        br_.sendTransform(tf::StampedTransform(transform_, ros::Time::now(), "world", "vehicle"));

                }
        } catch (cv_bridge::Exception& e) {
                ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }
}

int main(int argc, char** argv) {
        ros::init(argc, argv, "vehicle_visual_odometry");
        if(argc != 4) {
                ROS_ERROR("Invalid number of arguments for: focal_length, principle_point1, principle_point2");
                return -1;
        }
        cv::Point2d pp(atof(argv[2]),atof(argv[3]));
        Visual_Odometry visual_odometry(atof(argv[1]),pp);
        ros::spin();
        return 0;
}
