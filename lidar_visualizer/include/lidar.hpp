#include "ros/ros.h"
#include "lidar_visualizer/message.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <sensor_msgs/PointCloud.h>
#include <vector>
#include<dirent.h>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
class LidarReader
{
public:
   //std::string lidar_path,image_path,timestamp_path;
    std::fstream OpenFile;
    std::fstream OpenLidarTimeStampFile;
    std::string X1;
    std::string Y1;
    std::string Z1;
    std::string I1;
    std::string Time;
    std::string column0;
    std::string column1;
    std::vector<double>x_cord;
    std::vector<double>y_cord;
    std::vector<double>z_cord;
    std::vector<double>I_cord;
    std::vector<double>TimeVector;
    std::vector<std::string>LidarFiles;
    std::vector<std::string>ImageFiles;
    sensor_msgs::ImagePtr msg;
    int sizeOfCloud;
    ros::Publisher keypoints_publisher;
    LidarReader();
    DIR *LidarDIR;
    DIR *ImageDIR;
    void LidarTimeStampRead();
    void LidarPointCloud(sensor_msgs::PointCloud& pc,double centerX,double centerY,int& sizeOfCloud,std::string path);
    void getDirectory();
    void PublishLidarFrames();
    void LidarRead();
 };
