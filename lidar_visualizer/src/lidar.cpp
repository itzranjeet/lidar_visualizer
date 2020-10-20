#include "lidar.hpp"

LidarReader::LidarReader()
{
  ros::NodeHandle nh;
  keypoints_publisher = nh.advertise<sensor_msgs::PointCloud>("test_lidar", 10);
}

void LidarReader::getDirectory()
{
  struct dirent *entry;
  if (LidarDIR = opendir("/home/ranjeet/Desktop/Honda/ros_ws/src/lidar_visualizer/data/2011_09_26_drive_0005_extract/2011_09_26/2011_09_26_drive_0005_extract/velodyne_points/data"))
  {
    while (entry = readdir(LidarDIR))
    {
      if (strcmp(entry->d_name, ".") != 0 && strcmp(entry->d_name, "..") != 0)
        LidarFiles.push_back(entry->d_name);
    }

    closedir(LidarDIR);
    std::sort(LidarFiles.begin(), LidarFiles.end());
  }
}

void LidarReader::LidarRead()
{
  lidar_visualizer::message data;
  getline(OpenFile, X1, ' ');
  data.X = X1;
  x_cord.push_back(atof(data.X.c_str()));

  getline(OpenFile, Y1, ' ');
  data.Y = Y1;
  y_cord.push_back(atof(data.Y.c_str()));

  getline(OpenFile, Z1, ' ');
  data.Z = Z1;
  z_cord.push_back(atof(data.Z.c_str()));

  getline(OpenFile, I1, '\n');
  data.I = I1;
  I_cord.push_back(atof(data.I.c_str()));
}

void LidarReader::LidarPointCloud(sensor_msgs::PointCloud &pc, double centerX, double centerY, int &sizeOfCloud, std::string path)
{
  for (int i = 0; i < pc.points.size(); i++)
  {
    pc.points[i].x = x_cord[i];
    pc.points[i].y = y_cord[i];
    pc.points[i].z = z_cord[i];
    x_cord.clear();
    y_cord.clear();
    z_cord.clear();
  }
  std_msgs::Header frame;
  frame.frame_id = path;
  sensor_msgs::ChannelFloat32 depth_channel;
  for (int i = 0; i < pc.points.size(); i++)
  {
    depth_channel.values.push_back(I_cord[i]);
    I_cord.clear();
  }
  pc.channels.push_back(depth_channel);
}

void LidarReader::LidarTimeStampRead()
{
  OpenLidarTimeStampFile.open("/home/ranjeet/Desktop/Honda/ros_ws/src/lidar_visualizer/data/2011_09_26_drive_0005_extract/2011_09_26/2011_09_26_drive_0005_extract/velodyne_points/timestamps_start.txt", std::fstream::in);
  if (OpenLidarTimeStampFile.good())
  {
    while (ros::ok() and OpenLidarTimeStampFile.good())
    {
      getline(OpenLidarTimeStampFile, column0, ':');
      getline(OpenLidarTimeStampFile, column1, ':');
      getline(OpenLidarTimeStampFile, Time, '\n');
      TimeVector.push_back(atof(Time.c_str()));
    }
    OpenLidarTimeStampFile.close();
  }
}

void LidarReader::PublishLidarFrames()
{
  for (int i = 0; i < LidarFiles.size(); i++)
  {
    std::string str2 = "/home/ranjeet/Desktop/Honda/ros_ws/src/lidar_visualizer/data/2011_09_26_drive_0005_extract/2011_09_26/2011_09_26_drive_0005_extract/velodyne_points/data/" + LidarFiles[i];
    std::string path = str2.c_str();
    std::cout << path << std::endl;
    OpenFile.open(path, std::fstream::in);
    std::cout << "I am in file ->>>>" << LidarFiles[i] << std::endl;
    if (OpenFile.good())
    {
      while (ros::ok() and OpenFile.good())
      {
        LidarRead();
      }
      OpenFile.close();
    }

    sizeOfCloud = x_cord.size();
    std::cout << sizeOfCloud << std::endl;
    sensor_msgs::PointCloud keypoints;
    keypoints.points.resize(sizeOfCloud);
    keypoints.header.frame_id = "base_link";
    ros::Rate rate(1);
    LidarPointCloud(keypoints, 0.5, 0.5, sizeOfCloud, path);

    double t = TimeVector[i];
    std::cout << TimeVector[i] << std::endl;
    ros::Time Time(t);
    keypoints.header.stamp = Time;

    keypoints_publisher.publish(keypoints);
    ros::spinOnce();
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Node");
  LidarReader object;
  object.LidarTimeStampRead();
  object.getDirectory();
  object.PublishLidarFrames();
  std::string lidar_path, image_path, timestamp_path;
  return 0;
}
