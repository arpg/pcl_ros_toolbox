// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "jly_goicp.h"
#include <iostream>
#include <fstream>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl_ros_toolbox/load_point3d.hpp>
#include <pcl_ros_toolbox/load_pointcloud2.hpp>
#include <pcl_ros_toolbox/load_pcd.hpp>

int main (int argc, char *argv[])
{
	double startTime = pcl::getTime();
	ROS_INFO("Initializing...");

    ros::init(argc, argv, "pointcloud_publisher");
    ros::NodeHandle n;
    ros::Publisher cloud_pub = n.advertise<sensor_msgs::PointCloud2>("cloud", 1);
	std::string filepath;
	n.param("filepath", filepath);
	float pub_rate;
	n.param("pub_rate", pub_rate);
	std::string frame_id;
	n.param("frame_id", frame_id);

	double initTime = pcl::getTime();

	ROS_INFO("Loading pointcloud from file: %s...", filepath);

	std::string extension = filepath.substr(filepath.find_last_of(".") + 1);
	sensor_msgs::PointCloud2 cloud;
	int num_points;
	switch(extension)
	{
		case "txt":
			ROS_INFO("Loading pointcloud via POINT3D...");
			LoadPOINT3DFromFileAndConvertToPointCloud2(filepath, cloud);
			cloud.header.frame_id = frame_id;
			cloud.header.stamp = ros::Time::now();
			break;
		case "pcd":
			ROS_INFO("Loading pointcloud via PCD...");
			LoadPcd(filepath, cloud);
			cloud.header.frame_id = frame_id;
			cloud.header.stamp = ros::Time::now();
			break;
		case "bag":
			ROS_INFO("Loading pointcloud via ROSBAG (ignoring frame_id param)...");
			LoadPointCloud2sFromBag(filepath, cloud);
			break;
		default:
			ROS_INFO("Failed to load pointcloud. Extension '%s' unsupported.", extension.c_str());
			break;
	}
	double loadTime = pcl::getTime();

	ROS_INFO("Loaded cloud with %d points. Took %f sec.",num_points,loadTime-initTime);

	ROS_INFO("Publishing...");

    while(ros::ok())
    {
        cloud_pub.publish(cloud_msg);
        // ros::spinOnce();
        ros::Rate(pub_rate).sleep();
    }
}

