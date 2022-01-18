#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <ros/ros.h>

static inline bool LoadPcd(std::string input_file, sensor_msgs::PointCloud2& cloud_msg)
{
	int return_status = pcl::io::loadPCDFile (input_file, cloud_msg);
	if (return_status != 0)
	{
		ROS_ERROR("Failed to load PCD file.");
		return false;
	}

	return true;
}