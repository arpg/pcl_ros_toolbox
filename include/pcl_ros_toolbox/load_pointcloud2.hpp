#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>


static inline bool LoadPointCloud2sFromBag(std::string input_file, std::vector<std::string> input_topics, std::vector<sensor_msgs::PointCloud2>& cloud_msgs)
{
	rosbag::Bag inbag;
    inbag.open(input_file, rosbag::bagmode::Read);

    for(rosbag::MessageInstance const m: rosbag::View(inbag, rosbag::TopicQuery(input_topics)))
    {
        sensor_msgs::PointCloud2::ConstPtr cloud_ptr = m.instantiate<sensor_msgs::PointCloud2>();
        if (cloud_ptr != NULL)
		{
			cloud_msgs.push_back(*cloud_ptr);
		}
    }

    inbag.close();

	return 1;
}