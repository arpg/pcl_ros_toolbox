#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

static inline bool LoadPointCloud2FromBag(std::string input_file, std::string input_topic, sensor_msgs::PointCloud2& cloud_msg)
{
	rosbag::Bag inbag;
    inbag.open(input_file, rosbag::bagmode::Read);

    cloud_msg.header.stamp = ros::Time(0.f);;
    for(rosbag::MessageInstance const m: rosbag::View(inbag, rosbag::TopicQuery(input_topic)))
    {
        sensor_msgs::PointCloud2::ConstPtr cloud_ptr = m.instantiate<sensor_msgs::PointCloud2>();
        if (cloud_ptr->header.stamp.toSec()>cloud_msg.header.stamp.toSec())
        {
            cloud_msg = *cloud_ptr;
        }
    }

    inbag.close();

	return 1;
}

static inline bool LoadPointCloud2sFromBag(std::string input_file, std::vector<std::string> input_topics, std::vector<sensor_msgs::PointCloud2>& cloud_msgs, float throttle_rate=0.f)
{
	rosbag::Bag inbag;
    inbag.open(input_file, rosbag::bagmode::Read);

    ros::Time last_time(0.f);
    for(rosbag::MessageInstance const m: rosbag::View(inbag, rosbag::TopicQuery(input_topics)))
    {
        sensor_msgs::PointCloud2::ConstPtr cloud_ptr = m.instantiate<sensor_msgs::PointCloud2>();

        if (cloud_ptr != NULL)
		{
            if (throttle_rate>0.f && (cloud_ptr->header.stamp.toSec()-last_time.toSec())<=1.f/throttle_rate)
                continue;

			cloud_msgs.push_back(*cloud_ptr);
            last_time = cloud_msgs.back().header.stamp;
		}
    }

    inbag.close();

	return 1;
}