#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <ros/ros.h>
#include <tf2_ros/buffer.h>


static inline bool LoadTfsFromBag(std::string input_file, tf2_ros::Buffer& tf_buffer)
{
    bool success = true; 

	rosbag::Bag inbag;
    inbag.open(input_file, rosbag::bagmode::Read);

    std::vector<std::string> topics = {"/tf","/tf_static"};

    for(rosbag::MessageInstance const m: rosbag::View(inbag, rosbag::TopicQuery(topics)))
    {
        geometry_msgs::TransformStamped::ConstPtr tf_ptr = m.instantiate<geometry_msgs::TransformStamped>();
        if (tf_ptr != NULL)
		{
            bool is_static = strcmp(m.getTopic().c_str(),"/tf_static");
			success = success && tf_buffer.setTransform(*tf_ptr,"default_authority",is_static);
		}
    }

	return success;
}