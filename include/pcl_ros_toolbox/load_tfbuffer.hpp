#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <ros/ros.h>
#include <tf2_ros/buffer.h>


static inline bool LoadTfsFromBag(std::string input_file, tf2_ros::Buffer& tf_buffer)
{
    bool success = true; 

	rosbag::Bag inbag;
    inbag.open(input_file, rosbag::bagmode::Read);

    std::vector<std::string> topics = {std::string("/tf"),std::string("/tf_static")};

    ros::Time last_static_tf_time, last_tf_time;
    tf::tfMessage::Ptr last_static_tf, last_tf;

    for(rosbag::MessageInstance const m: rosbag::View(inbag, rosbag::TopicQuery(topics)))
    {
        bool is_static = strcmp(m.getTopic().c_str(),"/tf_static");
        if (m.getTime().toSec()>last_tf_time.toSec() && !is_static)
        {
            last_tf = m.instantiate<tf::tfMessage>();
            last_tf_time = m.getTime();
        }
        if (m.getTime().toSec()>last_static_tf_time.toSec() && is_static)
        {
            last_static_tf = m.instantiate<tf::tfMessage>();
            last_static_tf_time = m.getTime();
        }
    }

    if (last_tf != NULL)
    {
        for (uint i=0; i<last_tf->transforms.size(); i++)
        {
            geometry_msgs::TransformStamped stampedTf = last_tf->transforms[i];
            success = success && tf_buffer.setTransform(stampedTf,"default_authority",false);
        }
    }

    if (last_static_tf != NULL)
    {
        for (uint i=0; i<last_static_tf->transforms.size(); i++)
        {
            geometry_msgs::TransformStamped stampedTf = last_static_tf->transforms[i];
            success = success && tf_buffer.setTransform(stampedTf,"default_authority",true);
        }
    }

	return success;
}