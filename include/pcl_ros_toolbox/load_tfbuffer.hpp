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

    // determine buffer cache duration
    ros::Time bagStart(0.f), bagStop(0.f);
    for(rosbag::MessageInstance const m: rosbag::View(inbag, rosbag::TopicQuery(topics)))
    {
        if (bagStart.toSec()==0.f)
            bagStart = m.getTime();

        if (m.getTime().toSec()>bagStop.toSec())
            bagStop = m.getTime();
    }

    tf_buffer = tf2_ros::Buffer(ros::Duration(bagStop-bagStart));
    for(rosbag::MessageInstance const m: rosbag::View(inbag, rosbag::TopicQuery(topics)))
    {
        bool is_static = strcmp(m.getTopic().c_str(),"/tf_static");

        tf::tfMessage::Ptr tf_ptr = m.instantiate<tf::tfMessage>();
        for (uint i=0; i<tf_ptr->transforms.size(); i++)
        {
            geometry_msgs::TransformStamped stampedTf = tf_ptr->transforms[i];
            success = success && tf_buffer.setTransform(stampedTf,"default_authority",is_static);
        }
    }

    // if (last_tf != NULL)
    // {
    //     for (uint i=0; i<last_tf->transforms.size(); i++)
    //     {
    //         geometry_msgs::TransformStamped stampedTf = last_tf->transforms[i];
    //         success = success && tf_buffer.setTransform(stampedTf,"default_authority",false);
    //     }
    // }

    // if (last_static_tf != NULL)
    // {
    //     for (uint i=0; i<last_static_tf->transforms.size(); i++)
    //     {
    //         geometry_msgs::TransformStamped stampedTf = last_static_tf->transforms[i];
    //         success = success && tf_buffer.setTransform(stampedTf,"default_authority",true);
    //     }
    // }

	return success;
}