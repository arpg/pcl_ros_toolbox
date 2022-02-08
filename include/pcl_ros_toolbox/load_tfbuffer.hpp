#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <ros/ros.h>
#include <tf2_ros/buffer.h>

static inline float GetBagDuration(std::string input_file)
{
	rosbag::Bag inbag;
    inbag.open(input_file, rosbag::bagmode::Read);

    std::vector<std::string> topics = {std::string("/tf"),std::string("/tf_static")};

    ros::Time bagStart(0.f), bagStop(0.f);
    for(rosbag::MessageInstance const m: rosbag::View(inbag, rosbag::TopicQuery(topics)))
    {
        if (m.getTime().toSec()<bagStart.toSec() || bagStart.toSec()==0.f)
            bagStart = m.getTime();

        if (m.getTime().toSec()>bagStop.toSec())
            bagStop = m.getTime();
    }
    
    inbag.close();

    ros::Duration dur(bagStop-bagStart);
	return dur.toSec();
}

static inline bool LoadTfsFromBag(std::string input_file, tf2_ros::Buffer*& tf_buffer)
{
    float bag_dur = GetBagDuration(input_file);
    ROS_INFO("Loading tfbuffer length %f sec", bag_dur);

    bool success = true; 

	rosbag::Bag inbag;
    inbag.open(input_file, rosbag::bagmode::Read);

    std::vector<std::string> topics = {std::string("/tf"),std::string("/tf_static")};

    tf_buffer = new tf2_ros::Buffer(ros::Duration(bag_dur*1.05));
    for(rosbag::MessageInstance const m: rosbag::View(inbag, rosbag::TopicQuery(topics)))
    {
        bool is_static = (m.getTopic() == "/tf_static");

        tf::tfMessage::Ptr tf_ptr = m.instantiate<tf::tfMessage>();
        for (uint i=0; i<tf_ptr->transforms.size(); i++)
        {
            geometry_msgs::TransformStamped stampedTf = tf_ptr->transforms[i];
            if (stampedTf.header.frame_id.substr(0,1) == "/")
                stampedTf.header.frame_id = stampedTf.header.frame_id.substr(1); // remove leading slash from any tf frames, it shouldn't be there
            // ROS_INFO("Loading tf: %s %f %s", m.getTopic().c_str(), stampedTf.header.stamp.toSec(), stampedTf.header.frame_id.c_str());
            success = success && tf_buffer->setTransform(stampedTf,"default_authority",is_static);
        }
    }

    inbag.close();

	return success;
}