#include "ros/ros.h"

#include "nodelet/nodelet.h"
#include <pluginlib/class_list_macros.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>


#define MAX_NUM_CHANNELS 9

namespace pcl_ros_toolbox
{

typedef message_filters::sync_policies::ApproximateTime
        <sensor_msgs::PointCloud2,
        sensor_msgs::PointCloud2> approx_time_sync_policy_2;

typedef message_filters::sync_policies::ApproximateTime
        <sensor_msgs::PointCloud2,
        sensor_msgs::PointCloud2,
        sensor_msgs::PointCloud2,
        sensor_msgs::PointCloud2,
        sensor_msgs::PointCloud2,
        sensor_msgs::PointCloud2,
        sensor_msgs::PointCloud2,
        sensor_msgs::PointCloud2,
        sensor_msgs::PointCloud2> approx_time_sync_policy_9;

class ConcatenateServer : public nodelet::Nodelet
{
public:
  // ConcatenateServer();

  inline virtual void onInit()
  {
    // nh_ = getNodeHandle();
    // private_nh_ = getPrivateNodeHandle();

    pub_ = getPrivateNodeHandle().advertise<sensor_msgs::PointCloud2>("output", 1);

    subs_.clear();
    for (uint i=0; i<2; i++)
    {
      subs_.push_back(new message_filters::Subscriber<sensor_msgs::PointCloud2>(getPrivateNodeHandle(), "input_"+std::to_string(i), 5));
    }
    uint i=0;
    sync_ = new message_filters::Synchronizer <approx_time_sync_policy_2>(approx_time_sync_policy_2(5)
      ,*(subs_[i++])
      ,*(subs_[i++]) );
    // sync.registerCallback(boost::bind(&ConcatenateServer::callback, this));
    // sync.registerCallback(boost::bind(&ConcatenateServer::callback, this, _1, _2, _3, _4, _5, _6, _7, _8, _9));
    sync_->registerCallback(boost::bind(&ConcatenateServer::callback, this, _1, _2));

  };

  inline void callback(const sensor_msgs::PointCloud2ConstPtr& input0
                , const sensor_msgs::PointCloud2ConstPtr& input1
                // , const sensor_msgs::PointCloud2ConstPtr& input2
                // , const sensor_msgs::PointCloud2ConstPtr& input3
                // , const sensor_msgs::PointCloud2ConstPtr& input4
                // , const sensor_msgs::PointCloud2ConstPtr& input5
                // , const sensor_msgs::PointCloud2ConstPtr& input6
                // , const sensor_msgs::PointCloud2ConstPtr& input7
                // , const sensor_msgs::PointCloud2ConstPtr& input8 
                )
  {
    ROS_INFO("ConcatenateServer called.");

    ROS_INFO("Received input0 with size %d", input0->width);
    ROS_INFO("Received input1 with size %d", input1->width);
    // ROS_INFO("Received input2 with size %d", input2->width);
    // ROS_INFO("Received input3 with size %d", input3->width);
    // ROS_INFO("Received input4 with size %d", input4->width);
    // ROS_INFO("Received input5 with size %d", input5->width);
    // ROS_INFO("Received input6 with size %d", input6->width);
    // ROS_INFO("Received input7 with size %d", input7->width);
    // ROS_INFO("Received input8 with size %d", input8->width);

    pcl::PointCloud<pcl::PointXYZ> cloud0, cloud1;//, cloud0, cloud0, cloud0, cloud0, cloud0, 
   
		pcl::fromROSMsg(*input0, cloud0);
		pcl::fromROSMsg(*input1, cloud1);

    pcl::PointCloud<pcl::PointXYZ> concat_cloud = cloud0;
    concat_cloud += cloud1;

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(concat_cloud, output);

    ROS_INFO("Publishing output with size %d", output.width);

    pub_.publish(output);
    ros::spinOnce();
  }

private:
  // ros::NodeHandle& nh_;
  // ros::NodeHandle& private_nh_;
  message_filters::Synchronizer <approx_time_sync_policy_2>* sync_;
  std::vector<message_filters::Subscriber<sensor_msgs::PointCloud2>*> subs_;
  ros::Publisher pub_;
};

} // namespace pcl_ros_toolbox

PLUGINLIB_EXPORT_CLASS(pcl_ros_toolbox::ConcatenateServer, nodelet::Nodelet)
