#include "ros/ros.h"

#include "nodelet/nodelet.h"
#include <pluginlib/class_list_macros.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>


#define MAX_NUM_CHANNELS 9

namespace pcl_ros_toolbox
{

typedef message_filters::sync_policies::ApproximateTime
        <sensor_msgs::PointCloud2,
        sensor_msgs::PointCloud2> approx_time_sync_policy_2;

typedef message_filters::sync_policies::ApproximateTime
        <sensor_msgs::PointCloud2,
        sensor_msgs::PointCloud2,
        sensor_msgs::PointCloud2> approx_time_sync_policy_3;

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

typedef approx_time_sync_policy_2 approx_time_sync_policy;

class ApproxTimeConcatenateServer : public nodelet::Nodelet
{
public:
  // ApproxTimeConcatenateServer();

  inline virtual void onInit()
  {
    ROS_INFO("Initializing...");

    // nh_ = getNodeHandle();
    // private_nh_ = getPrivateNodeHandle();

    // getPrivateNodeHandle().param("num_inputs", num_inputs_, int(2));
    getPrivateNodeHandle().param("common_frame_id", common_frame_id_, std::string("world"));
    getPrivateNodeHandle().param("min_rate", min_rate_, double(4));
    getPrivateNodeHandle().param("sync_queue_size", sync_queue_size_, int(5));
    getPrivateNodeHandle().param("sub_queue_size", sub_queue_size_, int(5));

    approx_time_sync_policy sync_policy(sync_queue_size_);
    sync_policy.setMaxIntervalDuration(ros::Duration(1.0/min_rate_));
    // sync_policy.setInterMessageLowerBound(0, ros::Duration(0.05));
    // sync_policy.setInterMessageLowerBound(1, ros::Duration(0.05));
    // sync_policy.setInterMessageLowerBound(2, ros::Duration(0.05));

    pub_ = getPrivateNodeHandle().advertise<sensor_msgs::PointCloud2>("output", 1);

    subs_.clear();
    for (uint i=0; i<2; i++)
    {
      subs_.push_back(new message_filters::Subscriber<sensor_msgs::PointCloud2>(getPrivateNodeHandle(), "input_"+std::to_string(i), sub_queue_size_));
    }

    uint i=0;
    sync_ = new message_filters::Synchronizer <approx_time_sync_policy>(static_cast<const approx_time_sync_policy &>(sync_policy)
      ,*(subs_[i++])
      ,*(subs_[i++]) );
    // sync.registerCallback(boost::bind(&ApproxTimeConcatenateServer::callback, this));
    // sync.registerCallback(boost::bind(&ApproxTimeConcatenateServer::callback, this, _1, _2, _3, _4, _5, _6, _7, _8, _9));
    sync_->registerCallback(boost::bind(&ApproxTimeConcatenateServer::callback, this, _1, _2));

  };

  inline bool getTransformFromTree(std::string parent_frame_id, std::string child_frame_id, tf::StampedTransform& tform_msg, ros::Time stamp=ros::Time(0), double timeout=0.5)
  {
    // tf::StampedTransform tform_msg;
    static tf::TransformListener listener;

    if (parent_frame_id == child_frame_id)
    {
      ROS_WARN("No need to get transform for frame %s. Assuming Identity.", parent_frame_id.c_str());
      tform_msg.setIdentity();
      return true;
    }

    try
    {
      listener.waitForTransform(parent_frame_id, child_frame_id, ros::Time(0), ros::Duration(timeout));
      listener.lookupTransform(parent_frame_id, child_frame_id, stamp, tform_msg);
    }
    catch (tf::TransformException ex)
    {
      // ROS_ERROR("No link between %s and %s. Assuming Identity.", parent_frame_id.c_str(), child_frame_id.c_str());
      ROS_ERROR("%s",ex.what());
      tform_msg.setIdentity();
      return false;
    }

    // Eigen::Affine3d tform;
    // tf::transformTFToEigen(tf::Transform(tform_msg),tform);
    // (*tform_mat) = tform.matrix();
    return true;
  }

  inline bool transformAndConcatCloud(pcl::PointCloud<pcl::PointXYZ> partial_cloud, pcl::PointCloud<pcl::PointXYZ>& whole_cloud)
  {
    tf::StampedTransform tform_msg;
    // if(!getTransformFromTree(common_frame_id_, partial_cloud.header.frame_id, tform_msg, ros::Time().fromNSec(partial_cloud.header.stamp*1000) ))
    if(!getTransformFromTree(whole_cloud.header.frame_id, partial_cloud.header.frame_id, tform_msg ))
    {
      // ROS_WARN("Failed to get transform from tf tree.");
      return false;
    }
    pcl_ros::transformPointCloud (partial_cloud, partial_cloud, tform_msg);
    whole_cloud += partial_cloud;
    whole_cloud.header.stamp = partial_cloud.header.stamp;
    
    return true;
  }

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
    ROS_INFO("ApproxTimeConcatenateServer called.");

    ROS_INFO("Received input0 with size %d frame %s stamp %f", input0->width, input0->header.frame_id.c_str(), input0->header.stamp.toSec());
    ROS_INFO("Received input1 with size %d frame %s stamp %f", input1->width, input1->header.frame_id.c_str(), input1->header.stamp.toSec());
    // ROS_INFO("Received input2 with size %d", input2->width);
    // ROS_INFO("Received input3 with size %d", input3->width);
    // ROS_INFO("Received input4 with size %d", input4->width);
    // ROS_INFO("Received input5 with size %d", input5->width);
    // ROS_INFO("Received input6 with size %d", input6->width);
    // ROS_INFO("Received input7 with size %d", input7->width);
    // ROS_INFO("Received input8 with size %d", input8->width);

    pcl::PointCloud<pcl::PointXYZ> whole_cloud, cloud0, cloud1;//, cloud2;//, cloud0, cloud0, cloud0, cloud0, 
    whole_cloud.header.frame_id = common_frame_id_;
   
		pcl::fromROSMsg(*input0, cloud0);
		pcl::fromROSMsg(*input1, cloud1);
		// pcl::fromROSMsg(*input2, cloud2);

    if (!transformAndConcatCloud(cloud0, whole_cloud)) return;
    if (!transformAndConcatCloud(cloud1, whole_cloud)) return;

    if (whole_cloud.size()==0)
    {
      ROS_WARN("Empty cloud. Skipping...");
      return;
    }
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(whole_cloud, output);

    ROS_INFO("Publishing concatenated cloud with size %d frame %s", output.width, output.header.frame_id.c_str());

    pub_.publish(output);
    ros::spinOnce();
  }

private:
  // ros::NodeHandle& nh_;
  // ros::NodeHandle& private_nh_;
  message_filters::Synchronizer <approx_time_sync_policy>* sync_;
  std::vector<message_filters::Subscriber<sensor_msgs::PointCloud2>*> subs_;
  ros::Publisher pub_;

  // uint num_inputs_;
  std::string common_frame_id_;
  double min_rate_;
  int sync_queue_size_;
  int sub_queue_size_;
};

} // namespace pcl_ros_toolbox

PLUGINLIB_EXPORT_CLASS(pcl_ros_toolbox::ApproxTimeConcatenateServer, nodelet::Nodelet)
