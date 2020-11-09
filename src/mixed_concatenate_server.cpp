#include <queue>
#include <chrono>

#include "ros/ros.h"

#include "nodelet/nodelet.h"
#include <pluginlib/class_list_macros.h>

#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

#include <tf_conversions/tf_eigen.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

#define MAX_QUEUE_SIZE 10

namespace pcl_ros_toolbox
{

class MixedConcatenateServer : public nodelet::Nodelet
{
public:
  // MixedConcatenateServer();

  inline virtual void onInit()
  {
    // nh_ = getNodeHandle();
    // private_nh_ = getPrivateNodeHandle();
    
    getPrivateNodeHandle().param("termination_method", termination_method_, std::string("time_window"));
    getPrivateNodeHandle().param("termination_value", termination_value_, int(250));
    getPrivateNodeHandle().param("common_frame_id", common_frame_id_, std::string("world"));
    getPrivateNodeHandle().param("output_resolution_", output_resolution_, double(0.01));
    getPrivateNodeHandle().param("time_window_inconsistency_thresh", time_window_inconsistency_thresh_, int(2));
    
    if (termination_method_ == "message_window")
    {
      if (termination_value_ > MAX_QUEUE_SIZE)
      {
        ROS_FATAL("Tried to set message window size to %d but max possible size is %d. Aborting.", termination_value_, MAX_QUEUE_SIZE);
        return;
      }
      else
      {
        queue_size_ = termination_value_;
      }
    }
    if (termination_method_ == "time_window")
    {
      queue_size_ = MAX_QUEUE_SIZE;
    }

    pub_ = getPrivateNodeHandle().advertise<sensor_msgs::PointCloud2>("output", 5);
    sub_ = getPrivateNodeHandle().subscribe("input", queue_size_, &MixedConcatenateServer::callback, this);

    if (termination_method_ == "time_window")
    {
      publishTimer_ = getNodeHandle().createTimer(ros::Duration(termination_value_/1000.0), boost::bind(&MixedConcatenateServer::publishLoop, this, _1));
    }
  };

  inline void callback(const sensor_msgs::PointCloud2ConstPtr& input)
  {
    ROS_INFO("Received input with size %d", input->width);

    msg_queue_.push(input);
    
    if (msg_queue_.size() >= queue_size_)
    {
      while (msg_queue_.size()>queue_size_)
      { 
        msg_queue_.pop(); 
      }
      if (termination_method_ == "message_window")
      { 
        publishQueue(); 
      }
    }
  }

  inline void publishLoop(const ros::TimerEvent& event)
  {
    if (msg_queue_.empty()) return;
    
    ros::Time t0 = ros::Time::now();
    publishQueue();
    ros::Time t1 = ros::Time::now();
    double dt = (t1-t0).toSec();

    // publishQueue();

    // static ros::Time t0;
    // static ros::Time t1;
    // t1 = ros::Time::now();
    // double dt = (t1-t0).toSec();
    // t0 = t1;

    // static auto t0 = std::chrono::high_resolution_clock::now();
    // static auto t1 = std::chrono::high_resolution_clock::now();
    // t1 = std::chrono::high_resolution_clock::now();
    // double dt = (std::chrono::duration_cast<std::chrono::milliseconds>(t1-t0).count())/1000.0;
    // t0 = t1;

    if (termination_method_=="time_window")
    {
      static int inconsistent_count;
      if (dt > 1.25*(termination_value_/1000.0) && inconsistent_count>=time_window_inconsistency_thresh_)
      {
        ROS_WARN("Measured publish duration of %.2f. Reducing queue size to %d to maintain configured publish duration of %.2f.", dt, queue_size_ = (int)std::max(queue_size_-1, 2), (termination_value_/1000.0) );
        inconsistent_count = 0;
      }
      if (dt < 0.75*(termination_value_/1000.0) && inconsistent_count>=time_window_inconsistency_thresh_)
      {
        ROS_WARN("Measured publish duration of %.2f. Increasing queue size to %d to maintain configured publish duration of %.2f.", dt, queue_size_ = (int)std::min(queue_size_+1, MAX_QUEUE_SIZE), (termination_value_/1000.0) );
        inconsistent_count = 0;
      }
      inconsistent_count++;
    }
    
  }

  inline void publishQueue()
  {
    ros::Time last_time;
    pcl::PointCloud<pcl::PointXYZ> whole_cloud;
    // pcl::PCLPointCloud2 whole_cloud;

    while(!msg_queue_.empty())
    {
      last_time = msg_queue_.back()->header.stamp;

      std::string last_frame_id = msg_queue_.back()->header.frame_id;
      if (last_frame_id.empty())
      {
        ROS_WARN("No frame provided for pointcloud. Assuming %s.", common_frame_id_.c_str());
        last_frame_id = common_frame_id_;
      }

      tf::StampedTransform tform_msg;
      getTransformFromTree(common_frame_id_, last_frame_id, tform_msg, last_time);

      pcl::PointCloud<pcl::PointXYZ> cloud;
		  pcl::fromROSMsg(*(msg_queue_.back()), cloud);

      pcl_ros::transformPointCloud (cloud, cloud, tform_msg);
      whole_cloud += cloud;

      msg_queue_.pop();
    }

    // pcl::octree::OctreePointCloudSinglePoint<pcl::PointXYZ> octree (output_resolution_);
    // octree.setInputCloud (whole_cloud);
    // octree.addPointsFromInputCloud ();

    // pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    // sor.setInputCloud (&whole_cloud);
    // sor.setLeafSize (output_resolution_, output_resolution_, output_resolution_);
    // sor.filter (whole_cloud);
    
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(whole_cloud, output);
    output.header.frame_id = common_frame_id_;
    output.header.stamp = last_time;

    ROS_INFO("Publishing concatenated cloud (%d partial clouds, %d points)", queue_size_, output.width);

    pub_.publish(output);
    ros::spinOnce();
  }

  inline void getTransformFromTree(std::string parent_frame_id, std::string child_frame_id, tf::StampedTransform& tform_msg, ros::Time stamp=ros::Time(0), double timeout=0.1)
  {
    // tf::StampedTransform tform_msg;
    tf::TransformListener listener;

    if (parent_frame_id == child_frame_id)
    {
      ROS_WARN("No need to get transform for frame %s. Assuming Identity.", parent_frame_id.c_str());
      tform_msg.setIdentity();
      return;
    }

    try
    {
      listener.waitForTransform(parent_frame_id, child_frame_id, ros::Time::now(), ros::Duration(timeout));
      listener.lookupTransform(parent_frame_id, child_frame_id, stamp, tform_msg);
    }
    catch (tf::TransformException ex)
    {
      ROS_WARN("No link between %s and %s. Assuming Identity.", parent_frame_id.c_str(), child_frame_id.c_str());
      tform_msg.setIdentity();
    }

    // Eigen::Affine3d tform;
    // tf::transformTFToEigen(tf::Transform(tform_msg),tform);
    // (*tform_mat) = tform.matrix();
  }

private:
  // ros::NodeHandle& nh_;
  // ros::NodeHandle& private_nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  std::string termination_method_; // message_window or time_window
  int termination_value_; // number of messages or number of milliseconds
  int queue_size_; // max number of messages to store before we start dropping them
  std::string common_frame_id_; // all PCs will be transformed into this frame before concating to the whole PC
  double output_resolution_; // the whole cloud only stores unique points at this resolution
  int time_window_inconsistency_thresh_;

  std::queue<sensor_msgs::PointCloud2ConstPtr> msg_queue_;

  ros::Timer publishTimer_;
};

} // namespace pcl_ros_toolbox

PLUGINLIB_EXPORT_CLASS(pcl_ros_toolbox::MixedConcatenateServer, nodelet::Nodelet)
