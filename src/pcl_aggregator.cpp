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
#include <pcl/common/transforms.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/voxel_grid.h>

#include <boost/thread/mutex.hpp>

namespace pcl_ros_toolbox
{

class PclAggregator : public nodelet::Nodelet
{
public:
  typedef pcl::PointXYZI PCPoint;
  typedef pcl::PointCloud<PCPoint> PC;

  // MixedConcatenateServer();

  inline virtual void onInit()
  {
    // nh_ = getNodeHandle();
    // private_nh_ = getPrivateNodeHandle();
    
    getPrivateNodeHandle().param("duration_start", duration_start_, 0.0);
    getPrivateNodeHandle().param("duration_stop", duration_stop_, 100.0);
    getPrivateNodeHandle().param("aggregation_frequency", aggregation_frequency_, 1.0);
    getPrivateNodeHandle().param("publish_frequency", publish_frequency_, 1.0);
    getPrivateNodeHandle().param("common_frame_id", common_frame_id_, std::string("world"));
    getPrivateNodeHandle().param("voxelgrid_resolution", voxelgrid_resolution_, 0.0);

    if (duration_stop_ <= duration_start_)
    {
      ROS_WARN("duration_stop <= duration_start. Disabling duration time window. Beware large memory usage.");
      duration_start_ = -INFINITY;
      duration_stop_ = INFINITY;
    }

    pub_ = getPrivateNodeHandle().advertise<sensor_msgs::PointCloud2>("output", 5);
    sub_ = getPrivateNodeHandle().subscribe("input", 5, &PclAggregator::callback, this);

    publishTimer_ = getNodeHandle().createTimer(ros::Duration(1.0/publish_frequency_), boost::bind(&PclAggregator::publishLoop, this, _1));
    
    start_time_ = ros::Time::now();
  };

  inline ros::Time getTimeOfLastCloud(std::string frame) 
  {
    ros::Time time_of_last_cloud = ros::Time(0.0);

    if (msgs_.empty()) 
    {
      ROS_WARN("Tried to get time of last cloud for frame %s but msg array is empty.", frame.c_str());
      return time_of_last_cloud;
    }
    boost::mutex::scoped_lock lock(msgs_mtx_);
    
    for (uint i=0; i<msgs_.size(); i++) 
    {
      if (msgs_[i]->header.frame_id == frame)
      {
        time_of_last_cloud = msgs_[i]->header.stamp;
        break;
      } 
    }

    // ROS_INFO("Got time of last cloud from frame %s: %f.", frame.c_str(), time_of_last_cloud.toSec());
    return time_of_last_cloud;
  }

  inline void addMsgToArray(const sensor_msgs::PointCloud2ConstPtr& input)
  {
    boost::mutex::scoped_lock lock(msgs_mtx_);
    for (uint i=0; i<msgs_.size(); i++) 
    {
      if (msgs_[i]->header.frame_id == input->header.frame_id)
      {
        msgs_.erase(msgs_.begin()+i, msgs_.begin()+i+1);
        break;
      } 
    }
    msgs_.push_back(input);    
    // ROS_INFO("Added cloud with frame %s, time %f, size %d to msg array (now size %d).", input->header.frame_id.c_str(), input->header.stamp.toSec(), input->width*input->height, msgs_.size());
  }

  inline void callback(const sensor_msgs::PointCloud2ConstPtr& input)
  {
    // ROS_INFO("Received cloud, frame %s, size %d", input->header.frame_id.c_str(), input->width*input->height);

    ros::Time time_of_last_cloud = getTimeOfLastCloud(input->header.frame_id);
    if (time_of_last_cloud == ros::Time(0.0))
    {
      // first time with msg from this frame
      // go ahead and add it
      addMsgToArray(input);
    } else {
      // have gotten msg with this frame before
      // only add it if the appropriate amount of time has passeds
      double time_since_last_cloud = ros::Duration(input->header.stamp - time_of_last_cloud).toSec();
      // ROS_INFO("time since last cloud for frame %s: %f >=? %f", input->header.frame_id.c_str(), time_since_last_cloud, 1.0/aggregation_frequency_);
      if (time_since_last_cloud >= 1.0/aggregation_frequency_)
      {
        addMsgToArray(input);
      }
    }
  }

  inline void aggregateClouds()
  {
    if (msgs_.empty())
    {
      ROS_WARN("Tried to aggregate clouds but msg array is empty.");
      return;
    }
    boost::mutex::scoped_lock lock(msgs_mtx_);

    ROS_INFO("Aggregating clouds.");
    
    for (uint i=0; i<msgs_.size(); i++) 
    {
      tf::StampedTransform tform_msg;
      if(!getTransformFromTree(common_frame_id_, msgs_[i]->header.frame_id, tform_msg, msgs_[i]->header.stamp))
      {
        ROS_WARN("Tried to transform from tree but failed. Skipping msg.");
        continue;
      }
      
      PC original_cloud, transformed_cloud;
      pcl::fromROSMsg (*msgs_[i], original_cloud);
      
      Eigen::Matrix4f tform_mat;
      pcl_ros::transformAsMatrix(tform_msg, tform_mat);
      pcl::transformPointCloud(original_cloud, transformed_cloud, tform_mat);
      
      boost::mutex::scoped_lock lock(aggregate_cloud_mtx_);
      aggregate_cloud_ += transformed_cloud;
    }

    // ROS_INFO("Done aggregating clouds.");
  }

  inline void applyVoxelGrid()
  {
    ROS_INFO("Applying voxel grid filter.");

    pcl::VoxelGrid<PCPoint> voxel_filter;
    //PCLPointCloud voxel_filter;
    voxel_filter.setInputCloud (aggregate_cloud_.makeShared());
    voxel_filter.setLeafSize (voxelgrid_resolution_, voxelgrid_resolution_, voxelgrid_resolution_);
    voxel_filter.filter (aggregate_cloud_);
  }

  inline void publishCloud()
  {
    if (msgs_.empty())
    {
      ROS_WARN("Tried to publish clouds but msg array is empty.");
      return;
    }
    boost::mutex::scoped_lock lock(aggregate_cloud_mtx_);

    ROS_INFO("Publishing cloud of size %d to frame %s.", aggregate_cloud_.size(), common_frame_id_.c_str());

    sensor_msgs::PointCloud2 output_msg;
    pcl::toROSMsg(aggregate_cloud_,output_msg);
    output_msg.header.frame_id = common_frame_id_;
    output_msg.header.stamp = ros::Time::now();
    pub_.publish(output_msg);

    // aggregate_cloud_.clear();

    // ROS_INFO("Done publishing clouds.");
  }

  inline void publishLoop(const ros::TimerEvent& event)
  {
    if (start_time_==ros::Time(0.0))
      start_time_ = ros::Time::now();

    if (msgs_.empty())
    {
      ROS_WARN("Tried to run publish loop but msg array is empty.");
      return;
    }
    if (ros::Duration(ros::Time::now()-start_time_).toSec()<duration_start_ || ros::Duration(ros::Time::now()-start_time_).toSec()>duration_stop_)
    {
      ROS_WARN("Tried to run publish loop but out of configured time duration window: now %f, start time %f, dur start %f, dur stop %f", ros::Time::now().toSec(), start_time_.toSec(), duration_start_, duration_stop_);
      return;
    }

    ros::WallTime t_start = ros::WallTime::now();

    aggregateClouds();
    if (voxelgrid_resolution_>0.0)
      applyVoxelGrid();
    publishCloud();

    ros::WallTime t_stop = ros::WallTime::now();
    double actual_process_frequency = 1.0/(t_stop.toSec()-t_start.toSec());
    ROS_INFO("Actual process freq: %f, Desired publish freq: %f", actual_process_frequency, publish_frequency_);
  }

  inline bool getTransformFromTree(std::string parent_frame_id, std::string child_frame_id, tf::StampedTransform& tform_msg, ros::Time stamp=ros::Time(0), double timeout=1.0)
  {
    // tf::StampedTransform tform_msg;
    if (parent_frame_id == child_frame_id)
    {
      // ROS_WARN("No need to get transform for frame %s. Assuming Identity.", parent_frame_id.c_str());
      tform_msg.setIdentity();
      return true;
    }

    try
    {
      listener_.waitForTransform(parent_frame_id, child_frame_id, ros::Time::now(), ros::Duration(timeout));
      listener_.lookupTransform(parent_frame_id, child_frame_id, stamp, tform_msg);
    }
    catch (tf::TransformException ex)
    {
      ROS_WARN("No link between %s and %s.", parent_frame_id.c_str(), child_frame_id.c_str());
      tform_msg.setIdentity();
      return false;
    }

    // Eigen::Affine3d tform;
    // tf::transformTFToEigen(tf::Transform(tform_msg),tform);
    // (*tform_mat) = tform.matrix();

    return true;
  }

private:
  // ros::NodeHandle& nh_;
  // ros::NodeHandle& private_nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  std::string common_frame_id_; // all PCs will be transformed into this frame before concating to the whole PC
  double aggregation_frequency_, publish_frequency_;
  ros::Time time_of_last_aggregation_;

  boost::mutex msgs_mtx_, aggregate_cloud_mtx_;
  std::vector<sensor_msgs::PointCloud2ConstPtr> msgs_;

  ros::Time start_time_;
  double duration_start_, duration_stop_;

  PC aggregate_cloud_;

  ros::Timer publishTimer_, aggregationTimer_;

  tf::TransformListener listener_;
  
  double voxelgrid_resolution_;
};

} // namespace pcl_ros_toolbox

PLUGINLIB_EXPORT_CLASS(pcl_ros_toolbox::PclAggregator, nodelet::Nodelet)
