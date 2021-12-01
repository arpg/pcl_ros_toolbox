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

class PclReframer : public nodelet::Nodelet
{
public:
  typedef pcl::PointXYZ PCPoint;
  typedef pcl::PointCloud<PCPoint> PC;

  inline virtual void onInit()
  {
    getPrivateNodeHandle().param("output_frame", output_frame_, std::string("world"));

    pub_ = getPrivateNodeHandle().advertise<sensor_msgs::PointCloud2>("output", 5);
    sub_ = getPrivateNodeHandle().subscribe("input", 5, &PclReframer::callback, this);
  };

  inline void callback(const sensor_msgs::PointCloud2ConstPtr& input)
  {
    // ROS_INFO("Received cloud, frame %s, size %d", input->header.frame_id.c_str(), input->width*input->height);

    tf::StampedTransform tform_msg;
    if(!getTransformFromTree(output_frame_, input->header.frame_id, tform_msg, input->header.stamp))
    {
      ROS_WARN("Tried to transform from tree but failed. Skipping msg.");
      return;
    }

    sensor_msgs::PointCloud2 output_msg;
    output_msg = *input;
    
    if (input->header.frame_id != output_frame_)
    {
      PC original_cloud, transformed_cloud;
      pcl::fromROSMsg (*input, original_cloud);
      
      Eigen::Matrix4f tform_mat;
      pcl_ros::transformAsMatrix(tform_msg, tform_mat);
      pcl::transformPointCloud(original_cloud, transformed_cloud, tform_mat);

      pcl::toROSMsg(transformed_cloud,output_msg);
    }
      
    output_msg.header.frame_id = output_frame_;
    output_msg.header.stamp = input->header.stamp;
    pub_.publish(output_msg);
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
  std::string output_frame_; 

  tf::TransformListener listener_;
};

} // namespace pcl_ros_toolbox

PLUGINLIB_EXPORT_CLASS(pcl_ros_toolbox::PclReframer, nodelet::Nodelet)
