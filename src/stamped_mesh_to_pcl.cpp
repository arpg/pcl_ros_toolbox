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

#include <mesh_msgs/TriangleMeshStamped.h>

namespace pcl_ros_toolbox
{

class StampedMeshToPcl : public nodelet::Nodelet
{
public:

  inline virtual void onInit()
  {
    pub_ = getPrivateNodeHandle().advertise<sensor_msgs::PointCloud2>("output", 5);
    sub_ = getPrivateNodeHandle().subscribe("input", 1, &StampedMeshToPcl::callback, this);
  };

  inline void callback(const mesh_msgs::TriangleMeshStamped::ConstPtr& input)
  {
    // ROS_INFO("Received input with size %d", input->mesh.vertices.size());

    pcl::PointCloud<pcl::PointXYZ> pc;
    for (int i=0; i<input->mesh.vertices.size(); i++)
    {
        pcl::PointXYZ newPoint;
        newPoint.x = input->mesh.vertices[i].x;
        newPoint.y = input->mesh.vertices[i].y;
        newPoint.z = input->mesh.vertices[i].z;
        pc.points.push_back(newPoint);
    }
    sensor_msgs::PointCloud2 pc_msg;
    pcl::toROSMsg(pc, pc_msg);
    pc_msg.header = input->header;
    pub_.publish(pc_msg);
  }


  // inline void getTransformFromTree(std::string parent_frame_id, std::string child_frame_id, tf::StampedTransform& tform_msg, ros::Time stamp=ros::Time(0), double timeout=0.1)
  // {
  //   // tf::StampedTransform tform_msg;
  //   tf::TransformListener listener;

  //   if (parent_frame_id == child_frame_id)
  //   {
  //     ROS_WARN("No need to get transform for frame %s. Assuming Identity.", parent_frame_id.c_str());
  //     tform_msg.setIdentity();
  //     return;
  //   }

  //   try
  //   {
  //     listener.waitForTransform(parent_frame_id, child_frame_id, ros::Time::now(), ros::Duration(timeout));
  //     listener.lookupTransform(parent_frame_id, child_frame_id, stamp, tform_msg);
  //   }
  //   catch (tf::TransformException ex)
  //   {
  //     ROS_WARN("No link between %s and %s. Assuming Identity.", parent_frame_id.c_str(), child_frame_id.c_str());
  //     tform_msg.setIdentity();
  //   }

  //   // Eigen::Affine3d tform;
  //   // tf::transformTFToEigen(tf::Transform(tform_msg),tform);
  //   // (*tform_mat) = tform.matrix();
  // }

private:
  // ros::NodeHandle& nh_;
  // ros::NodeHandle& private_nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
};

} // namespace pcl_ros_toolbox

PLUGINLIB_EXPORT_CLASS(pcl_ros_toolbox::StampedMeshToPcl, nodelet::Nodelet)
