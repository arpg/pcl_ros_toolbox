#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <pcl_conversions/pcl_conversions.h>
#include <goicp/EstimateTransform.h>
#include <pcl_ros_toolbox/load_pointcloud2.hpp>

class GroundtruthTrajectoryGenerator
{
public:
	GroundtruthTrajectoryGenerator(ros::NodeHandle&);
	void Run();
	void ReadInputs();
	void GetTrajectory();
	void WriteOutputs();
private:
	ros::NodeHandle nh;
	std::string input_bagfile, cloud_topic, gt_input_bagfile, gt_cloud_topic, gt_path_topic, output_bagfile;
	ros::ServiceClient icp_client;
    tf::TransformListener tf_listener;
	std::vector<sensor_msgs::PointCloud2> model_msgs, data_msgs;
	nav_msgs::Path gt_path;
};