#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>
#include <pcl_conversions/pcl_conversions.h>
#include <goicp/EstimateTransform.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

class GroundtruthTrajectoryGenerator
{
public:
	GroundtruthTrajectoryGenerator(ros::NodeHandle& );
	void processBag();
private:
	ros::NodeHandle nh;
	std::string input_bagfile, output_bagfile, cloud_topic, gt_input_bagfile, gt_path_topic;
	ros::ServiceClient icp_client;
    tf::TransformListener tf_listener;
};