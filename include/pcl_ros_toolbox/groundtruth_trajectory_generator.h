#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>
#include <pcl_conversions/pcl_conversions.h>
#include <goicp/EstimateTransform.h>

class GroundtruthTrajectoryGenerator
{
public:
	GroundtruthTrajectoryGenerator(ros::NodeHandle& );
	void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& );
	void gtCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& );
	void processCloud();
private:
	ros::NodeHandle nh;
	std::string bagfile, cloud_topic, gt_cloud_topic, gt_path_topic;
	ros::ServiceClient icp_client;
    ros::Subscriber cloud_sub, gt_cloud_sub;
    ros::Publisher gt_path_pub;
    tf::TransformListener tf_listener;
	sensor_msgs::PointCloud2::ConstPtr gt_cloud_msg, cloud_msg;
};