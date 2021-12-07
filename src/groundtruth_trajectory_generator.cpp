#include <pcl_ros_toolbox/groundtruth_trajectory_generator.h>

GroundtruthTrajectoryGenerator::GroundtruthTrajectoryGenerator(ros::NodeHandle& nh_) : nh(nh_)
{
    nh.param<std::string>("bagfile", bagfile, std::string("darpa_final.bag").c_str());
    nh.param<std::string>("cloud_topic", cloud_topic, std::string("horiz/os_cloud_node/points").c_str());
    nh.param<std::string>("gt_cloud_topic", gt_cloud_topic, std::string("/darpa_final_cloud").c_str());
    nh.param<std::string>("gt_path_topic", gt_path_topic, std::string("/darpa_final_path").c_str());

    icp_client = nh.serviceClient<goicp::EstimateTransform>("/go_icp_server/estimate_transform");

    cloud_sub = nh.subscribe(cloud_topic, 1, &GroundtruthTrajectoryGenerator::cloudCallback, this);
    gt_cloud_sub = nh.subscribe(gt_cloud_topic, 1, &GroundtruthTrajectoryGenerator::gtCloudCallback, this);
    
    gt_path_pub = nh.advertise<nav_msgs::Path>(gt_path_topic, 1, true);    
}

void GroundtruthTrajectoryGenerator::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    cloud_msg = msg;
    processCloud();
}

void GroundtruthTrajectoryGenerator::gtCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    gt_cloud_msg = msg;
}

void GroundtruthTrajectoryGenerator::processCloud()
{
    if (!cloud_msg)
    {
        ROS_WARN("Cloud has not been received. Cannot process cloud. Skipping...");
        return;
    }
    if (!gt_cloud_msg)
    {
        ROS_WARN("Groundtruth cloud has not been received. Cannot process cloud. Skipping...");
        return;
    }
}
