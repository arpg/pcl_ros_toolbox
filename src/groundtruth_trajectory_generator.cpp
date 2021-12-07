#include <pcl_ros_toolbox/groundtruth_trajectory_generator.h>

GroundtruthTrajectoryGenerator::GroundtruthTrajectoryGenerator(ros::NodeHandle& nh_) : nh(nh_)
{
    n.param<string>("bagfile", bagfile, std::string("darpa_final.bag").c_str());
    n.param<string>("cloud_topic", cloud_topic, std::string("horiz/os_cloud_node/points").c_str());
    n.param<string>("gt_cloud_topic", gt_cloud_topic, std::string("/darpa_final_cloud").c_str());
    n.param<string>("gt_path_topic", gt_path_topic, std::string("/darpa_final_path").c_str());

    icp_client = n.serviceClient<goicp::EstimateTransform>("/go_icp_server/estimate_transform");

    cloud_sub = nh.subscribe(cloud_topic, 1, cloudCallback, this);
    gt_cloud_sub = nh.subscribe(gt_cloud_topic, 1, gtCloudCallback, this);
    
    gt_path_pub = nh.advertise<geometry_msgs::Path>(gt_path_topic, 1, true);    
}

GroundtruthTrajectoryGenerator::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    cloud_msg = msg;
    processCloud();
}

GroundtruthTrajectoryGenerator::gtCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    gt_cloud_msg = msg;
}

GroundtruthTrajectoryGenerator::processCloud()
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
