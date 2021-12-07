#include <pcl_ros_toolbox/groundtruth_trajectory_generator.h>

GroundtruthTrajectoryGenerator::GroundtruthTrajectoryGenerator(ros::NodeHandle& nh_) : nh(nh_)
{
    nh.param<std::string>("input_bagfile", input_bagfile, std::string("darpa_final.bag").c_str());
    nh.param<std::string>("cloud_topic", cloud_topic, std::string("horiz/os_cloud_node/points").c_str());
    nh.param<std::string>("gt_input_bagfile", gt_input_bagfile, std::string("/darpa_final_gt_cloud").c_str());
    nh.param<std::string>("gt_path_topic", gt_path_topic, std::string("/darpa_final_path").c_str());
    nh.param<std::string>("output_bagfile", output_bagfile, std::string("darpa_final_path.bag").c_str());

    icp_client = nh.serviceClient<goicp::EstimateTransform>("/go_icp_server/estimate_transform");

    processBag();
}

void GroundtruthTrajectoryGenerator::processBag()
{
    rosbag::Bag inbag;
    inbag.open(input_bagfile, rosbag::bagmode::Read);

    for(rosbag::MessageInstance const m: rosbag::View(inbag))
    {
        sensor_msgs::PointCloud2::ConstPtr cloud_msg = m.instantiate<sensor_msgs::PointCloud2>();
        if (cloud_msg != NULL)
            ROS_INFO("on cloud");
    }

    inbag.close();
}
