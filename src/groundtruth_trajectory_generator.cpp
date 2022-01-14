#include <pcl_ros_toolbox/groundtruth_trajectory_generator.h>

GroundtruthTrajectoryGenerator::GroundtruthTrajectoryGenerator(ros::NodeHandle& nh_) : nh(nh_)
{
    nh.param<std::string>("input_bagfile", input_bagfile, std::string("darpa_final_cloud.bag").c_str());
    nh.param<std::string>("cloud_topic", cloud_topic, std::string("horiz/os_cloud_node/points").c_str());
    nh.param<std::string>("gt_input_bagfile", gt_input_bagfile, std::string("darpa_final_gt_cloud.bag").c_str());
    nh.param<std::string>("gt_cloud_topic", gt_cloud_topic, std::string("darpa_final_gt_cloud").c_str());
    nh.param<std::string>("gt_path_topic", gt_path_topic, std::string("darpa_final_gt_path").c_str());
    nh.param<std::string>("output_bagfile", output_bagfile, std::string("darpa_final_gt_path.bag").c_str());

    std::string icp_client_topic;
    nh.param<std::string>("icp_client_topic", icp_client_topic, std::string("/go_icp_server/estimate_transform").c_str());
    icp_client = nh.serviceClient<goicp::EstimateTransform>(icp_client_topic);

    Run();
}

void GroundtruthTrajectoryGenerator::Run()
{
    ReadInputs();
    GetTrajectory();
    WriteOutputs();
}


void GroundtruthTrajectoryGenerator::ReadInputs()
{
    // std::vector<sensor_msgs::PointCloud2> model_msgs_;
    LoadPointCloud2sFromBag(gt_input_bagfile, std::vector<std::string>{gt_cloud_topic}, model_msgs);

    // std::vector<sensor_msgs::PointCloud2> data_msgs_;
    LoadPointCloud2sFromBag(input_bagfile, std::vector<std::string>{cloud_topic}, data_msgs);
}

void GroundtruthTrajectoryGenerator::GetTrajectory()
{
    if (model_msgs.size()<1)
    {
        ROS_ERROR("Model_msgs unpopulated. Skipping groundtruth trajectory generation...");
        return;
    }
    if (data_msgs.size()<1)
    {
        ROS_ERROR("Data_msgs unpopulated. Skipping groundtruth trajectory generation...");
        return;
    }
    if (model_msgs.size()>1)
    {
        ROS_ERROR("Model_msgs is size %d but only size 1 is supported. Skipping groundtruth trajectory generation...", model_msgs.size());
        return;
    }

    goicp::EstimateTransform srv;
	srv.request.model_cloud = model_msgs[0];

    // geometry_msgs::TransformStamped initial_tform_inv;
    // initial_tform_inv.header = "unassigned";
    for (uint i=0; i<data_msgs.size(); i++)
    {
	    srv.request.data_cloud = data_msgs[i];

        if(icp_client.call(srv))
        {
            geometry_msgs::TransformStamped tform = srv.response.tform;
            ROS_INFO("Client received transform: %f %f %f %f %f %f %f", tform.transform.translation.x, tform.transform.translation.y, tform.transform.translation.z, tform.transform.rotation.w, tform.transform.rotation.x, tform.transform.rotation.y, tform.transform.rotation.z);
            // if (strcmp(initial_tform_inv.header,"unassigned"))
            // {
            //     initial_tform_inv = tform.inverse();
            // }
            geometry_msgs::Point tform_pt;
            tform_pt.x = tform.transform.translation.x;
            tform_pt.y = tform.transform.translation.y;
            tform_pt.z = tform.transform.translation.z;
            geometry_msgs::Quaternion tform_quat;
            tform_quat.x = tform.transform.rotation.x;
            tform_quat.y = tform.transform.rotation.y;
            tform_quat.z = tform.transform.rotation.z;
            tform_quat.w = tform.transform.rotation.w;
            geometry_msgs::Pose tform_pose;
            tform_pose.position = tform_pt;
            tform_pose.orientation = tform_quat;
            geometry_msgs::PoseStamped tform_posestamped;
            tform_posestamped.header = tform.header;
            tform_posestamped.pose = tform_pose;
            gt_path.poses.push_back(tform_posestamped);
        }
        else
        {
            ROS_ERROR("Failed to call '%s' service.", icp_client.getService().c_str());
        }
        // ros::Rate(1./ros::Duration(0.01)).sleep();
    }
}

void GroundtruthTrajectoryGenerator::WriteOutputs()
{
    if (gt_path.poses.size()<1)
    {
        ROS_ERROR("Groundtruth path unpopulated. Skipping output bag writing...");
        return;
    }

    rosbag::Bag outbag;
    outbag.open(output_bagfile, rosbag::bagmode::Write);

    // for(rosbag::MessageInstance const m: rosbag::View(inbag))
    // {
        // sensor_msgs::PointCloud2::ConstPtr cloud_msg = m.instantiate<sensor_msgs::PointCloud2>();
        // if (cloud_msg != NULL)
        //     ROS_INFO("on cloud");
    // }

    outbag.close();
}