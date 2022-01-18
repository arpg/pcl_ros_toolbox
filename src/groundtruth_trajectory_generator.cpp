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
    ROS_INFO("Run Complete!");
}


void GroundtruthTrajectoryGenerator::ReadInputs()
{
    ROS_INFO("Loading model cloud from bag...");
    LoadPointCloud2sFromBag(gt_input_bagfile, std::vector<std::string>{gt_cloud_topic}, model_msgs);

    ROS_INFO("Loading data clouds from bag...");
    LoadPointCloud2sFromBag(input_bagfile, std::vector<std::string>{cloud_topic}, data_msgs);

    ROS_INFO("Loading tf tree from bag...");
    LoadTfsFromBag(input_bagfile, tf_buffer);
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
    std::vector<std::string> tf_strings;
    tf_buffer._getFrameStrings(tf_strings);
    if (tf_strings.size()<1)
    {
        ROS_ERROR("Tf_buffer unpopulated. Skipping groundtruth trajectory generation...");
        return;
    }

    ROS_INFO("Starting Trajectory Generation...");

    goicp::EstimateTransform srv;
	srv.request.model_cloud = model_msgs[0];

    // geometry_msgs::TransformStamped initial_tform_inv;
    // initial_tform_inv.header = "unassigned";
    for (uint i=0; i<data_msgs.size(); i++)
    {
	    srv.request.data_cloud = data_msgs[i];

        try
        {
            geometry_msgs::TransformStamped init_tform = tf_buffer.lookupTransform(srv.request.model_cloud.header.frame_id, srv.request.data_cloud.header.frame_id, srv.request.data_cloud.header.stamp); // set initial tform to tform between model to data
            srv.request.init_tform = init_tform.transform;
        }
        catch (tf2::TransformException &ex) 
        {
            // ROS_WARN("%s",ex.what());
            ROS_WARN("Failed to lookup transform %s->%s. Skipping cloud %d/%d...", i+1, data_msgs.size(), srv.request.data_cloud.header.frame_id, srv.request.model_cloud.header.frame_id);
            ros::Duration(0.01).sleep();
            continue;
        }

        if(icp_client.call(srv))
        {
            geometry_msgs::Transform tform = srv.response.tform;
            ROS_INFO("Client received %d/%d transform (data->model): %f %f %f %f %f %f %f", i+1, data_msgs.size(), tform.translation.x, tform.translation.y, tform.translation.z, tform.rotation.w, tform.rotation.x, tform.rotation.y, tform.rotation.z);
            // if (strcmp(initial_tform_inv.header,"unassigned"))
            // {
            //     initial_tform_inv = tform.inverse();
            // }
            geometry_msgs::Point tform_pt;
            tform_pt.x = tform.translation.x;
            tform_pt.y = tform.translation.y;
            tform_pt.z = tform.translation.z;
            geometry_msgs::Quaternion tform_quat;
            tform_quat.x = tform.rotation.x;
            tform_quat.y = tform.rotation.y;
            tform_quat.z = tform.rotation.z;
            tform_quat.w = tform.rotation.w;
            geometry_msgs::Pose tform_pose;
            tform_pose.position = tform_pt;
            tform_pose.orientation = tform_quat;
            geometry_msgs::PoseStamped tform_posestamped;
            tform_posestamped.header.frame_id = srv.request.model_cloud.header.frame_id;
            tform_posestamped.header.stamp = srv.request.data_cloud.header.stamp;
            tform_posestamped.pose = tform_pose;
            gt_path.poses.push_back(tform_posestamped);
        }
        else
        {
            ROS_ERROR("Failed to call '%s' service.", icp_client.getService().c_str());
        }
        // ros::Rate(1./ros::Duration(0.01)).sleep();
    }
    // gt_path.header = gt_path.poses.back().header;
    gt_path.header.frame_id = "map";
    gt_path.header.stamp = ros::Time::now();

    ROS_INFO("Completed Trajectory Generation with path of length %d.", gt_path.poses.size());
}

void GroundtruthTrajectoryGenerator::WriteOutputs()
{
    if (gt_path.poses.size()<1)
    {
        ROS_ERROR("Groundtruth path unpopulated. Skipping output bag writing...");
        return;
    }

    ROS_INFO("Writing path to bag...");

    rosbag::Bag outbag;
    outbag.open(output_bagfile, rosbag::bagmode::Write);

    outbag.write(gt_path_topic, ros::Time::now(), gt_path);

    outbag.close();
}