#include <pcl_ros_toolbox/groundtruth_trajectory_generator.h>

GroundtruthTrajectoryGenerator::GroundtruthTrajectoryGenerator(ros::NodeHandle& nh_) : nh(nh_)
{
    nh.param<std::string>("input_bagfile", input_bagfile, std::string("darpa_final_cloud.bag").c_str());
    nh.param<std::string>("cloud_topic", cloud_topic, std::string("cloud").c_str());
    nh.param<bool>("output_cloud", output_cloud, true);
    nh.param<std::string>("gt_input_pcdfile", gt_input_pcdfile, std::string("darpa_final_gt_cloud.bag").c_str());
    nh.param<std::string>("gt_cloud_frame", gt_cloud_frame, std::string("map").c_str());
    nh.param<bool>("output_gt_cloud", output_gt_cloud, true);
    nh.param<std::string>("gt_cloud_topic", gt_cloud_topic, std::string("gt_cloud").c_str());
    nh.param<std::string>("gt_path_topic", gt_path_topic, std::string("darpa_final_gt_path").c_str());
    nh.param<std::string>("output_bagfile", output_bagfile, std::string("darpa_final_gt_path.bag").c_str());
    
    nh.param<float>("cloud_throttle_rate", cloud_throttle_rate, 0.f); // rate at which input clouds are throttled, to improve performance / reduce calls to goicp. 0 is unthrottled.
    nh.param<bool>("require_tfs", require_tfs, false);
    nh.param<bool>("publish_path", publish_path, false);

    if (publish_path)
        path_pub = nh.advertise<nav_msgs::Path>("gt_path", 1);

    std::string icp_client_topic;
    nh.param<std::string>("icp_client_topic", icp_client_topic, std::string("/go_icp_server/estimate_transform").c_str());
    icp_client = nh.serviceClient<goicp::EstimateTransform>(icp_client_topic);

    Run();
}

GroundtruthTrajectoryGenerator::GroundtruthTrajectoryGenerator(GroundtruthTrajectoryGenerator& rhs)
{

}

void GroundtruthTrajectoryGenerator::Run()
{
    ReadInputs();
    while (ros::ok())
    {
        GetTrajectory();
        WriteOutputs();
        if (publish_path)
        {
            ROS_INFO("Publishing path...");
            path_pub.publish(gt_path);
        }
        ROS_INFO("Run Complete!");
        ros::Rate(1).sleep();
    }
}


void GroundtruthTrajectoryGenerator::ReadInputs()
{
    double startTime = pcl::getTime();

    ROS_INFO("Loading tf tree from bag...");
    if (!LoadTfsFromBag(input_bagfile, tf_buffer))
        ROS_ERROR("Error loading tf tree.");
    // ROS_INFO("tf has frames: %s", tf_buffer->allFramesAsString().c_str());
    // std::vector<std::string> tf_strings;
    // tf_buffer._getFrameStrings(tf_strings);
    // for (uint i=0; i<tf_strings.size(); i++)
    // {
    //     ROS_INFO("tf has string %s", tf_strings[i].c_str());
    // }
    // if (tf_strings.size()<1)
    // {
    //     if (require_tfs)
    //     {
    //         ROS_ERROR("Tf_buffer unpopulated. Skipping groundtruth trajectory generation...");
    //         return;
    //     }
    //     else
    //     {
    //         ROS_WARN("Tf_buffer unpopulated but tfs not required. Assuming Identity...");
    //     }
    // }

    ROS_INFO("Loading model cloud from pcd...");
    // LoadPointCloud2sFromBag(gt_input_bagfile, std::vector<std::string>{gt_cloud_topic}, model_msgs);
    sensor_msgs::PointCloud2 model_msg;
    LoadPcd(gt_input_pcdfile, model_msg);
    model_msg.header.frame_id = gt_cloud_frame;
    model_msg.header.stamp = ros::Time::now();
    model_msgs.push_back(model_msg);

    ROS_INFO("Loading data clouds from bag...");
    LoadPointCloud2sFromBag(input_bagfile, std::vector<std::string>{cloud_topic}, data_msgs, cloud_throttle_rate);

    double stopTime = pcl::getTime();
    ROS_INFO("Loading took %f sec.", stopTime-startTime);
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

    ROS_INFO("Starting Trajectory Generation...");

    goicp::EstimateTransform srv;
	srv.request.model_cloud = model_msgs[0];

    // geometry_msgs::TransformStamped initial_tform_inv;
    // initial_tform_inv.header = "unassigned";
    for (uint i=0; i<data_msgs.size(); i++)
    {
	    srv.request.data_cloud = data_msgs[i];

        geometry_msgs::TransformStamped init_tform;
        try
        {
            init_tform = tf_buffer->lookupTransform(srv.request.model_cloud.header.frame_id, srv.request.data_cloud.header.frame_id, srv.request.data_cloud.header.stamp); // set initial tform to tform between model to data
        }
        catch (tf2::TransformException &ex) 
        {
            // ROS_WARN("%s",ex.what());
            if (require_tfs)
            {
                ROS_WARN("Failed to lookup transform %s->%s:\n%s\nSkipping cloud %d/%d...", srv.request.data_cloud.header.frame_id.c_str(), srv.request.model_cloud.header.frame_id.c_str(), ex.what(), i+1, data_msgs.size());
                // ros::Duration(0.01).sleep();
                continue;
            }
            else
            {
                // tf lookup failed but not required so just send identity
                init_tform.transform.translation.x = 0.f;
                init_tform.transform.translation.y = 0.f;
                init_tform.transform.translation.z = 0.f;
                init_tform.transform.rotation.x = 0.f;
                init_tform.transform.rotation.y = 0.f;
                init_tform.transform.rotation.z = 0.f;
                init_tform.transform.rotation.w = 1.f;
            }
        }
        srv.request.init_tform = init_tform.transform;

        if(icp_client.call(srv))
        {
            geometry_msgs::Transform tform = srv.response.tform;
            ROS_INFO("Client received %d/%d transform (%s->%s): %f %f %f %f %f %f %f", i+1, data_msgs.size(), srv.request.data_cloud.header.frame_id.c_str(), srv.request.model_cloud.header.frame_id.c_str(), tform.translation.x, tform.translation.y, tform.translation.z, tform.rotation.x, tform.rotation.y, tform.rotation.z, tform.rotation.w);
            // if (strcmp(initial_tform_inv.header,"unassigned"))
            // {
            //     initial_tform_inv = tform.inverse();
            // }
            // service return data->model, but the path we wish to generate is model->data
            float qlen2 = pow(tform.rotation.x,2)+pow(tform.rotation.y,2)+pow(tform.rotation.z,2)+pow(tform.rotation.w,2);
            geometry_msgs::PoseStamped tform_posestamped;
            tform_posestamped.header.frame_id = srv.request.model_cloud.header.frame_id;
            tform_posestamped.header.stamp = srv.request.data_cloud.header.stamp;
            tform_posestamped.pose.position.x = -tform.translation.x;
            tform_posestamped.pose.position.y = -tform.translation.y;
            tform_posestamped.pose.position.z = -tform.translation.z;
            tform_posestamped.pose.orientation.x = -tform.rotation.x/qlen2;
            tform_posestamped.pose.orientation.y = -tform.rotation.y/qlen2;
            tform_posestamped.pose.orientation.z = -tform.rotation.z/qlen2;
            tform_posestamped.pose.orientation.w = tform.rotation.w/qlen2;
            gt_path.poses.push_back(tform_posestamped);
        }
        else
        {
            ROS_ERROR("Failed to call '%s' service.", icp_client.getService().c_str());
        }
        // ros::Rate(1./ros::Duration(0.01)).sleep();
    }
    // if (gt_path.poses.size()>0)
    //     gt_path.header = gt_path.poses.back().header;
    gt_path.header.frame_id = srv.request.model_cloud.header.frame_id;
    // gt_path.header.stamp = gt_path.poses.back().frame_id;

    ROS_INFO("Completed Trajectory Generation with path of length %d.", gt_path.poses.size());
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

    if (output_gt_cloud)
    {
        ROS_INFO("Writing gt cloud to bag...");
        outbag.write(gt_cloud_topic, ros::Time::now(), model_msgs[0]);
    }
    if (output_cloud)
    {
        ROS_INFO("Writing data clouds to bag...");
        for (uint i=0; i<data_msgs.size(); i++)
        {
            outbag.write(cloud_topic, ros::Time::now(), data_msgs[i]);
        }
    }
    ROS_INFO("Writing path to bag...");
    outbag.write(gt_path_topic, ros::Time::now(), gt_path);

    outbag.close();
}