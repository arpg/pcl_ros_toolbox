#include <ros/ros.h>
#include <pcl_ros_toolbox/groundtruth_trajectory_generator.h>

int main (int argc, char *argv[])
{
    ros::init(argc, argv, "groundtruth_trajectory_generator");
    ros::NodeHandle nh("~");

	GroundtruthTrajectoryGenerator gtg(nh);

    ros::spin();
	return 0;
}