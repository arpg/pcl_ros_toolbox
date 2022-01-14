#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

typedef struct _POINT3D
{
	float x, y, z;
} POINT3D;

static inline bool LoadPOINT3DFromFile(string FName, int & N, POINT3D ** p)
{
	int i;
	ifstream ifile;

	ifile.open(FName.c_str(), ifstream::in);
	if(!ifile.is_open())
	{
		cout << "Unable to open point file '" << FName << "'" << endl;
		return 0;
	}
	ifile >> N; // First line has number of points to follow
	*p = (POINT3D *)malloc(sizeof(POINT3D) * N);
	for(i = 0; i < N; i++)
	{
		ifile >> (*p)[i].x >> (*p)[i].y >> (*p)[i].z;
	}

	ifile.close();

	return 1;
}

static inline bool LoadPOINT3DFromFileAndConvertToPointCloud2(std::string input_file, sensor_msgs::PointCloud2& cloud_msg)
{
	ROS_INFO("Loading pointcloud from file.");

	POINT3D* cloud3d;
	int num_points;
	if(!LoadPOINT3DFromFile (input_file, num_points, &cloud3d))
		return 0;

	ROS_INFO("Loaded pointcloud with %d points.",num_points);
	ROS_INFO("Converting pointcloud to pcl.");

	pcl::PointCloud<pcl::PointXYZ> cloud;
	for (uint i=0; i<num_points; i++)
	{
		cloud.points.push_back(pcl::PointXYZ((cloud3d+i)->x,(cloud3d+i)->y,(cloud3d+i)->z));
	}

	ROS_INFO("Converting pointcloud to ros msg.");

    // sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);

	return 1;
}