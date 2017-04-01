#include <model_point.h>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#include <iostream>
#include <vector>
#include <ctime>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <Eigen/Dense>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/icp.h>
using namespace Eigen;
using namespace Eigen::internal;
using namespace Eigen::Architecture;

float get_Distance(float*** grid_value, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointXYZ &p1_key, pcl::PointXYZ &p2_key, const double min_x, const double min_y, const double min_z, const double max_x, const double max_y, const double max_z, const float resolution=0.02f)
{
	/*
	参数列表说明：
	grid_value为距离场 12*12*12
	cloud 为扫描点云一个关键点的占据网格中的点
	p1_key 为模型中一个关键点坐标
	p2_key 为扫描点云中一个关键点坐标
	min_x到max_z为tsdf的边界 即border中的6个值
	*/
	float x_transform = p2_key.x - p1_key.x;		//p2是扫面点云关键点
	float y_transform = p2_key.y - p1_key.y;
	float z_transform = p2_key.z - p1_key.z;
	Eigen::Matrix4f transform_mat = Eigen::Matrix4f::Identity();	//平移矩阵
	transform_mat(0, 3) = x_transform;
	transform_mat(1, 3) = y_transform;
	transform_mat(2, 3) = z_transform;
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::transformPointCloud(*cloud, *transformed_cloud, transform_mat);		//平移变换

	float distance_total = 100000000;				//最后总距离
	float distance_temp = 0;		//用于迭代36轮的每一轮的距离

	float theta = M_PI / 18; // The angle of rotation in radians

	int best_theta = 0;
	for (int i = 0; i < 36; i++)
	{
		//先旋转-
		Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();	//平移矩阵
		transform_1(0, 3) = -p1_key.x;
		transform_1(1, 3) = -p1_key.y;
		transform_1(2, 3) = -p1_key.z;
		pcl::transformPointCloud(*transformed_cloud, *transformed_cloud, transform_1);		//平移变换

		Eigen::Matrix4f transform_2 = Eigen::Matrix4f::Identity();	//旋转矩阵
		transform_2(0, 0) = cos(theta);
		transform_2(1, 0) = sin(theta);
		transform_2(0, 1) = -sin(theta);
		transform_2(1, 1) = cos(theta);
		pcl::transformPointCloud(*transformed_cloud, *transformed_cloud, transform_2);		//旋转

		transform_1(0, 3) = p1_key.x;
		transform_1(1, 3) = p1_key.y;
		transform_1(2, 3) = p1_key.z;
		pcl::transformPointCloud(*transformed_cloud, *transformed_cloud, transform_2);		//平移变换

																							//
		pcl::octree::OctreePointCloud<pcl::PointXYZ>octree_temp(resolution);
		octree_temp.defineBoundingBox(min_x, min_y, min_z, max_x, max_y, max_z);
		octree_temp.setInputCloud(transformed_cloud);
		octree_temp.addPointsFromInputCloud();
		std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> pointGrid;
		int num_center = octree_temp.getOccupiedVoxelCenters(pointGrid);			//得到旋转后存在点云体素的中心 存放在pointGrid中

		for (int p = 1; p < pointGrid.size(); p++)			//
		{
			int x = (cloud->points[p].x - min_x) / resolution;
			int y = (cloud->points[p].y - min_y) / resolution;
			int z = (cloud->points[p].z - min_z) / resolution;
			distance_temp += (grid_value[x][y][z] * grid_value[x][y][z] * grid_value[x][y][z] * grid_value[x][y][z]);
		}
		distance_temp /= (pointGrid.size() - 1);
		if (distance_total < distance_temp)
		{
			distance_total = distance_temp;
			best_theta = (i + 1)*theta;			//根据循环中的i确定旋转角度
		}
			

	}

	return distance_total;
}



