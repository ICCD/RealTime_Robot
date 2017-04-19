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

float get_Distance(Eigen::Matrix4f &key_transform,float grid_value[12][12][12], pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ &p1_key, pcl::PointXYZ &p2_key, const double min_x, const double min_y, const double min_z, const double max_x, const double max_y, const double max_z, const float resolution=0.02f)
{
	/*
	�����б�˵����
	grid_valueΪ���볡 12*12*12
	cloud Ϊɨ�����һ���ؼ����ռ�������еĵ�
	p1_key Ϊģ����һ���ؼ�������
	p2_key Ϊɨ�������һ���ؼ�������
	min_x��max_zΪtsdf�ı߽� ��border�е�6��ֵ
	*/
	

	float x_transform = p1_key.x - p2_key.x;		//p2��ɨ����ƹؼ���
	float y_transform = p1_key.y - p2_key.y;
	float z_transform = p1_key.z - p2_key.z;
	Eigen::Matrix4f transform_mat = Eigen::Matrix4f::Identity();	//ƽ�ƾ���
	transform_mat(0, 3) = x_transform;
	transform_mat(1, 3) = y_transform;
	transform_mat(2, 3) = z_transform;
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::transformPointCloud(*cloud, *transformed_cloud, transform_mat);		//ƽ�Ʊ任��ʹ�����ؼ����غ�

	float distance_total = 100000000;				//����ܾ���
	float distance_temp = 0;		//���ڵ���36�ֵ�ÿһ�ֵľ���

	float theta = M_PI / 18; // The angle of rotation in radians

	double best_theta = 0;

	Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();	//ƽ�ƾ���1��ʹռ������ƽ�Ƶ�ԭ�㣬��ԭ�������ת
	transform_1(0, 3) = -p1_key.x;
	transform_1(1, 3) = -p1_key.y;
	

	Eigen::Matrix4f transform_2 = Eigen::Matrix4f::Identity();	//��ת����ÿ����ת10��
	transform_2(0, 0) = cos(theta);
	transform_2(1, 0) = sin(theta);
	transform_2(0, 1) = -sin(theta);
	transform_2(1, 1) = cos(theta);

	Eigen::Matrix4f transform_3 = Eigen::Matrix4f::Identity();	//ƽ�ƾ�����ռ����������ת���ԭ��
	transform_3(0, 3) = p1_key.x;
	transform_3(1, 3) = p1_key.y;
	

	for (int i = 0; i < 36; i++)
	{
		//����ת-
		
		pcl::transformPointCloud(*transformed_cloud, *transformed_cloud, transform_3*transform_2*transform_1);		//ƽ�ƣ���ת��ƽ��

		pcl::octree::OctreePointCloud<pcl::PointXYZ>octree_temp(resolution);
		octree_temp.defineBoundingBox(min_x, min_y, min_z, max_x, max_y, max_z);
		octree_temp.setInputCloud(transformed_cloud);
		octree_temp.addPointsFromInputCloud();
		std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> pointGrid;
		int num_center = octree_temp.getOccupiedVoxelCenters(pointGrid);			//�õ���ת����ڵ������ص����� �����pointGrid��
		

		

		for (int p = 1; p < pointGrid.size(); p++)			//ע��p�Ǵ�1��ʼ��
		{
			int x = (pointGrid[p].x - min_x) / resolution;
			int y = (pointGrid[p].y - min_y) / resolution;
			int z = (pointGrid[p].z - min_z) / resolution;
			//cout << "grid_value" << grid_value[x][y][z] << endl;
			//cout << x << "    " << y << "    " << z << "    " << grid_value[x][y][z] << endl;
			distance_temp += (grid_value[x][y][z] * grid_value[x][y][z] * grid_value[x][y][z] * grid_value[x][y][z]);
		}
		distance_temp /= (pointGrid.size() - 1);
		cout << distance_temp << "distance_temp" << i << endl;

		if (distance_temp < distance_total)
		{
			distance_total = distance_temp;
			best_theta = (i + 1);			//����ѭ���е�iȷ����ת�Ƕ�
			//cout << distance_total << "distance_total" << endl;
		}
			

	}

	

	Eigen::Matrix4f transform2 = Eigen::Matrix4f::Identity();	//��ת(��һ���ص�ԭ��Ĺ���)
	transform2(0, 0) = cos(best_theta);
	transform2(1, 0) = sin(best_theta);
	transform2(0, 1) = -sin(best_theta);
	transform2(1, 1) = cos(best_theta);

	Eigen::Matrix4f transform3 = Eigen::Matrix4f::Identity();	//�߶�����
	float change_size = p1_key.z / p2_key.z;
	transform3(0, 0) = change_size;
	transform3(1, 1) = change_size;
	transform3(2, 2) = change_size;

	
	key_transform = transform3*transform_3*transform2*transform_1*transform_mat;		//�����ؼ���֮��ı任����


	//cout << "best_theta" << best_theta << endl;
	return distance_total;
}



