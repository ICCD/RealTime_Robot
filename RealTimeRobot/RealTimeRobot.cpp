// RealTimeRobot.cpp : �������̨Ӧ�ó������ڵ㡣
//

#include "stdafx.h"
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h> 
#include <pcl/visualization/pcl_visualizer.h>
#include <cstdlib>
#include <pcl/keypoints/harris_3D.h>
#include <model_point.h>
#include <scan_point.h>
#include <matching.h>
#include <key_point.h>
#include <function.h>

using namespace pcl;
using namespace Eigen;
using namespace Eigen::internal;
using namespace Eigen::Architecture;


int main()
{
	/*
	
	//���ص���
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("11351.pcd", *cloud);
	pcl::PointCloud<pcl::PointXYZ>::Ptr mcloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("11351.pcd", *mcloud);

	//Eigen::Matrix4f transform_mat = Eigen::Matrix4f::Identity();	//ƽ�ƾ���
	//transform_mat(0, 3) = 0.8;
	//transform_mat(1, 3) = 0.8;
	//pcl::transformPointCloud(*mcloud, *mcloud, transform_mat);		//ƽ�Ʊ任
	
	//ʵ����ScanPoint���󣬴������
	ScanPoint scanpoint =  ScanPoint(cloud);

	scanpoint.get_Area(cloud);

	//ʵ����Model���󣬴������
	ModelPoint modelpoint = ModelPoint(mcloud);

 	pcl::PointCloud<pcl::PointXYZ>::Ptr mkeypoint(new pcl::PointCloud<pcl::PointXYZ>);
	*mkeypoint = modelpoint.getKeypoint();

	//��ȡ�ؼ���
	pcl::PointCloud<pcl::PointXYZ>::Ptr keypoint(new pcl::PointCloud<pcl::PointXYZ>);
	*keypoint = scanpoint.getKeypoint();
	keyPointICP(keypoint,mkeypoint,cloud,mcloud);
	vector<KeyPoint>  skeyPoint;
	skeyPoint.resize((*keypoint).points.size());
	int count = 0;//����
	for each (pcl::PointXYZ onepoint in *keypoint)
	{
		KeyPoint p = KeyPoint(onepoint);
		p.getOccupiedGrid(cloud);
		p.get_TSDF(cloud);
		//p.get_Vector3D(scanpoint.surface);
		skeyPoint[count++]  = p;
	}
	

	std::cout << "No Bug" << std::endl;
	int o = 0;
	while (true){
		o--;
	}
    return 0;

	*/
	clock_t start = clock();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("5.pcd", *cloud);

	ModelPoint modelpoint = ModelPoint(cloud);
	modelpoint.getArea(cloud);
	modelpoint.getKeypoint();

	int temp = 0;

	for (int i = 0; i < modelpoint.key_coordinates.size(); i++)
	{
		KeyPoint k_point(modelpoint.key_coordinates[i]);

		k_point.get_Vector3D(modelpoint.surface);
		for(int j=0;j<3;j++)
			if (k_point.vector3D[j] > 0.16)
			{
				temp++;
				break;
			}
		k_point.getOccupiedGrid(cloud);
		k_point.get_TSDF(cloud);
		modelpoint.keyPoint.push_back(k_point);
	}
	std::cout << modelpoint.key_coordinates.size() <<"              "<< temp << std::endl;

	

	clock_t ends = clock();
	cout << "Running Time : " << (double)(ends - start) / CLOCKS_PER_SEC << endl;

/*
	modelpoint.getArea(cloud);
	std::cout << modelpoint.surface.size() << std::endl;
	for (int i = 0; i < modelpoint.surface.size(); i++)
	{
		std::cout << modelpoint.surface[i].Area << "  " << modelpoint.surface[i].IsVertical << "aaaa" << std::endl;

	}


	modelpoint.getKeypoint();

	
		KeyPoint k_point(modelpoint.key_coordinates[9]);
		std::cout << "1" << std::endl;

		k_point.get_Vector3D(modelpoint.surface);
		std::cout << k_point.vector3D[0] << k_point.vector3D[1] << k_point.vector3D[2] << std::endl;


		k_point.getOccupiedGrid(cloud);								//����ռ������
		std::cout << k_point.Occupiedgrid.cloud->size() << std::endl;

		k_point.get_TSDF(cloud);

		int pp = 0;						//����������
		for (int i = 0; i < 12; i++)
			for (int j = 0; j < 12; j++)
				for (int k = 0; k < 12; k++)
					std::cout << k_point.grid_value[i][j][k] << std::endl;
*/
	

	return 0;

}

