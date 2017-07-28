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
#include"stdlib.h"
#include <time.h>
using namespace std;
using namespace pcl;
using namespace Eigen;
using namespace Eigen::internal;
using namespace Eigen::Architecture;


int main()
{
	

	//���ص���
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr mcloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("T0_m8111.pcd", *cloud);
	pcl::io::loadPCDFile("chair1.pcd", *mcloud);


	//ʵ����Model���󣬴������
	ModelPoint modelpoint = ModelPoint(mcloud);
	modelpoint.getKeypoint();
	modelpoint.getArea(mcloud);

	clock_t start = clock();		//��ʱ��ʼ
	//ʵ����ScanPoint���󣬴������
	ScanPoint scanpoint = ScanPoint(cloud);
	scanpoint.getKeypoint();
	scanpoint.get_Area(cloud);
	//keyPointICP(keypoint,mkeypoint,cloud,mcloud);/*����չʾ�ú���*/
	scanpoint.keyPoint.resize((scanpoint.key_coordinates).points.size());
	modelpoint.keyPoint.resize((modelpoint.key_coordinates).points.size());
	int count = 0;//����
	for each (pcl::PointXYZ onepoint in scanpoint.key_coordinates)
	{
		KeyPoint p = KeyPoint(onepoint);
		p.getOccupiedGrid(cloud);
		//p.get_TSDF(cloud);
		p.get_Vector3D(scanpoint.surface);
		scanpoint.keyPoint[count++] = p;
		//std::cout << count << std::endl;
	}
	count = 0;
	for each (pcl::PointXYZ onepoint in modelpoint.key_coordinates)
	{
		KeyPoint p = KeyPoint(onepoint);
		p.getOccupiedGrid(mcloud);
		p.get_TSDF(mcloud);
		p.get_Vector3D(modelpoint.surface);
		modelpoint.keyPoint[count++] = p;
	}
	vector<PairPoint> pairpoint;
	int point_num = 0;
	int pp = 0;
	for each (KeyPoint k in modelpoint.keyPoint)
	{
		//int singlePointPair = 0;
		for each (KeyPoint sk in scanpoint.keyPoint)
		{
			/*cout << "111Occu:" << k.Occupiedgrid.Number << "  222Occu:" << sk.Occupiedgrid.Number << endl;
			cout << "111height:" << k.Key_coordinate.z << "  222height:" << sk.Key_coordinate.z << endl;
			cout << "111area:" << k.vector3D[0]<<k.vector3D[1]<<k.vector3D[2] << "  222area:" << sk.vector3D[0] << sk.vector3D[1] << sk.vector3D[2] << endl;
			cout <<++pp<< endl;*/
			Eigen::Matrix4f matrix;
			bool keydistance = (get_Distance(matrix, k, sk) <3);
			if (match_by_height(k.Key_coordinate, sk.Key_coordinate) && match_by_area(k.vector3D, sk.vector3D) && match_by_occupied(k.Occupiedgrid, sk.Occupiedgrid)&& keydistance) {
				PairPoint p;
				p.point_i = k;
				p.point_j = sk;
				pairpoint.push_back(p);
				//pairpoint[point_num++] = p;
				//singlePointPair++;
			}
			//std::cout << match_by_height(k.Key_coordinate, sk.Key_coordinate) << match_by_area(k.vector3D, sk.vector3D) << match_by_occupied(k.Occupiedgrid, sk.Occupiedgrid) << std::endl;
		}
		/*if (singlePointPair >= 4) {
			for (int i = 0; i < singlePointPair; i++) {

			}
			float get = get_Distance(k.grid_value, pairpoint[ppnum - singlePointPair].point_j.Occupiedgrid.cloud, k.Key_coordinate, pairpoint[ppnum - singlePointPair].point_j.Key_coordinate,
				k.Border[0], k.Border[1], k.Border[2], k.Border[3], k.Border[4], k.Border[5]);

		}*/
	}
	cout << pairpoint.size() << "aaaa" << endl;
	Eigen::Matrix4f matrix = Ransac(pairpoint, 20,cloud,mcloud);
	pcl::transformPointCloud(*cloud, *cloud, matrix);		//ƽ�Ʊ任��ʹ�����ؼ����غ�

																	//���浽PCD�ļ�
	pcl::io::savePCDFileASCII("mcloud1.pcd", *mcloud); //�����Ʊ��浽PCD�ļ���
	pcl::io::savePCDFileASCII("transformed_cloud3.pcd", *cloud);
	clock_t ends = clock();			//��ʱ����
	cout << "Running Time : " << (double)(ends - start) / CLOCKS_PER_SEC << endl;			//���뻯Ϊ�룬���ΪԤ����һ�����ݿ�ģ�͵�ʱ��
	//��ʾ���ƴ���
	
	system("pcl_viewer.exe 11351.pcd transformed_cloud.pcd");


	int o = 0;
	while (true) {
		o--;
	}
	return 0;


	/*
	clock_t start = clock();		//��ʱ��ʼ

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("5.pcd", *cloud);

	ModelPoint modelpoint = ModelPoint(cloud);	//ģ�Ͷ����ʼ��
	modelpoint.getArea(cloud);	//�ָ�ƽ��
	modelpoint.getKeypoint();	//�õ��ؼ���

	int temp = 0;

	for (int i = 0; i < modelpoint.key_coordinates.size(); i++)		//���ι������еĹؼ������
	{
		KeyPoint k_point(modelpoint.key_coordinates[i]);

		k_point.get_Vector3D(modelpoint.surface);				//�õ���ά����


		for(int j=0;j<3;j++)
			if (k_point.vector3D[j] > 0.16)
			{
				temp++;
				break;
			}


		k_point.getOccupiedGrid(cloud);						//�õ�ռ������
		k_point.get_TSDF(cloud);							//�õ�tsdf
		modelpoint.keyPoint.push_back(k_point);				//���ؼ��������ӵ�ģ�Ͷ�����
	}
	std::cout << modelpoint.key_coordinates.size() <<"              "<< temp << std::endl;

	Eigen::Matrix4f t_m = Eigen::Matrix4f::Identity();
	int k = 9;	//ģ��
	int d = 9;	//
	double score = get_Distance(t_m, modelpoint.keyPoint[k].grid_value, modelpoint.keyPoint[d].Occupiedgrid.cloud, modelpoint.keyPoint[k].Key_coordinate, modelpoint.keyPoint[d].Key_coordinate, modelpoint.keyPoint[k].Border[0], modelpoint.keyPoint[k].Border[1], modelpoint.keyPoint[k].Border[2], modelpoint.keyPoint[k].Border[3], modelpoint.keyPoint[k].Border[4], modelpoint.keyPoint[k].Border[5]);
	cout << "score=" << score << endl;

	clock_t ends = clock();			//��ʱ����
	cout << "Running Time : " << (double)(ends - start) / CLOCKS_PER_SEC << endl;			//���뻯Ϊ�룬���ΪԤ����һ�����ݿ�ģ�͵�ʱ��
	*/

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

