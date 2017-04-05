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
int main()
{
	//���ص���
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("11351.pcd", *cloud);			//ɨ�����
	pcl::PointCloud<pcl::PointXYZ>::Ptr mcloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("70761_c.pcd", *mcloud);

	//ʵ����ScanPoint���󣬴������
	ScanPoint scanpoint =  ScanPoint(cloud);
	scanpoint.get_Area(cloud);
	scanpoint.getKeypoint();
	//ʵ����Model���󣬴������

	ModelPoint modelpoint = ModelPoint(mcloud);
	modelpoint.getArea(mcloud);
	modelpoint.getKeypoint();
	//��ȡ�ؼ���
	

		
	scanpoint.keyPoint.resize((scanpoint.key_coordinates).points.size());		//ɨ����Ƴ�ʼ��
	int count = 0;//����
	for each (pcl::PointXYZ onepoint in scanpoint.key_coordinates)
	{
		KeyPoint p = KeyPoint(onepoint);

		p.getOccupiedGrid(cloud);
		p.get_TSDF(cloud);
		p.get_Vector3D(scanpoint.surface);

		scanpoint.keyPoint[count++]  = p;
	}

	modelpoint.keyPoint.resize((modelpoint.key_coordinates).points.size());		//ģ�͵��Ƴ�ʼ��		��������ģ��
	int mcount = 0;//����
	for each (pcl::PointXYZ onepoint in modelpoint.key_coordinates)
	{
		KeyPoint p = KeyPoint(onepoint);

		p.getOccupiedGrid(cloud);
		p.get_TSDF(cloud);
		p.get_Vector3D(modelpoint.surface);

		modelpoint.keyPoint[mcount++] = p;
	}
	pcl::PointCloud<pcl::PointXYZ>::Ptr p_temp1(pcl::PointCloud<pcl::PointXYZ>);

	//keyPointICP(scanpoint.key_coordinates, modelpoint.key_coordinates, cloud, mcloud);
	vector<PairPoint> pairpoint;     //��Źؼ���Եļ���
	int ppnum = 0;
	for each (KeyPoint k in modelpoint.keyPoint)
	{
		int singlePointPair = 0;
		for each (KeyPoint sk in scanpoint.keyPoint)
		{
			if (match_by_height(k.Key_coordinate, sk.Key_coordinate) && match_by_area(k.vector3D, sk.vector3D) && match_by_occupied(k.Occupiedgrid, sk.Occupiedgrid)) {
				PairPoint p;
				p.point_i = k;
				p.point_j = sk;
				pairpoint[ppnum++] = p;
				singlePointPair++;
			}
		}
		if (singlePointPair >= 4) {
			for (int i = 0; i < singlePointPair; i++) {

			}
			float get = get_Distance(k.grid_value, pairpoint[ppnum - singlePointPair].point_j.Occupiedgrid.cloud, k.Key_coordinate, pairpoint[ppnum - singlePointPair].point_j.Key_coordinate,
				k.Border[0], k.Border[1], k.Border[2], k.Border[3], k.Border[4], k.Border[5]);

		}
	}
	

	std::cout << "No Bug" << std::endl;
	int o = 0;
	while (true){
		o--;
	}
    return 0;
}

