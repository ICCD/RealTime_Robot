// RealTimeRobot.cpp : 定义控制台应用程序的入口点。
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
	
	
	//加载点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("11351.pcd", *cloud);
	pcl::PointCloud<pcl::PointXYZ>::Ptr mcloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("11351.pcd", *mcloud);

	Eigen::Matrix4f transform_mat = Eigen::Matrix4f::Identity();	//平移矩阵
	transform_mat(0, 3) = 0.8;
	transform_mat(1, 3) = 0.8;
	pcl::transformPointCloud(*mcloud, *mcloud, transform_mat);		//平移变换
	
	//实例化ScanPoint对象，传入点云
	ScanPoint scanpoint =  ScanPoint(cloud);
	scanpoint.getKeypoint();
	scanpoint.get_Area(cloud);

	//实例化Model对象，传入点云
	ModelPoint modelpoint = ModelPoint(mcloud);
	modelpoint.getKeypoint();
	modelpoint.getArea(mcloud);

	//keyPointICP(keypoint,mkeypoint,cloud,mcloud);/*测试展示用函数*/
	scanpoint.keyPoint.resize((scanpoint.key_coordinates).points.size());
	modelpoint.keyPoint.resize((modelpoint.key_coordinates).points.size());
	int count = 0;//计数
	for each (pcl::PointXYZ onepoint in scanpoint.key_coordinates)
	{
		KeyPoint p = KeyPoint(onepoint);
		p.getOccupiedGrid(cloud);
		p.get_TSDF(cloud);
		p.get_Vector3D(scanpoint.surface);
		scanpoint.keyPoint[count++]  = p;
		//std::cout << count << std::endl;
	}
	count = 0;
	for each (pcl::PointXYZ onepoint in modelpoint.key_coordinates)
	{
		KeyPoint p = KeyPoint(onepoint);
		p.getOccupiedGrid(cloud);
		p.get_TSDF(cloud);
		p.get_Vector3D(modelpoint.surface);
		modelpoint.keyPoint[count++] = p;
	}
	vector<PairPoint> pairpoint;
	int point_num = 0;
	for each (KeyPoint k in modelpoint.keyPoint)
	{
		int singlePointPair = 0;
		for each (KeyPoint sk in scanpoint.keyPoint)
		{
			if (match_by_height(k.Key_coordinate, sk.Key_coordinate) && match_by_area(k.vector3D, sk.vector3D) && match_by_occupied(k.Occupiedgrid, sk.Occupiedgrid)) {
				PairPoint p;
				p.point_i = k;
				p.point_j = sk;
				pairpoint[point_num++] = p;
				singlePointPair++;
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
	std::cout << "No Bug" << std::endl;
	int o = 0;
	while (true){
		o--;
	}
    return 0;

	
	/*
	clock_t start = clock();		//计时开始

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("5.pcd", *cloud);

	ModelPoint modelpoint = ModelPoint(cloud);	//模型对象初始化
	modelpoint.getArea(cloud);	//分割平面
	modelpoint.getKeypoint();	//得到关键点

	int temp = 0;

	for (int i = 0; i < modelpoint.key_coordinates.size(); i++)		//依次构造所有的关键点对象
	{
		KeyPoint k_point(modelpoint.key_coordinates[i]);

		k_point.get_Vector3D(modelpoint.surface);				//得到三维向量
		
		
		for(int j=0;j<3;j++)			
			if (k_point.vector3D[j] > 0.16)
			{
				temp++;
				break;
			}
		

		k_point.getOccupiedGrid(cloud);						//得到占据网格
		k_point.get_TSDF(cloud);							//得到tsdf
		modelpoint.keyPoint.push_back(k_point);				//将关键点对象添加到模型对象中
	}
	std::cout << modelpoint.key_coordinates.size() <<"              "<< temp << std::endl;

	Eigen::Matrix4f t_m = Eigen::Matrix4f::Identity();	
	int k = 9;
	int d = 9;
	double score = get_Distance(t_m, modelpoint.keyPoint[k].grid_value, modelpoint.keyPoint[d].Occupiedgrid.cloud, modelpoint.keyPoint[k].Key_coordinate, modelpoint.keyPoint[d].Key_coordinate, modelpoint.keyPoint[k].Border[0], modelpoint.keyPoint[k].Border[1], modelpoint.keyPoint[k].Border[2], modelpoint.keyPoint[k].Border[3], modelpoint.keyPoint[k].Border[4], modelpoint.keyPoint[k].Border[5]);
	cout << "score=" << score << endl;

	clock_t ends = clock();			//计时结束
	cout << "Running Time : " << (double)(ends - start) / CLOCKS_PER_SEC << endl;			//毫秒化为秒，结果为预处理一个数据库模型的时间
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


		k_point.getOccupiedGrid(cloud);								//测试占据网格
		std::cout << k_point.Occupiedgrid.cloud->size() << std::endl;

		k_point.get_TSDF(cloud);

		int pp = 0;						//测试向量场
		for (int i = 0; i < 12; i++)
			for (int j = 0; j < 12; j++)
				for (int k = 0; k < 12; k++)
					std::cout << k_point.grid_value[i][j][k] << std::endl;
*/
	

	return 0;

}

