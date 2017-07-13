#pragma once
//此文件存储公用函数，常数，宏定义
#include <key_point.h>
#include <pcl\registration\icp.h>
#include <pcl/pcl_base.h>
#include <pcl/common/transforms.h>
#include <pcl/pcl_macros.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <time.h>
#include <iostream>
#include"stdlib.h"

#define PI 3.1415926    //定义π
#define RANSAC_TIMES  100 //?

using namespace Eigen;
using namespace Eigen::internal;
using namespace Eigen::Architecture;
#define Random(x) (rand() % x) 

//关键点对应后生成的点对
struct PairPoint {
	KeyPoint point_i;
	KeyPoint point_j;
};
double pointdistance(pcl::PointXYZ p1, pcl::PointXYZ p2) {
	
	return sqrt((p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y) + (p1.z - p2.z)*(p1.z - p2.z));
}

///<summary>
///匹配关系筛选函数
///</summary>
Eigen::Matrix4f Ransac(vector<PairPoint> pairpoint,int ransac_times, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr mcloud) {
	Eigen::Matrix4f matrix;
	vector<Eigen::Matrix4f> em;
	double minidistance = 10000000.0;
	Eigen::Matrix4f bestMatrix;
	int p=0;
	int best_innum = 0;
	for (int i = 0; i < pairpoint.size(); i++)
	{
		//随机取一对关键点
		//int int_rand = rand() % pairpoint.size();
		PairPoint pair = pairpoint[i];
		// 根据该对关键点计算旋转矩阵
		get_Distance(matrix, pair.point_i,pair.point_j);
		//旋转扫描点云的关键点集合
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_i(new pcl::PointCloud<pcl::PointXYZ>); // 创建点云,模型点云
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_j(new pcl::PointCloud<pcl::PointXYZ>);
		
			(*cloud_i).height = 1; //设置点云高度
			(*cloud_i).width = pairpoint.size(); //设置点云宽度
			(*cloud_i).points.resize((*cloud_i).width * (*cloud_i).height); //变形，无序

			(*cloud_j).width = pairpoint.size(); //设置点云宽度
			(*cloud_j).height = 1; //设置点云高度
			(*cloud_j).points.resize((*cloud_j).width * (*cloud_j).height); //变形，无序

			int pointnum = 0;
			for each (PairPoint pp in pairpoint)
			{
				(*cloud_i).points[pointnum] = pp.point_i.Key_coordinate;
				(*cloud_j).points[pointnum++] = pp.point_j.Key_coordinate;
			}
			pcl::transformPointCloud(*cloud_j, *cloud_j, matrix);		//变换
		//计算此次匹配的局内点数
			Eigen::Matrix4f noUse;
			int	inline_num = 0;
		
			for (int j = 0; j < pairpoint.size(); j++)
			{
				double dis = pointdistance((*cloud_i).points[j], (*cloud_j).points[j]);
				//cout << dis << "ffffffffff" << endl;
				float  des = get_Distance(noUse, pairpoint[j].point_i, pairpoint[j].point_j);
	
				if (dis<0.15&&des<100)
				{
					inline_num++;
				}
			}
			if (inline_num > best_innum) {
				bestMatrix = matrix;
				best_innum = inline_num;
				p = 0;
				em.clear();
			}else if(inline_num=best_innum) {
				cout << p++ << "+++++++++++++++++++++++++++++" << endl;
				em.push_back(matrix);
			//pcl::transformPointCloud(*cloud, *cloud, matrix);
			//pcl::io::savePCDFileASCII("mcloud1.pcd", *mcloud); //将点云保存到PCD文件中
			//pcl::io::savePCDFileASCII("transformed_cloud1.pcd", *cloud);
			//system("pcl_viewer.exe  mcloud1.pcd transformed_cloud1.pcd");
			}
			cout << inline_num << "================================" << endl;
		
	}
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr gg(new pcl::PointCloud<pcl::PointXYZ>);
	//for each (Eigen::Matrix4f var in em)
	//{
	//	pcl::transformPointCloud(*cloud, *gg, var);
	//	pcl::io::savePCDFileASCII("mcloud1.pcd", *mcloud); //将点云保存到PCD文件中
	//	pcl::io::savePCDFileASCII("transformed_cloud1.pcd", *gg);
	//	system("pcl_viewer.exe  mcloud1.pcd transformed_cloud1.pcd");
	//}
		return bestMatrix;
}

void  keyPointICP(pcl::PointCloud<pcl::PointXYZ>::Ptr SpointCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr mPointCloud , pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr mcloud) {
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp; //创建ICP对象，用于ICP配准
	icp.setInputCloud(mcloud); //设置输入点云
	icp.setInputTarget(cloud); //设置目标点云（输入点云进行仿射变换，得到目标点云）
	pcl::PointCloud<pcl::PointXYZ> Final; //存储结果
										  //进行配准，结果存储在Final中
	icp.align(Final);
	//输出最终的变换矩阵（4x4）
	std::cout << icp.getFinalTransformation() << std::endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::transformPointCloud(*mcloud,*transformed_cloud, icp.getFinalTransformation());

	//保存到PCD文件
	pcl::io::savePCDFileASCII("mcloud.pcd", *mcloud); //将点云保存到PCD文件中
	pcl::io::savePCDFileASCII("transformed_cloud.pcd", *transformed_cloud);
	//显示点云窗口
	system("pcl_viewer.exe 11351.pcd mcloud.pcd");
	system("pcl_viewer.exe 11351.pcd transformed_cloud.pcd");
	//pcl::PointCloud<pcl::PointXYZ> cheng;
	//cheng.width = (*mcloud).width;
	//cheng.height = (*mcloud).height;
	//cheng.is_dense = false;
	//cheng.points.resize(cheng.width*cheng.height);
	//int tnum = 0;
	//for each (pcl::PointXYZ v in *mcloud)
	//{
	//	cheng.points[tnum].x = v.x + 1;
	//	cheng.points[tnum].y= v.y + 0.5;
	//	cheng.points[tnum++].z = v.z;
	//}
	//pcl::io::savePCDFileASCII("70761_c.pcd", cheng); //将点云保存到PCD文件中
	int o = 0;
	while (true) {
		o--;
	}
}

/**
*输入：关键点对：PairPoint
*输出：4*4矩阵 ：Matrix4
*用途：根据关键点对计算刚性变换矩阵
**/
//Eigen::Matrix4f getMatrixfromPairPoint(PairPoint p) {
//	
//}
bool match_by_height(pcl::PointXYZ &key1, pcl::PointXYZ &key2)		
{
	float temp = key1.z / key2.z;
	if (temp >= float(2 / 3) || temp <= 1.5)
		return true;
	return false;
}
bool match_by_area(vector<double> v1, vector<double> v2)
{
	if ((v1[0] / v2[0]) > 3 || (v1[0] / v2[0]) < float(1 / 3) || (v1[1] / v2[1]) > 3 || (v1[1] / v2[1]) < float(1 / 3) || (v1[2] / v2[2]) > 3 || (v1[2] / v2[2]) < float(1 / 3))
		return false;
	return true;
}
bool match_by_occupied(OccupiedGrid &o1, OccupiedGrid &o2)
{
	if (o2.Number == 0) return false;	//defined by xueyu 解決除數為零問題
	float temp = o1.Number / o2.Number;
	if (temp > 2 || temp < 0.5)
		return false;
	return true;
}

