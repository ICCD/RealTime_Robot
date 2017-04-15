#pragma once
//此文件存储公用函数，常数，宏定义
#include <key_point.h>
#include <pcl\registration\icp.h>
#include <pcl/pcl_base.h>
#include <pcl/common/transforms.h>
#include <pcl/pcl_macros.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>


#define PI 3.1415926    //定义π
#define RANSAC_TIMES  100 //?

using namespace Eigen;

//关键点对应后生成的点对
struct PairPoint {
	KeyPoint point_i;
	KeyPoint point_j;
};

///<summary>
///匹配关系筛选函数
///</summary>
void Ransac(vector<PairPoint> pairpoint,int ransac_times) {
	int *count = new int[RANSAC_TIMES];

	for (int i = 0; i < RANSAC_TIMES; i++) {
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_i; // 创建点云
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_j;
		 //填充点云数据
		(*cloud_i).width = 5; //设置点云宽度
		(*cloud_i).height = 1; //设置点云高度
		(*cloud_i).is_dense = false; //非密集型
		(*cloud_i).points.resize((*cloud_i).width * (*cloud_i).height); //变形，无序

		(*cloud_j).width = 5; //设置点云宽度
		(*cloud_j).height = 1; //设置点云高度
		(*cloud_j).is_dense = false; //非密集型
		(*cloud_j).points.resize((*cloud_j).width * (*cloud_j).height); //变形，无序
		 //设置这些点的坐标
		int pointnum = 0;
		for each (PairPoint pair in pairpoint)
		{
			(*cloud_i).points[pointnum] = pair.point_i.Key_coordinate;
			(*cloud_j).points[pointnum] = pair.point_j.Key_coordinate;
		}
		//*********************************
		// ICP配准
		//*********************************
		pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp; //创建ICP对象，用于ICP配准
		icp.setInputCloud(cloud_i); //设置输入点云
		icp.setInputTarget(cloud_j); //设置目标点云（输入点云进行仿射变换，得到目标点云）
		pcl::PointCloud<pcl::PointXYZ> Final; //存储结果
											  //进行配准，结果存储在Final中
		icp.align(Final);
		//输出最终的变换矩阵（4x4）
		std::cout << icp.getFinalTransformation() << std::endl;
		icp.getFitnessScore();
		
	}
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


